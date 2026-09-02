#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <esp_camera.h>
#include <esp_wifi.h>
#include <Dynamixel2Arduino.h>

#include "q8Dynamixel.h"
#include "klog.h"
#include "pinMapping.h"

// ============================================================================
// XIAO ESP32S3 Sense (OV2640) 카메라 핀 — Seeed 공식 예제
// (esp32 arduino core CameraWebServer 예제의 camera_pins.h,
//  CAMERA_MODEL_XIAO_ESP32S3 항목 값을 그대로 사용)
// D6/D7(=GPIO43/44, Dynamixel UART)와 겹치지 않음을 확인함.
// ============================================================================
#define CAM_PWDN_PIN    -1
#define CAM_RESET_PIN   -1
#define CAM_XCLK_PIN    10
#define CAM_SIOD_PIN    40
#define CAM_SIOC_PIN    39
#define CAM_Y9_PIN      48
#define CAM_Y8_PIN      11
#define CAM_Y7_PIN      12
#define CAM_Y6_PIN      14
#define CAM_Y5_PIN      16
#define CAM_Y4_PIN      18
#define CAM_Y3_PIN      17
#define CAM_Y2_PIN      15
#define CAM_VSYNC_PIN   38
#define CAM_HREF_PIN    47
#define CAM_PCLK_PIN    13

// ============================================================================
// 전역 객체
// ============================================================================
Dynamixel2Arduino q8dxl(Serial1, DXL_DIR_PIN);
q8Dynamixel       q8(q8dxl);
AsyncUDP          udp;
WiFiServer        camServer(80);

// cameraReady/camServer: 카메라 전용 태스크(core 0)에서만 접근 — setup()에서 초기화 후
// cameraTask 시작, loop()/motor 경로는 더 이상 참조하지 않는다.
// ============================================================================
// 진단 로그: 시리얼 + UDP 브로드캐스트(192.168.4.255:9999)
// 모터 전원이 켜져 있으면 USB가 열거되지 않아(klog.h 주석 참고) 시리얼만으로는
// 구동 중 로그를 볼 수 없다.
// ============================================================================
void klog(const char* fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;

  // USB 미연결 시 write가 막힐 수 있어 연결 여부를 확인하고 쓴다.
  if (Serial) Serial.write((const uint8_t*)buf, n);
  udp.broadcastTo((uint8_t*)buf, n, KLOG_PORT);
}

// softAP 채널. 기본값 1은 주변 AP가 가장 붐비는 대역이라 UDP 유실의 직접 원인이 된다.
// 2026-08-24 현장 스캔: ch1에 신호 100짜리 AP, ch11에 90, ch6은 최강 간섭원이 50 -> 6 선택.
// 장소가 바뀌면 `nmcli dev wifi list --rescan yes`로 다시 재서 고를 것 - 측정 없이 바꾸면 악화된다.
static const int AP_CHANNEL = 6;

bool cameraReady = false;

// 카메라가 airtime을 독점하면 UDP 모션 패킷이 유실돼 500ms 워치독이 물린다 — 20fps 상한
static const uint32_t CAM_MIN_FRAME_INTERVAL_MS = 50;

// 안전 정지 상태 (500ms 무수신 시 torque off 1회)
// lastValidPacketMs: WiFi 콜백 태스크(쓰기)와 loop 태스크(읽기)가 공유 -> volatile 유지
// torqueSafetyTripped: loop 태스크만 읽고 쓴다(복구도 loop에서 수행) -> volatile 불필요
volatile unsigned long lastValidPacketMs = 0;
bool torqueSafetyTripped = false;

// 모션 패킷 중복 폐기용 마지막 seq (onUdpPacket 콜백에서만 접근하는 단일 쓰기자)
bool haveSeq = false;
uint16_t lastSeq = 0;

// 모션 패킷 tick 유효 범위: 단일 회전 Position 모드 0~4095, 중립 1024 벗어나면 폐기
static const int32_t TICK_MIN = 0;
static const int32_t TICK_MAX = 4095;

// ============================================================================
// Dynamixel 접근 직렬화용 큐
// AsyncUDP onPacket 콜백은 WiFi 태스크에서 실행되므로 q8dxl(UART, half-duplex)에
// 직접 접근하면 loop() 태스크와 경합한다. 콜백은 검증/파싱만 하고 명령을 큐에
// 넣으며, 실제 Dynamixel UART 접근은 loop() 태스크 한 곳에서만 수행한다
// (원본 q8bot의 단일 태스크 소유 방식과 동일).
//
// 모션(motionQueue)과 커맨드(cmdQueue)를 분리한다:
// - motionQueue: 크기 1 + xQueueOverwrite. 모션은 "최신 자세"만 의미가 있으므로
//   오래된 목표를 큐에 쌓아두면 처리 지연이 누적된다 — 항상 최신 값으로 덮어쓴다.
// - cmdQueue: torque on/off, jump는 유실되면 안 되는 이벤트이므로 기존 FIFO(크기 4)
//   유지. processDxlQueue는 cmdQueue를 먼저 소비하고, 그다음 motionQueue를 소비한다.
// ============================================================================
struct MotionCmd {
  int32_t ticks[8];
  uint16_t dur;
};

static QueueHandle_t motionQueue = NULL;
static QueueHandle_t cmdQueue = NULL;

// ============================================================================
// UDP 패킷 처리 (docs/protocol.md 참조)
// 아래 handle*는 WiFi 콜백 태스크에서 실행 — 검증 후 큐 적재만 하고 반환한다.
// ============================================================================
uint8_t xorChecksum(const uint8_t* data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) sum ^= data[i];
  return sum;
}

void handleMotionPacket(const uint8_t* data) {
  uint16_t seq = data[0] | (data[1] << 8);
  if (haveSeq && seq == lastSeq) return;  // 중복 패킷 무시
  haveSeq = true;
  lastSeq = seq;

  MotionCmd motionCmd;
  for (int i = 0; i < 8; i++) {
    int32_t v = data[2 + i * 2] | (data[2 + i * 2 + 1] << 8);
    if (v < TICK_MIN || v > TICK_MAX) return;  // 범위 밖 값이 하나라도 있으면 패킷 전체 폐기
    motionCmd.ticks[i] = v;
  }
  motionCmd.dur = data[18] | (data[19] << 8);

  lastValidPacketMs = millis();
  xQueueOverwrite(motionQueue, &motionCmd);  // 항상 최신 자세만 유지(크기 1 큐)
}

void handleCommandPacket(const uint8_t* data) {
  uint8_t cmd = data[1];
  if (cmd != 0 && cmd != 1 && cmd != 4) return;  // 알 수 없는 cmd는 폐기

  lastValidPacketMs = millis();
  xQueueSend(cmdQueue, &cmd, 0);  // 큐가 가득 차면 드롭(non-blocking, 콜백을 막지 않음)
}

void onUdpPacket(AsyncUDPPacket packet) {
  size_t len = packet.length();
  const uint8_t* data = packet.data();

  if (len == 21) {
    if (xorChecksum(data, 20) != data[20]) return;  // 체크섬 불일치 폐기
    handleMotionPacket(data);
  } else if (len == 3) {
    if (data[0] != 0xFF) return;                    // 커맨드 매직 불일치
    if (xorChecksum(data, 2) != data[2]) return;
    handleCommandPacket(data);
  }
  // 그 외 길이는 폐기
}

// ============================================================================
// 안전 정지: 마지막 유효 패킷 후 500ms 경과 시 torque off 1회 (loop 태스크에서만 호출)
// ============================================================================
void checkSafety() {
  if (!torqueSafetyTripped && millis() - lastValidPacketMs > 500) {
    q8.disableTorque();
    torqueSafetyTripped = true;
    // 로그만으로 "통신 끊겨 안전정지"와 "보드 재부팅"을 구분할 수 있어야 한다.
    // 재부팅이면 이 줄 없이 [BOOT]가 바로 나온다.
    klog("[SAFE] t=%lu 500ms 무수신 -> torque off\n", (unsigned long)millis());
  }
}

// ============================================================================
// cmdQueue/motionQueue 소비 — 모든 Dynamixel UART 접근은 이 함수(loop 태스크)에서만
// 수행한다. cmdQueue(유실 불가 이벤트)를 먼저 소비하고, 없으면 motionQueue(최신
// 자세)를 소비한다. 반환값은 뭔가 처리했는지 여부(loop의 idle 판단용).
// ============================================================================
bool processDxlQueue() {
  uint8_t ctrlCmd;
  if (xQueueReceive(cmdQueue, &ctrlCmd, 0) == pdTRUE) {
    // 안전 정지로 torque off된 상태에서 유효 명령 수신 시 재활성화 후 적용
    // (단, 명령 자체가 torque off면 재활성화 펄스 없이 tripped 해제만)
    if (torqueSafetyTripped && ctrlCmd == 0) {
      torqueSafetyTripped = false;  // 이미 torque off 상태 -> 명령 결과와 동일
      return true;
    }
    if (torqueSafetyTripped) {
      q8.enableTorque();
      torqueSafetyTripped = false;
    }

    switch (ctrlCmd) {
      case 0: q8.disableTorque(); break;
      case 1: q8.enableTorque(); break;
      case 4:
        q8.jump();
        // jump()는 내부에서 약 7.3초 delay(blocking)로 진행된다. 그동안 loop는
        // checkSafety를 호출하지 못하므로 종료 시점 기준으로 lastValidPacketMs를
        // 갱신해 직후 오탐(안전 정지)을 막는다. (jump 전용 유예 타이머를 두는
        // 것보다 단순하고, jump는 이미 loop 태스크를 독점하므로 다른 패킷도
        // 그 사이 처리되지 않기 때문에 이 갱신만으로 충분하다.)
        lastValidPacketMs = millis();
        // jump() 진행 중(약 7.3초) 쌓인 motionQueue의 자세는 최대 7.3초 전 목표라
        // 착지 직후 그대로 적용되면 위험 — 큐를 비워 다음 모션 패킷부터 반영되게 한다.
        xQueueReset(motionQueue);
        break;
      default: break;
    }
    return true;
  }

  MotionCmd motionCmd;
  if (xQueueReceive(motionQueue, &motionCmd, 0) == pdTRUE) {
    if (torqueSafetyTripped) {
      q8.enableTorque();
      torqueSafetyTripped = false;
    }
    q8.ensureProfile(motionCmd.dur);
    q8.bulkWrite(motionCmd.ticks);
    return true;
  }

  return false;
}

// ============================================================================
// 카메라 MJPEG 스트리밍 (QVGA, 포트 80)
// ============================================================================
bool cameraInit() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_Y2_PIN;
  config.pin_d1 = CAM_Y3_PIN;
  config.pin_d2 = CAM_Y4_PIN;
  config.pin_d3 = CAM_Y5_PIN;
  config.pin_d4 = CAM_Y6_PIN;
  config.pin_d5 = CAM_Y7_PIN;
  config.pin_d6 = CAM_Y8_PIN;
  config.pin_d7 = CAM_Y9_PIN;
  config.pin_xclk = CAM_XCLK_PIN;
  config.pin_pclk = CAM_PCLK_PIN;
  config.pin_vsync = CAM_VSYNC_PIN;
  config.pin_href = CAM_HREF_PIN;
  config.pin_sccb_sda = CAM_SIOD_PIN;
  config.pin_sccb_scl = CAM_SIOC_PIN;
  config.pin_pwdn = CAM_PWDN_PIN;
  config.pin_reset = CAM_RESET_PIN;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;                       // PSRAM 여유 큼 — 캡처/전송 병렬화로 fps 개선
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;     // 오래된 프레임 대신 항상 최신 프레임 사용

  return esp_camera_init(&config) == ESP_OK;
}

void handleCameraClient() {
  if (!cameraReady) return;

  WiFiClient client = camServer.available();
  if (!client) return;

  // 작은 write가 Nagle로 지연 결합되면 프레임 지연/버스트가 커진다.
  client.setNoDelay(true);

  // HTTP 응답 헤더를 한 번의 write로 합쳐 보낸다(기존엔 println() 3회 = TCP 세그먼트 최대 3개).
  // 끝의 빈 줄(헤더/바디 구분 CRLF)은 아래 프레임 헤더의 선행 "\r\n"이 대신 제공한다 —
  // 그래야 첫 프레임을 포함해 바이트 시퀀스가 기존과 동일하게 유지된다.
  char httpHeader[128];
  int hLen = snprintf(httpHeader, sizeof(httpHeader),
      "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n");
  // snprintf는 절단 시 "쓰였을 길이"를 반환한다 - 그대로 write에 넘기면 버퍼 밖을 읽는다.
  if (hLen > (int)sizeof(httpHeader) - 1) hLen = sizeof(httpHeader) - 1;
  client.write((const uint8_t*)httpHeader, hLen);

  int fbFailures = 0;
  uint32_t lastFrameMs = millis() - CAM_MIN_FRAME_INTERVAL_MS;  // 첫 프레임은 지연 없이 전송
  while (client.connected()) {
    // checkSafety/processDxlQueue 호출 없음: 카메라는 전용 태스크(core 0)로 분리되어
    // 더 이상 loop()를 점유하지 않는다 — 모션/안전 정지는 loop 태스크(core 1)가
    // 독립적으로 최대 속도로 처리한다. Dynamixel UART 접근은 여전히 loop 태스크
    // 한 곳(processDxlQueue)에서만 이루어진다.

    // 프레임 레이트 상한: UDP 모션 패킷과 airtime을 나눠 써야 한다(millis() 랩어라운드 안전 형태).
    uint32_t now = millis();
    if (now - lastFrameMs < CAM_MIN_FRAME_INTERVAL_MS) {
      vTaskDelay(pdMS_TO_TICKS(CAM_MIN_FRAME_INTERVAL_MS - (now - lastFrameMs)));
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      // 일시적 캡처 실패로 세션을 끊으면 브라우저가 스스로 복구하지 못한다
      // (<img> MJPEG은 자동 재연결이 없다). 몇 번은 참고 계속 간다.
      if (++fbFailures > 5) break;
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    fbFailures = 0;

    // 파트 헤더를 스택 버퍼 하나에 만들어 write 1회로 보낸다(기존엔 println() 2회 +
    // printf 1회 = TCP 세그먼트 최대 3개). 선행 "\r\n"은 이전 프레임 바디 뒤의 구분자 역할
    // (기존 코드의 client.println() 자리) — 이 CRLF가 어긋나면 멀티파트 경계가 깨져
    // 브라우저가 이후 프레임을 영영 못 만든다.
    size_t len = fb->len;
    char partHeader[128];
    int pLen = snprintf(partHeader, sizeof(partHeader),
        "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)len);
    if (pLen > (int)sizeof(partHeader) - 1) pLen = sizeof(partHeader) - 1;  // 위와 동일 이유
    client.write((const uint8_t*)partHeader, pLen);
    size_t sent = client.write(fb->buf, len);
    esp_camera_fb_return(fb);
    lastFrameMs = millis();

    // 부분 전송이면 멀티파트 경계가 깨져 브라우저가 이후 프레임을 영영 못 만든다.
    // 그대로 계속 보내면 "TCP는 살아있는데 화면만 멈춘" 상태가 되고 브라우저의
    // onerror도 안 뜬다 — 끊어서 재연결로 복구되게 한다.
    if (sent != len || !client.connected()) break;
  }
  client.stop();
}

// 카메라 전용 FreeRTOS 태스크(core 0, WiFi/lwIP와 같은 코어) — 모션 처리를
// 담당하는 loop() 태스크(core 1, Arduino 기본 태스크)와 분리해 스트리밍이
// 모션 지연에 영향을 주지 않게 한다. 우선순위를 낮게 두어(tskIDLE_PRIORITY+1)
// WiFi/lwIP 태스크를 방해하지 않는다.
void cameraTask(void* param) {
  for (;;) {
    handleCameraClient();
    vTaskDelay(1);  // 접속 클라이언트 없을 때 바쁜 대기 방지, core 0 다른 태스크에 양보
  }
}

// ============================================================================
void setup() {
  Serial.begin(115200);

  // 점프/보행 중 토크가 죽고 보드가 재부팅되는 현상의 원인 구분용.
  // BROWNOUT = 전원 새그(모터 피크 전류), PANIC/WDT = 펌웨어 크래시,
  // POWERON = 실제 전원 끊김(커넥터/배터리). 원인마다 대책이 완전히 다르다.
  esp_reset_reason_t rr = esp_reset_reason();
  const char* rrName =
      rr == ESP_RST_BROWNOUT ? "BROWNOUT(전원 새그)" :
      rr == ESP_RST_POWERON  ? "POWERON(전원 인가/끊김)" :
      rr == ESP_RST_PANIC    ? "PANIC(크래시)" :
      rr == ESP_RST_INT_WDT  ? "INT_WDT" :
      rr == ESP_RST_TASK_WDT ? "TASK_WDT" :
      rr == ESP_RST_SW       ? "SW(소프트 리셋)" : "기타";
  Serial.printf("\n[BOOT] reset reason = %d (%s)\n", (int)rr, rrName);
  // 이 시점엔 WiFi가 없어 UDP로 못 나간다 - softAP 기동 후 아래에서 다시 내보낸다.

  // Dynamixel half-duplex UART: RX=D7, TX=D6 (D라벨 고정)
  Serial1.begin(1000000, SERIAL_8N1, DXL_RX_PIN, DXL_TX_PIN);
  q8.begin();

  lastValidPacketMs = millis();

  motionQueue = xQueueCreate(1, sizeof(MotionCmd));
  cmdQueue = xQueueCreate(4, sizeof(uint8_t));
  if (motionQueue == NULL || cmdQueue == NULL) {
    Serial.println("[RTOS] Failed to create dxl queues - halting");
    while (1) { delay(1000); }  // Dynamixel 직렬화 불가 상태로 동작 금지
  }

  // softAP는 조용히 실패한다(브라운아웃 직후 RF 캘리브레이션/NVS 접근 실패 등).
  // 반환값을 안 보면 AP 없는 상태로 계속 돌아가 노트북이 영영 붙지 못한다.
  // 스택을 완전히 내렸다가 재시도하고, 그래도 안 되면 재부팅이 유일한 복구 수단.
  bool apUp = false;
  for (int attempt = 1; attempt <= 3 && !apUp; attempt++) {
    WiFi.mode(WIFI_AP);
    apUp = WiFi.softAP("kangyangi", "kangyangi", AP_CHANNEL);
    if (!apUp) {
      Serial.printf("[WIFI] softAP 기동 실패 (%d/3)\n", attempt);
      WiFi.mode(WIFI_OFF);
      delay(500);
    }
  }
  if (!apUp) {
    Serial.println("[WIFI] softAP 기동 불가 - 재부팅");
    Serial.flush();
    delay(200);
    ESP.restart();
  }
  esp_wifi_set_ps(WIFI_PS_NONE);  // softAP 절전 해제 — UDP 모션 패킷 지연/지터 감소

  // listen 실패 시 AP는 보이는데 제어만 안 먹는 상태가 된다 - 겉보기 정상이라 최악.
  if (!udp.listen(8888)) {
    Serial.println("[WIFI] UDP 8888 listen 실패 - 재부팅");
    Serial.flush();
    delay(200);
    ESP.restart();
  }
  udp.onPacket(onUdpPacket);

  // 재부팅 원인은 UDP 로그의 첫 줄로도 반드시 남아야 한다 - 모터 전원이 켜진
  // 상태에선 위쪽 Serial 출력을 볼 방법이 없기 때문이다.
  klog("[BOOT] reset reason = %d (%s)\n", (int)rr, rrName);
  klog("[WIFI] AP up, ip=%s\n", WiFi.softAPIP().toString().c_str());

  cameraReady = cameraInit();
  if (cameraReady) {
    camServer.begin();
    // core 0(WiFi/lwIP와 같은 코어)에 낮은 우선순위로 배치 — core 1의 모션 루프와
    // 자원을 다투지 않는다.
    // 스택 8192: WiFiClient/printf 경로 포함 시 4096은 여유 불확실 — esp32 카메라 예제 관례값
    xTaskCreatePinnedToCore(cameraTask, "camera", 8192, NULL, tskIDLE_PRIORITY + 1, NULL, 0);
  } else {
    Serial.println("[CAM] init failed - camera disabled, motor control continues");
  }
}

void loop() {
  checkSafety();
  q8.telemetry();  // 자체적으로 20ms 간격 조절 — 모션 주기에 영향 없음
  bool processed = processDxlQueue();
  if (!processed) delay(1);  // 큐가 빌 때만 idle 태스크에 양보(와치독 여유), 있으면 최대 속도 유지
}
