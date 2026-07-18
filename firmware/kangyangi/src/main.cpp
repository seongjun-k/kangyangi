#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <esp_camera.h>
#include <esp_wifi.h>
#include <Dynamixel2Arduino.h>
#include <I2S.h>  // arduino-esp32 3.x core: driver/i2s.h를 감싼 I2SClass(PDM RX 지원)

#include "q8Dynamixel.h"
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
// XIAO ESP32S3 Sense 온보드 PDM 마이크 핀 (Sense 확장보드, Seeed 공식 문서 값)
// GPIO41=DATA, GPIO42=CLK — 카메라 핀(10-18,38-40,47,48)과 Dynamixel D6/D7(=43/44)
// 어느 것과도 겹치지 않음.
// ============================================================================
#define MIC_DATA_PIN    41
#define MIC_CLK_PIN     42

// ============================================================================
// 전역 객체
// ============================================================================
Dynamixel2Arduino q8dxl(Serial1, DXL_DIR_PIN);
q8Dynamixel       q8(q8dxl);
AsyncUDP          udp;
WiFiServer        camServer(80);
WiFiServer        micServer(81);

// cameraReady/camServer: 카메라 전용 태스크(core 0)에서만 접근 — setup()에서 초기화 후
// cameraTask 시작, loop()/motor 경로는 더 이상 참조하지 않는다.
bool cameraReady = false;

// micReady/micServer/I2S: 마이크 전용 태스크(core 0)에서만 접근 — cameraReady와 동일 패턴.
bool micReady = false;

// 안전 정지 상태 (500ms 무수신 시 torque off 1회)
// lastValidPacketMs: WiFi 콜백 태스크(쓰기)와 loop 태스크(읽기)가 공유 -> volatile 유지
// torqueSafetyTripped: loop 태스크만 읽고 쓴다(복구도 loop에서 수행) -> volatile 불필요
volatile unsigned long lastValidPacketMs = 0;
bool torqueSafetyTripped = false;

// 모션 패킷 중복 폐기용 마지막 seq (onUdpPacket 콜백에서만 접근하는 단일 쓰기자)
bool haveSeq = false;
uint16_t lastSeq = 0;

// 모션 패킷 tick 유효 범위: 중립 4096(HOMING_OFFSET) 기준 0~8191(13bit 미만) 벗어나면 폐기
static const int32_t TICK_MIN = 0;
static const int32_t TICK_MAX = 8191;

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

  if (len == 19) {
    if (xorChecksum(data, 18) != data[18]) return;  // 체크섬 불일치 폐기
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

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println();

  while (client.connected()) {
    // checkSafety/processDxlQueue 호출 없음: 카메라는 전용 태스크(core 0)로 분리되어
    // 더 이상 loop()를 점유하지 않는다 — 모션/안전 정지는 loop 태스크(core 1)가
    // 독립적으로 최대 속도로 처리한다. Dynamixel UART 접근은 여전히 loop 태스크
    // 한 곳(processDxlQueue)에서만 이루어진다.
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) break;

    client.println("--frame");
    client.println("Content-Type: image/jpeg");
    client.printf("Content-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.println();
    esp_camera_fb_return(fb);

    if (!client.connected()) break;
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
// 마이크 오디오 스트리밍 (raw PCM 16kHz/16bit/mono, 포트 81)
// ============================================================================
// DMA 버퍼 512샘플 x 16bit x 2버퍼 -> 오디오는 16kHz*2byte=32KB/s로 카메라(수백KB/s
// JPEG) 대비 대역폭이 미미하다. 버퍼 크기를 키울 이유가 없어 기본값 근처로 둔다.
static const int MIC_SAMPLE_RATE = 16000;
static const int MIC_DMA_BUF_LEN = 512;

bool micInit() {
  // PDM RX 모드에서는 클럭이 ws(fs) 슬롯으로 출력된다(번들 I2S 라이브러리
  // _applyPinSetting 매핑) — Seeed 공식 예제와 동일하게 (bck, ws, data_out,
  // data_in, mck) 인자 순서 중 ws 자리에 CLK, data_in 자리에 DATA를 넣는다.
  I2S.setAllPins(-1, MIC_CLK_PIN, MIC_DATA_PIN, -1, -1);
  I2S.setBufferSize(MIC_DMA_BUF_LEN);
  return I2S.begin(PDM_MONO_MODE, MIC_SAMPLE_RATE, 16) == 1;
}

void handleMicClient() {
  if (!micReady) return;

  WiFiClient client = micServer.available();
  if (!client) return;  // 클라이언트 없음 -> I2S.read 호출 자체를 하지 않아 CPU/버스 낭비 없음

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: audio/L16;rate=16000;channels=1");
  client.println();

  uint8_t buf[MIC_DMA_BUF_LEN * 2];  // 16bit 샘플 -> 2byte
  while (client.connected()) {
    // checkSafety/processDxlQueue 호출 없음: 카메라 태스크와 동일하게 모션 처리는
    // loop() 태스크(core 1)가 독립적으로 담당한다. client.write가 블로킹돼도
    // 영향받는 것은 이 태스크(core 0)뿐이다.
    int n = I2S.read(buf, sizeof(buf));
    if (n <= 0) continue;
    client.write(buf, n);
    if (!client.connected()) break;
  }
  client.stop();
}

// 마이크 전용 FreeRTOS 태스크(core 0) — cameraTask와 동일한 패턴으로 core 1의
// 모션 루프와 분리한다. 우선순위 낮게(tskIDLE_PRIORITY+1)두어 WiFi/lwIP를 방해하지 않는다.
void micTask(void* param) {
  for (;;) {
    handleMicClient();
    vTaskDelay(1);  // 접속 클라이언트 없을 때 바쁜 대기 방지, core 0 다른 태스크에 양보
  }
}

// ============================================================================
void setup() {
  Serial.begin(115200);

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

  WiFi.mode(WIFI_AP);
  WiFi.softAP("kangyangi", "kangyangi");
  esp_wifi_set_ps(WIFI_PS_NONE);  // softAP 절전 해제 — UDP 모션 패킷 지연/지터 감소

  udp.listen(8888);
  udp.onPacket(onUdpPacket);

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

  micReady = micInit();
  if (micReady) {
    micServer.begin();
    // 스택 8192: cameraTask와 동일 근거(WiFiClient/printf 경로 포함 시 4096 여유 불확실)
    xTaskCreatePinnedToCore(micTask, "mic", 8192, NULL, tskIDLE_PRIORITY + 1, NULL, 0);
  } else {
    Serial.println("[MIC] init failed - mic disabled, motor/camera control continues");
  }
}

void loop() {
  checkSafety();
  bool processed = processDxlQueue();
  if (!processed) delay(1);  // 큐가 빌 때만 idle 태스크에 양보(와치독 여유), 있으면 최대 속도 유지
}
