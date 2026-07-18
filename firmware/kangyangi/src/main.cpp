#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <esp_camera.h>
#include <Dynamixel2Arduino.h>

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
// 전역 객체
// ============================================================================
Dynamixel2Arduino q8dxl(Serial1, DXL_DIR_PIN);
q8Dynamixel       q8(q8dxl);
AsyncUDP          udp;
WiFiServer        camServer(80);

bool cameraReady = false;

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
// ============================================================================
enum DxlCmdType : uint8_t {
  DXL_CMD_MOTION = 0,
  DXL_CMD_CONTROL = 1,
};

struct DxlCommand {
  uint8_t type;       // DxlCmdType
  int32_t ticks[8];   // type == MOTION일 때 사용
  uint8_t ctrlCmd;    // type == CONTROL일 때 사용 (0=disable,1=enable,4=jump)
};

static QueueHandle_t dxlQueue = NULL;

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

  DxlCommand dxlCmd;
  dxlCmd.type = DXL_CMD_MOTION;
  for (int i = 0; i < 8; i++) {
    int32_t v = data[2 + i * 2] | (data[2 + i * 2 + 1] << 8);
    if (v < TICK_MIN || v > TICK_MAX) return;  // 범위 밖 값이 하나라도 있으면 패킷 전체 폐기
    dxlCmd.ticks[i] = v;
  }

  lastValidPacketMs = millis();
  xQueueSend(dxlQueue, &dxlCmd, 0);  // 큐가 가득 차면 드롭(non-blocking, 콜백을 막지 않음)
}

void handleCommandPacket(const uint8_t* data) {
  uint8_t cmd = data[1];
  if (cmd != 0 && cmd != 1 && cmd != 4) return;  // 알 수 없는 cmd는 폐기

  DxlCommand dxlCmd;
  dxlCmd.type = DXL_CMD_CONTROL;
  dxlCmd.ctrlCmd = cmd;

  lastValidPacketMs = millis();
  xQueueSend(dxlQueue, &dxlCmd, 0);
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
// dxlQueue 소비 — 모든 Dynamixel UART 접근은 이 함수(loop 태스크)에서만 수행
// ============================================================================
void processDxlQueue() {
  DxlCommand dxlCmd;
  if (xQueueReceive(dxlQueue, &dxlCmd, 0) != pdTRUE) return;

  // 안전 정지로 torque off된 상태에서 유효 명령 수신 시 재활성화 후 적용
  // (단, 명령 자체가 torque off면 재활성화 펄스 없이 tripped 해제만)
  if (torqueSafetyTripped && dxlCmd.type == DXL_CMD_CONTROL && dxlCmd.ctrlCmd == 0) {
    torqueSafetyTripped = false;  // 이미 torque off 상태 -> 명령 결과와 동일
    return;
  }
  if (torqueSafetyTripped) {
    q8.enableTorque();
    torqueSafetyTripped = false;
  }

  if (dxlCmd.type == DXL_CMD_MOTION) {
    q8.bulkWrite(dxlCmd.ticks);
  } else {
    switch (dxlCmd.ctrlCmd) {
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
        break;
      default: break;
    }
  }
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
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

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
    checkSafety();      // 스트리밍 중에도 안전 정지 감시 유지
    processDxlQueue();  // 스트리밍 루프가 loop()를 점유하므로 여기서도 모션 큐 소비(미소비 시 로봇 고정)

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

// ============================================================================
void setup() {
  Serial.begin(115200);

  // Dynamixel half-duplex UART: RX=D7, TX=D6 (D라벨 고정)
  Serial1.begin(1000000, SERIAL_8N1, DXL_RX_PIN, DXL_TX_PIN);
  q8.begin();

  lastValidPacketMs = millis();

  dxlQueue = xQueueCreate(4, sizeof(DxlCommand));
  if (dxlQueue == NULL) {
    Serial.println("[RTOS] Failed to create dxlQueue - halting");
    while (1) { delay(1000); }  // Dynamixel 직렬화 불가 상태로 동작 금지
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP("kangyangi", "kangyangi");

  udp.listen(8888);
  udp.onPacket(onUdpPacket);

  cameraReady = cameraInit();
  if (cameraReady) {
    camServer.begin();
  } else {
    Serial.println("[CAM] init failed - camera disabled, motor control continues");
  }
}

void loop() {
  checkSafety();
  processDxlQueue();
  handleCameraClient();
}
