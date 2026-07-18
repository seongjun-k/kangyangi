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
volatile unsigned long lastValidPacketMs = 0;
bool torqueSafetyTripped = false;

// 모션 패킷 중복 폐기용 마지막 seq
bool haveSeq = false;
uint16_t lastSeq = 0;

// ============================================================================
// UDP 패킷 처리 (docs/protocol.md 참조)
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

  int32_t ticks[8];
  for (int i = 0; i < 8; i++) {
    ticks[i] = data[2 + i * 2] | (data[2 + i * 2 + 1] << 8);
  }
  q8.bulkWrite(ticks);

  lastValidPacketMs = millis();
  torqueSafetyTripped = false;
}

void handleCommandPacket(const uint8_t* data) {
  uint8_t cmd = data[1];
  switch (cmd) {
    case 0: q8.disableTorque(); break;
    case 1: q8.enableTorque(); break;
    case 4: q8.jump(); break;
    default: break;  // 알 수 없는 cmd는 무시
  }

  lastValidPacketMs = millis();
  torqueSafetyTripped = false;
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
// 안전 정지: 마지막 유효 패킷 후 500ms 경과 시 torque off 1회
// ============================================================================
void checkSafety() {
  if (!torqueSafetyTripped && millis() - lastValidPacketMs > 500) {
    q8.disableTorque();
    torqueSafetyTripped = true;
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
    checkSafety();  // 스트리밍 중에도 안전 정지 감시 유지

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
  handleCameraClient();
}
