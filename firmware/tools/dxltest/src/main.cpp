// Dynamixel 버스 진단·셋업 스케치 — kangyangi 본 펌웨어 대신 임시로 플래시해서 사용.
// 배선은 본 펌웨어와 동일: UART1 RX=D7 TX=D6, DIR=D8, half-duplex, protocol 2.0.
//
// 동작:
//  - SETUP_TARGET_ID == 0 : ID 1~20을 1M/57600 baud로 스캔만 반복 (읽기 전용)
//  - SETUP_TARGET_ID != 0 : 해당 ID를 찾아 baud 1M + DRIVE_MODE time-based(bit2)로
//    셋업(EEPROM 쓰기, 1회) 후 검증 출력. 새 모터(공장값 57600)도 잡아서 처리.
//    새 모터의 ID 변경까지 하려면 FACTORY_NEW_ID에 현재 ID(공장값 1)를 지정.
//
// 사용 후 반드시 본 펌웨어(firmware/kangyangi)를 복구 플래시할 것.
#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#define SETUP_TARGET_ID 0   // 0=스캔만, N=해당 ID 셋업
#define FACTORY_NEW_ID  0   // 0=ID 변경 안 함, N=이 ID를 SETUP_TARGET_ID로 변경

const uint8_t DXL_DIR_PIN = D8;
const uint8_t DXL_RX_PIN = D7;
const uint8_t DXL_TX_PIN = D6;

Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);

bool beginAt(uint32_t baud) {
  dxl.begin(baud);
  dxl.setPortProtocolVersion(2.0);
  return true;
}

void scanAt(uint32_t baud) {
  beginAt(baud);
  Serial.printf("[DXLTEST] scan @ %lu baud:", (unsigned long)baud);
  bool found = false;
  for (uint8_t id = 1; id <= 20; id++) {
    if (dxl.ping(id)) {
      int32_t hwerr = dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id);
      int32_t volt = dxl.readControlTableItem(ControlTableItem::PRESENT_INPUT_VOLTAGE, id);
      int32_t temp = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, id);
      Serial.printf(" ID%d(HWERR=0x%02lX V=%.1f T=%ldC)", id, (long)hwerr, volt / 10.0, (long)temp);
      found = true;
    }
  }
  if (!found) Serial.print(" (none)");
  Serial.println();
}

#if SETUP_TARGET_ID
void setupMotor() {
  uint8_t id = SETUP_TARGET_ID;

  // 공장 초기 모터(57600, ID=FACTORY_NEW_ID)면 ID부터 변경
#if FACTORY_NEW_ID
  beginAt(57600);
  if (dxl.ping(FACTORY_NEW_ID)) {
    dxl.torqueOff(FACTORY_NEW_ID);
    bool ok = dxl.setID(FACTORY_NEW_ID, id);
    Serial.printf("[DXLTEST] ID %d->%d: %s\n", FACTORY_NEW_ID, id, ok ? "OK" : "FAIL");
    delay(200);
  }
#endif

  // 57600에서 잡히면 baud를 1M(BAUD_RATE=3)으로 변경
  beginAt(57600);
  if (dxl.ping(id)) {
    dxl.torqueOff(id);
    bool ok = dxl.writeControlTableItem(ControlTableItem::BAUD_RATE, id, 3);
    Serial.printf("[DXLTEST] ID%d baud->1M: %s\n", id, ok ? "OK" : "FAIL");
    delay(200);
  }

  // 1M에서 DRIVE_MODE bit2(time-based profile) 설정 + 검증
  beginAt(1000000);
  if (!dxl.ping(id)) {
    Serial.printf("[DXLTEST] ID%d no response @1M - setup failed\n", id);
    return;
  }
  int32_t cur = dxl.readControlTableItem(ControlTableItem::DRIVE_MODE, id);
  if (cur >= 0 && !(cur & 0x04)) {
    dxl.torqueOff(id);
    bool ok = dxl.writeControlTableItem(ControlTableItem::DRIVE_MODE, id, cur | 0x04);
    Serial.printf("[DXLTEST] ID%d DRIVE_MODE %ld->%ld: %s\n",
                  id, (long)cur, (long)(cur | 0x04), ok ? "OK" : "FAIL");
    delay(200);
  }
  int32_t dm = dxl.readControlTableItem(ControlTableItem::DRIVE_MODE, id);
  Serial.printf("[DXLTEST] ID%d model=%d DRIVE_MODE=%ld time-based=%s\n",
                id, dxl.getModelNumber(id), (long)dm,
                (dm >= 0 && (dm & 0x04)) ? "YES" : "NO");
}
#endif

void setup() {
  Serial.begin(115200);
  delay(5000);  // 호스트가 CDC 포트를 열 시간 확보
  Serial1.begin(1000000, SERIAL_8N1, DXL_RX_PIN, DXL_TX_PIN);

  // 일괄 time-based profile 설정: ID 11~18 중 DRIVE_MODE bit2 미설정인 모터만 EEPROM 쓰기
  beginAt(1000000);
  for (uint8_t id = 11; id <= 18; id++) {
    if (!dxl.ping(id)) continue;
    int32_t dm = dxl.readControlTableItem(ControlTableItem::DRIVE_MODE, id);
    if (dm >= 0 && !(dm & 0x04)) {
      dxl.torqueOff(id);
      bool ok = dxl.writeControlTableItem(ControlTableItem::DRIVE_MODE, id, dm | 0x04);
      Serial.printf("[DXLTEST] ID%d DRIVE_MODE %ld->%ld: %s\n",
                    id, (long)dm, (long)(dm | 0x04), ok ? "OK" : "FAIL");
      delay(100);
    }
  }

  // 에러 진단 + 리부팅: 부팅 시 1회 — HWERR 원인을 먼저 읽고 reboot으로 래치 해제
  for (uint8_t id = 11; id <= 18; id++) {
    if (!dxl.ping(id)) {
      Serial.printf("[DXLTEST] ID%d: no response, skip\n", id);
      continue;
    }
    int32_t hwerr = dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id);
    int32_t volt = dxl.readControlTableItem(ControlTableItem::PRESENT_INPUT_VOLTAGE, id);
    Serial.printf("[DXLTEST] ID%d pre-reboot: HWERR=0x%02lX V=%.1f -> reboot\n",
                  id, (long)hwerr, volt / 10.0);
    dxl.reboot(id);
    delay(300);
  }
}

void loop() {
#if SETUP_TARGET_ID
  setupMotor();
#else
  scanAt(1000000);
  scanAt(57600);
#endif
  delay(2000);
}
