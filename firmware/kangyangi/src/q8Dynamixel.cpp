/*
  q8Dynamixel.h - Wrapper for the Robotis Dynamixel2Arduino library.
  Created by Eric Wu, April 21st, 2024.
  Released into the public domain.
*/

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <q8Dynamixel.h>
#include <klog.h>

using namespace ControlTableItem;

// Constructor that takes an object of type Dynamixel2Arduino as an argument
q8Dynamixel::q8Dynamixel(Dynamixel2Arduino& dxl) : _dxl(dxl) {
  // Constructor implementation
  // Initialize any other members if needed
}

void q8Dynamixel::begin(){
  _dxl.begin(_baudrate);
  _dxl.setPortProtocolVersion(_protocolVersion);
  setOpMode();

  // Fill the members of structure for bulkWrite using internal packet buffer
  _bw_infos.packet.p_buf = nullptr;
  _bw_infos.packet.is_completed = false;
  _bw_infos.p_xels = _info_xels_bw;
  _bw_infos.xel_count = 0;

  for (int i = 0; i < _idCount; i++){
    _bw_data_xel[i].goal_position = 0;
    _info_xels_bw[i].id = _DXL[i];
    _info_xels_bw[i].addr = 116; // Goal Position of X serise.
    _info_xels_bw[i].addr_length = 4; // Goal Position
    _info_xels_bw[i].p_data = reinterpret_cast<uint8_t*>(&_bw_data_xel[i]);
    _bw_infos.xel_count++;
  }
  _bw_infos.is_info_changed = true;

  setProfile(1000);
  expandArrays();
}

void q8Dynamixel::enableTorque(){
  // 브라운아웃 등으로 래치된 하드웨어 에러(빨간 LED 점멸)는 reboot으로만 해제된다.
  // 에러 모터만 리부팅. 리부팅은 RAM 레지스터(프로파일/게인)를 초기화하므로
  // 재적용 없이는 Profile Velocity=0(최대 속도)이 되어 위험 — setProfile 재실행 필수.
  bool rebooted = false;
  for (int i = 0; i < _idCount; i++){
    int32_t hwerr = _dxl.readControlTableItem(HARDWARE_ERROR_STATUS, _DXL[i]);
    if (hwerr > 0){
      // 어떤 에러로 래치됐는지 남긴다 — bit0=입력전압, bit2=과열, bit5=과부하.
      // 점프/보행 중 토크가 죽는 원인이 전원 새그인지 과부하인지 이걸로 갈린다.
      int32_t volt = _dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, _DXL[i]);
      klog("[DXL] ID%d 에러 래치 HWERR=0x%02lX V=%.1f -> reboot\n",
           _DXL[i], (long)hwerr, volt / 10.0);
      _dxl.reboot(_DXL[i]);
      rebooted = true;
    }
  }
  if (rebooted){
    delay(300);  // reboot 후 재기동 대기(진단 스케치에서 300ms로 실기 확인)
    setProfile(_prevProfile);
  }
  _dxl.torqueOn(BROADCAST_ID);
}

void q8Dynamixel::disableTorque(){
  _dxl.torqueOff(BROADCAST_ID);
}

void q8Dynamixel::setOpMode(){
  // 관절 실사용 범위(약 260도, 4096틱=360도 이내)가 한 바퀴 안에 들어가므로
  // 멀티턴을 추적하는 Extended Position 대신 단일 회전 Position 모드를 쓴다.
  // 전원이 끊겨도 present position이 항상 물리 각도 그대로 복원되어
  // 재부팅 시 팬텀 풀턴이 발생하지 않는다.
  //
  // ESP32만 재부팅되고 모터 전원은 유지될 수 있어 실제 하드웨어 Torque Enable 상태를
  // 소프트웨어가 알 수 없다. Operating Mode(EEPROM)는 Torque Off 상태에서만 쓸 수
  // 있으므로 현재 상태와 무관하게 항상 먼저 강제로 끈다.
  disableTorque();

  // setOperatingMode()는 내부적으로 ping()이 채워주는 모델 번호 캐시를 참조한다.
  // ping 없이 바로 호출하면 캐시가 비어 미등록 모델로 처리되어 아무것도 쓰지 않고
  // 조용히 실패한다(실기로 setOperatingMode 반환값이 매번 FAIL임을 확인해 원인 특정).
  for (int i = 0; i < _idCount; i++){
    _dxl.ping(_DXL[i]);
    _dxl.setOperatingMode(_DXL[i], OP_POSITION);
  }
}

void q8Dynamixel::setProfile(uint16_t dur){
  setGain(400);
  for (int i = 0; i < _idCount; i++){
    _dxl.writeControlTableItem(PROFILE_VELOCITY, _DXL[i], dur);
    _dxl.writeControlTableItem(PROFILE_ACCELERATION, _DXL[i], dur / 3);
  }
}

void q8Dynamixel::ensureProfile(uint16_t dur){
  // _prevProfile은 jump()도 갱신하는 단일 SSoT(현재 적용된 프로파일) - 여기서만 비교/설정
  if (dur == _prevProfile) return;
  setProfile(dur);
  _prevProfile = dur;
}

void q8Dynamixel::setGain(uint16_t p_gain){
  // for Time-based Extended Pos, Profile velocity is the move duration (ms).
  for (int i = 0; i < _idCount; i++){
    _dxl.writeControlTableItem(POSITION_P_GAIN, _DXL[i], p_gain);
  }
}

void q8Dynamixel::bulkWrite(int32_t values[8]){
  // 8 motors move to their respective positions
  for (int i = 0; i < _idCount; i++){
    _bw_data_xel[i].goal_position = values[i];
  }
  _bw_infos.is_info_changed = true;

  _dxl.bulkWrite(&_bw_infos);
}

// 재부팅 원인(브라운아웃 vs 크래시) 규명용 텔레메트리.
// 모터 입력전압 = DXL_P2 레일 전압이고, XIAO도 D2 숏키를 거쳐 같은 레일에서 급전받는다.
// 여기서 새그가 잡히면 재부팅 원인이 전원으로 확정된다.
// 1초 요약만으로는 수십 ms짜리 새그를 놓치므로 20ms마다 표본화해 최저치를 남긴다.
void q8Dynamixel::telemetry(){
  uint32_t now = millis();
  if (now - _telemLastSample < 20) return;
  // 백오프 중 — 20ms마다 재시도하면 UART 타임아웃으로 모션 주기가 흔들린다.
  // millis() 랩어라운드 안전을 위해 부호 있는 차로 비교한다(now < until 직접 비교는 49.7일마다 깨진다).
  if ((int32_t)(now - _telemBackoffUntil) < 0) return;
  _telemLastSample = now;

  // 1개만 읽는다 — 8개 전부 읽으면 UART 시간을 먹어 모션 주기가 흔들린다.
  int32_t raw = _dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, _DXL[0]);
  if (raw <= 0) {
    _telemBackoffUntil = now + 1000;  // 통신 실패(모터 전원 off 등) - 1초 후 재시도
    return;
  }
  float v = raw / 10.0f;
  if (v < _telemMin) _telemMin = v;

  // 위험 전압은 1초 요약을 기다리지 않고 즉시 내보낸다.
  // 브라운아웃으로 리셋되면 아직 안 나간 출력은 그대로 사라지기 때문이다.
  if (v < _telemAlarm){
    klog("[TLM] t=%lu V=%.1f *** 전압 새그 ***\n", (unsigned long)now, v);
    return;
  }

  if (now - _telemLastReport >= 1000){
    _telemLastReport = now;
    klog("[TLM] t=%lu V=%.1f min=%.1f\n", (unsigned long)now, v, _telemMin);
    _telemMin = 99.0f;
  }
}

// jump()처럼 loop를 독점하는 구간에서 delay() 대신 쓴다 — 정작 전류 피크가 걸리는
// 그 시간 동안 표본이 하나도 안 남으면 텔레메트리의 의미가 없다.
void q8Dynamixel::telemetryDelay(uint32_t ms){
  uint32_t start = millis();
  while (millis() - start < ms){
    telemetry();
    delay(1);
  }
}

// 도약 P게인. 8개 모터가 setProfile(0)로 동시에 최대 출력을 내는 지점이라
// 전체 소비 전류의 피크가 여기서 결정된다.
// 2026-08-24 실측: gain 800에서 도약 100ms 후 모터 전압이 5.0V -> 3.3V로 붕괴,
// 보드가 그 자리에서 브라운아웃(klog가 [JUMP] 3/3 전에 끊김). 800 -> 600으로 하향.
// 근본 원인은 전원 계통(벌크 커패시터/배터리 방전율)이고 이건 완화책이다 —
// 전원을 보강했으면 800으로 되돌려 점프 높이를 회복시킬 것.
static const uint16_t JUMP_GAIN = 600;

void q8Dynamixel::jump(){
  // 단계 로그 + flush — 재부팅이 어느 단계에서 터지는지가 원인 판별의 핵심이다.
  // 도약(setProfile(0)+gain 800)이 전류 피크 지점이라 여기서 끊기면 전원이 범인.
  klog("[JUMP] 1/3 웅크리기\n");
  setProfile(500);
  telemetryDelay(100);
  bulkWrite(_lowArray);
  telemetryDelay(1000);

  klog("[JUMP] 2/3 도약(전류 피크)\n");
  setProfile(0);
  setGain(JUMP_GAIN);
  telemetryDelay(100);
  bulkWrite(_highArray);
  telemetryDelay(100);
  bulkWrite(_restArray);
  telemetryDelay(5000);

  klog("[JUMP] 3/3 복귀\n");
  setProfile(500);
  telemetryDelay(100);
  bulkWrite(_idleArray);
  telemetryDelay(1000);
  _prevProfile = 500;
  klog("[JUMP] 완료\n");
}

int32_t q8Dynamixel::_deg2Dxl(float deg){
  // Dynamixel joint 0 to 360 deg is 0 to 4096
  const float friendlyPerDxl = 360.0 / 4096.0 / _gearRatio;
  int angleDxl = static_cast<int>(deg / friendlyPerDxl + 0.5) + _zeroOffset;
  return angleDxl;
}

void q8Dynamixel::expandArrays(){
  // _idlePos/_jumpLow/_jumpHigh/_jumpRest 각각 값이 다를 수 있어 배열은 따로 두되,
  // 채우는 방식(4쌍 반복)만 테이블로 묶는다.
  const float* srcs[4] = {_idlePos, _jumpLow, _jumpHigh, _jumpRest};
  int32_t* dsts[4] = {_idleArray, _lowArray, _highArray, _restArray};
  for (int a = 0; a < 4; a++){
    for (int i = 0; i < 4; i++){
      dsts[a][i*2] = _deg2Dxl(srcs[a][0]);
      dsts[a][i*2+1] = _deg2Dxl(srcs[a][1]);
    }
  }
}
