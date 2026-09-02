/*
  q8Dynamixel.h - Wrapper for the Robotis Dynamixel2Arduino library.
  Created by Eric Wu, April 21st, 2024.
  Released into the public domain.
*/
#ifndef q8Dynamixel_h
#define q8Dynamixel_h

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;

class q8Dynamixel
{
  public:
    q8Dynamixel(Dynamixel2Arduino& dxl);
    void begin();
    void enableTorque();
    void disableTorque();
    void setOpMode();
    void setProfile(uint16_t dur);
    void ensureProfile(uint16_t dur);
    void setGain(uint16_t p_gain);
    void bulkWrite(int32_t values[8]);
    void jump();
    void telemetry();                  // 전압 표본화 + 1초 요약 (loop와 jump 양쪽에서 호출)
    void telemetryDelay(uint32_t ms);  // delay() 대체 — 대기 중에도 전압을 표본화한다

  private:
    Dynamixel2Arduino& _dxl; // Member variable to store the object of Dynamixel2Arduino

    const uint8_t BROADCAST_ID = 254;

    uint32_t _baudrate = 1000000;
    float _protocolVersion = 2.0;
    static const uint8_t _idCount = 8;
    const uint8_t _DXL[_idCount] = {11, 12, 13, 14, 15, 16, 17, 18};  // 모터 ID 설정 시 동일 번호로 부여할 것
    // 단일 회전 Position 모드(0~4095) 안에서 관절 실사용 범위(-40~220도)가
    // 전부 들어오도록 중앙을 1024로 잡음(협의 SSoT: docs/protocol.md, python ZERO_OFFSET과 일치).
    const int16_t _zeroOffset = 1024;
    const uint8_t _gearRatio = 1;
    uint16_t _prevProfile = 1000;  // begin()의 setProfile(1000)과 일치시켜 초기 비교값을 정의(기존엔 미초기화였음)
    // 재부팅 원인 규명용 텔레메트리 상태 (loop 태스크에서만 접근)
    // XL-330-M288 동작 범위 3.7~6.0V, 권장 5.0V. 4.5V 아래면 레일이 무너지는 중이고
    // 같은 레일에서 급전받는 XIAO가 브라운아웃될 위험 구간이다.
    static constexpr float _telemAlarm = 4.5f;
    uint32_t _telemLastSample = 0;
    uint32_t _telemLastReport = 0;
    uint32_t _telemBackoffUntil = 0;  // 읽기 실패 후 재시도 대기 마감 시각(ms)
    float _telemMin = 99.0f;

    int32_t _deg2Dxl(float deg);
    void expandArrays();
    const float _idlePos[2]   = {30, 150};
    int32_t _idleArray[8];
    const float _jumpLow[2]   = {-40, 220};
    int32_t _lowArray[8];
    const float _jumpHigh[2] = {90, 90};
    int32_t _highArray[8];
    const float _jumpRest[2]  = {30, 150};
    int32_t _restArray[8];

    // Struct definitions for bw (bulk write)
    struct bw_data_xel{
      int32_t goal_position;
    } __attribute__((packed));

    struct bw_data_xel _bw_data_xel[_idCount];
    DYNAMIXEL::InfoBulkWriteInst_t _bw_infos;
    DYNAMIXEL::XELInfoBulkWrite_t _info_xels_bw[_idCount];
};

#endif
