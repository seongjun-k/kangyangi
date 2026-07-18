# Kangyangi UDP 제어 프로토콜 (SSoT)

노트북(Python) → XIAO ESP32-S3(AP 192.168.4.1) UDP **포트 8888**. 리틀엔디언.

## 모션 패킷 (19 bytes)

| 오프셋 | 크기 | 필드 |
|---|---|---|
| 0 | 2 | seq (uint16 LE, 송신마다 +1, 랩어라운드 허용) |
| 2 | 16 | angle×8 (uint16 LE ×8) — **Dynamixel raw tick (0–8191, extended position mode, 중앙 4096)**, ID 11→18 순서 |
| 18 | 1 | checksum — 앞 18바이트 XOR |

- 각도 변환(deg→tick)은 파이썬 측에서 수행 (`deg2dxl`).
- seq가 직전 값과 같으면 무시(중복 패킷). 순서 역전은 단순 폐기하지 않음(랩어라운드 고려 안 함 — ponytail: 필요해지면 보강).

## 커맨드 패킷 (3 bytes)

| 오프셋 | 크기 | 필드 |
|---|---|---|
| 0 | 1 | 0xFF (모션 패킷과 구분용 매직) |
| 1 | 1 | cmd: 0=torque off, 1=torque on, 4=jump |
| 2 | 1 | checksum — 앞 2바이트 XOR |

## 안전 규칙 (생략 불가)

- 펌웨어는 **500ms 무수신 시 torque off** (안전정지).
- 안전정지 후 유효한 모션 패킷을 다시 수신하면 펌웨어가 torque를 자동 재활성화한다.
- checksum 불일치·길이 불일치(19도 3도 아님) 패킷은 폐기.

## 구현 위치 (변경 시 함께 수정)

- 펌웨어 수신부: firmware/kangyangi/src/main.cpp:60 (모션 패킷 처리), :76 (커맨드 패킷 처리), :89 (길이/체크섬 검증), :107 (500ms 안전 정지)
- 파이썬 송신부: python/q8bot/udp_link.py — 모션 패킷 `q8_udp.move_all()`(L83-96), 커맨드 패킷 `q8_udp._send_cmd()`(L58-64)
