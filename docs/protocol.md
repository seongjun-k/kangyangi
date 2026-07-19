# Kangyangi UDP 제어 프로토콜 (SSoT)

노트북(Python) → XIAO ESP32-S3(AP 192.168.4.1) UDP **포트 8888**. 리틀엔디언.

## 모션 패킷 (21 bytes)

| 오프셋 | 크기 | 필드 |
|---|---|---|
| 0 | 2 | seq (uint16 LE, 송신마다 +1, 랩어라운드 허용) |
| 2 | 16 | angle×8 (uint16 LE ×8) — **Dynamixel raw tick (0–8191, extended position mode, 중앙 4096)**, ID 1→8 순서 |
| 18 | 2 | dur (uint16 LE, ms) — PROFILE_VELOCITY에 적용할 프로파일 시간. 0=최고속. 펌웨어는 직전 값과 다를 때만 적용(`ensureProfile`) |
| 20 | 1 | checksum — 앞 20바이트 XOR |

- 각도 변환(deg→tick)은 파이썬 측에서 수행 (`deg2dxl`).
- seq가 직전 값과 같으면 무시(중복 패킷). 순서 역전은 단순 폐기하지 않음(랩어라운드 고려 안 함 — ponytail: 필요해지면 보강).
- dur의 "ms" 의미는 모터 Drive Mode가 time-based profile(비트2)로 EEPROM 설정돼 있다는 전제(원본 q8bot 세팅 절차). 미설정이면 velocity-based로 해석됨 — 실기에서 확인 필요.

## 커맨드 패킷 (3 bytes)

| 오프셋 | 크기 | 필드 |
|---|---|---|
| 0 | 1 | 0xFF (모션 패킷과 구분용 매직) |
| 1 | 1 | cmd: 0=torque off, 1=torque on, 4=jump |
| 2 | 1 | checksum — 앞 2바이트 XOR |

## 안전 규칙 (생략 불가)

- 펌웨어는 **500ms 무수신 시 torque off** (안전정지).
- 안전정지 후 유효한 모션 패킷을 다시 수신하면 펌웨어가 torque를 자동 재활성화한다.
- checksum 불일치·길이 불일치(21도 3도 아님) 패킷은 폐기.
- 파이썬은 torque on 동안 마지막 모션 패킷을 **150ms 무송신 시 재송신(keepalive)** — 위 워치독 하에서 정지 자세 유지. UDP 유실 대비 워치독 대비 3배 이상 여유.

## 구현 위치 (변경 시 함께 수정)

- 펌웨어 수신부: firmware/kangyangi/src/main.cpp:104 (모션 패킷 처리), :122 (커맨드 패킷 처리), :134 (길이/체크섬 검증), :148 (500ms 안전 정지)
- 파이썬 송신부: python/q8bot/udp_link.py — 모션 패킷 `q8_udp.move_all()`(L115-)/`send_raw_ticks()`(L132-), 커맨드 패킷 `q8_udp._send_cmd()`(L96-), keepalive `_keepalive_loop()`(L77-)
- 시뮬레이터: python/sim/mock_robot.py (동일 규칙 재현)
