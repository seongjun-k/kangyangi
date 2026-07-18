#include <Arduino.h>

// DYNAMIXEL TTL Comms 방향 제어 핀
// 원본(q8bot, XIAO C3) raw GPIO 8 = XIAO S3의 D8 (D라벨 핀 넘버링은 보드마다 다름 — 항상 D라벨 사용)
const uint8_t DXL_DIR_PIN = D8;

// Dynamixel UART(half-duplex, Serial1) 핀
const uint8_t DXL_RX_PIN = D7;
const uint8_t DXL_TX_PIN = D6;
