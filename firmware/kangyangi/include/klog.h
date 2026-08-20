/*
  klog.h - 진단 로그 출력 (시리얼 + UDP 브로드캐스트)

  모터 전원이 켜져 있으면 부스트 레일 DXL_P2가 D2 숏키를 통해 XIAO의 VUSB를
  5V로 붙들고 있어 USB VBUS attach 이벤트가 발생하지 않는다 -> USB가 아예
  열거되지 않아 시리얼 디버깅이 불가능하다(2026-08-19 실기 재현 확인).
  그래서 진단 로그는 UDP 브로드캐스트로도 내보낸다.

  한계: 브라운아웃으로 리셋되면 WiFi 스택이 함께 죽어 직전 몇 줄이 유실될 수 있다.
  시리얼만큼 신뢰할 수 없으므로 "마지막 줄이 없다"를 근거로 결론내지 말 것.
*/
#ifndef klog_h
#define klog_h

#include <Arduino.h>

static const uint16_t KLOG_PORT = 9999;

// printf 형식. 개행은 호출부에서 붙인다.
void klog(const char* fmt, ...);

#endif
