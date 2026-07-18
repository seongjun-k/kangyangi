# Kangyangi (강양이)

Quadruped robot based on [q8bot](https://github.com/EricYufengWu/q8bot), ported to XIAO ESP32-S3 Sense.

- Control: Xbox controller → laptop (Python IK/gait) → WiFi UDP → ESP32-S3 (AP mode, 192.168.4.1)
- Actuators: Dynamixel XL-330 x8 (UART half-duplex)
- Camera: QVGA MJPEG stream to laptop
