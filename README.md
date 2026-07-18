# Kangyangi (강양이)

Quadruped robot based on [q8bot](https://github.com/EricYufengWu/q8bot), ported to XIAO ESP32-S3 Sense.

## Architecture

Xbox controller → laptop (Python, `web_operate.py`) → WiFi UDP → XIAO ESP32-S3 (AP mode, 192.168.4.1) → UART half-duplex → Dynamixel XL-330 x8 (ID 1–8).

- Web control UI: laptop `:8080` (browser — keyboard/gamepad + push-to-talk voice)
- Camera (QVGA MJPEG): robot `:80`
- Microphone stream (raw PCM): robot `:81`
- Voice control: push-to-talk (`V` key or gamepad RB), Korean offline STT via [vosk](https://alphacephei.com/vosk/)

## Running

```bash
cd python/q8bot
pip install -r ../requirements.txt
python3 web_operate.py            # connects to the real robot (192.168.4.1)
```

Then open `http://localhost:8080/` in a browser.

### Simulation (no hardware)

```bash
cd python/sim
python3 run_sim.py --headless     # self-checking, no animation
python3 run_sim.py                # live matplotlib animation
```

`run_sim.py` uses `mock_robot.py` to emulate the firmware's UDP protocol and safety-stop behavior (see `docs/protocol.md`), so gait/IK/UDP can be verified without the physical robot.

## License

MIT. Based on [q8bot](https://github.com/EricYufengWu/q8bot) by Yufeng (Eric) Wu (MIT License) — see [LICENSE](LICENSE).
