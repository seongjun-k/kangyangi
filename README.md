# Kangyangi (강양이)

Quadruped robot based on [q8bot](https://github.com/EricYufengWu/q8bot), ported to the Seeed XIAO ESP32-S3 Sense.

The laptop does all the thinking (IK, gait generation, voice recognition) and streams raw joint ticks over WiFi UDP. The firmware is a thin, watchdog-protected servo driver.

## Architecture

```
Xbox controller ─USB─┐
                     ├─> laptop (Python)  ─WiFi UDP:8888─> XIAO ESP32-S3 (AP 192.168.4.1)
browser :8080  ──────┘   IK + gait + STT                    │
                                                            └─UART half-duplex─> XL-330 x8 (ID 11–18)
```

| Endpoint | Where | What |
|---|---|---|
| `http://localhost:8080/` | laptop | control UI (keyboard / gamepad / push-to-talk voice) |
| `http://localhost:8080/calib` | laptop | joint zero-offset calibration wizard |
| `http://192.168.4.1/` | robot | camera stream (QVGA MJPEG) |
| `http://192.168.4.1:81/` | robot | microphone stream (raw PCM) |
| `192.168.4.1:8888` | robot | UDP control — see [docs/protocol.md](docs/protocol.md) |

The robot boots as a WiFi AP: SSID `kangyangi`, password `kangyangi`.

## Hardware

- Seeed XIAO ESP32-S3 **Sense** (camera + mic expansion board required)
- 8x Dynamixel XL-330-M288-T, IDs **11–18**, 1 Mbps, protocol 2.0
- Half-duplex TTL bus on `D6` (TX), `D7` (RX), direction on `D8`

> XIAO C3 → S3 is pin-compatible by **D-label only** — the raw GPIO numbers differ. Always refer to pins by D-label.

Motor EEPROM setup (done once via the `dxltest` tool, see below):

| Register | Value | Why |
|---|---|---|
| Operating Mode | `3` (Position, single-turn) | Extended/multiturn mode resets its revolution counter on motor power loss, which made the legs spin a full turn on reboot |
| Drive Mode bit2 | set (all) | time-based profile — the `dur` field in the UDP packet is milliseconds |
| Drive Mode bit0 | set on ID 13, 14, 17, 18 | reverse direction, mirrors the right-side legs |
| Baud Rate | `3` (1 Mbps) | matches firmware |

## Repo layout

```
firmware/kangyangi/     main firmware (PlatformIO, Arduino framework)
firmware/tools/dxltest/ standalone Dynamixel bus diagnostic / provisioning sketch
python/q8bot/           control stack — web UI, IK, gait, UDP link, voice
python/sim/             hardware-free simulator (mock firmware + matplotlib viz)
docs/protocol.md        UDP packet format (SSoT)
hardware/               Altium project, BOM, assembly guide (gitignored)
```

## Running

### 1. Flash the firmware

```bash
pio run -d firmware/kangyangi                                   # build
pio run -d firmware/kangyangi -t upload --upload-port /dev/ttyACM0
```

The board is native-USB (CDC), so it usually shows up as `/dev/ttyACM0` and resets itself for upload. If it doesn't, hold **BOOT** while re-plugging the USB cable to force bootloader mode.

**The robot collapses when the board reboots** (torque starts off) — lay it down or hold it before flashing.

### 2. Connect the laptop to the robot's AP

Join the `kangyangi` WiFi network. The laptop keeps its normal internet connection if you use a second adapter for the robot.

### 3. Start the control server

```bash
cd python/q8bot
../../.venv/bin/pip install -r ../requirements.txt
../../.venv/bin/python web_operate.py   # --ip 127.0.0.1 to target the simulator
                                        # --port 8080, --debug
```

Use the venv's interpreter, not a bare `python3`. Voice recognition is behind an import guard, so a system interpreter without `vosk` starts the server normally and silently recognises nothing.

Open `http://localhost:8080/`.

## Calibration

`calibration.json` stores the per-joint tick that corresponds to 0°. Without it every joint falls back to `ZERO_OFFSET = 1024`, which is only correct if the horns happened to be splined on perfectly.

Recalibrate after any leg reassembly:

1. Assemble each leg in the **reference pose** — both upper links horizontal and pointing the same direction (mirrored on the opposite side). Approximate is fine; spline teeth make exact impossible.
2. Open `http://localhost:8080/calib` and enter calibration mode. The server holds every joint at its stored zero offset.
3. Nudge each joint (±1 tick ≈ 0.088°) until it is physically at true zero.
4. Save. `calibration.json` is written and applied immediately — no restart.
5. Back on the main page, command a stance and confirm the feet drop *down*. A leg that folds upward is assembled one half-turn off — take it apart and reseat it.

## Controls

| Key | Action | | Key | Action |
|---|---|---|---|---|
| `W` / `S` | forward / backward | | `G` | switch gait |
| `A` / `D` | turn left / right | | `J` | jump |
| `Q` / `E` | forward-left / forward-right | | `H` | greet |
| `R` | reset | | `P` | paw |
| `C` | show workspace range | | `V` | voice push-to-talk (hold) |

The page must have focus for keys to register. An Xbox controller works through the browser Gamepad API with the same action mapping (RB = push-to-talk).

Gaits: `TROT`, `TROT_HIGH`, `TROT_LOW`, `TROT_FAST`, `WALK`, `CRAWL`, `BOUND`, `PRONK`.

Voice (Korean, offline via [vosk](https://alphacephei.com/vosk/), streamed from the robot's mic): 앞으로/전진, 뒤로/후진, 왼쪽, 오른쪽, 멈춰/정지, 인사, 앉아/쉬어, 일어나/준비, 점프/뛰어, 손. Sentences containing a negation are ignored entirely, and "점프"/"손" require an exact word match so that "손님" doesn't make the robot shake hands.

## Safety

The protocol is fail-safe by design (full rules in [docs/protocol.md](docs/protocol.md)):

- The firmware **disables torque after 500 ms without a valid packet**, so a crashed or disconnected laptop drops the robot rather than leaving it running blind.
- The Python side re-sends the last motion packet every 150 ms, giving a 3x margin against UDP loss.
- Packets with a bad checksum or wrong length are discarded.
- Torque re-enables automatically once valid motion packets resume.

## Simulation (no hardware)

```bash
cd python/sim
python3 run_sim.py --headless     # self-checking, no animation
python3 run_sim.py                # live matplotlib animation
```

`mock_robot.py` reimplements the firmware's packet parsing, watchdog and safety-stop behaviour, so gait, IK and the UDP layer can be verified without powering the robot.

## Diagnostics

`firmware/tools/dxltest` is a separate PlatformIO project that flashes in place of the main firmware to talk to the bus directly. It scans IDs 1–20 at 1 Mbps and 57600 baud and prints hardware error status, input voltage, drive mode, operating mode, torque state and present position for every motor found — the fastest way to answer "is the motor even there, and what does it think its configuration is".

It also provisions motors: set `SETUP_TARGET_ID` (and `FACTORY_NEW_ID` for a factory-fresh motor still on ID 1 / 57600 baud) to assign an ID, switch to 1 Mbps and set the time-based drive mode.

```bash
pio run -d firmware/tools/dxltest -t upload
```

Reflash the main firmware when you're done.

## Verification

```bash
pio run -d firmware/kangyangi          # firmware build
python3 -m py_compile <changed .py>    # python syntax check
cd python/sim && python3 run_sim.py --headless
```

## License

MIT. Based on [q8bot](https://github.com/EricYufengWu/q8bot) by Yufeng (Eric) Wu (MIT License) — see [LICENSE](LICENSE).
