# SE09 vehicle dashboard

Browser dashboard for the Pico's existing binary-v2 MQTT firmware. The local Python server connects to your existing TCP MQTT broker; the browser talks only to that server. No broker WebSocket configuration is required.

## Run on your laptop

Use the same LAN broker IPv4 address and topic root as the Pico's ignored `config/wifi_secrets.h`. The root normally is `robot/<MQTT_CLIENT_ID>/v2`. It must match exactly. This dashboard retains the uploaded default `robot/mtk3smp-pico/v2`; if your client ID is `se09-car`, pass `--root robot/se09-car/v2`.

```sh
cd dashboard
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install -r requirements.txt
python3 app.py BROKER_IP --root robot/YOUR_CLIENT_ID/v2
```

On Windows activate with `.venv\Scripts\activate` and use `python` if needed. Replace `BROKER_IP` and `YOUR_CLIENT_ID` with your configuration. If no broker IP is supplied, the app prompts for it. The server prints the local browser address when ready. Defaults: MQTT TCP port 1883, local browser port 8080; override with `--broker-port` and `--port`.

The Pico must run the MQTT firmware (`make NET=lwip MQTT=1`), not the normal non-MQTT build. MQTT firmware boots stopped until START. The dashboard never sends START on startup, reconnect, page reload or automatically after a fault.

## What is displayed

- **Vehicle controller:** mission state and reported subsystem fault flags.
- **Wheels/motion:** measured left/right RPM; propulsion PWM; actual motor mode; global speed level; recent 120-sample RPM trend; raw motor-fault labels; bounded-move progress counts.
- **Line/barcode:** left/right black-detection flags, centre flag, reported barcode letter.
- **Terrain:** reported height in millimetres and terrain phase.
- **Obstacle:** nearest distance and side clearance, angle of the closest return, and the live detour stage and chosen side.
- **Connection:** live heartbeat age, Wi-Fi/broker/subscription flags and bridge drop flags.
- **Commands/events:** exact binary START/STOP, Pico acknowledgment and a bounded connection/mission/fault event list.

All values come from received MQTT payloads. The current two-sensor Pico configuration leaves the centre flag unset; an unset flag does not prove a centre sensor is installed or reading white. Barcode decoding is currently disabled. Terrain and obstacle cards show **No data** until their producers submit a valid measurement. The car checks the two 45° departure paths while stopped; exact obstacle width is not required or displayed. An ultrasonic echo fault is shown under vehicle faults.

The matching diagnostics firmware additionally publishes normalized IR darkness, target RPM, controller stop reason and telemetry-drop count on `/controller`. Older firmware does not publish these fields. Raw ADC values, tilt, turn rate and total distance remain unavailable. Wheel progress is counts for bounded moves, not cumulative distance. The global speed level is a legacy selection, not the per-wheel fine-RPM target. A displayed mission state reports controller intent; actual wheel RPM reports measured motion.

## Freshness and commands

Each telemetry channel independently expires after 2 seconds without a valid live sample. Its last received value remains visible but faded and labelled **Stale**. Live heartbeats do not freshen old measurements. A reconnect requires new live samples before telemetry becomes fresh again. Retained telemetry and malformed packets never count as live data. No-data and measured zero remain distinct.

Heartbeat expires after 15 seconds. START is available only when the broker/subscriptions and a fresh heartbeat confirm all connection flags. The Pico may still reject START due to startup delay, stale line data, a junction stop or fault. STOP stays available while the local server has a broker connection. No command is retained; the server does not replay commands after reconnect.

The Pico ACK confirms acceptance for processing, not physical movement. Inspect mission state, wheel RPM and faults after commanding START. The car can bypass an obstacle with confirmed departure clearance during an active run and resume after reacquiring the line. The first-junction halt is still enabled.

MQTT QoS 0 does not guarantee delivery. The browser reports uncertainty if its HTTP request times out. Faults need investigation before a new START. This is a local dashboard with Host/Origin and command-token checks, bound to loopback; it is not designed for public Internet exposure. Broker TLS/auth configuration is not added by this dashboard or the current Pico MQTT client.

## Protocol

Subscriptions use the configured root and these suffixes:

| Topic | Bytes | Decode |
|---|---:|---|
| `controller` |25 (v1) / 31 (v2)| versioned startup guard, readiness, normalized IR, target RPM, line age and drop count; see VEHICLE_CONTROLLER.md |
| `status` |1| Wi-Fi1, broker2, command subscription4, TX drop8, RX drop16 |
| `ack` |1| START in bit 0, rejection or failure in bit 1, RESET in bit 2, completed RESET in bit 3 |
| `vehicle` |1| state bits0..2, fault mask bits3..7 |
| `motion` |9| S2_STATUS packed bits: mode0:2, level2:3, faults5:4, L RPM9:9, R RPM18:9, L PWM27:7, R PWM34:7, L progress41:13, R progress54:13 |
| `line` |1| L/C/R black bits0/1/2, barcode bits3..5 |
| `terrain` |2| height0:11, phase11:3 |
| `avoidance` |1 / 4| phase at bits 0:4, side at 4:2 (0 none, 1 left, 2 right). Four-byte packets add stop cause at 8:4, phase at the stop at 12:4 and current forward range at 16:12. |
| `obstacle` |6 / 7| nearest0:12, width12:11, left23:12, right35:12. Seven-byte packets add angle minus 30° at 47:7 and a saved-sweep flag at bit 54. Width 2047 and clearance 4095 mean unavailable. A saved sweep can report a confirmed open side as 4094. |

Bit numbering starts at the least significant bit of byte0. Fields use the firmware's little-endian packing. `offset:width` notation is used above. `mqtt_protocol.py` validates lengths, ranges and unused bits before displaying telemetry. START publishes `0x01`, STOP `0x00`, and stopped-only RESET `0x02` to `/command`, QoS0, non-retained. Strings such as `START` are not valid Pico commands.

## Tests

State/decoder/HTTP tests (Python standard library):

```sh
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

Browser integration requires Mosquitto, Chromium and Playwright in addition to requirements.txt:

```sh
python3 -m pip install playwright
python3 tests/test_browser.py --screenshot /tmp/se09-dashboard.png
```

Set `BUDDY1_MOSQUITTO` or `BUDDY1_BROWSER` if executable paths are not discoverable. The integration test starts its own local broker and simulated-Pico protocol fixture, tests actual HTTP/browser commands and all telemetry topics, then shuts down its processes. The fixture is explicitly not hardware validation. See TEST_RESULTS.md for executed outcomes.

## If START is rejected

Replace both the dashboard and Pico firmware with matching current versions. A heartbeat confirms network connectivity, not that the motors and sensors are ready. The Vehicle controller panel reports the actual START blocker. Wait at least five seconds after power-up and start on a straight section, not a crossing. Read the named blocker before resetting faults. If it says diagnostics unavailable, the Pico is running older firmware or the topic root does not match. Send the blocker text and IR/target values for diagnosis; a generic one-byte rejected ACK alone cannot identify the cause.

The **Obstacle scan** card shows measured forward distance even when a target is outside the stopping threshold. A measured forward range beyond 27 cm clears the obstacle interlock. Two complete out-of-range pulses within the 30 ms deadline can also clear it. Two matching RCWL-9610A no-target pulses around 47.1 ms can also clear it. Missing and other late echoes keep the interlock blocked. Hold a flat target 40–60 cm straight ahead while the car is stopped to verify the sensor. If the fault stays on, check sensor power, common ground, GP16 trigger, and the 3.3 V ECHO signal at GP17. Use a divider for a 5 V ECHO output. If the panel shows **Stale**, check the Pico’s Wi-Fi and broker connection first.

Connect the scan servo to the Robo Pico servo header marked **GP12**. At power-up the firmware holds the servo at its centre pulse and waits 350 ms before ranging. Check that the sensor faces straight ahead at that position. If it points to one side, mark the powered centre position, switch off, and re-seat the servo horn or adjust the sensor bracket. Small corrections can be made with `S5_SERVO_CENTRE_US` in `config/s5_servo_calibration.h` (see `config/s5_servo_calibration.example.h`). To test detection followed by a single clearance scan over USB, use the [stationary bench build](../testing/subsystem5/TEST_LOG.md#stationary-hardware-check).

## Trace obstacle avoidance

Open USB serial at 115200 baud. The car slows at 350 mm and stops at 220 mm. The servo checks 135° (45° left), 45° (45° right), then returns to 90° for a confirmed forward reading. Two consecutive agreeing measurements confirm a bearing; two real no-target pulses confirm open space. Echo timeouts remain unavailable.

The log reports `[S5] clearance forward=... left=... right=...`. Values 4094 and 4095 mean open and unavailable respectively. The car uses a 45° departure when the diagonal has at least 412 mm clearance and forward room is at least 200 mm. Otherwise it can select a 90° stationary pivot when turn clearance is at least 100 mm. A new echo on the actual 90° heading must confirm at least 335 mm before the 185 mm lateral drive. A blocked live 45° heading can widen to 90° before translating. Width fitting cannot block this decision. The unused width field stays 2047 for wire compatibility and is hidden in the dashboard.

Watch the phase and chosen side: turn outward by the selected 45° or 90°, drive the bounded departure, align parallel, pass the obstacle, turn inward by the same angle, search for the original line, centre the axle, realign, and confirm fresh line/forward data before resuming. A blocked return corridor makes the car advance farther alongside the obstacle, within a 500 mm pass budget. The top command panel shows an active detour as progress; if it fails, that panel and rejected START feedback show the recorded stop cause and phase, including during power-bank testing without USB. Missing echoes, both departure paths blocked, motion failure, and failure to confirm the line stop the car.

The dashboard retains the latest scan until Reset or the next detour. **Forward now** shows the current straight-ahead reading separately. Restart the Python process after updating so it serves the revised dashboard.

## Reset without power cycling

With matching firmware, **Reset** is available when fresh controller data reports IDLE or FAULT and the command link is ready. The Pico waits for the wheels to stop before clearing the state. For a failed detour, place the car back on the line first. Press Reset once and wait for **Reset complete**, then press START to begin a fresh run. A confirmed nearby obstacle starts a stationary clearance scan before wheel motion; missing echoes still block START. The Pico keeps its Wi-Fi and broker sessions during this reset. Restart the Python dashboard process after updating the repository so it serves the current Reset rules.

RESET sends `02`. The Pico replies with `04` for acceptance or `06` for rejection. An accepted reset reports `0C` on completion or `0E` on failure. Controller diagnostics v2 carry the pending flag, latched timing gap and limit in 31 bytes. Version 1 diagnostics can still be displayed.

For cold-start or reconnect timing, compare the USB serial `wifi: link up` and `mqtt: ready for commands` messages. The Pico initiates MQTT as soon as the hotspot gives it an IP and publishes readiness when the broker acknowledges its command subscription.

## Wheel stall and packet-format mismatch

The current interface shows **Clearance v3** in the top strip and has no width field. A page that still shows WIDTH and lacks the chosen-side/recovery fields is an older dashboard. Restart `app.py` from the `dashboard` directory of the same checkout used to rebuild the Pico firmware; leave the user's ignored Wi-Fi and calibration headers in place. For the current recovery changes use branch `fix/subsystem5-departure-recovery` until it is merged into main.

An invalid-length packet is now shown in the main notice with its received and expected byte counts. The current C obstacle publisher sends exactly seven bytes; the decoder accepts that format plus the legacy six-byte format. Do not reinterpret other lengths as valid measurements. Check the topic root, any other publishers using that root, and that the firmware and Python server were updated together. Repeated identical errors remain in the notice without filling the event history; a valid packet clears that channel's error.

**Wheel stalled** is a separate motor-controller fault: a commanded wheel has not supplied encoder pulses while drive power is being requested. A malformed obstacle packet does not itself set that motor fault. With the chassis safely raised, check whether both wheels physically turn. If neither turns, check the motor supply, battery/power-bank route, board power switch and motor connections. If a wheel turns but the fault remains, check that wheel's encoder power, common ground and signal (GP2 left, GP3 right on this firmware). The filtered RPM or zero bounded-move progress after a stop does not identify which connection failed. Resolve the cause before Reset and START; the stall stop remains enabled.

Host integration checks, from the repository root (a host C compiler is required):

```sh
python3 testing/protocol/test_obstacle_publisher.py -v
cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=undefined,bounds \
  -DPROHIBIT_DEF_SIZE_T -D_PICO_RP2040_ -DCNF_SMP=1 -Iinclude -Iconfig \
  testing/subsystem2/test_stall.c -o /tmp/test_stall
/tmp/test_stall
```

The protocol test compiles and executes the actual C publisher/packer, then decodes its bytes with the dashboard. The motor test covers valid encoder feedback, missing feedback on each side, a latched stop and explicit clearing. These tests do not establish the car's supply voltage, torque, wiring or encoder signals.
