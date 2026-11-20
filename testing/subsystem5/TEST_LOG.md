# Subsystem 5 tests

## Current clearance workflow

Production driving no longer estimates obstacle width. The earlier flat-face/corner fitter rejected real scans when wide-beam echoes did not agree with its geometric assumptions. That missing estimate previously vetoed the detour. The scanner now measures the actual two 45° departure headings and a fresh forward range, using up to five pings per bearing and two consecutive agreeing results. A timeout is unavailable, not open space.

The planner chooses the clearer usable side and uses a 185 mm lateral target (262 mm diagonal at 45°). It requires 412 mm departure clearance, including the 150 mm stopping margin, and at least 200 mm forward room. These are bounded maneuvers, not a measurement of object width. Wide obstacles, narrow gaps and chassis/servo/encoder calibration still require physical track validation. If the inward return corridor is blocked, the car advances alongside the obstacle in 60 mm steps, up to a total 500 mm pass budget. Two fresh IR samples confirming a crossing near the predicted line trigger axle centring and a 45° alignment turn; fresh centred-line and forward-range evidence are required for handoff.

The dashboard shows clearance, chosen side and recovery phase. It hides width. The MQTT packet retains its unused width sentinel (2047) for compatibility. Historical width fitter tests and calibration utilities remain for analysis; they are not used by the production or bench scanner.

## Stationary hardware check

Build `make -C build_make S5_BENCH=1 -j4` and flash the generated `mtk3pico_smp1_usb_cdc_s5bench.uf2`. Motors remain disabled. GP12 drives the servo, GP16 triggers the ultrasonic sensor, and GP17 receives its level-shifted echo. Open USB serial at 115200 baud. Verify that the startup servo checks point at the printed angles and return to centre. Adjust the ignored `config/s5_servo_calibration.h` using the example header if needed.

Place a flat target 200–220 mm ahead. After two near echoes, the same scanner used for driving checks 135°, 45° and 90°. Expect `clearance forward=... left=... right=...`, followed by `plan=LEFT`, `plan=RIGHT` or a stopped plan. Make one departure path clear, then swap sides. Confirm that unavailable echoes never select a direction. Remove the target or move it beyond 270 mm to rearm. Bench-only rearming after three timeouts does not authorize production movement.

## Track check (physical validation pending)

1. Verify motor direction and actual encoder-bounded 45° turns with wheels raised. Confirm the 60 mm sensor-to-axle setting (`S3_ADVANCE_MM`) and servo bearings on the mounted car.
2. On a clear test track, start line following and place an obstacle ahead. Confirm slowing, stopping, then a clearance scan while the wheels are stopped.
3. Leave sufficient room on the left and verify departure, passing, inward search, axle centring, realignment and resumed line following. Repeat on the right and with a deeper obstacle that requires extra passing steps.
4. Block both departure paths and verify STOP. Also check echo loss, operator STOP and a missing return line. Save the USB log and actual stop/rejoin locations. No host or build check substitutes for these measurements.

## Host regressions

From the repository root:

```sh
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined,bounds \
  testing/subsystem5/test_avoidance.c app_program/subsystem5/s5_avoid.c \
  app_program/vehicle_controller/vehicle_core.c -o /tmp/test_avoidance
/tmp/test_avoidance
cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=undefined,bounds \
  -ffunction-sections -fdata-sections -DPROHIBIT_DEF_SIZE_T -D_PICO_RP2040_ -DCNF_SMP=1 \
  -Iinclude -Iconfig testing/subsystem5/test_clearance_scan.c \
  app_program/subsystem5/s5_avoid.c -Wl,--gc-sections -o /tmp/test_clearance_scan
/tmp/test_clearance_scan
cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=undefined,bounds \
  -DPROHIBIT_DEF_SIZE_T -D_PICO_RP2040_ -DCNF_SMP=1 -DTM_MQTT=1 -Iinclude -Iconfig \
  testing/subsystem5/test_controller_events.c app_program/subsystem5/s5_avoid.c \
  app_program/vehicle_controller/vehicle_core.c -o /tmp/test_controller_events
/tmp/test_controller_events
```

The scanner replay checks actual production bearing commands, echo confirmation, cancellation and bounded scan duration. The controller replay exercises both complete detours through real dispatch and simulated motor acknowledgements. Pure planner tests cover blocked returns, missing width/edges, lost echoes, stale line data and cancellation. Hardware remains unverified.

## Historical data representation and results

The following tables and earlier records describe previous width-estimation revisions. Current driving uses the clearance workflow above.


| Field | Range | Representation |
|---|---|---|
| Captured pulse and state | 0–59,999 µs, 6 capture states and 7 echo outcomes | 16 bits for pulse duration and 3 bits for state in both ISR and task storage |
| Trigger, rise and freshness timestamps | 0–2³² − 1 µs or ms, rolling | 32 bits each, using wrap-safe elapsed time |
| Sweep range and closest distance | 0–4000 mm, 4094 open, 4095 unavailable | 12 bits per entry and closest-distance field; 12 bits on wire for measured distance |
| Sweep bearing | 30°–150° by default, optionally 15°–165°, in 5° steps | Implicit scan index. Reported bearings stay in the standard range of the 7-bit wire field. |
| Estimated width | 0–2000 mm, 2047 unavailable | 11 bits in the profile and on wire |
| Obstacle front projection | 0–511 mm, saturated above 511 | 9 bits in the profile |
| Background range near each edge | 0–4000 mm, 4094 open, 4095 unavailable | 12 bits each in the local profile |
| Departure heading range | 0–4000 mm, 4094 open, 4095 unavailable | 12 bits each at 45° left and right in the profile and MQTT |
| Left and right lateral bounds | Signed displacement | Signed 13 bits, one validity bit and one sweep-limit bit per side |
| Forward interlock status | Open confirmation and blocked flags | 1 bit each in the range gate |
| Ultrasonic echo fault | Set or clear | 1 bit in the vehicle fault mask |
| Detour phase and side | 0–14, none/left/right | 4 and 2 bits in RAM and the low six bits of the MQTT avoidance byte |
| Stop cause, phase at stop, and forward range | 16 cause codes, phases 0–14, range 0–4095 | 4, 4 and 12 bits in the extended MQTT avoidance packet |
| Movement request and reply | 7 planner actions, 5 reply states | 3 bits each in the planner |
| Travel, pass distance and axle offset | 0–511 mm | 9 bits each in planner storage |
| Lateral bypass offset | 0–255 mm | 8 bits in planner storage |
| Crossing count, reverse authorization and input flags | 0–2 crossing samples, boolean flags | 2-bit count and one bit per flag |
| Motion queue maneuver argument | Forward 1–2000, 90° 2048–2049, reverse 2050–4049, 45° 4094–4095 | 12-bit argument within a 16-bit command |

The obstacle packet uses seven bytes. Its saved-profile flag distinguishes sweep readings from forward samples. The extended avoidance packet uses four bytes. The dashboard also accepts legacy six-byte obstacle packets and one-byte avoidance packets. A motion command uses two bytes with a separate queue timestamp. The ISR and scan task share capture state on one core. The task arms and drains it with local interrupts masked. Vehicle snapshots shared with MQTT are mutex-protected.

## Results

| ID | Check | Result |
|---|---|---|
| S5-U01 | Compute nearest point, angle, width and missing clearance | Passed on host |
| S5-U02 | Leave width unavailable when the obstacle extends beyond the fine scan | Passed on host |
| S5-U03 | Pack angle and reject an out-of-bounds angle | Passed on host |
| S5-I01 | Decode the actual C encoder's output in the dashboard decoder | Passed on host |
| S5-I02 | Show valid values and missing clearance in Chromium via real Mosquitto with a simulated Pico | Passed |
| S5-I03 | Verify retained and malformed packets do not refresh obstacle data | Passed in dashboard unit suite |
| S5-H01 | Verify servo direction, echo timing, stop distance and scan accuracy using measured obstacles | Pending Pico W bench test |
| S5-H02 | Verify timing under concurrent Wi-Fi, motion and scan tasks | Pending Pico W bench test |
| S5-I04 | Simulate left and right detours, a blocked return, IR reacquisition and stop cases | Passed on host |
| S5-I05 | Display chosen side and line-reacquired stage from live MQTT packets | Passed in browser with simulated Pico |
| S5-H03 | Measure left/right bypass success, alignment, collision clearance and end-to-end duration | Pending track test |
| S5-U04 | Clear the interlock after two open pings, including a long out-of-range pulse | Passed on host |
| S5-U05 | Preserve the interlock after a single open ping following a close return or fault | Passed on host |
| S5-U06 | Reject a scan with close returns at every angle as evidence of side clearance | Passed on host |
| S5-I06 | Complete either detour with a confirmed open side and no measured width | Passed in controller simulation |
| S5-H04 | Verify GP12 centres the sensor straight ahead at startup and check a target at known range | Pending Pico W bench test |
| S5-U07 | Keep a silent input interlocked at boot and reject implausibly short pulses | Passed on host |
| S5-U08 | Accept a measured echo while preserving stuck-high and timer fault behavior | Passed on host |
| S5-U09 | Accept a complete out-of-range pulse as sensor response evidence | Passed on host |
| S5-I07 | Exercise GP12 pulse selection and GP16 trigger with high ECHO using modeled RP2040 registers | Passed on host |
| S5-I08 | Wait for two near echoes before sweeping, report width, and rearm only after two clear echoes | Passed with simulated bench input |
| S5-U10 | Distinguish pre-trigger high from missing rising or falling edges | Passed on host with the range interlock check |
| S5-I09 | Retry a missed fine-scan echo at 90° and rearm after three echo timeouts | Passed with simulated bench input. The retry produced a 28 mm profile from scripted returns. |
| S5-I10 | Scan a scripted target at 280 mm with one unavailable edge bearing | Passed with simulated bench input. The next usable return resolved the edge and the profile reported 48 mm. |
| S5-U11 | Slow the approach and stop at 220 mm, then require measured width and space for the 45° diagonal | Passed in controller simulation. |
| S5-I11 | Detect and profile a target when one timeout separates two valid readings | Passed in a host bench replay. Two timeouts or a farther reading cleared the pending detection. Open-side confirmation still required consecutive open pings. |
| S5-I12 | Boot the sensor-only test and report measured range, open space and an echo timeout | Pico build and boot-path inspection passed. A host replay printed the expected 200 mm, 400 mm, no-target and timeout reports. Physical ranging is pending. |

Run the controller and planner test from the repository root:

```sh
cc -std=c11 -Wall -Wextra -Werror testing/subsystem5/test_avoidance.c \
  app_program/subsystem5/s5_avoid.c app_program/vehicle_controller/vehicle_core.c \
  -o /tmp/test_avoidance
/tmp/test_avoidance
```

The RCWL-9610A bench replay used obstacle echoes around 160–170 mm and consecutive open pulses around 47.1 ms. It reported a 116 mm width and a 90° left turn. Host tests also confirmed that a single open pulse or an unknown echo cannot release the interlock. The updated bench and MQTT firmware built successfully. Track testing is pending.

## Workflow revision verification

Results on 19 November 2026 for firmware commit 4ae863a. The table above retains earlier test history. The current echo deadline supersedes the historical silence and late-pulse behavior.

| Check | Result |
|---|---|
| Host scan and echo regression | Passed with undefined-behavior and bounds sanitizers. Covers packed samples, off-centre/high-angle targets, clipped profiles, and late, out-of-order and simultaneous edges. |
| Host controller and planner regression | Passed with the same sanitizers. Covers both detours, bounded reverse and rescanning, clear path continuation, rejecting old scans, and confirmation of line crossing and handoff. |
| Dashboard unit suite | All 26 tests passed, including the reverse phase in the existing avoidance byte. |
| Pico MQTT, stationary bench and UART self-test firmware | All three linked and generated UF2 files. |
| Optional Buddy 1 demo profile | Both baseline and revised trees failed to link because the demo source list omits the vehicle controller referenced by telemetry. |
| Pico bench and track | Pending physical GP12/GP16/GP17, echo accuracy and open-track behavior, concurrency timing, and left/right bypass and recovery measurements. |

[Host test output](logs/2026-11-19_host.txt)

The static section comparison uses the same Pico SDK 2.2.0 and Arm GCC 14.2.1 MQTT configuration for both commits.

| Build | Text | Data | BSS | Static RAM (data + BSS) |
|---|---:|---:|---:|---:|
| 5e667d1 baseline | 359,812 B | 112 B | 82,720 B | 82,832 B |
| 4ae863a workflow revision | 361,764 B | 112 B | 82,736 B | 82,848 B |

The sweep array shrank from 50 to 38 bytes. Arm layout checks show the avoidance input shrinking from 44 to 32 bytes and its output from 8 to 2 bytes. The allocated scan task stack stays at 2,048 bytes. Static RAM grew by 16 bytes with the edge-capture and retrace state. These measurements cover representation and build size. Measure scan timing, motor-loop latency and recovery accuracy on the robot before recording system performance.

Run the scan and echo regression from the repository root:

```sh
cc -std=c11 -Wall -Wextra -Werror testing/subsystem5/test_scan.c \
  app_program/subsystem5/s5_profile.c -o /tmp/test_s5_scan
/tmp/test_s5_scan
python -m unittest discover -s dashboard/tests
```

## Integrated controller and interrupt regression

| Check | Result |
|---|---|
| Controller dispatcher with simulated motor acknowledgements | Passed. Checks unavailable and confirmed open ranges, loss of range between START and dispatch, STOP priority, motor settling, scan dispatch after the motor stops, and cancellation on link loss. |
| Host echo, avoidance, and Buddy 4 safety regressions | Passed. |
| Dashboard unit and browser tests | All 26 unit tests passed. The Chromium test with Mosquitto and a simulated Pico confirmed that echo loss disables START, a clear path re-enables it, and STOP remains available. |
| Pico W firmware | MQTT with and without Subsystem 4, single-core UART, sensor-only, and stationary servo-scan profiles produced UF2 files. |
| Robot | On-car timing, forward echo and detour behavior still need measurement. |

The controller snapshot decreased from 124 to 52 bytes. The mission state and stop reason each use 3 bits, each blocker uses 4 bits, each IR darkness reading uses 10 bits, and each readiness flag uses 1 bit. The MQTT controller message is 31 bytes, its command payload is 1 byte, and the S2 queue stores 2-byte command words. The MQTT build with Subsystem 4 used 82,992 bytes of static RAM on baseline `9a56007` and 82,912 bytes after the change, measured as ELF data plus BSS with Pico SDK 2.2.0 and Arm GCC 12.2.1. These figures exclude runtime allocations.

Run the dispatcher regression from the repository root.

```sh
cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=undefined,bounds \
  -DPROHIBIT_DEF_SIZE_T -D_PICO_RP2040_ -DCNF_SMP=1 -DTM_MQTT=1 -Iinclude -Iconfig \
  testing/subsystem5/test_controller_events.c app_program/subsystem5/s5_avoid.c \
  app_program/vehicle_controller/vehicle_core.c -o /tmp/test_controller_events
/tmp/test_controller_events
```

With the integrated MQTT firmware and the wheels raised, hold a flat target about 45 cm straight ahead. Check that the USB serial `[S5] forward` log reports a measured range and that START becomes available once the other checks pass. Confirm STOP clears the wheel commands. Then verify the module's empty-track response before attempting a grounded detour.

## Diagonal bypass and stopped reset

Host regressions covered both bypass sides, the measured-width requirement, a blocked return corridor, line recovery and the original heading. The S2 host check verified that each new turn uses the correct direction and 45° encoder targets. Controller and dashboard tests covered reset for a failed detour with no motor fault bits, clearing a stopped motor fault, and reporting a reset timeout. Track clearance, measured turn angles, and time to complete the photo's workflow still need measurements on the car.

## Scan retention and recovery

Controller regression replayed a failed width estimate at 208 mm followed by a 606 mm forward reading and a STOP request. The sweep and failure cause remained available through STOP. Reset cleared the detour without starting motion. The browser replay with Mosquitto showed the two distances separately and verified that Reset remained available with an older motor-mode report. Host tests also checked one stationary retry, 45° side selection, and departure after a bounded retrace. The physical width estimate and detour still need a track test.

## Flat-face width regression

A synthetic sweep modelled a 220 mm face at 209 mm with stable corner echoes extending beyond the aimed rays. The previous estimator reported 415 mm on those inputs. The earlier flat-face estimator reported 220 mm and the law-of-cosines calculation reports 222 mm using the stable corner ranges. The host tests also passed for a ray-like scan and an ambiguous sweep with no measurable edge-range change. These are simulated returns. Physical accuracy still needs a stationary check with a measured box.

The follow-up host test covers a narrow face at 215 mm with only 4–5 mm of corner-range change. It also recovers the earlier synthetic 220 mm width when one adjacent bearing misses and another outer sample sees the front instead of the corner. A scan with no verified background on one side still reports width as unavailable and records which side failed.

The two recorded track sweeps reproduced a left sweep-limit stop. At 150°, the near return was 240–245 mm, while the right sweep found background at 925–942 mm at 30°. The replay leaves the width unconfirmed and verifies the right bracket. The controller regression covers a 30 mm retrace followed by a fresh scan when authorized, and a stopped outcome when a retrace is unavailable or the second scan still reaches the limit. The measured 230–234 mm departure readings do not qualify for the detour.

The motor-off bench capture showed a 197 mm centre return, mixed near and far echoes at 135°, and five returns of 800–2013 mm at 140°. Replaying those readings kept 135° unresolved and confirmed background at 140°, within 10° of the last confirmed hit. The original replay estimated 192 mm. With the corner agreement check, the same replay leaves the width unconfirmed because the left corner ranges are inconsistent. The independent 45° departure checks reported unavailable on the left and 208 mm on the right.

A second bench capture used a measured 80 mm object and reported 212 mm. The replay confirms both edge brackets but rejects the width. The final near returns on each side are 9 mm farther than the preceding near returns, so they do not establish stable corner distances. The replay preserves the measured background and conservative motion bounds. The fixed-point cosine calculation gives 283 mm for the raw last-hit readings of 212 mm and 207 mm at 85° separation. Applying the beam correction fitted from the 80 mm target gives 87 mm using the same ranges at 23.8° separation. This is a replay of the calibration target. It needs an independent bench check.

## Clearance revision validation — 20 November 2026

Tested firmware and dashboard commit: `8cb693168e55aede8de1caae8d209aa6cb3a1b31`.

| Check | Result |
|---|---|
| Production scanner replay (normal and motor-off bench) | Passed with undefined-behavior and bounds sanitizers. Confirms bearing order, repeated echoes, unknown/open distinction, cancellation, and a simulated worst-case scan below 3 seconds. |
| Planner and controller core | Passed with address and undefined-behavior sanitizers. Both sides complete without width/edge estimates; blocked return retries, range/line failures and stop paths remain covered. |
| Production controller dispatcher | Passed with undefined-behavior and bounds sanitizers. Both complete detours exercise queued commands, acknowledgements, IR crossing, axle centring, heading restoration and resumed wheel commands. |
| Dashboard | 28 unit/HTTP tests and 13 browser/MQTT checks passed. Width is absent even when legacy packets contain it or mark it unknown. |
| RP2040 firmware builds | Default dual-core USB/Wi-Fi, MQTT, stationary bench, and single-core UART self-test profiles generated UF2 files without compiler warnings. SDK 2.2.0, Arm GCC 14.2.1. |
| Real MQTT connection | Not tested; the compile check used the repository's example settings through an external include directory. No live credentials were added. |
| Pico W and track | Not run. Confirm servo/motor direction, stopping distance, the 185 mm lateral target, line capture and clearance around the actual vehicle on hardware. |

[Saved host output](logs/2026-11-20_clearance_host.txt) · [Browser/MQTT output](logs/2026-11-20_clearance_browser.txt) · [Build summary](logs/2026-11-20_clearance_builds.txt)
