Imported donor history below; it does not describe validation of this merge. See [merge validation](../MERGE_VALIDATION.md) for results from the merged source.

# Dashboard telemetry verification

## Passed

- 23 Python protocol/state/HTTP tests (`python3 -m unittest discover -s tests -p 'test_*.py' -v`). Covers existing binary commands and local HTTP checks, every telemetry subscription/decoder, field bounds and padding, independent freshness, retained/malformed sample rejection, reconnect invalidation, fault labels and bounded history.
- JavaScript syntax check (`node --check web/app.js`).
- Real Mosquitto 2.0.21 plus Chromium 155 / Playwright integration, using paho-mqtt 2.1.0 and a simulated Pico protocol fixture.
- CLI startup/address prompt, no automatic START, actual HTTP-to-broker binary START/STOP, ACK display, rapid START/STOP ordering, all five telemetry topics through MQTT/HTTP/browser, RPM trend rendering, missing-data placeholders, independent stale indicators, motor/vehicle faults, mobile width, heartbeat expiry and broker loss.
- Browser integration reported no JavaScript errors. All test-started broker/server/browser processes were shut down.

Browser results:

```text
PASS CLI: empty address reprompted; valid IP starts local HTTP dashboard; no startup command
PASS browser: controls, heartbeat, all five telemetry cards and missing-data placeholders render
PASS browser: retained status does not enable START; connected STOP remains available
PASS end-to-end: button -> HTTP -> real Mosquitto -> binary 01/00, QoS 0, non-retained -> fixture ACK -> browser
PASS browser: rapid START then STOP preserves command order
PASS end-to-end telemetry: all five binary topics decoded over real broker, HTTP and browser
PASS browser: mobile layout has no horizontal overflow
PASS browser: independent telemetry expiry and motor/mission fault rendering
PASS browser: stale heartbeat disables START and keeps STOP available
PASS browser: broker loss disables controls; no auto-command; no JavaScript errors
```

## Limits

The protocol fixture is software, not the physical Pico. Actual car Wi-Fi, broker delivery from the Pico, sensors, motor movement and track completion were not tested. Terrain/obstacle test values demonstrate decoding only; they do not imply those producers are implemented in the current firmware. No firmware changes were needed for this dashboard update. No credentials, virtual environments or bytecode caches are included in the distributable.

The additive controller diagnostic packet and specific START blocker/IR display also passed unit and real-broker/browser tests. START disables for a fresh blocker while STOP remains available; clearing the blocker permits retry. The Pico startup/junction fix was validated in line-controller host tests and both firmware builds. Physical motion remains unverified.

Timing/reset update: version2 timing-gap/limit decoding and stopped-only RESET availability passed unit tests. The browser integration sends exact binary02 through the local broker, receives ACK04 and verifies the fixture remains IDLE without any automatic START. Physical car timing remains unverified.

## Compatibility and stall diagnosis — 20 November 2026

Tested revision: `1cdf0dd`. The reported screenshots contain an older dashboard layout (WIDTH still visible) and invalid-length obstacle telemetry. The current production C publisher sends seven bytes and the current decoder accepts that packet. The screenshots also report a separate wheel-stall motor fault. Hardware power, physical movement and encoder signals have not been observed.

- 32 dashboard unit/HTTP tests passed, including received/expected packet lengths, a persistent visible notice, suppression of duplicate error events, clearing on a valid packet, and motor-stall guidance.
- The actual C obstacle publisher and packer were compiled and run; their seven-byte output decoded successfully with the Python decoder.
- The production motor core passed its host regression with undefined-behavior/bounds sanitizers: encoder feedback on both wheels, missing feedback on either wheel, latched stop, no restart while faulted, explicit clear, and velocity-mode bounded-progress semantics.
- All 13 browser/MQTT checkpoints passed, with additional assertions for the Clearance v3 marker, visible mismatch banner, recovery on a valid packet, and mobile layout. The simulated heartbeat was refreshed at scenario boundaries to prevent test duration from accidentally expiring it before the deliberate stale-heartbeat scenario.
- No firmware control code or safety limits were changed in this diagnostic revision. Physical car testing remains blocked on the observation of whether the wheels turn and how the motors are powered.

[Host output](../testing/subsystem5/logs/2026-11-20_compatibility_host.txt) · [Browser output](../testing/subsystem5/logs/2026-11-20_compatibility_browser.txt)
