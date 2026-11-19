# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |
| INT-03 | MQTT startup and reconnect deadlines | Dashboard readiness | host | `testing/integration/test_mqtt_startup.c` | First IP-ready poll attempts MQTT. Subscription sends readiness on the next poll. A stalled connection retries after its deadline. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

The MQTT regression passed with Pico SDK 2.2.0 lwIP headers. Run it with the local firmware Wi-Fi header in place.

```sh
port=lib/liblwip/sysdepend/pico_rp2040
cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=undefined,bounds \
  -D_PICO_RP2040_ -DCNF_SMP=1 -I"$PICO_SDK_PATH/lib/lwip/src/include" \
  -I"$port" -I"$port/include" -Iconfig -Iinclude \
  testing/integration/test_mqtt_startup.c -o /tmp/test_mqtt_startup
/tmp/test_mqtt_startup
```
