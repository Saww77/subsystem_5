/* Replay the production scan task's scanner, with the servo/ranger simulated. */
#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem5/subsystem5.c"

static const unsigned *echoes;
static unsigned echo_count, ping_count, angle_count, angles[8];
static unsigned published, published_range, cancel_at;
static uint32_t clock_ms, last_ping_at;

INT tm_printf(const UB *format, ...) { (void)format; return 0; }
ER tk_dly_tsk(RELTIM delay) { clock_ms += delay; return E_OK; }
void s5_hw_angle(unsigned angle)
{
    assert(angle_count < sizeof(angles) / sizeof(*angles));
    angles[angle_count++] = angle;
}
S5_PING s5_hw_ping(void)
{
    assert(ping_count < echo_count);
    assert(!ping_count || clock_ms - last_ping_at >= PING_INTERVAL_MS);
    last_ping_at = clock_ms;
    unsigned range = echoes[ping_count++];
    clock_ms += 60; /* Include even a long no-target pulse in the time budget. */
    return (S5_PING){.range_mm = range};
}
BOOL vehicle_obstacle_scan_current(uint32_t request)
{
    assert(request == 42);
    return !cancel_at || ping_count < cancel_at;
}
void vehicle_submit_obstacle_range(unsigned range)
{
    assert(servo_angle == S5_CENTRE_ANGLE);
    ++published;
    published_range = range;
}

static void reset(const unsigned *script, unsigned count)
{
    echoes = script;
    echo_count = count;
    ping_count = angle_count = published = cancel_at = 0;
    clock_ms = last_ping_at = 0;
    servo_angle = S5_CENTRE_ANGLE;
}

static void test_scan_to_plan(void)
{
    const unsigned script[] = {850, 840, 610, 600, 220, 218};
    reset(script, sizeof(script) / sizeof(*script));
    uint32_t request = 42;
    S5_PROFILE scan;
    assert(scan_obstacle(&scan, FALSE, &request));
    assert(ping_count == 6 && angle_count == 3);
    assert(angles[0] == 135 && angles[1] == 45 && angles[2] == 90);
    assert(scan.nearest_mm == 218 && scan.front_mm == 218);
    assert(published == 1 && published_range == 218);
    assert(scan.width_mm == S5_UNKNOWN_WIDTH && !scan.left_edge_valid && !scan.right_edge_valid);
    unsigned offset, pass;
    S5_FAILURE reason;
    assert(s5_plan_bypass(&scan, &offset, &pass, &reason) == S5_LEFT);
    assert(offset == S5_BYPASS_OFFSET_MM && reason == S5_FAIL_NONE);
    assert(clock_ms < 3000);
}

static void test_missing_echo_and_open_side(void)
{
    const unsigned script[] = {S5_UNKNOWN_RANGE, S5_UNKNOWN_RANGE, S5_UNKNOWN_RANGE,
        S5_UNKNOWN_RANGE, S5_UNKNOWN_RANGE, S5_OPEN_RANGE, S5_OPEN_RANGE, 220, 220};
    reset(script, sizeof(script) / sizeof(*script));
    S5_PROFILE scan;
    assert(scan_obstacle(&scan, TRUE, NULL)); /* Same scanner in motor-off bench. */
    assert(scan.left_path_mm == S5_UNKNOWN_RANGE && scan.right_path_mm == S5_OPEN_RANGE);
    assert(!published && servo_angle == S5_CENTRE_ANGLE);
    unsigned offset, pass;
    S5_FAILURE reason;
    assert(s5_plan_bypass(&scan, &offset, &pass, &reason) == S5_RIGHT);
}

static void test_no_echo_is_bounded(void)
{
    unsigned script[15];
    for (unsigned i = 0; i < 15; ++i) script[i] = S5_UNKNOWN_RANGE;
    reset(script, 15);
    S5_PROFILE scan;
    assert(scan_obstacle(&scan, FALSE, NULL));
    assert(ping_count == 15 && clock_ms < 3000);
    unsigned offset, pass;
    S5_FAILURE reason;
    assert(s5_plan_bypass(&scan, &offset, &pass, &reason) == S5_NO_SIDE);
    assert(reason == S5_FAIL_TARGET);
}

static void test_cancel_and_clear_path(void)
{
    const unsigned script[] = {850, 850, 600, 600, 700, 700};
    reset(script, 6);
    cancel_at = 1;
    uint32_t request = 42;
    S5_PROFILE scan;
    assert(!scan_obstacle(&scan, FALSE, &request));
    assert(servo_angle == S5_CENTRE_ANGLE && !published && ping_count == 1);
    reset(script, 6);
    assert(scan_obstacle(&scan, FALSE, &request));
    assert(scan.nearest_mm == S5_OPEN_RANGE && published_range == 700);
}

int main(void)
{
    test_scan_to_plan();
    test_missing_echo_and_open_side();
    test_no_echo_is_bounded();
    test_cancel_and_clear_path();
    puts("PASS production clearance scan, width-independent plan, echo loss, timing and cancellation");
}
