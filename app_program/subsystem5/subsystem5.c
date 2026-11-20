#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "s5_profile.h"
#include "../subsystem2/subsystem2.h"
#include "../vehicle_controller/vehicle_controller.h"

#ifdef CPU_RP2040
#define SERVO_SETTLE_MS 120
#define SERVO_SWEEP_SETTLE_MS 240
#define SERVO_HOME_SETTLE_MS 350
#define PING_INTERVAL_MS 60
#define CLEARANCE_ATTEMPTS 5u
#define RANGE_LOG_INTERVAL_MS 2000u
#define SCAN_WAKE 1u
_Static_assert(S5_CENTRE_ANGLE + S5_BYPASS_TURN_DEG <= S5_SCAN_MAX_ANGLE &&
               S5_CENTRE_ANGLE - S5_BYPASS_TURN_DEG >= S5_SCAN_MIN_ANGLE,
               "The servo must reach both departure headings");

static ID scan_event;
static unsigned servo_angle = S5_CENTRE_ANGLE;
static const char *const echo_status[] = {
    [S5_ECHO_QUIET] = "rise timeout",
    [S5_ECHO_PULSE] = "pulse",
    [S5_ECHO_HIGH] = "high before trigger",
    [S5_ECHO_TIMING] = "timing fault",
    [S5_ECHO_FALL_TIMEOUT] = "fall timeout",
    [S5_ECHO_LATE] = "late fall",
    [S5_ECHO_NO_TARGET] = "no target",
};
#if APP_S5_SENSOR_TEST
EXPORT void subsystem5_sensor_test(void)
{
    tm_printf((UB*)"[S5 SENSOR] GP16 trigger, GP17 echo. Hold a flat target in front of the sensor.\n");
    if (!s5_hw_init_ranging()) {
        tm_printf((UB*)"[S5 SENSOR] Could not start ultrasonic capture.\n");
        return;
    }

    for (;;) {
        tk_dly_tsk(350);
        S5_PING ping = s5_hw_ping();
        if (s5_range_measured(ping.range_mm))
            tm_printf((UB*)"[S5 SENSOR] range=%u mm pulse=%u us\n",
                      (unsigned)ping.range_mm, (unsigned)ping.pulse_us);
        else
            tm_printf((UB*)"[S5 SENSOR] echo=%s pulse=%u us range=%s\n",
                      echo_status[ping.state], (unsigned)ping.pulse_us,
                      ping.range_mm == S5_OPEN_RANGE ? "open" : "unavailable");
    }
}
#endif

static void aim(unsigned angle)
{
    unsigned change = angle > servo_angle ? angle - servo_angle : servo_angle - angle;
    s5_hw_angle(angle);
    servo_angle = angle;
    tk_dly_tsk(change > 15u ? SERVO_SWEEP_SETTLE_MS : SERVO_SETTLE_MS);
}

static void log_ping(const char *tag, const char *context, unsigned angle, S5_PING ping)
{
    if (s5_range_measured(ping.range_mm))
        tm_printf((UB*)"%s %s angle=%u echo=%u us range=%u mm\n",
                  tag, context, angle, (unsigned)ping.pulse_us, (unsigned)ping.range_mm);
    else
        tm_printf((UB*)"%s %s angle=%u echo=%s pulse=%u us range=%s\n",
                  tag, context, angle, echo_status[ping.state], (unsigned)ping.pulse_us,
                  ping.range_mm == S5_OPEN_RANGE ? "open" : "unavailable");
}

/* Confirm a bearing, including real no-target pulses. A timeout is never
 * clearance. Servo settling also gives the transducer its inter-ping quiet time. */
static unsigned read_bearing(unsigned angle, BOOL verbose, const uint32_t *request)
{
    aim(angle);
    unsigned previous = S5_UNKNOWN_RANGE;
    for (unsigned attempt = 0; attempt < CLEARANCE_ATTEMPTS; ++attempt) {
        if (request && !vehicle_obstacle_scan_current(*request)) return S5_UNKNOWN_RANGE;
        if (attempt) tk_dly_tsk(PING_INTERVAL_MS);
        S5_PING ping = s5_hw_ping();
        if (verbose) log_ping("[S5 BENCH]", "clearance", angle, ping);
        unsigned confirmed = s5_confirm_scan_range(previous, ping.range_mm);
        if (s5_range_available(confirmed)) return confirmed;
        previous = ping.range_mm;
    }
    return S5_UNKNOWN_RANGE;
}

static BOOL scan_obstacle(S5_PROFILE *profile, BOOL verbose, const uint32_t *request)
{
    const char *tag = verbose ? "[S5 BENCH]" : "[S5]";
    tm_printf((UB*)"%s checking left and right 45-degree departure clearance\n", tag);
    *profile = (S5_PROFILE){
        .nearest_mm = S5_UNKNOWN_RANGE, .width_mm = S5_UNKNOWN_WIDTH,
        .angle_offset_deg = S5_MAX_ANGLE - S5_CENTRE_ANGLE,
        .left_mm = S5_UNKNOWN_RANGE, .right_mm = S5_UNKNOWN_RANGE,
        .left_path_mm = S5_UNKNOWN_RANGE, .right_path_mm = S5_UNKNOWN_RANGE,
    };
    profile->left_path_mm = read_bearing(S5_CENTRE_ANGLE + S5_BYPASS_TURN_DEG, verbose, request);
    if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;
    profile->right_path_mm = read_bearing(S5_CENTRE_ANGLE - S5_BYPASS_TURN_DEG, verbose, request);
    if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;
    /* Refresh the forward measurement after returning home; never interpret a
     * side-facing echo as a forward interlock sample. */
    unsigned forward = read_bearing(S5_CENTRE_ANGLE, verbose, request);
    if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;
    if (request) vehicle_submit_obstacle_range(forward);
    profile->nearest_mm = forward;
    profile->front_mm = s5_range_measured(forward) ?
        (forward < S5_MAX_PROJECTED_FRONT_MM ? forward : S5_MAX_PROJECTED_FRONT_MM) : 0;
    profile->left_mm = profile->left_path_mm;
    profile->right_mm = profile->right_path_mm;
    tm_printf((UB*)"%s clearance forward=%u mm left=%u mm right=%u mm (4094=open, 4095=unavailable)\n",
              tag, forward, (unsigned)profile->left_path_mm, (unsigned)profile->right_path_mm);
    if (s5_range_available(forward) && forward > S5_CLEAR_MM)
        profile->nearest_mm = S5_OPEN_RANGE;
    return TRUE;

cancelled:
    aim(S5_CENTRE_ANGLE);
    return FALSE;
}

static void log_forward(S5_PING ping, const char *context)
{
    static BOOL logged;
    static uint32_t last_log_at;
    SYSTIM time;
    tk_get_tim(&time);
    if (logged && time.lo - last_log_at < RANGE_LOG_INTERVAL_MS) return;
    logged = TRUE;
    last_log_at = time.lo;
    log_ping("[S5]", context, S5_CENTRE_ANGLE, ping);
}

static BOOL setup(void)
{
    BOOL ready = s5_hw_init();
    if (!ready)
        tm_printf((UB*)"[S5] servo or ultrasonic capture failed to start\n");
    else
        tm_printf((UB*)"[S5] GP%u centred: %u us. Waiting for sensor to settle.\n",
                  S5_SERVO_PIN, S5_SERVO_CENTRE_US);
    /* The driver holds centre as soon as PWM is enabled. Let the sensor settle
       there before accepting the first range on either firmware profile. */
    tk_dly_tsk(SERVO_HOME_SETTLE_MS);
    return ready;
}

static void scan_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    if (!setup()) return;
    tm_printf((UB*)"[S5] GP16 trigger, GP17 echo. Entry=%u mm, clear=%u mm\n",
              S5_SCAN_STOP_MM, S5_CLEAR_MM);

    for (;;) {
        S5_PING ping = s5_hw_ping();
        vehicle_submit_obstacle_range(ping.range_mm);
        log_forward(ping, "forward");
        UINT events;
        ER wake = tk_wai_flg(scan_event, SCAN_WAKE, TWF_ORW | TWF_BITCLR, &events, PING_INTERVAL_MS);

        uint32_t request;
        if (wake == E_OK && vehicle_obstacle_scan_request(&request)) {
            S5_PROFILE profile;
            BOOL scanned = scan_obstacle(&profile, FALSE, &request);
            vehicle_finish_obstacle_scan(scanned ? &profile : NULL, request);
            /* The sweep finishes with a forward ping. */
            tk_dly_tsk(PING_INTERVAL_MS);
        } else if (wake != E_TMOUT) {
            /* A canceled request must still respect the transducer's quiet time. */
            tk_dly_tsk(PING_INTERVAL_MS);
        }
    }
}

#if APP_S5_BENCH
#define BENCH_PING_MS 350
#define BENCH_CONFIRMATIONS 2u
#define BENCH_TIMEOUT_CONFIRMATIONS 3u

static unsigned bench_wait_for_echo(BOOL want_target)
{
    unsigned confirmed = 0;
    unsigned timeouts = 0;
    for (;;) {
        tk_dly_tsk(BENCH_PING_MS);
        S5_PING ping = s5_hw_ping();
        log_ping("[S5 BENCH]", want_target ? "waiting for target" : "waiting to rearm",
                 S5_CENTRE_ANGLE, ping);

        unsigned range = ping.range_mm;
        bool measured = s5_range_measured(range);
        bool timed_out = ping.state == S5_ECHO_QUIET || ping.state == S5_ECHO_FALL_TIMEOUT ||
                         ping.state == S5_ECHO_LATE;
        if (!timed_out) timeouts = 0;
        else if (timeouts < BENCH_TIMEOUT_CONFIRMATIONS) ++timeouts;

        bool matches = want_target ? measured && range <= S5_SCAN_STOP_MM :
                       range == S5_OPEN_RANGE || (measured && range > S5_CLEAR_MM);
        if (matches) ++confirmed;
        else if (!want_target || timeouts != 1u) confirmed = 0;
        if (confirmed == BENCH_CONFIRMATIONS) return range;
        /* In this motor-off test a missing target can restart the detector. */
        if (!want_target && timeouts == BENCH_TIMEOUT_CONFIRMATIONS) {
            tm_printf((UB*)"[S5 BENCH] Three echo timeouts. Restarting target detection.\n");
            return S5_UNKNOWN_RANGE;
        }
    }
}

static void bench_print_clearance(const char *side, unsigned range)
{
    if (s5_range_measured(range))
        tm_printf((UB*)"[S5 BENCH] %s clearance=%u mm\n", side, range);
    else
        tm_printf((UB*)"[S5 BENCH] %s clearance=%s\n", side,
                  range == S5_OPEN_RANGE ? "open" : "unavailable");
}

static void bench_print_plan(const S5_PROFILE *profile)
{
    if (profile->nearest_mm == S5_OPEN_RANGE) {
        tm_printf((UB*)"[S5 BENCH] plan=CONTINUE turn=0 deg\n");
        return;
    }

    bench_print_clearance("left 45 deg", profile->left_path_mm);
    bench_print_clearance("right 45 deg", profile->right_path_mm);
    unsigned offset, pass;
    S5_FAILURE reason;
    S5_SIDE side = s5_plan_bypass(profile, &offset, &pass, &reason);
    if (side == S5_NO_SIDE) {
        tm_printf((UB*)"[S5 BENCH] plan=STOP turn=0 deg reason=%u\n", (unsigned)reason);
        return;
    }
    tm_printf((UB*)"[S5 BENCH] plan=%s turn=%u deg diagonal=%u mm lateral_offset=%u mm pass=%u mm\n",
              side == S5_LEFT ? "LEFT" : "RIGHT", S5_BYPASS_TURN_DEG,
              s5_diagonal_mm(offset), offset, pass);
}

EXPORT void subsystem5_bench(void)
{
    tm_printf((UB*)"[S5 BENCH] GP12 servo, GP16 trigger, GP17 echo. Drive outputs off.\n");
    tm_printf((UB*)"[S5 BENCH] Scan needs two forward echoes within %u mm. One timeout is allowed between them.\n",
              S5_SCAN_STOP_MM);
    if (!setup()) return;

    /* The servo has no position feedback. Make the commanded movement easy to
       verify before trusting the bearings in the scan log. */
    const unsigned check_angles[] = {
#ifdef S5_SERVO_RIGHT_OUTER_US
        S5_SCAN_MIN_ANGLE,
#endif
        S5_MIN_ANGLE, 45u, 135u, S5_MAX_ANGLE,
#ifdef S5_SERVO_LEFT_OUTER_US
        S5_SCAN_MAX_ANGLE,
#endif
        S5_CENTRE_ANGLE,
    };
    tm_printf((UB*)"[S5 BENCH] Keep the box out of the sweep for the startup background check.\n");
    for (unsigned i = 0; i < sizeof(check_angles) / sizeof(check_angles[0]); ++i) {
        tm_printf((UB*)"[S5 BENCH] Servo check: watch sensor turn to %u deg (PWM %u us).\n",
                  check_angles[i], s5_servo_pulse_us(check_angles[i]));
        aim(check_angles[i]);
        tk_dly_tsk(1000);
        log_ping("[S5 BENCH]", "background check", check_angles[i], s5_hw_ping());
    }

    for (;;) {
        unsigned detected = bench_wait_for_echo(TRUE);
        tm_printf((UB*)"[S5 BENCH] obstacle detected at %u mm. Checking departure clearance.\n", detected);
        S5_PROFILE profile;
        if (scan_obstacle(&profile, TRUE, NULL))
            bench_print_plan(&profile);
        else
            tm_printf((UB*)"[S5 BENCH] plan=STOP turn=0 deg. Profile unavailable.\n");
        tm_printf((UB*)"[S5 BENCH] Servo centred. Remove target or move it beyond %u mm to rearm.\n",
                  S5_CLEAR_MM);
        (void)bench_wait_for_echo(FALSE);
        tm_printf((UB*)"[S5 BENCH] Test rearmed.\n");
    }
}
#endif
#endif

EXPORT ER subsystem5_init(void)
{
#ifdef CPU_RP2040
    T_CFLG event = {0};
    scan_event = tk_cre_flg(&event);
    if (scan_event < E_OK) return scan_event;
#endif
    return E_OK;
}

void subsystem5_request_scan(void)
{
#ifdef CPU_RP2040
    if (scan_event > 0) (void)tk_set_flg(scan_event, SCAN_WAKE);
#endif
}

EXPORT ER subsystem5_start(void)
{
#ifdef CPU_RP2040
    vehicle_enable_obstacle_monitor();
    if (scan_event <= 0) return E_OBJ;
    ID task = app_start_task((FP)scan_task, S5_PRI_MAIN, TP_PRC2, "obstacle_scan");
    return task < E_OK ? task : E_OK;
#else
    return E_OK;
#endif
}
