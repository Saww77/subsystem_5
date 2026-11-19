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
#define FINE_ATTEMPTS 5u
#define RANGE_LOG_INTERVAL_MS 2000u
#define SCAN_WAKE 1u
#define BYPASS_SCAN_STEPS (S5_BYPASS_TURN_DEG / S5_FINE_STEP)

_Static_assert(S5_BYPASS_TURN_DEG % S5_FINE_STEP == 0 && BYPASS_SCAN_STEPS <= S5_CENTRE_INDEX,
               "The sweep must sample the actual departure heading");

static ID scan_event;
static unsigned servo_angle = S5_CENTRE_ANGLE;
#if __has_include("s5_width_calibration.h")
#include "s5_width_calibration.h"
_Static_assert(S5_WIDTH_CAL_MIN_MM > 0 && S5_WIDTH_CAL_MIN_MM <= S5_WIDTH_CAL_MAX_MM &&
               S5_WIDTH_CAL_MAX_MM <= S5_CLEAR_MM, "Width calibration must fit the scan range");
_Static_assert(S5_WIDTH_LEFT_EXCESS_TENTHS <= 750u && S5_WIDTH_RIGHT_EXCESS_TENTHS <= 750u,
               "Width calibration angles must fit the sweep");
static const S5_WIDTH_CALIBRATION width_calibration = {
    .left_excess_tenths = S5_WIDTH_LEFT_EXCESS_TENTHS,
    .right_excess_tenths = S5_WIDTH_RIGHT_EXCESS_TENTHS,
    .min_face_mm = S5_WIDTH_CAL_MIN_MM,
    .max_face_mm = S5_WIDTH_CAL_MAX_MM,
};
#else
static const S5_WIDTH_CALIBRATION width_calibration = {0};
#endif
static const char *const echo_status[] = {
    [S5_ECHO_QUIET] = "rise timeout",
    [S5_ECHO_PULSE] = "pulse",
    [S5_ECHO_HIGH] = "high before trigger",
    [S5_ECHO_TIMING] = "timing fault",
    [S5_ECHO_FALL_TIMEOUT] = "fall timeout",
    [S5_ECHO_LATE] = "late fall",
    [S5_ECHO_NO_TARGET] = "no target",
};
static const char *const width_status[] = {
    [S5_WIDTH_OK] = "no width estimate",
    [S5_WIDTH_FACE] = "no consistent forward face",
    [S5_WIDTH_LEFT_EDGE] = "no verified background near left edge",
    [S5_WIDTH_RIGHT_EDGE] = "no verified background near right edge",
    [S5_WIDTH_BOTH_EDGES] = "no verified background near either edge",
    [S5_WIDTH_LEFT_ECHO] = "left corner ranges do not agree",
    [S5_WIDTH_RIGHT_ECHO] = "right corner ranges do not agree",
    [S5_WIDTH_BOTH_ECHOES] = "corner ranges do not identify a width",
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

static unsigned read_bearing(unsigned angle, BOOL verbose, bool fine, unsigned edge_reference)
{
    aim(angle);
    unsigned previous = S5_UNKNOWN_RANGE;
    const unsigned attempts = fine ? FINE_ATTEMPTS : 3u;
    bool edge = s5_range_measured(edge_reference);
    unsigned ranges[FINE_ATTEMPTS];
    for (unsigned attempt = 0; attempt < attempts; ++attempt) {
        if (attempt) tk_dly_tsk(PING_INTERVAL_MS);
        S5_PING ping = s5_hw_ping();
        if (verbose) log_ping("[S5 BENCH]", attempt ? "recheck" : "scan", servo_angle, ping);
        if (!fine && s5_range_measured(ping.range_mm)) return ping.range_mm;

        ranges[attempt] = ping.range_mm;
        unsigned confirmed = s5_confirm_scan_range(previous, ping.range_mm);
        if (s5_range_available(confirmed) && (!edge || !s5_background_beyond(confirmed, edge_reference)))
            return confirmed;
        /* A missed ping can separate two matching measured echoes.
         * Open space still needs two consecutive results. */
        if (!s5_range_available(ping.range_mm) && s5_range_measured(previous)) continue;
        previous = ping.range_mm;
    }
    return edge ? s5_edge_background(ranges, attempts, edge_reference) : S5_UNKNOWN_RANGE;
}

static unsigned read_at(unsigned index, BOOL verbose, bool fine, unsigned edge_reference)
{
    return read_bearing(s5_scan_bearing(index), verbose, fine, edge_reference);
}

/* Find the echo transition to one degree wherever the returns are consistent. */
static void refine_width_edge(unsigned *hit, unsigned *hit_mm, unsigned *background, int side,
                              unsigned reference, BOOL verbose, const uint32_t *request)
{
    while (*background - *hit > 1u) {
        if (request && !vehicle_obstacle_scan_current(*request)) return;
        unsigned middle = (*hit + *background) / 2u;
        unsigned angle = (unsigned)((int)S5_CENTRE_ANGLE + side * (int)middle);
        unsigned range = read_bearing(angle, verbose, true, reference);
        if (s5_same_obstacle(range, reference)) {
            *hit = middle;
            *hit_mm = range;
        } else if (s5_background_beyond(range, reference)) {
            *background = middle;
        } else break;
    }
}

/* Consecutive missing bearings bound the scan work in the driving profile. */
static unsigned scan_edge(S5_SCAN *samples, unsigned anchor, unsigned reference,
                          int step, BOOL verbose, const uint32_t *request)
{
    int index = (int)anchor;
    unsigned misses = 0;
    while (index + step >= 0 && index + step < (int)S5_SCAN_POINTS &&
           (verbose || misses < 4u)) {
        if (request && !vehicle_obstacle_scan_current(*request)) break;
        index += step;
        unsigned range = read_at((unsigned)index, verbose, true, reference);
        s5_scan_set(samples, (unsigned)index, range);
        const char *side = step < 0 ? "left" : "right";
        if (s5_background_beyond(range, reference)) {
            if (verbose)
                tm_printf((UB*)"[S5 BENCH] %s edge confirmed at angle=%u deg\n", side, servo_angle);
            return (unsigned)index;
        } else if (s5_same_obstacle(range, reference)) {
            misses = 0;
            if (verbose)
                tm_printf((UB*)"[S5 BENCH] %s angle=%u deg: %u mm grouped with %u mm target\n",
                          side, servo_angle, range, reference);
        } else {
            ++misses;
            if (verbose)
                tm_printf((UB*)"[S5 BENCH] %s angle=%u deg: edge unconfirmed\n", side, servo_angle);
        }
    }
    if (verbose)
        tm_printf((UB*)"[S5 BENCH] %s sweep limit at %u deg: no confirmed edge\n",
                  step < 0 ? "left" : "right", servo_angle);
    return (unsigned)index;
}

static BOOL scan_obstacle(S5_PROFILE *profile, BOOL verbose, const uint32_t *request)
{
    const char *tag = verbose ? "[S5 BENCH]" : "[S5]";
    S5_SCAN samples = {0};
    tm_printf((UB*)"%s confirming forward target before GP12 coarse sweep\n", tag);
    unsigned centre = read_at(S5_CENTRE_INDEX, verbose, true, S5_UNKNOWN_RANGE);
    s5_scan_set(&samples, S5_CENTRE_INDEX, centre);
    for (unsigned i = S5_COARSE_FIRST; i <= S5_COARSE_LAST; i += S5_COARSE_STEP) {
        if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;
        if (i == S5_CENTRE_INDEX && s5_range_available(centre)) continue;
        s5_scan_set(&samples, i, read_at(i, verbose, false, S5_UNKNOWN_RANGE));
    }

    unsigned anchor = s5_scan_target(&samples);
    if (anchor == S5_NO_TARGET) {
        aim(S5_CENTRE_ANGLE);
        unsigned forward = s5_hw_ping().range_mm;
        if (request) vehicle_submit_obstacle_range(forward);
        /* A missing echo on any coarse bearing leaves that sector unresolved. */
        for (unsigned i = S5_COARSE_FIRST; i <= S5_COARSE_LAST; i += S5_COARSE_STEP)
            if (!s5_range_available(s5_scan_get(&samples, i))) return FALSE;
        if (!s5_range_available(forward) || forward <= S5_CLEAR_MM) return FALSE;
        *profile = (S5_PROFILE){.nearest_mm = S5_OPEN_RANGE};
        tm_printf((UB*)"%s no obstacle in the coarse sweep\n", tag);
        return TRUE;
    }

    unsigned coarse = s5_scan_get(&samples, anchor);
    if (verbose)
        tm_printf((UB*)"%s fine scan starting at %u deg\n", tag, s5_scan_bearing(anchor));
    unsigned fine = anchor == S5_CENTRE_INDEX && s5_range_available(centre) ?
                    centre : read_at(anchor, verbose, true, S5_UNKNOWN_RANGE);
    if (!s5_same_obstacle(fine, coarse) || fine > S5_CLEAR_MM) {
        tm_printf((UB*)"%s fine scan could not confirm the %u mm target at %u deg\n",
                  tag, coarse, servo_angle);
        goto cancelled;
    }
    s5_scan_set(&samples, anchor, fine);

    unsigned first = scan_edge(&samples, anchor, fine, -1, verbose, request);
    unsigned last = scan_edge(&samples, anchor, fine, 1, verbose, request);
    if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;

    S5_WIDTH_SCAN width_scan;
    *profile = s5_profile(&samples, first, last, anchor, &width_scan);
    bool calibrated = width_scan.off_axis && width_calibration.min_face_mm &&
                      width_scan.face_mm >= width_calibration.min_face_mm &&
                      width_scan.face_mm <= width_calibration.max_face_mm;
    if (width_scan.on_rays || (width_scan.off_axis && (verbose || calibrated))) {
        refine_width_edge(&width_scan.left_hit_deg, &width_scan.left_hit_mm,
                          &width_scan.left_background_deg, 1, fine, verbose, request);
        refine_width_edge(&width_scan.right_hit_deg, &width_scan.right_hit_mm,
                          &width_scan.right_background_deg, -1, fine, verbose, request);
        if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;
    }
    if (verbose && width_scan.face_mm)
        tm_printf((UB*)"%s width scan depth=%u mm left=%u..%u deg right=%u..%u deg off_axis=%u hits L=%u R=%u mm\n",
                  tag, width_scan.face_mm, width_scan.left_hit_deg, width_scan.left_background_deg,
                  width_scan.right_hit_deg, width_scan.right_background_deg, (unsigned)width_scan.off_axis,
                  width_scan.left_hit_mm, width_scan.right_hit_mm);
    if (calibrated) {
        profile->width_mm = s5_calibrated_width(&width_scan, &width_calibration);
        profile->width_status = profile->width_mm == S5_UNKNOWN_WIDTH ? S5_WIDTH_BOTH_ECHOES : S5_WIDTH_OK;
    } else if (width_scan.on_rays) {
        unsigned refined = s5_refined_ray_width(&width_scan);
        if (refined != S5_UNKNOWN_WIDTH) profile->width_mm = refined;
    }
    profile->left_path_mm = read_at(S5_CENTRE_INDEX - BYPASS_SCAN_STEPS, verbose, true, S5_UNKNOWN_RANGE);
    if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;
    profile->right_path_mm = read_at(S5_CENTRE_INDEX + BYPASS_SCAN_STEPS, verbose, true, S5_UNKNOWN_RANGE);
    if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;
    aim(S5_CENTRE_ANGLE);
    if (request) vehicle_submit_obstacle_range(s5_hw_ping().range_mm);
    tm_printf((UB*)"%s 45-degree path L=%u mm R=%u mm (4094=open, 4095=unavailable)\n",
              tag, (unsigned)profile->left_path_mm, (unsigned)profile->right_path_mm);
    if (profile->width_mm == S5_UNKNOWN_WIDTH) {
        tm_printf((UB*)"%s width=unavailable reason=%s centre=%u mm nearest_distance=%u mm\n",
                  tag, width_status[profile->width_status], s5_scan_get(&samples, S5_CENTRE_INDEX),
                  (unsigned)profile->nearest_mm);
        if (profile->left_clipped)
            tm_printf((UB*)"%s left sweep limit: near return %u mm at %u deg (PWM %u us)\n",
                      tag, s5_scan_get(&samples, 0), S5_SCAN_MAX_ANGLE, s5_servo_pulse_us(S5_SCAN_MAX_ANGLE));
        if (profile->right_clipped)
            tm_printf((UB*)"%s right sweep limit: near return %u mm at %u deg (PWM %u us)\n",
                      tag, s5_scan_get(&samples, S5_SCAN_POINTS - 1), S5_SCAN_MIN_ANGLE,
                      s5_servo_pulse_us(S5_SCAN_MIN_ANGLE));
        if (!verbose) {
            for (unsigned i = first; i <= last; ++i)
                tm_printf((UB*)"[S5] width echo angle=%u deg range=%u mm\n",
                          s5_scan_bearing(i), s5_scan_get(&samples, i));
        }
        if (verbose && (!profile->left_edge_valid || !profile->right_edge_valid))
            tm_printf((UB*)"%s Use a box narrow enough to see past both edges, with a background over %u mm farther away.\n",
                      tag, S5_PROFILE_TOLERANCE_MM);
    } else {
        tm_printf((UB*)"%s profile nearest_distance=%u mm width=%u mm angle=%u deg method=%s\n",
                  tag, (unsigned)profile->nearest_mm, (unsigned)profile->width_mm,
                  s5_servo_angle(profile->angle_offset_deg), calibrated ? "calibrated cosine" : "edge cosine");
    }
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
        tm_printf((UB*)"[S5 BENCH] obstacle detected at %u mm. Starting scan and turn planning.\n", detected);
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
