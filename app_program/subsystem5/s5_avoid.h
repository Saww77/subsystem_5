#ifndef S5_AVOID_H
#define S5_AVOID_H

#include <stdbool.h>
#include <stdint.h>
#include "s5_profile.h"

#define S5_RANGE_MAX_AGE_MS 300u
#define S5_SLOW_RANGE_MM 350u
#define S5_MANEUVER_STOP_MM 150u
#define S5_MAX_PASS_MM 500u
#define S5_APPROACH_RPM 20u
/* Fixed lateral departure, independent of an unreliable acoustic width fit.
 * Validate this offset and the motor turn calibration on the actual chassis. */
#define S5_BYPASS_OFFSET_MM 185u
#define S5_BYPASS_TURN_DEG 45u
#define S5_TURN_CLEARANCE_MM 100u
#define S5_REVERSE_MM 30u
#define S5_LONG_REVERSE_MM 60u
#define S5_SEARCH_MARGIN_MM 40u

/* Phase and side are also sent in the low six bits of /avoidance. */
typedef enum {
    S5_OFF, S5_PROFILE_PATH, S5_TURN_OUT, S5_SHIFT_OUT, S5_ALIGN_OUTSIDE,
    S5_PASS, S5_TURN_IN, S5_FIND_LINE, S5_CENTER_LINE, S5_ALIGN_LINE,
    S5_ALIGN_RETRY, S5_COMPLETE, S5_FAILED, S5_CHECK_LINE, S5_BACK_UP
} S5_PHASE;
typedef enum { S5_NO_SIDE, S5_LEFT, S5_RIGHT } S5_SIDE;
typedef enum { S5_WAIT, S5_STOP, S5_ADVANCE, S5_PIVOT_LEFT, S5_PIVOT_RIGHT, S5_REVERSE, S5_RESCAN } S5_ACTION;
typedef enum { S5_MOTION_NONE, S5_MOTION_QUEUED, S5_MOTION_PENDING, S5_MOTION_DONE, S5_MOTION_FAILED } S5_MOTION;
typedef enum {
    S5_FAIL_NONE, S5_FAIL_SCAN_TIMEOUT, S5_FAIL_TARGET, S5_FAIL_WIDTH, S5_FAIL_ROOM,
    S5_FAIL_SIDE, S5_FAIL_ECHO, S5_FAIL_CLOSE, S5_FAIL_MOTION, S5_FAIL_LINE,
    S5_FAIL_STOP, S5_FAIL_INTERLOCK, S5_FAIL_PHASE_TIMEOUT,
    S5_FAIL_LEFT_LIMIT, S5_FAIL_RIGHT_LIMIT, S5_FAIL_BOTH_LIMITS
} S5_FAILURE;

typedef struct {
    uint32_t now, range_at, profile_at, line_at;
    S5_PROFILE profile;
    unsigned range_mm : 12;
    unsigned sensor_to_axle_mm : 9;
    unsigned motion : 3;
    bool stopped : 1, range_valid : 1, profile_valid : 1;
    bool line_hit : 1, line_junction : 1, line_centered : 1, can_retrace : 1;
    bool near_original_line : 1, can_retrace_far : 1;
} S5_AVOID_INPUT;
typedef struct {
    uint32_t phase_at, last_line_at;
    unsigned phase:4, side:2, line_samples:2, reverse_available:1;
    unsigned reverse_long:1, scan_retried:1, failure:4, failed_phase:4;
    unsigned offset_mm:8, next_pass_mm:9, passed_mm:9;
} S5_AVOID;
typedef struct {
    uint16_t action : 3;
    uint16_t distance_mm : 9;
} S5_AVOID_OUTPUT;

S5_SIDE s5_plan_bypass(const S5_PROFILE *profile, unsigned *offset_mm, unsigned *pass_mm,
                       S5_FAILURE *failure);
unsigned s5_diagonal_mm(unsigned lateral_mm);
unsigned s5_reentry_detection_mm(unsigned offset_mm, unsigned sensor_to_axle_mm);
void s5_avoid_begin(S5_AVOID *state, uint32_t now);
void s5_avoid_abort(S5_AVOID *state, S5_FAILURE reason, uint32_t now);
S5_AVOID_OUTPUT s5_avoid_step(S5_AVOID *state, const S5_AVOID_INPUT *in);
#endif
