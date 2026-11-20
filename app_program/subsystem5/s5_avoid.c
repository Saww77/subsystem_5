#include "s5_avoid.h"

#define FRONT_MARGIN_MM 15u
#define MAX_OFFSET_MM (S5_CLEAR_MM - FRONT_MARGIN_MM)
#define FOLLOW_AHEAD_MARGIN_MM 130u
#define MIN_PASS_MM 60u
#define RETRY_STEP_MM 60u
#define MOTION_SETTLE_MS 260u
#define SCAN_TIMEOUT_MS 14000u
#define PHASE_TIMEOUT_MS 6500u

_Static_assert(S5_CLEAR_MM + FOLLOW_AHEAD_MARGIN_MM <= S5_MAX_PASS_MM,
               "The maximum profiled distance must fit the bypass budget");
_Static_assert(S5_MAX_PASS_MM < 512u, "Bypass distances must fit the 9-bit fields");
_Static_assert(S5_BYPASS_OFFSET_MM > 0 && S5_BYPASS_OFFSET_MM <= MAX_OFFSET_MM,
               "The clearance departure offset must fit the maneuver envelope");
_Static_assert(MAX_OFFSET_MM <= 255u, "Lateral offset must fit in 8 bits");
_Static_assert((MAX_OFFSET_MM + S5_SEARCH_MARGIN_MM) * 1415u <= 511000u,
               "Diagonal search must fit in 9 bits");

static bool at_or_after(uint32_t time, uint32_t reference)
{
    return (uint32_t)(time - reference) < UINT32_C(0x80000000);
}

static S5_AVOID_OUTPUT go(S5_AVOID *state, S5_PHASE phase, uint32_t now)
{
    state->phase = phase;
    state->phase_at = now;
    state->line_samples = 0;
    state->last_line_at = now;
    return (S5_AVOID_OUTPUT){.action = S5_WAIT};
}

void s5_avoid_abort(S5_AVOID *state, S5_FAILURE reason, uint32_t now)
{
    if (state->phase == S5_FAILED) return;
    state->failure = reason;
    state->failed_phase = state->phase;
    go(state, S5_FAILED, now);
}

static S5_AVOID_OUTPUT fail(S5_AVOID *state, S5_FAILURE reason, uint32_t now)
{
    s5_avoid_abort(state, reason, now);
    return (S5_AVOID_OUTPUT){.action = S5_STOP};
}

void s5_avoid_begin(S5_AVOID *state, uint32_t now)
{
    *state = (S5_AVOID){.phase = S5_PROFILE_PATH, .phase_at = now};
}

unsigned s5_diagonal_mm(unsigned lateral_mm)
{
    /* At 45 degrees, both projections are equal. Round up the travel. */
    return (lateral_mm * 1415u + 999u) / 1000u;
}

unsigned s5_reentry_detection_mm(unsigned offset_mm, unsigned sensor_to_axle_mm, bool turn_90)
{
    if (offset_mm <= S5_SEARCH_MARGIN_MM) return 0;
    unsigned search_mm = offset_mm - S5_SEARCH_MARGIN_MM;
    if (!turn_90) search_mm = s5_diagonal_mm(search_mm);
    return search_mm > sensor_to_axle_mm ? search_mm - sensor_to_axle_mm : 0;
}

static bool departure_room(unsigned offset, unsigned front)
{
    return offset <= MAX_OFFSET_MM && offset + FRONT_MARGIN_MM <= front;
}

S5_SIDE s5_plan_bypass(const S5_PROFILE *profile, unsigned *offset_mm, unsigned *pass_mm, unsigned *turn_deg,
                       S5_FAILURE *failure)
{
    *failure = S5_FAIL_TARGET;
    if (!s5_range_measured(profile->nearest_mm) || profile->nearest_mm > S5_CLEAR_MM)
        return S5_NO_SIDE;

    /* Width and edge fitting are diagnostic only. Choose between the actual
     * departure headings using repeat-confirmed range, then recheck forward
     * range at each stopped phase. The return corridor is checked separately. */
    *failure = S5_FAIL_ROOM;
    if (profile->front_mm < S5_TURN_CLEARANCE_MM) return S5_NO_SIDE;

    unsigned left_path = profile->left_path_mm, right_path = profile->right_path_mm;
    *failure = !s5_range_available(left_path) || !s5_range_available(right_path) ?
               S5_FAIL_ECHO : S5_FAIL_SIDE;
    unsigned needed = s5_diagonal_mm(S5_BYPASS_OFFSET_MM) + S5_MANEUVER_STOP_MM;
    bool room = departure_room(S5_BYPASS_OFFSET_MM, profile->front_mm);
    bool left = room && s5_range_available(left_path) && left_path >= needed;
    bool right = room && s5_range_available(right_path) && right_path >= needed;
    *turn_deg = S5_BYPASS_TURN_DEG;
    if (!left && !right) {
        /* A diagonal may intersect the obstacle even though a stationary pivot
         * is possible. Turn square to the track, then require a NEW forward
         * range on that heading before any lateral translation is dispatched.
         * The 45-degree scan alone never authorizes a blind 90-degree drive. */
        *turn_deg = 90u;
        left = s5_range_available(left_path) && left_path >= S5_TURN_CLEARANCE_MM;
        right = s5_range_available(right_path) && right_path >= S5_TURN_CLEARANCE_MM;
    }
    if (!left && !right) return S5_NO_SIDE;

    *failure = S5_FAIL_NONE;
    bool use_left = left && (!right || left_path >= right_path);
    *offset_mm = S5_BYPASS_OFFSET_MM;
    unsigned forward_goal = profile->nearest_mm + FOLLOW_AHEAD_MARGIN_MM;
    /* A square departure has no forward projection. */
    unsigned departure_forward = *turn_deg == 90u ? 0u : *offset_mm;
    *pass_mm = forward_goal > departure_forward + MIN_PASS_MM ?
               forward_goal - departure_forward : MIN_PASS_MM;
    return use_left ? S5_LEFT : S5_RIGHT;
}

S5_AVOID_OUTPUT s5_avoid_step(S5_AVOID *state, const S5_AVOID_INPUT *in)
{
    S5_PHASE phase = state->phase;
    S5_AVOID_OUTPUT wait = {.action = S5_WAIT};
    if (phase == S5_OFF || phase == S5_COMPLETE || phase == S5_FAILED) return wait;

    uint32_t elapsed = in->now - state->phase_at;
    bool range_fresh = in->range_valid && in->now - in->range_at <= S5_RANGE_MAX_AGE_MS;
    if (in->motion == S5_MOTION_FAILED) return fail(state, S5_FAIL_MOTION, in->now);
    if (elapsed >= (phase == S5_PROFILE_PATH ? SCAN_TIMEOUT_MS : PHASE_TIMEOUT_MS)) {
        S5_FAILURE reason = phase == S5_PROFILE_PATH && !in->profile_valid ? S5_FAIL_SCAN_TIMEOUT :
                            !range_fresh ? S5_FAIL_ECHO : S5_FAIL_PHASE_TIMEOUT;
        return fail(state, reason, in->now);
    }
    if (phase == S5_PROFILE_PATH) {
        /* The scan belongs to this stop and has completed with the servo centred. */
        if (!in->profile_valid || in->profile_at == state->phase_at ||
            !at_or_after(in->profile_at, state->phase_at) || !in->stopped) return wait;
        if (in->profile.nearest_mm == S5_OPEN_RANGE) {
            if (!range_fresh) return wait;
            if (in->range_mm > S5_CLEAR_MM) return go(state, S5_CHECK_LINE, in->now);
        }
        S5_PROFILE profile = in->profile;
        if (range_fresh && in->range_mm < profile.front_mm) profile.front_mm = in->range_mm;
        unsigned offset, pass, turn;
        S5_FAILURE reason;
        S5_SIDE side = s5_plan_bypass(&profile, &offset, &pass, &turn, &reason);
        if (side != S5_NO_SIDE) {
            if (!range_fresh) return wait;
            state->side = side;
            state->turn_90 = turn == 90u;
            state->offset_mm = offset;
            state->next_pass_mm = pass;
            return go(state, S5_TURN_OUT, in->now);
        }
        if (!state->scan_retried && (reason == S5_FAIL_TARGET || reason == S5_FAIL_WIDTH ||
                                    reason == S5_FAIL_ECHO)) {
            state->scan_retried = true;
            go(state, S5_PROFILE_PATH, in->now);
            return (S5_AVOID_OUTPUT){.action = S5_RESCAN};
        }
        /* Change vantage point only along an authorized, previously tracked segment. */
        if (state->reverse_available && in->line_centered && !in->line_junction) {
            state->reverse_available = false;
            state->reverse_long = state->reverse_long && reason == S5_FAIL_ROOM &&
                                  profile.front_mm <= S5_CLEAR_MM - S5_LONG_REVERSE_MM &&
                                  range_fresh && in->range_mm <= S5_CLEAR_MM - S5_LONG_REVERSE_MM;
            return go(state, S5_BACK_UP, in->now);
        }
        return fail(state, reason, in->now);
    }

    if (phase == S5_BACK_UP) {
        if (in->line_junction || !in->line_centered) return fail(state, S5_FAIL_LINE, in->now);
        if (!range_fresh) return fail(state, S5_FAIL_ECHO, in->now);
        if (in->motion == S5_MOTION_DONE) return go(state, S5_PROFILE_PATH, in->now);
        if (in->motion != S5_MOTION_NONE || !in->stopped || elapsed < MOTION_SETTLE_MS ||
            !at_or_after(in->range_at, state->phase_at + MOTION_SETTLE_MS)) return wait;
        return (S5_AVOID_OUTPUT){.action = S5_REVERSE,
                                 .distance_mm = state->reverse_long ? S5_LONG_REVERSE_MM : S5_REVERSE_MM};
    }

    bool forward = phase == S5_SHIFT_OUT || phase == S5_PASS ||
                   phase == S5_FIND_LINE || phase == S5_CENTER_LINE;
    if (phase == S5_CHECK_LINE) {
        if (!in->stopped || !range_fresh || elapsed < MOTION_SETTLE_MS ||
            !at_or_after(in->range_at, state->phase_at + MOTION_SETTLE_MS)) return wait;
        if (in->range_mm <= S5_CLEAR_MM) return fail(state, S5_FAIL_CLOSE, in->now);
        if (in->line_junction) return fail(state, S5_FAIL_LINE, in->now);
        if (!in->line_centered || !at_or_after(in->line_at, state->phase_at + MOTION_SETTLE_MS))
            return wait;
        return go(state, S5_COMPLETE, in->now);
    }
    if ((in->motion == S5_MOTION_QUEUED || in->motion == S5_MOTION_PENDING) &&
        (!range_fresh || in->range_mm < S5_TURN_CLEARANCE_MM ||
         (forward && in->range_mm <= S5_MANEUVER_STOP_MM)))
        return fail(state, range_fresh ? S5_FAIL_CLOSE : S5_FAIL_ECHO, in->now);

    if (phase == S5_FIND_LINE && (in->motion == S5_MOTION_PENDING || in->motion == S5_MOTION_DONE)) {
        if (in->near_original_line && at_or_after(in->line_at, state->phase_at) && in->line_at != state->last_line_at) {
            state->last_line_at = in->line_at;
            state->line_samples = in->line_hit ? state->line_samples + 1u : 0u;
        }
        if (state->line_samples >= 2) {
            go(state, S5_CENTER_LINE, in->now);
            return (S5_AVOID_OUTPUT){.action = S5_STOP};
        }
    }

    if (in->motion == S5_MOTION_QUEUED || in->motion == S5_MOTION_PENDING) return wait;
    if (in->motion == S5_MOTION_DONE) {
        switch (phase) {
        case S5_TURN_OUT:
            state->widening = false;
            return go(state, S5_SHIFT_OUT, in->now);
        case S5_SHIFT_OUT: return go(state, S5_ALIGN_OUTSIDE, in->now);
        case S5_ALIGN_OUTSIDE: case S5_ALIGN_RETRY: return go(state, S5_PASS, in->now);
        case S5_PASS:
            state->passed_mm += state->next_pass_mm;
            return go(state, S5_TURN_IN, in->now);
        case S5_TURN_IN: return go(state, S5_FIND_LINE, in->now);
        case S5_FIND_LINE: return fail(state, S5_FAIL_LINE, in->now);
        case S5_CENTER_LINE: return go(state, S5_ALIGN_LINE, in->now);
        case S5_ALIGN_LINE: return go(state, S5_CHECK_LINE, in->now);
        default: return fail(state, S5_FAIL_MOTION, in->now);
        }
    }

    if (!in->stopped || elapsed < MOTION_SETTLE_MS ||
        !range_fresh || !at_or_after(in->range_at, state->phase_at + MOTION_SETTLE_MS)) return wait;
    if (!forward) {
        if (in->range_mm < S5_TURN_CLEARANCE_MM) return fail(state, S5_FAIL_CLOSE, in->now);
        if (phase == S5_TURN_OUT && !state->turn_90 &&
            in->range_mm < state->offset_mm + FRONT_MARGIN_MM)
            return fail(state, S5_FAIL_ROOM, in->now);
        bool outward = phase == S5_TURN_OUT || phase == S5_ALIGN_RETRY || phase == S5_ALIGN_LINE;
        bool left = outward ? state->side == S5_LEFT : state->side == S5_RIGHT;
        return (S5_AVOID_OUTPUT){.action = left ? S5_PIVOT_LEFT : S5_PIVOT_RIGHT,
                                 .turn_90 = state->turn_90 && !state->widening};
    }
    unsigned distance = phase == S5_SHIFT_OUT ? (state->turn_90 ? state->offset_mm : s5_diagonal_mm(state->offset_mm)) :
                        phase == S5_PASS ? state->next_pass_mm :
                        phase == S5_CENTER_LINE ? in->sensor_to_axle_mm :
                        state->turn_90 ? state->offset_mm + S5_SEARCH_MARGIN_MM :
                        s5_diagonal_mm(state->offset_mm + S5_SEARCH_MARGIN_MM);
    if (in->range_mm < distance + S5_MANEUVER_STOP_MM) {
        if (phase == S5_SHIFT_OUT && !state->turn_90 && in->range_mm >= S5_TURN_CLEARANCE_MM) {
            /* The first 45-degree turn is complete, but its live echo does not
             * permit departure. Add only the remaining 45 degrees, then use
             * square-route distances and 90-degree alignment turns. */
            state->turn_90 = state->widening = true;
            state->next_pass_mm += state->offset_mm;
            return go(state, S5_TURN_OUT, in->now);
        }
        if (phase != S5_FIND_LINE || state->passed_mm == S5_MAX_PASS_MM)
            return fail(state, S5_FAIL_CLOSE, in->now);
        unsigned remaining = S5_MAX_PASS_MM - state->passed_mm;
        state->next_pass_mm = remaining < RETRY_STEP_MM ? remaining : RETRY_STEP_MM;
        return go(state, S5_ALIGN_RETRY, in->now);
    }
    if (phase == S5_FIND_LINE) {
        if (in->line_hit || in->line_junction) return fail(state, S5_FAIL_LINE, in->now);
        state->last_line_at = in->line_at;
    }
    return (S5_AVOID_OUTPUT){.action = S5_ADVANCE, .distance_mm = distance};
}
