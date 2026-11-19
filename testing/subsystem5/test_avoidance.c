/* Host tests for the production controller and planner. */
#include "../../app_program/vehicle_controller/vehicle_core.h"
#include <assert.h>
#include <stdio.h>

static void step(VC_MODEL *model, VC_INPUT *input, VC_OUTPUT *output, unsigned ms)
{
    input->avoidance.now += ms;
    if (input->avoidance.range_valid) input->avoidance.range_at = input->avoidance.now;
    vc_core_step(model, input, output);
}

static VC_INPUT track_input(void)
{
    return (VC_INPUT){
        .ready = true, .line_valid = true, .link_ok = true,
        .left_rpm = 30, .right_rpm = 30,
        .avoidance = {.stopped = true, .range_valid = true, .range_mm = 1000,
                      .sensor_to_axle_mm = 60},
    };
}

static void profile(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out, S5_PROFILE scan)
{
    in->start = true;
    step(model, in, out, 20);
    assert(model->state == VC_FOLLOW_LINE && !out->stop);
    in->start = false;
    in->avoidance.range_mm = S5_SCAN_STOP_MM + 1;
    step(model, in, out, 20);
    assert(out->left_rpm == 20 && out->right_rpm == 20);

    S5_RANGE_GATE gate = {0};
    assert(s5_accept_range(&gate, S5_SCAN_STOP_MM + 1) && !gate.blocked);
    assert(s5_accept_range(&gate, S5_SCAN_STOP_MM) && gate.blocked);
    assert(s5_accept_range(&gate, S5_CLEAR_MM) && gate.blocked);
    assert(s5_accept_range(&gate, S5_CLEAR_MM + 1) && !gate.blocked);
    in->obstacle_blocked = in->obstacle_detected = true;
    in->avoidance.range_mm = S5_SCAN_STOP_MM;
    step(model, in, out, 20);
    assert(out->stop && model->state == VC_AVOIDING && model->avoidance.phase == S5_PROFILE_PATH);

    in->avoidance.profile = scan;
    in->avoidance.profile_valid = true;
    in->avoidance.profile_at = in->avoidance.now - 20;
    step(model, in, out, 20);
    assert(model->avoidance.phase == S5_PROFILE_PATH); /* ignore pre-stop scan */
    in->avoidance.profile_at = in->avoidance.now + 2000;
    step(model, in, out, 2000);
}

static S5_PROFILE obstacle(bool left)
{
    return (S5_PROFILE){.nearest_mm = S5_SCAN_STOP_MM, .front_mm = S5_SCAN_STOP_MM, .width_mm = 90,
                        .left_mm = 320, .right_mm = 330,
                        .left_path_mm = left ? 850 : 600, .right_path_mm = left ? 600 : 850,
                        .left_edge_mm = -50, .right_edge_mm = 40,
                        .left_edge_valid = true, .right_edge_valid = true};
}

static void start_move(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out,
                       S5_ACTION action, unsigned travel)
{
    in->avoidance.motion = S5_MOTION_NONE;
    step(model, in, out, 260);
    assert(out->maneuver.action == action && out->maneuver.distance_mm == travel);
    in->avoidance.motion = S5_MOTION_PENDING;
}

static void end_move(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out)
{
    in->avoidance.motion = S5_MOTION_DONE;
    step(model, in, out, 500);
    in->avoidance.motion = S5_MOTION_NONE;
}

static void reach_inward_turn(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out, bool left, bool open_side)
{
    S5_ACTION outward = left ? S5_PIVOT_LEFT : S5_PIVOT_RIGHT;
    S5_ACTION inward = left ? S5_PIVOT_RIGHT : S5_PIVOT_LEFT;
    S5_PROFILE scan = obstacle(left);
    if (open_side) {
        if (left) scan.left_path_mm = S5_OPEN_RANGE;
        else scan.right_path_mm = S5_OPEN_RANGE;
    }
    profile(model, in, out, scan);
    assert(model->avoidance.side == (left ? S5_LEFT : S5_RIGHT));

    start_move(model, in, out, outward, 0);
    end_move(model, in, out);
    in->avoidance.range_mm = open_side ? S5_OPEN_RANGE : 1000;
    in->obstacle_blocked = in->obstacle_detected = false;
    start_move(model, in, out, S5_ADVANCE, left ? 262 : 248);
    end_move(model, in, out);
    start_move(model, in, out, inward, 0);
    end_move(model, in, out);
    start_move(model, in, out, S5_ADVANCE, left ? 165 : 175);
    end_move(model, in, out);
    start_move(model, in, out, inward, 0);
    end_move(model, in, out);
    assert(model->state == VC_RECOVER_LINE && model->avoidance.phase == S5_FIND_LINE);
}

static void test_rejoin(bool left, bool retry, bool open_side)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    reach_inward_turn(&model, &in, &out, left, open_side);
    S5_ACTION outward = left ? S5_PIVOT_LEFT : S5_PIVOT_RIGHT;
    S5_ACTION inward = left ? S5_PIVOT_RIGHT : S5_PIVOT_LEFT;
    if (retry) {
        in.avoidance.range_mm = 240;
        step(&model, &in, &out, 260);
        assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_ALIGN_RETRY);
        start_move(&model, &in, &out, outward, 0);
        end_move(&model, &in, &out);
        in.avoidance.range_mm = 1000;
        start_move(&model, &in, &out, S5_ADVANCE, 60);
        end_move(&model, &in, &out);
        start_move(&model, &in, &out, inward, 0);
        end_move(&model, &in, &out);
    }

    start_move(&model, &in, &out, S5_ADVANCE, left ? 319 : 305);
    in.avoidance.line_hit = true;
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    assert(model.avoidance.line_samples == 0); /* a marking too early is not the original line */
    in.avoidance.near_original_line = true;
    in.avoidance.line_junction = true; /* both outer sensors may cross together */
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    assert(!out.stop);
    step(&model, &in, &out, 20);
    assert(!out.stop); /* the same reading can't confirm the line twice */
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    assert(out.stop && out.maneuver.action == S5_STOP && model.avoidance.phase == S5_CENTER_LINE);

    in.avoidance.line_hit = in.avoidance.line_junction = false;
    start_move(&model, &in, &out, S5_ADVANCE, 60);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, outward, 0);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_CHECK_LINE);
    in.avoidance.range_valid = false;
    step(&model, &in, &out, 260);
    assert(model.state == VC_RECOVER_LINE && out.maneuver.action == S5_WAIT);
    in.avoidance.range_valid = true;
    step(&model, &in, &out, 20);
    assert(model.state == VC_RECOVER_LINE); /* crossing evidence cannot authorize handoff */
    in.avoidance.line_centered = true;
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    assert(model.state == VC_FOLLOW_LINE && model.avoidance.phase == S5_COMPLETE);
    step(&model, &in, &out, 20);
    assert(!out.stop && out.left_rpm == 30 && out.right_rpm == 30);
}

static void test_abort_paths(void)
{
    enum { OPERATOR_STOP, LINK_LOST, STALE_LINE, NO_ECHO, MOTOR_REJECTED, NO_LINE };
    for (unsigned cause = OPERATOR_STOP; cause <= NO_LINE; ++cause) {
        VC_MODEL model = {0};
        VC_INPUT in = track_input();
        VC_OUTPUT out;
        reach_inward_turn(&model, &in, &out, false, false);
        start_move(&model, &in, &out, S5_ADVANCE, 305);
        switch (cause) {
        case OPERATOR_STOP: in.stop = true; break;
        case LINK_LOST: in.link_ok = false; break;
        case STALE_LINE: in.line_age_ms = VC_LINE_MAX_AGE_MS + 1; break;
        case NO_ECHO: in.avoidance.range_valid = false; break;
        case MOTOR_REJECTED: in.avoidance.motion = S5_MOTION_FAILED; break;
        case NO_LINE: in.avoidance.motion = S5_MOTION_DONE; break;
        }
        step(&model, &in, &out, 20);
        assert(out.stop && !vc_core_running(&model) && model.avoidance.phase == S5_FAILED);
        in = track_input();
        in.start = true;
        step(&model, &in, &out, 20);
        assert(out.stop && !vc_core_running(&model)); /* repositioning needs actual IR evidence */
    }
}

static void test_unsafe_profile_and_scan_timeout(void)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    S5_PROFILE scan = obstacle(false);
    scan.left_path_mm = 350;
    scan.right_path_mm = 190;
    profile(&model, &in, &out, scan);
    assert(out.stop && model.avoidance.phase == S5_FAILED && model.avoidance.failure == S5_FAIL_SIDE);

    model = (VC_MODEL){0};
    in = track_input();
    in.start = true;
    step(&model, &in, &out, 20);
    in.start = false;
    in.obstacle_blocked = in.obstacle_detected = true;
    step(&model, &in, &out, 20);
    step(&model, &in, &out, 14000);
    assert(out.stop && model.avoidance.phase == S5_FAILED && model.avoidance.failure == S5_FAIL_SCAN_TIMEOUT);
}

static void test_reentry_limit(void)
{
    S5_AVOID model = {.phase = S5_FIND_LINE, .side = S5_LEFT,
                     .offset_mm = 185, .passed_mm = S5_MAX_PASS_MM};
    S5_AVOID_INPUT in = {.now = 300, .range_at = 300, .stopped = true,
                         .range_valid = true, .range_mm = 240};
    S5_AVOID_OUTPUT out = s5_avoid_step(&model, &in);
    assert(out.action == S5_STOP && model.phase == S5_FAILED);
}

static void test_width_and_diagonal_clearance(void)
{
    S5_PROFILE scan = obstacle(true);
    unsigned offset, pass;
    S5_FAILURE reason;
    assert(s5_plan_bypass(&scan, &offset, &pass, &reason) == S5_LEFT);
    assert(offset == 185 && pass == 165);
    assert(s5_reentry_detection_mm(offset, 60) == 146);
    scan.front_mm = 195; /* Only the shorter right-hand offset fits. */
    assert(s5_plan_bypass(&scan, &offset, &pass, &reason) == S5_RIGHT);
    scan.front_mm = 175;
    assert(s5_plan_bypass(&scan, &offset, &pass, &reason) == S5_NO_SIDE && reason == S5_FAIL_ROOM);
    scan.front_mm = S5_SCAN_STOP_MM;
    scan.width_mm = S5_UNKNOWN_WIDTH;
    assert(s5_plan_bypass(&scan, &offset, &pass, &reason) == S5_NO_SIDE && reason == S5_FAIL_WIDTH);
    scan = obstacle(true);
    scan.left_path_mm = S5_UNKNOWN_RANGE;
    assert(s5_plan_bypass(&scan, &offset, &pass, &reason) == S5_RIGHT);

    scan.front_mm = 215;
    scan.left_path_mm = 235;
    scan.right_path_mm = 222;
    assert(s5_plan_bypass(&scan, &offset, &pass, &reason) == S5_NO_SIDE && reason == S5_FAIL_SIDE);
}

static void test_stationary_retry(void)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    S5_PROFILE scan = obstacle(true);
    scan.width_mm = S5_UNKNOWN_WIDTH;
    profile(&model, &in, &out, scan);
    assert(out.maneuver.action == S5_RESCAN && model.avoidance.phase == S5_PROFILE_PATH);
    step(&model, &in, &out, 20);
    assert(out.maneuver.action == S5_WAIT);
    in.avoidance.profile = obstacle(true);
    in.avoidance.profile_at = in.avoidance.now + 2000;
    step(&model, &in, &out, 2000);
    assert(model.avoidance.phase == S5_TURN_OUT && model.avoidance.side == S5_LEFT);
    start_move(&model, &in, &out, S5_PIVOT_LEFT, 0);
}

static void test_clipped_scan(bool can_retrace)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.avoidance.line_centered = true;
    in.avoidance.can_retrace = can_retrace;
    S5_PROFILE scan = obstacle(true);
    scan.width_mm = S5_UNKNOWN_WIDTH;
    scan.left_edge_valid = false;
    scan.left_clipped = true;
    scan.left_path_mm = 234;
    scan.right_path_mm = 230;
    profile(&model, &in, &out, scan);
    if (!can_retrace) {
        assert(out.stop && model.avoidance.phase == S5_FAILED);
        assert(model.avoidance.failure == S5_FAIL_LEFT_LIMIT);
        return;
    }
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_BACK_UP);
    start_move(&model, &in, &out, S5_REVERSE, S5_REVERSE_MM);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_PROFILE_PATH);
    in.avoidance.range_mm += S5_REVERSE_MM;
    in.avoidance.profile_at = in.avoidance.now + 2000;
    step(&model, &in, &out, 2000);
    assert(out.stop && model.avoidance.phase == S5_FAILED);
    assert(model.avoidance.failure == S5_FAIL_LEFT_LIMIT);
}

static void test_departure_room_after_retrace(void)
{
    S5_AVOID state = {.phase = S5_PROFILE_PATH, .phase_at = 100,
                      .reverse_available = true, .reverse_long = true};
    S5_AVOID_INPUT in = {.now = 500, .profile_at = 450, .range_at = 500, .range_mm = 170,
                         .profile_valid = true, .stopped = true, .range_valid = true,
                         .line_centered = true, .profile = obstacle(true)};
    /* The newer forward measurement rules out a departure despite the original profile. */
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_BACK_UP);
    in.now = in.range_at = 800;
    S5_AVOID_OUTPUT out = s5_avoid_step(&state, &in);
    assert(out.action == S5_REVERSE && out.distance_mm == S5_LONG_REVERSE_MM);
    in.motion = S5_MOTION_DONE;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_PROFILE_PATH);
    in.now = in.range_at = in.profile_at = 950;
    in.motion = S5_MOTION_NONE;
    in.range_mm = 230;
    in.profile.front_mm = in.profile.nearest_mm = 230;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_TURN_OUT);
}

static void test_reverse_and_rescan(bool path_clears)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.avoidance.line_centered = in.avoidance.can_retrace = true;
    S5_PROFILE blocked = obstacle(true);
    blocked.front_mm = blocked.nearest_mm = 90;
    profile(&model, &in, &out, blocked);
    assert(model.avoidance.phase == S5_BACK_UP);
    start_move(&model, &in, &out, S5_REVERSE, S5_REVERSE_MM);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_PROFILE_PATH);
    step(&model, &in, &out, 20);
    assert(model.avoidance.phase == S5_PROFILE_PATH); /* previous scan can't be reused */

    if (path_clears) {
        in.avoidance.profile = (S5_PROFILE){.nearest_mm = S5_OPEN_RANGE};
        in.avoidance.range_mm = S5_CLEAR_MM + 1;
        in.obstacle_detected = in.obstacle_blocked = false;
    }
    in.avoidance.profile_at = in.avoidance.now + 2000;
    step(&model, &in, &out, 2000);
    if (!path_clears) {
        assert(out.stop && model.avoidance.phase == S5_FAILED);
        return;
    }
    assert(model.avoidance.phase == S5_CHECK_LINE);
    in.avoidance.line_at = in.avoidance.now + 260;
    step(&model, &in, &out, 260);
    assert(model.state == VC_FOLLOW_LINE && model.avoidance.phase == S5_COMPLETE);
}

int main(void)
{
    test_rejoin(true, false, false);
    test_rejoin(false, true, false);
    test_rejoin(true, false, true);
    test_rejoin(false, false, true);
    test_abort_paths();
    test_unsafe_profile_and_scan_timeout();
    test_reentry_limit();
    test_width_and_diagonal_clearance();
    test_stationary_retry();
    test_clipped_scan(false);
    test_clipped_scan(true);
    test_departure_room_after_retrace();
    test_reverse_and_rescan(true);
    test_reverse_and_rescan(false);
    puts("PASS obstacle avoidance, both sides, blocked return retry, recovery, and cancellation");
}
