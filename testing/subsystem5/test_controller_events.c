/* Host exercise of the production dispatcher with a simulated motion owner. */
#include <assert.h>
#include <stdio.h>
#include "../../app_program/vehicle_controller/vehicle_controller.c"

static uint32_t clock_ms;
static unsigned event_flags, stop_count, command_count, scan_count, clear_count, reset_count;
static bool reset_result;
static unsigned connection = 7;
static bool motor_busy;
static S2_STATUS motor;
static S2_REPLY motor_reply;
static uint16_t queued_command;

ID tk_cre_mtx(const T_CMTX *config) { assert(config->mtxatr == TA_INHERIT); return 1; }
ER tk_del_mtx(ID id) { assert(id == 1); return E_OK; }
ER tk_loc_mtx(ID id, TMO time) { assert(id == 1 && time == TMO_FEVR); return E_OK; }
ER tk_unl_mtx(ID id) { assert(id == 1); return E_OK; }
ID tk_cre_flg(const T_CFLG *config) { assert(!config->iflgptn); return 2; }
ER tk_set_flg(ID id, UINT bits) { assert(id == 2); event_flags |= bits; return E_OK; }
ER tk_get_tim(SYSTIM *time) { time->lo = clock_ms; return E_OK; }
INT tm_printf(const UB *format, ...) { (void)format; return 0; }
ID app_start_task(FP entry, PRI priority, UW cpu, const char *name)
{
    (void)entry; (void)priority; (void)cpu; (void)name;
    assert(0); /* The test drives the worker directly. */
    return E_OBJ;
}
ER tk_wai_flg(ID id, UINT wanted, UINT mode, UINT *seen, TMO timeout)
{
    (void)id; (void)wanted; (void)mode; (void)seen; (void)timeout;
    assert(0);
    return E_OBJ;
}

unsigned s1_connection_status(void) { return connection; }
ER s1_publish_reset_result(BOOL ok) { ++reset_count; reset_result = ok; return E_OK; }
uint32_t s2_core_timing_fault_gap_us(void) { return 0; }
bool s2_core_distance_reached(unsigned mm, const S2_STATUS *status)
{ (void)mm; (void)status; return false; }
bool s2_core_maneuver_finished(unsigned arg, const S2_STATUS *status)
{ (void)arg; (void)status; return false; }
bool motion_get_feedback(S2_STATUS *status, S2_REPLY *reply, bool *busy)
{
    *status = motor;
    if (reply) *reply = motor_reply;
    *busy = motor_busy;
    return true;
}

void motion_stop(void)
{
    ++stop_count;
    queued_command = 0;
    motor_busy = true;
}

bool motion_set_wheel_rpm_tracked(unsigned left, unsigned right, uint16_t *command)
{
    assert(!queued_command && left && right);
    ++command_count;
    queued_command = (uint16_t)(1 + left + right * 128);
    *command = queued_command;
    motor_busy = true;
    return true;
}

bool motion_clear_faults(void) { ++clear_count; return true; }
bool motion_maneuver(unsigned mm, bool right, uint16_t *command)
{ (void)mm; (void)right; (void)command; assert(0); return false; }
bool motion_maneuver_turn_45(bool right, uint16_t *command)
{ (void)right; (void)command; assert(0); return false; }
bool motion_maneuver_reverse(unsigned mm, uint16_t *command)
{ (void)mm; (void)command; assert(0); return false; }
void subsystem5_request_scan(void) { ++scan_count; }

static void motor_acknowledge(unsigned reason)
{
    assert(queued_command);
    motor_reply.command = queued_command;
    queued_command = 0;
    ++motor_reply.sequence;
    motor_reply.reason = reason;
    motor_reply.accepted = reason == S2_REPLY_OK;
    motor_busy = motor_reply.accepted;
    s2_put_bits(&motor, 0, 2, motor_reply.accepted);
    vehicle_notify_feedback();
}

static void motor_stopped(void)
{
    motor_busy = false;
    s2_put_bits(&motor, 0, 2, 0);
    vehicle_notify_feedback();
}

static TMO work(void)
{
    assert(event_flags == VC_WAKE);
    event_flags = 0;
    return vehicle_service_once();
}

int main(void)
{
    uint32_t request;
    VEHICLE_SNAPSHOT snapshot;
    assert(vehicle_init() == E_OK);
    vehicle_enable_obstacle_monitor();
    vehicle_submit_line(0, 0, 0, 30, 30, FALSE, 0);
    assert(work() == TMO_FEVR && !vehicle_request_start());
    assert(vehicle_get_snapshot(&snapshot) && snapshot.obstacle_no_echo);

    vehicle_submit_obstacle_range(S5_OPEN_RANGE);
    assert(!vehicle_request_start());
    vehicle_submit_obstacle_range(S5_OPEN_RANGE);
    assert(work() == TMO_FEVR && !command_count);

    assert(vehicle_request_start());
    vehicle_request_stop();
    assert(work() == TMO_FEVR && stop_count == 1 && !command_count);
    assert(!vehicle_request_start());
    motor_stopped();
    work();
    assert(vehicle_request_start());
    vehicle_submit_obstacle_range(S5_UNKNOWN_RANGE);
    assert(work() == TMO_FEVR && !command_count);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.last_start_reject == VC_START_OBSTACLE);

    vehicle_submit_obstacle_range(600);
    assert(vehicle_request_start());
    assert(work() > 0 && vehicle_state() == VC_FOLLOW_LINE && command_count == 1);
    vehicle_submit_line(0, 0, 0, 24, 24, FALSE, 0);
    work();
    assert(command_count == 1);
    motor_acknowledge(S2_REPLY_SETTLING);
    work();
    assert(command_count == 1); /* Wait before retrying a settling rejection. */
    clock_ms += VC_REFRESH_MS;
    vehicle_submit_line(0, 0, 0, 30, 30, FALSE, 0);
    work();
    assert(command_count == 2);
    motor_acknowledge(S2_REPLY_OK);
    work();

    vehicle_submit_obstacle_range(S5_SCAN_STOP_MM);
    work();
    assert(vehicle_state() == VC_AVOIDING && stop_count == 2 && !scan_count);
    assert(!vehicle_obstacle_scan_request(&request));
    motor_stopped();
    work();
    assert(scan_count == 1 && vehicle_obstacle_scan_request(&request));
    assert(vehicle_obstacle_scan_current(request));

    connection = 0;
    vehicle_notify_feedback();
    assert(work() == TMO_FEVR && vehicle_state() == VC_FAULT);
    assert(!vehicle_obstacle_scan_current(request));
    S5_PROFILE stale = {.nearest_mm = S5_SCAN_STOP_MM};
    vehicle_finish_obstacle_scan(&stale, request);
    assert(!event_flags && command_count == 2);

    /* A cancelled detour can be reset even with an empty motor fault mask. */
    connection = 7;
    vehicle_submit_obstacle_range(600);
    assert(vehicle_request_reset());
    assert(work() > 0 && !reset_count && !vehicle_request_start());
    motor_stopped();
    assert(work() == TMO_FEVR && reset_count == 1 && reset_result && !clear_count);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.state == VC_IDLE &&
           snapshot.avoidance_phase == S5_OFF && !snapshot.reset_pending);
    assert(command_count == 2);

    s2_put_bits(&motor, 5, 4, 4);
    vehicle_notify_feedback();
    assert(work() == TMO_FEVR && vehicle_state() == VC_FAULT);
    assert(vehicle_request_reset());
    assert(work() > 0 && !clear_count);
    motor_stopped();
    assert(work() > 0 && clear_count == 1 && reset_count == 1);
    s2_put_bits(&motor, 5, 4, 0);
    vehicle_notify_feedback();
    assert(work() == TMO_FEVR && reset_count == 2 && reset_result);
    assert(vehicle_state() == VC_IDLE && command_count == 2);

    assert(vehicle_request_reset());
    work();
    clock_ms += VC_RESET_TIMEOUT_MS;
    vehicle_notify_feedback();
    assert(work() == TMO_FEVR && reset_count == 3 && !reset_result);
    motor_stopped();
    work();
    vehicle_submit_line(108, 0, 26, 30, 30, FALSE, 0);
    vehicle_submit_obstacle_range(606);
    assert(vehicle_request_start());
    work();
    motor_acknowledge(S2_REPLY_OK);
    work();
    vehicle_submit_obstacle_range(208);
    work();
    motor_stopped();
    work();
    assert(vehicle_obstacle_scan_request(&request));

    S5_PROFILE unresolved = {.nearest_mm = 208, .front_mm = 203, .width_mm = S5_UNKNOWN_WIDTH,
                             .angle_offset_deg = 60, .left_path_mm = S5_OPEN_RANGE,
                             .right_path_mm = S5_UNKNOWN_RANGE};
    uint32_t first_request = request;
    clock_ms += 20;
    vehicle_finish_obstacle_scan(&unresolved, request);
    assert(work() > 0 && vehicle_obstacle_scan_request(&request) && request != first_request);
    clock_ms += 20;
    vehicle_finish_obstacle_scan(&unresolved, request);
    assert(work() == TMO_FEVR && vehicle_state() == VC_FAULT);
    vehicle_submit_obstacle_range(606);
    vehicle_request_stop();
    work();
    assert(vehicle_get_snapshot(&snapshot) && snapshot.state == VC_IDLE && snapshot.obstacle_profile);
    assert(snapshot.near_mm == 208 && snapshot.obstacle_forward_mm == 606 &&
           snapshot.width_mm == S5_UNKNOWN_WIDTH && snapshot.avoidance_failure == S5_FAIL_WIDTH &&
           snapshot.avoidance_failed_phase == S5_PROFILE_PATH);
    motor_stopped();
    work();
    assert(vehicle_request_reset());
    work();
    motor_stopped();
    work();
    assert(vehicle_get_snapshot(&snapshot) && !snapshot.obstacle_profile &&
           snapshot.avoidance_failure == S5_FAIL_NONE && reset_count == 4 && reset_result);
    puts("PASS controller events, stationary rescan, preserved failure and profile, and stopped reset");
}
