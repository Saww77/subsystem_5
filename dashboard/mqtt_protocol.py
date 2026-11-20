"""Pico binary-v2 wire format. No dependency on the firmware or MQTT client."""
STATES = ('IDLE', 'FOLLOW_LINE', 'TURNING', 'AVOIDING', 'RECOVER_LINE', 'FAULT')
MOTOR_MODES = ('IDLE', 'VELOCITY', 'DISTANCE', 'TURN')
MOTOR_FAULTS = ((1, 'Encoder counts'), (2, 'Control timing'), (4, 'Wheel stalled'), (8, 'Command timeout'))
VEHICLE_FAULT_NO_ECHO = 1 << 3
START_BLOCK_OBSTACLE = 8
VEHICLE_FAULTS = ((1, 'Motor'), (2, 'Encoder'), (4, 'IMU'),
                  (VEHICLE_FAULT_NO_ECHO, 'Ultrasonic echo'), (16, 'Wi-Fi'))
TELEMETRY_TOPICS = ('vehicle', 'motion', 'line', 'terrain', 'obstacle', 'controller', 'avoidance')
AVOIDANCE_PHASES = (
    'Inactive', 'Checking side clearance', 'Turning outward', 'Leaving the line',
    'Aligning alongside the obstacle', 'Passing obstacle', 'Turning toward the line',
    'Searching for the line', 'Centering on the line', 'Realigning with the track',
    'Continuing past the obstacle', 'Line reacquired', 'Detour stopped',
    'Checking the rejoined path', 'Reversing to rescan',
)
AVOIDANCE_SIDES = (None, 'Left', 'Right')
AVOIDANCE_FAILED = 12
AVOIDANCE_FAILURES = (
    None,
    'Servo sweep timed out',
    'The sweep could not confirm the obstacle',
    'The sweep could not confirm both edges for a width estimate',
    'Too little room to turn onto the departure diagonal',
    'Neither departure path had enough confirmed turn clearance',
    'The ultrasonic sensor lost a usable echo',
    'An obstacle was too close for the next move',
    'The motor controller could not finish the move',
    'The line could not be confirmed',
    'Stopped by operator request',
    'A vehicle safety interlock stopped the detour',
    'The detour phase timed out',
    'A near echo persisted at the left sweep limit',
    'A near echo persisted at the right sweep limit',
    'Near echoes persisted at both sweep limits',
)

def command_payload(command):
    if command == 'start': return b'\x01'
    if command == 'stop': return b'\x00'
    if command == 'reset': return b'\x02'
    raise ValueError('command must be start, stop or reset')

def decode_telemetry(kind, payload):
    sizes = dict(vehicle=(1,), motion=(9,), line=(1,), terrain=(2,),
                 obstacle=(6, 7), controller=(25, 31), avoidance=(1, 4))
    if len(payload) not in sizes.get(kind, ()):
        raise ValueError('Incorrect binary telemetry length: received ' + str(len(payload)) +
                         ' bytes; expected ' + ' or '.join(map(str, sizes.get(kind, ()))) + ' bytes')
    word = int.from_bytes(payload, 'little')
    def bits(offset, width): return (word >> offset) & ((1 << width) - 1)
    if kind == 'controller':
        if (payload[0],len(payload)) not in ((1,25),(2,31)) or payload[1] >= len(STATES) or payload[2] > 6 or payload[3] > 8 or payload[24] > 8 or (payload[0]==1 and payload[4] & 128) or payload[5] > 15:
            raise ValueError('Invalid controller diagnostics')
        raw = [int.from_bytes(payload[i:i+2], 'little') for i in (6,8,10)]
        if max(raw)>1000 or max(payload[12:14])>126 or any(x%2 for x in payload[12:14]):
            raise ValueError('Invalid controller sensor or target range')
        blocks = ('Ready for START', 'Motor startup not ready', 'Wi-Fi/MQTT not ready',
                  'Motor fault latched', 'STOP / reset pending', 'No valid IR readings',
                  'IR readings are stale', 'Junction / line stop active', 'Obstacle interlock active')
        hints = ('Controller start checks pass.', 'Wait at least five seconds after powering the Pico.',
                 'Wait for Wi-Fi, broker and command subscription.',
                 'Inspect motor faults before resetting or restarting.',
                 'Allow the controller to process STOP before retrying.',
                 'Check the IR sensor AO connections and ADC read errors on USB serial.',
                 'The sensor task is not supplying fresh data; inspect USB serial.',
                 'Place the car on a straight section away from crossings and wait for fresh IR readings.',
                 'Move nearby objects away and wait for the ultrasonic sensor to confirm the path is clear.')
        age = int.from_bytes(payload[14:16], 'little')
        return dict(state=STATES[payload[1]], stop_reason=payload[2], start_block=payload[3],
                    start_block_label=blocks[payload[3]], start_hint=hints[payload[3]],
                    motor_ready=bool(payload[4]&1), line_valid=bool(payload[4]&2),
                    line_stop=bool(payload[4]&4), obstacle_blocked=bool(payload[4]&8),
                    start_pending=bool(payload[4]&16), stop_pending=bool(payload[4]&32),
                    link_ready=bool(payload[4]&64), motor_faults=payload[5],
                    left_dark=raw[0], centre_dark=raw[1], right_dark=raw[2],
                    left_target_rpm=payload[12], right_target_rpm=payload[13],
                    line_age_ms=None if age==65535 else age,
                    uptime_ms=int.from_bytes(payload[16:20],'little'),
                    telemetry_drops=int.from_bytes(payload[20:24],'little'),
                    last_start_reject=payload[24], last_reject_label=blocks[payload[24]],
                    last_reject_hint=hints[payload[24]], reset_supported=payload[0]>=2,
                    reset_pending=bool(payload[4]&128),
                    timing_gap_us=int.from_bytes(payload[25:29],'little') if payload[0]>=2 else None,
                    timing_limit_us=int.from_bytes(payload[29:31],'little') if payload[0]>=2 else None)
    if kind == 'avoidance':
        phase, side = bits(0, 4), bits(4, 2)
        if bits(6, 2) or phase >= len(AVOIDANCE_PHASES) or side >= len(AVOIDANCE_SIDES):
            raise ValueError('Invalid avoidance payload')
        result = dict(phase=phase, phase_label=AVOIDANCE_PHASES[phase], side=AVOIDANCE_SIDES[side])
        if len(payload) == 4:
            failure, failed_phase = bits(8, 4), bits(12, 4)
            forward = bits(16, 12)
            if (word >> 28 or failure >= len(AVOIDANCE_FAILURES) or
                    failed_phase >= len(AVOIDANCE_PHASES) or 4000 < forward < 4094 or
                    (failure and phase != AVOIDANCE_FAILED)):
                raise ValueError('Invalid detour diagnostics')
            result.update(failure=AVOIDANCE_FAILURES[failure],
                          failed_phase=AVOIDANCE_PHASES[failed_phase] if failure else None,
                          forward_mm=forward if forward <= 4000 else None,
                          forward_open=forward == 4094)
        return result
    if kind == 'vehicle':
        state, faults = bits(0, 3), bits(3, 5)
        if state >= len(STATES): raise ValueError('Unknown vehicle state')
        return dict(state=STATES[state], state_code=state, faults=faults,
                    fault_labels=[label for bit, label in VEHICLE_FAULTS if faults & bit])
    if kind == 'motion':
        fields = dict(mode=bits(0, 2), speed_level=bits(2, 3), faults=bits(5, 4),
                      left_rpm=bits(9, 9), right_rpm=bits(18, 9),
                      left_pwm=bits(27, 7), right_pwm=bits(34, 7),
                      left_progress=bits(41, 13), right_progress=bits(54, 13))
        if word >> 67 or fields['speed_level'] > 4 or max(fields['left_rpm'], fields['right_rpm']) > 300 or max(fields['left_pwm'], fields['right_pwm']) > 100:
            raise ValueError('Invalid motion range or reserved bits')
        fields['mode_name'] = MOTOR_MODES[fields['mode']]
        fields['fault_labels'] = [label for bit, label in MOTOR_FAULTS if fields['faults'] & bit]
        return fields
    if kind == 'line':
        ir, barcode = bits(0, 3), bits(3, 3)
        if word >> 6 or barcode > 4: raise ValueError('Invalid line payload')
        return dict(ir_bits=ir, left_black=bool(ir & 1), centre_black=bool(ir & 2),
                    right_black=bool(ir & 4), barcode_code=barcode,
                    barcode='None' if barcode == 0 else 'ABCD'[barcode - 1])
    if kind == 'terrain':
        height, phase = bits(0, 11), bits(11, 3)
        if word >> 14 or height > 2000 or phase > 5: raise ValueError('Invalid terrain payload')
        return dict(height_mm=height, phase=phase)
    fields = dict(nearest_mm=bits(0, 12), width_mm=bits(12, 11),
                  left_mm=bits(23, 12), right_mm=bits(35, 12))
    if len(payload) == 7:
        angle = bits(47, 7) + 30
        if word >> 55 or angle > 150:
            raise ValueError('Invalid obstacle angle or reserved bits')
        fields['angle_deg'] = angle
        if bits(54, 1):
            fields['saved_profile'] = True
            for side in ('left', 'right'):
                if fields[side + '_mm'] == 4094:
                    fields[side + '_mm'] = None
                    fields[side + '_open'] = True
        for key, unknown in (('width_mm', 2047), ('left_mm', 4095), ('right_mm', 4095)):
            if fields[key] == unknown:
                fields[key] = None
    elif word >> 47:
        raise ValueError('Invalid obstacle reserved bits')
    for key, maximum in (('nearest_mm', 4000), ('width_mm', 2000),
                         ('left_mm', 4000), ('right_mm', 4000)):
        if fields[key] is not None and fields[key] > maximum:
            raise ValueError('Invalid obstacle range')
    return fields
