#include "s1_comm.h"
#include "../vehicle_controller/vehicle_controller.h"
#include "../subsystem2/subsystem2.h"
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_bridge.h"

static ER publish(unsigned route, const unsigned char *data, unsigned length)
{
#if TM_MQTT
    return (ER)mqtt_bridge_send(route, data, length);
#else
    (void)route; (void)data; (void)length;
    return E_NOSPT;
#endif
}

ER s1_publish_vehicle(unsigned state, unsigned fault_mask)
{
    unsigned char data;
    if (state > 7u || fault_mask > 31u) { return E_PAR; }
    data = (unsigned char)(state | (fault_mask << 3u));
    return publish(MQTT_ROUTE_VEHICLE, &data, 1u);
}

ER s1_publish_line(unsigned ir_bits, unsigned barcode)
{
    unsigned char data[1];
    if (!s1_wire_line(data, ir_bits, barcode)) { return E_PAR; }
    return publish(MQTT_ROUTE_LINE, data, sizeof(data));
}

ER s1_publish_terrain(unsigned height_mm, unsigned phase)
{
    unsigned char data[2];
    if (!s1_wire_terrain(data, height_mm, phase)) { return E_PAR; }
    return publish(MQTT_ROUTE_TERRAIN, data, sizeof(data));
}

ER s1_publish_obstacle(unsigned nearest_mm, unsigned width_mm,
                       unsigned left_mm, unsigned right_mm, unsigned angle_deg, BOOL profile)
{
    unsigned char data[7];
    if (!s1_wire_obstacle(data, nearest_mm, width_mm, left_mm, right_mm, angle_deg)) { return E_PAR; }
    s1_wire_put(data, 54u, 1u, profile != FALSE);
    return publish(MQTT_ROUTE_OBSTACLE, data, sizeof(data));
}

unsigned s1_connection_status(void)
{
#if TM_MQTT
    return mqtt_bridge_status();
#else
    return 0;
#endif
}

ER s1_publish_reset_result(BOOL ok)
{
    unsigned char ack = S1_ACK_RESET | S1_ACK_COMPLETE | (ok ? 0u : S1_ACK_REJECTED);
    return publish(MQTT_ROUTE_ACK, &ack, 1u);
}

void s1_comm_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;
#if TM_MQTT
    enum { COMMAND_BURST = 4, TELEMETRY_MS = 1000 };
    unsigned char frame[MQTT_BRIDGE_FRAME_MAX];
    uint32_t telemetry_at = 0;
    for (;;) {
        SYSTIM time;
        tk_get_tim(&time);
        uint32_t elapsed = time.lo - telemetry_at;
        if (elapsed >= TELEMETRY_MS) {
            telemetry_at = time.lo;
            elapsed = 0;
            s1_publish_telemetry();
        }

        unsigned char acks[COMMAND_BURST];
        unsigned count = 0, budget;
        bool stop_seen = false;
        for (budget = 0; budget < COMMAND_BURST; ++budget) {
            int wait = budget ? TMO_POL : TELEMETRY_MS - (int)elapsed;
            int length = mqtt_bridge_receive(frame, wait);
            if (length == E_OBJ) return;
            if (length < 0) break;
            if (length < 2 || frame[0] != MQTT_ROUTE_COMMAND) continue;
            unsigned op;
            unsigned char ack;
            if (!s1_command_decode(frame + 1, (unsigned)length - 1u, &op)) {
                ack = (unsigned char)((frame[1] & 1u) | S1_ACK_REJECTED);
            } else {
                stop_seen |= op == S1_CMD_STOP;
                ack = op == S1_CMD_RESET ? S1_ACK_RESET : (unsigned char)op;
            }
            acks[count++] = ack;
        }
        /* A STOP anywhere in the batch cancels movement before ACKs are sent. */
        if (stop_seen) vehicle_request_stop();
        for (unsigned i = 0; i < count; ++i) {
            unsigned char ack = acks[i];
            if (ack == S1_CMD_START && (stop_seen || !vehicle_request_start())) ack |= S1_ACK_REJECTED;
            if (ack == S1_ACK_RESET && (stop_seen || !vehicle_request_reset())) ack |= S1_ACK_REJECTED;
            (void)publish(MQTT_ROUTE_ACK, &ack, 1u);
        }
        /* Yield to sensing and controller tasks during a sustained burst. */
        if (budget == COMMAND_BURST) (void)tk_dly_tsk(1);
    }
#endif
}
