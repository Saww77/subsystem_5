#include "s1_comm.h"
#include "../subsystem2/subsystem2.h"
#include "../vehicle_controller/vehicle_controller.h"
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_bridge.h"
ER s1_publish_motion(void)
{
#if TM_MQTT
    S2_STATUS s;
    if(!motion_get_status(&s))return E_OBJ;
    return (ER)mqtt_bridge_send(MQTT_ROUTE_MOTION,s.bytes,sizeof(s.bytes));
#else
    return E_NOSPT;
#endif
}
#if TM_MQTT
static void account(ER result)
{ if(result!=E_OK)vehicle_record_telemetry_drop(); }
#endif
#if TM_MQTT
static void publish_controller(const VEHICLE_SNAPSHOT *s)
{
    unsigned char bytes[31]={0};
    uint32_t age=s->timestamp_ms-s->line_timestamp_ms;
    if(!s->line_valid || age>65535u)age=65535u;
    bytes[0]=2;bytes[1]=s->state;bytes[2]=s->reason;bytes[3]=s->start_block;
    bytes[4]=(s->motor_ready?1u:0u)|(s->line_valid?2u:0u)|(s->line_stop?4u:0u)|
             (s->obstacle_blocked?8u:0u)|(s->start_pending?16u:0u)|
             (s->stop_pending?32u:0u)|(s->link_ready?64u:0u)|(s->reset_pending?128u:0u);
    bytes[5]=s->motor_faults;
    unsigned values[3]={(unsigned)s->l,(unsigned)s->c,(unsigned)s->r};
    for(unsigned i=0;i<3;i++){bytes[6+2*i]=values[i]&255;bytes[7+2*i]=values[i]>>8;}
    bytes[12]=s->left_rpm;bytes[13]=s->right_rpm;
    bytes[14]=age&255;bytes[15]=age>>8;
    for(unsigned i=0;i<4;i++){
        bytes[16+i]=(s->timestamp_ms>>(8*i))&255;
        bytes[20+i]=(s->telemetry_drops>>(8*i))&255;
    }
    bytes[24]=s->last_start_reject;
    for(unsigned i=0;i<4;i++)bytes[25+i]=(s->timing_fault_gap_us>>(8*i))&255;
    bytes[29]=s->control_gap_limit_us&255;bytes[30]=s->control_gap_limit_us>>8;
    account((ER)mqtt_bridge_send(MQTT_ROUTE_CONTROLLER,bytes,sizeof(bytes)));
}

static void publish_obstacle(const VEHICLE_SNAPSHOT *s)
{
    static BOOL logged, previous_width;
    static ER previous_result;
    static uint32_t last_log_at;
    ER result = s1_publish_obstacle(s->near_mm, s->width_mm, s->clear_left_mm,
                                   s->clear_right_mm, s5_servo_angle(s->obstacle_angle_offset), s->obstacle_profile);
    account(result);
    BOOL width = s->width_mm != S5_UNKNOWN_WIDTH;
    if (logged && width == previous_width && result == previous_result &&
        s->timestamp_ms - last_log_at < 3000u) return;
    logged = TRUE;
    previous_width = width;
    previous_result = result;
    last_log_at = s->timestamp_ms;
    const char *outcome = result == E_OK ? "queued" : "queue rejected";
    if (width)
        tm_printf((UB*)"[S1 MQTT] /obstacle %s near=%u mm width=%u mm result=%d\n",
                  outcome, (unsigned)s->near_mm, (unsigned)s->width_mm, result);
    else
        tm_printf((UB*)"[S1 MQTT] /obstacle %s near=%u mm width=unavailable result=%d\n",
                  outcome, (unsigned)s->near_mm, result);
}
#endif
void s1_publish_telemetry(void)
{
#if TM_MQTT
    static unsigned previous_link = ~0u;
    unsigned link = s1_connection_status() & 3u;
    if (link != previous_link) {
        previous_link = link;
        tm_printf((UB*)"[S1 MQTT] telemetry link Wi-Fi=%u broker=%u\n", link & 1u, link >> 1u);
    }
    VEHICLE_SNAPSHOT s;
    if(link!=3u || !vehicle_get_snapshot(&s))return;
    unsigned faults=(s.motor_faults || s.reason==VC_REASON_MOTOR)?S1_FAULT_MOTOR:0;
    if(s.motor_faults&1u)faults|=S1_FAULT_ENCODER;
    if(s.obstacle_no_echo)faults|=S1_FAULT_NO_ECHO;
    if(s.reason==VC_REASON_LINK)faults|=S1_FAULT_WIFI;
    account(s1_publish_vehicle(s.state,faults));
    publish_controller(&s);
    unsigned char avoidance[4] = {
        (unsigned char)(s.avoidance_phase | (s.avoidance_side << 4)),
        (unsigned char)(s.avoidance_failure | (s.avoidance_failed_phase << 4)),
        (unsigned char)s.obstacle_forward_mm,
        (unsigned char)(s.obstacle_forward_mm >> 8),
    };
    account((ER)mqtt_bridge_send(MQTT_ROUTE_AVOIDANCE, avoidance, sizeof(avoidance)));
    /* No fabricated zero measurements during startup or absent subsystems. */
    if(motion_ready())account(s1_publish_motion());
    if(s.line_valid && (uint32_t)(s.timestamp_ms-s.line_timestamp_ms)<=VC_LINE_MAX_AGE_MS){
        unsigned ir=(s.l>=600?1u:0u)|(s.c>=600?2u:0u)|(s.r>=600?4u:0u);
        account(s1_publish_line(ir,s.barcode));
    }
    if(s.terrain_valid && (uint32_t)(s.timestamp_ms-s.terrain_timestamp_ms)<=VC_SENSOR_MAX_AGE_MS)
        account(s1_publish_terrain(s.height_mm,s.terrain_phase));
    if (s.obstacle_valid && (s.obstacle_profile ||
        (uint32_t)(s.timestamp_ms - s.obstacle_timestamp_ms) <= S5_RANGE_MAX_AGE_MS))
        publish_obstacle(&s);
#endif
}
