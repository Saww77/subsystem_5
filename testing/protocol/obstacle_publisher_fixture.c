/* Exercise the actual publisher and C packer, not a Python packet replica. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../app_program/subsystem1/s1_comm.h"
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_bridge.h"
static unsigned count;
int mqtt_bridge_send(unsigned route, const unsigned char *data, unsigned length)
{
    assert(route == MQTT_ROUTE_OBSTACLE && length == 7);
    assert(s1_wire_get(data, 0, 12) == 220);
    assert(s1_wire_get(data, 12, 11) == 2047);
    assert(s1_wire_get(data, 23, 12) == 4094);
    assert(s1_wire_get(data, 35, 12) == 4095);
    assert(s1_wire_get(data, 47, 7) == 60);
    assert(s1_wire_get(data, 54, 1) == 1);
    assert(fwrite(data, 1, length, stdout) == length);
    ++count;
    return E_OK;
}
int main(void)
{
    assert(s1_publish_obstacle(220, 2047, 4094, 4095, 90, TRUE) == E_OK);
    assert(count == 1);
}
