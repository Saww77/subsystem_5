/* Host regression for MQTT bring-up and reconnect with a simulated lwIP port. */
#include <assert.h>
#include <stdio.h>
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c"

static u32_t now_ms;
static unsigned connects, disconnects, heartbeats, status_byte;

struct mqtt_client_s { int unused; };
static mqtt_client_t broker_client;

u32_t sys_now(void) { return now_ms; }
INT tm_printf(const UB *format, ...) { (void)format; return 0; }
mqtt_client_t *mqtt_client_new(void) { return &broker_client; }
int ip4addr_aton(const char *address, ip4_addr_t *out)
{
    assert(address && out);
    out->addr = 0;
    return 1;
}
err_t mqtt_client_connect(mqtt_client_t *c, const ip_addr_t *address, u16_t port,
                          mqtt_connection_cb_t callback, void *arg,
                          const struct mqtt_connect_client_info_t *info)
{
    (void)arg;
    assert(c == &broker_client && address && port && callback && info->client_id);
    ++connects;
    return ERR_OK;
}
void mqtt_disconnect(mqtt_client_t *c) { assert(c == &broker_client); ++disconnects; }
void mqtt_set_inpub_callback(mqtt_client_t *c, mqtt_incoming_publish_cb_t pub,
                            mqtt_incoming_data_cb_t data, void *arg)
{
    (void)arg;
    assert(c == &broker_client && pub && data);
}
err_t mqtt_sub_unsub(mqtt_client_t *c, const char *topic, u8_t qos,
                    mqtt_request_cb_t callback, void *arg, u8_t subscribe)
{
    (void)arg;
    assert(c == &broker_client && topic && !qos && callback && subscribe);
    return ERR_OK;
}
err_t mqtt_publish(mqtt_client_t *c, const char *topic, const void *payload,
                   u16_t length, u8_t qos, u8_t retain, mqtt_request_cb_t callback, void *arg)
{
    (void)arg;
    assert(c == &broker_client && !strcmp(topic, topics[MQTT_ROUTE_STATUS]));
    assert(length == 1 && !qos && retain && callback);
    status_byte = *(const unsigned char *)payload;
    ++heartbeats;
    return ERR_OK;
}
int mqtt_bridge_ready(void) { return 1; }
int mqtt_bridge_service_take(unsigned char frame[MQTT_BRIDGE_FRAME_MAX]) { (void)frame; return -1; }
int mqtt_bridge_service_deliver(unsigned route, const unsigned char *data, unsigned length)
{ (void)route; (void)data; (void)length; return 0; }
void mqtt_bridge_service_status(unsigned bits) { (void)bits; }

int main(void)
{
    mqtt_utk_poll(0);
    now_ms = 50;
    mqtt_utk_poll(0);
    assert(connects == 0);
    ++now_ms;
    mqtt_utk_poll(1);
    assert(connects == 1); /* Attempt MQTT on the first poll with an IP. */
    connection_cb(&broker_client, NULL, MQTT_CONNECT_ACCEPTED);
    mqtt_utk_poll(1);
    assert(heartbeats == 1 && status_byte == (MQTT_LINK_WIFI | MQTT_LINK_BROKER));
    now_ms += 20;
    subscribe_done_cb(NULL, ERR_OK);
    mqtt_utk_poll(1);
    assert(heartbeats == 2 && status_byte == 7); /* No periodic heartbeat delay. */

    mqtt_utk_poll(0);
    assert(disconnects == 1);
    ++now_ms;
    mqtt_utk_poll(1);
    assert(connects == 2);
    now_ms += HANDSHAKE_MS;
    mqtt_utk_poll(1);
    assert(disconnects == 2 && connects == 2);
    now_ms += RETRY_MS;
    mqtt_utk_poll(1);
    assert(connects == 3);
    puts("PASS immediate MQTT connect, subscription heartbeat, and bounded retry");
}
