#include "s1_wire.h"

void s1_wire_put(unsigned char *bytes, unsigned bit, unsigned count, unsigned value)
{
    unsigned i;
    for (i = 0; i < count; ++i) {
        unsigned k = bit + i;
        bytes[k / 8u] = (unsigned char)((bytes[k / 8u] & ~(1u << (k % 8u))) |
                           (((value >> i) & 1u) << (k % 8u)));
    }
}

unsigned s1_wire_get(const unsigned char *bytes, unsigned bit, unsigned count)
{
    unsigned i;
    unsigned value = 0;
    for (i = 0; i < count; ++i) {
        unsigned k = bit + i;
        value |= ((unsigned)(bytes[k / 8u] >> (k % 8u)) & 1u) << i;
    }
    return value;
}

int s1_command_decode(const unsigned char *data, unsigned length, unsigned *opcode)
{
    if (data == 0 || opcode == 0 || length != 1u || data[0] > S1_CMD_RESET) { return 0; }
    *opcode = data[0];
    return 1;
}

int s1_wire_line(unsigned char out[1], unsigned ir, unsigned barcode)
{
    if (out == 0 || ir > 7u || barcode > 4u) { return 0; }
    out[0] = (unsigned char)(ir | (barcode << 3u));
    return 1;
}

int s1_wire_terrain(unsigned char out[2], unsigned height_mm, unsigned phase)
{
    if (out == 0 || height_mm > 2000u || phase > 5u) { return 0; }
    out[0] = 0; out[1] = 0;
    s1_wire_put(out, 0u, 11u, height_mm);
    s1_wire_put(out, 11u, 3u, phase);
    return 1;
}

int s1_wire_obstacle(unsigned char out[7], unsigned nearest_mm, unsigned width_mm,
                     unsigned left_mm, unsigned right_mm, unsigned angle_deg)
{
    if (!out || nearest_mm > 4000u || (width_mm > 2000u && width_mm != 2047u) ||
        (left_mm > 4000u && left_mm < 4094u) || left_mm > 4095u ||
        (right_mm > 4000u && right_mm < 4094u) || right_mm > 4095u ||
        angle_deg < 30u || angle_deg > 150u) return 0;
    for (unsigned i = 0; i < 7u; ++i) out[i] = 0;
    s1_wire_put(out, 0u, 12u, nearest_mm);
    s1_wire_put(out, 12u, 11u, width_mm);
    s1_wire_put(out, 23u, 12u, left_mm);
    s1_wire_put(out, 35u, 12u, right_mm);
    s1_wire_put(out, 47u, 7u, angle_deg - 30u);
    return 1;
}
