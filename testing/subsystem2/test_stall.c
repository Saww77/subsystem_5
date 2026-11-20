/* Run the production motor controller with a simulated clock and encoders. */
#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem2/motion_core.h"
#include "../../app_program/subsystem2/motion_config.h"
#include "../../app_program/subsystem2/motion_hw.h"
#include "../../app_program/subsystem2/motion_transport.h"
/* The target masks local interrupts around encoder_word. Host calls are serial. */
#undef DI
#undef EI
#define DI(saved) ((saved) = 0)
#define EI(saved) ((void)(saved))
#include "../../app_program/subsystem2/motion_core.c"

static uint64_t clock_us;
static unsigned output[30];
void s2_hw_pwm_init(void) {}
ER s2_hw_encoder_init(void) { return E_OK; }
uint64_t s2_hw_time_us(void) { return clock_us; }
void s2_hw_pwm_write(unsigned pin, unsigned level)
{
    assert(pin < 30 && level <= 1000);
    output[pin] = level;
}
static void tick(unsigned left, unsigned right)
{
    for (unsigned i = 0; i < left; ++i)
        subsystem2_encoder_gpio_irq(LEFT_ENC_A, S2_GPIO_EDGE_RISE);
    for (unsigned i = 0; i < right; ++i)
        subsystem2_encoder_gpio_irq(RIGHT_ENC_A, S2_GPIO_EDGE_RISE);
    clock_us += 10000;
    s2_core_service();
}
static void stopped(void)
{
    assert(!s2_core_busy());
    assert(!output[LEFT_IN_A] && !output[LEFT_IN_B]);
    assert(!output[RIGHT_IN_A] && !output[RIGHT_IN_B]);
}
int main(void)
{
    clock_us = 10000;
    s2_core_init();
    assert(s2_core_set_wheel_rpm(30, 30));
    assert(!s2_core_clear_faults());
    for (unsigned i = 0; i < 200; ++i) tick(3, 3);
    assert(s2_core_busy() && !s2_core_faults());
    S2_STATUS snapshot;
    s2_core_snapshot(&snapshot);
    assert(s2_get_bits(&snapshot, 9, 9) > 0 && s2_get_bits(&snapshot, 18, 9) > 0);
    /* Bounded-move progress is zero in velocity mode even with good encoders. */
    assert(!s2_get_bits(&snapshot, 41, 13) && !s2_get_bits(&snapshot, 54, 13));
    s2_core_stop();
    stopped();
    for (unsigned side = 0; side < 2; ++side) {
        s2_core_init();
        assert(s2_core_maneuver(S2_MANEUVER_TURN | side));
        for (unsigned i = 0; i < 200 && !s2_core_faults(); ++i)
            tick(side ? 3 : 0, side ? 0 : 3);
        assert(s2_core_faults() == FAULT_STALL);
        stopped();
        for (unsigned i = 0; i < 100; ++i) tick(3, 3);
        assert(s2_core_faults() == FAULT_STALL); /* Never resumes on its own. */
        assert(!s2_core_maneuver(S2_MANEUVER_TURN));
        assert(s2_core_clear_faults());
        stopped();
        assert(!s2_core_faults());
    }
    puts("PASS motor stall guard, each missing encoder, latched stop, explicit clear and velocity progress semantics");
}
