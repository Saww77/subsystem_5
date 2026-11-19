#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

#define S5_PRI_MAIN 20

#define S5_SERVO_PIN 12u

void subsystem5_request_scan(void);

#if APP_S5_BENCH
EXPORT void subsystem5_bench(void);
#endif
#if APP_S5_SENSOR_TEST
EXPORT void subsystem5_sensor_test(void);
#endif

#endif
