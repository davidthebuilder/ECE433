#ifndef PROX_SENSOR_H_
#define PROX_SENSOR_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>



bool Prox_sensor_X_Axis_Armature_status_check(void);
bool Prox_sensor_Y_Axis_Armature_status_check(void);
bool Prox_sensor_elevator_status_check(void);

#endif // PROX_SENSOR_H_
