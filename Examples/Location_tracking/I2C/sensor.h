
#include "stm32f10x.h"

void initSensor();
int updateSensor();

int16_t   OUT_X_ACCEL, OUT_Y_ACCEL, OUT_Z_ACCEL;
int16_t   OUT_X_MAG,OUT_Y_MAG,OUT_Z_MAG;
int16_t   OUT_X_GYRO,OUT_Y_GYRO,OUT_Z_GYRO;
