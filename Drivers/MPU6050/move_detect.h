//
// Created by 沈俊儒 on 2020/7/3.
//

#ifndef MOVE_DETECT_H
#define MOVE_DETECT_H

#define RING_BUFFER_SIZE 32
#define WINDOW_SIZE 10
#define STABLE_GUARANTEE_CNT 5
#define STABILITY_THRESHOLD 0.06

#include <i2c.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"
#include "sd_hal_mpu6050.h"

extern volatile uint8_t Move_Is_Stable;

struct Vector_3D {
    double x, y, z;
};

extern struct Vector_3D Position;

struct Vector_3D Move_Next(void);
void Move_Update_Stability(void);

#endif //MOVE_DETECT_H
