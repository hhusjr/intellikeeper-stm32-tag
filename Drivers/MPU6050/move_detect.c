//
// Created by 沈俊儒 on 2020/7/3.
//

#include "move_detect.h"
#include "math.h"
#include "../Zigbee/zigbee.h"

SD_MPU6050 mpu1;
static volatile struct Vector_3D Acc_RingBuffer[RING_BUFFER_SIZE];
static volatile uint16_t RingBuffer_Head, RingBuffer_Tail, RingBuffer_Length;
static volatile struct Vector_3D Acc_Sum;

volatile uint8_t Move_Is_Stable = 1;

static void RingBuffer_Push(struct Vector_3D data) {
    Acc_RingBuffer[RingBuffer_Tail] = data;
    RingBuffer_Tail = (RingBuffer_Tail + 1) % RING_BUFFER_SIZE;
    RingBuffer_Length++;
}

static uint8_t RingBuffer_Empty(void) {
    return RingBuffer_Tail % RING_BUFFER_SIZE == RingBuffer_Head % RING_BUFFER_SIZE;
}

static struct Vector_3D RingBuffer_Pop(void) {
    struct Vector_3D result = Acc_RingBuffer[RingBuffer_Head];
    RingBuffer_Head = (RingBuffer_Head + 1) % RING_BUFFER_SIZE;
    RingBuffer_Length--;
    return result;
}

struct Vector_3D Move_Next(void) {
    SD_MPU6050_Init(&hi2c1, &mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_250s);
    SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu1);

    struct Vector_3D result;
    result.x = mpu1.Accelerometer_X * 1.0 * mpu1.Acce_Mult;
    result.y = mpu1.Accelerometer_Y * 1.0 * mpu1.Acce_Mult;
    result.z = mpu1.Accelerometer_Z * 1.0 * mpu1.Acce_Mult;

    if (RingBuffer_Length < WINDOW_SIZE) {
        for (uint16_t i = 0; i < WINDOW_SIZE; i++) {
            RingBuffer_Push(result);
            Acc_Sum.x += result.x;
            Acc_Sum.y += result.y;
            Acc_Sum.z += result.z;
        }
    } else {
        struct Vector_3D head = RingBuffer_Pop();
        Acc_Sum.x += result.x - head.x;
        Acc_Sum.y += result.y - head.y;
        Acc_Sum.z += result.z - head.z;
        RingBuffer_Push(result);
    }

    struct Vector_3D ret;
    ret.x = Acc_Sum.x / WINDOW_SIZE;
    ret.y = Acc_Sum.y / WINDOW_SIZE;
    ret.z = Acc_Sum.z / WINDOW_SIZE;

    return ret;
}

#define CLOSE(a, b) (fabs(a - b) <= STABILITY_THRESHOLD)
void Move_Update_Stability(void) {
    struct Vector_3D previous = Move_Next(), p;
    for (uint8_t i = 0; i < STABLE_GUARANTEE_CNT; i++) {
        p = Move_Next();
        if (!CLOSE(previous.x, p.x) || !CLOSE(previous.y, p.y) || !CLOSE(previous.z, p.z)) {
            Move_Is_Stable = 0;
            return;
        }
        previous = p;
    }
    Move_Is_Stable = 1;
}
