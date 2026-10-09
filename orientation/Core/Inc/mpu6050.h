#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define MPU6050_ADDR   (0x68 << 1)
#define REG_WHO_AM_I         0x75  // Hardware ID register. Always returns the fixed constant 0x68.
#define REG_PWR_MGMT_1       0x6B  // Power Management 1. Controls sleep modes and clock sources.
#define REG_INT_ENABLE       0x38  // Interrupt Enable. Configures what events trigger the physical INT pin.
#define REG_SMPLRT_DIV       0x19  // Sample Rate Divider. Divides the base rate set by CONFIG.
#define REG_CONFIG           0x1A  // Digital Low Pass Filter. Also selects the base sample rate.
#define REG_ACCEL_XOUT_H     0x3B  // Starting memory address of the 14-byte raw sensor data block.

#define SAMPLE_RATE_HZ       1000.0f
#define DT                   (WINDOW_SIZE / SAMPLE_RATE_HZ)  // 0.005 s
#define WINDOW_SIZE          5     // size of mpu6050 read window
#define TAU     0.5f                    // what you tune
#define ALPHA   (TAU / (TAU + DT))      // derived, never touch
#define RAD_TO_DEG           57.2957795f  // rad to deg

struct State{
  float roll;
  float pitch;
  float yaw;

  float a_x, a_y, a_z, g_x, g_y, g_z, tt;
};

void MPU6050_Init(I2C_HandleTypeDef *hi2c);

float Num_To_Accel(const int16_t *x);
float Num_To_Rate(const int16_t *x);
float Num_To_Temp(const int16_t *x);

void convert_accel_and_temp(const int16_t *accel_x, const int16_t *accel_y, const int16_t *accel_z, const int16_t *Temp, float *a_x, float *a_y, float *a_z, float *tt);
void convert_gyro(const int16_t *gyro_x, const int16_t *gyro_y, const int16_t *gyro_z, float *g_x, float *g_y, float *g_z);
void set_bits(const uint8_t *buffer, int16_t *Accel_X, int16_t *Accel_Y, int16_t *Accel_Z, int16_t *Temp, int16_t *Gyro_X, int16_t *Gyro_Y, int16_t *Gyro_Z);
void append_values(float A_X_Arr[], float A_Y_Arr[], float A_Z_Arr[], float Temp_Arr[], float G_X_Arr[], float G_Y_Arr[], float G_Z_Arr[], const float *a_x, const float *a_y, const float *a_z, const float *tt, const float *g_x, const float *g_y, const float *g_z, int index);
void update_state(float A_X_Arr[], float A_Y_Arr[], float A_Z_Arr[], float Temp_Arr[], float G_X_Arr[], float G_Y_Arr[], float G_Z_Arr[], struct State *state);
void simpleSort(float arr[], int size);
float median_value(const float Arr[]);
float mean_value(float Arr[]);
void update_roll_pitch_yaw(struct State *state, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
