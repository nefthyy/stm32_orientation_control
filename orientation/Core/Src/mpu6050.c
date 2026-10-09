#include "main.h"
#include <math.h>

static int primed = 0;

void MPU6050_Init(I2C_HandleTypeDef *hi2c){

  uint8_t check_address = 0;
  uint8_t config_data = 0;

  if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, REG_WHO_AM_I, 1, &check_address, 1, 100) != HAL_OK){
    Error_Handler();
  }

  if (check_address != 0x68){
    Error_Handler();
  }

  config_data = 0x00;   // Clear SLEEP, run off the internal 8 MHz oscillator.
  HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_PWR_MGMT_1, 1, &config_data, 1, 100);

  config_data = 0x03;   // DLPF_CFG=3 -> 44 Hz bandwidth, 1 kHz base rate.
  HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_CONFIG, 1, &config_data, 1, 100);

  config_data = 0x00;   // SMPLRT_DIV=9 -> 1000/(1+9) = 100 Hz output rate.
  HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_SMPLRT_DIV, 1, &config_data, 1, 100);

  config_data = 0x01;   // DATA_RDY_EN: pulse the INT pin on each new sample.
  HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_INT_ENABLE, 1, &config_data, 1, 100);

}

float Num_To_Accel(const int16_t *x){
  return *x / 16384.0f * 9.81f;
}

float Num_To_Rate(const int16_t *x){
  return *x / 131.0f;
}

float Num_To_Temp(const int16_t *x){
  float y = (*x / 340.0f) + 36.53f;
  return (y * 9.0f / 5.0f) + 32.0f;
}

void convert_accel_and_temp(const int16_t *accel_x, const int16_t *accel_y, const int16_t *accel_z,
                            const int16_t *Temp, float *a_x, float *a_y, float *a_z, float *tt)
{

  if (accel_x != NULL && a_x != NULL){
    *a_x = Num_To_Accel(accel_x);
  }
  if (accel_y != NULL && a_y != NULL){
    *a_y = Num_To_Accel(accel_y);
  }
  if (accel_z != NULL && a_z != NULL){
    *a_z = Num_To_Accel(accel_z);
  }

  if (Temp != NULL && tt != NULL){
    *tt = Num_To_Temp(Temp);
  }

}

void convert_gyro(const int16_t *gyro_x, const int16_t *gyro_y, const int16_t *gyro_z, float *g_x, float *g_y, float *g_z)
{

  if (gyro_x != NULL && g_x != NULL){
    *g_x = Num_To_Rate(gyro_x);
  }
  if (gyro_y != NULL && g_y != NULL){
    *g_y = Num_To_Rate(gyro_y);
  }
  if (gyro_z != NULL && g_z != NULL){
    *g_z = Num_To_Rate(gyro_z);
  }

}


void set_bits(const uint8_t *buffer, int16_t *Accel_X, int16_t *Accel_Y, int16_t *Accel_Z, int16_t *Temp, int16_t *Gyro_X, int16_t *Gyro_Y, int16_t *Gyro_Z){
  *Accel_X = (int16_t)((buffer[0]  << 8) | buffer[1]);
  *Accel_Y = (int16_t)((buffer[2]  << 8) | buffer[3]);
  *Accel_Z = (int16_t)((buffer[4]  << 8) | buffer[5]);

  *Temp    = (int16_t)((buffer[6]  << 8) | buffer[7]);

  *Gyro_X  = (int16_t)((buffer[8]  << 8) | buffer[9]);
  *Gyro_Y  = (int16_t)((buffer[10] << 8) | buffer[11]);
  *Gyro_Z  = (int16_t)((buffer[12] << 8) | buffer[13]);
}

// sort arrays we want median values from and just append for other ones
void append_values(float A_X_Arr[], float A_Y_Arr[], float A_Z_Arr[], float Temp_Arr[], float G_X_Arr[], float G_Y_Arr[], float G_Z_Arr[],
              const float *a_x, const float *a_y, const float *a_z, const float *tt, const float *g_x, const float *g_y, const float *g_z, int index)
{
  A_X_Arr[index] = *a_x;
  A_Y_Arr[index] = *a_y;
  A_Z_Arr[index] = *a_z;
  G_X_Arr[index] = *g_x;
  G_Y_Arr[index] = *g_y;
  G_Z_Arr[index] = *g_z;
  Temp_Arr[index] = *tt;
}


void simpleSort(float arr[], int size) {
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                float temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}


// take median values of the acc and temp
float median_value(const float Arr[]){
  float t[WINDOW_SIZE];
  for (int i = 0; i < WINDOW_SIZE; i++){
    t[i] = Arr[i];
  }
  simpleSort(t, WINDOW_SIZE);
  int mid = (int) WINDOW_SIZE / 2;
  if(WINDOW_SIZE == 0){
    return 0;
  } else if (WINDOW_SIZE % 2 == 0) {
    return (t[mid] + t[mid-1]) / 2;
  } else {
    return (t[mid]);
  }
}

// take mean values of gyroscope
float mean_value(float Arr[]){
  if(WINDOW_SIZE == 0){
    return 0.0;
  } else {
    float sum = 0.0;
    for (int i = 0; i < WINDOW_SIZE; i++){
      sum += Arr[i];
    }
    return sum / WINDOW_SIZE;
  }

}

void update_state(float A_X_Arr[], float A_Y_Arr[], float A_Z_Arr[], float Temp_Arr[], float G_X_Arr[], float G_Y_Arr[], float G_Z_Arr[], struct State *state){
  state->a_x = median_value(A_X_Arr);
  state->a_y = median_value(A_Y_Arr);
  state->a_z = median_value(A_Z_Arr);
  state->tt = median_value(Temp_Arr);
  state->g_x = mean_value(G_X_Arr);
  state->g_y = mean_value(G_Y_Arr);
  state->g_z = mean_value(G_Z_Arr);
}



void update_roll_pitch_yaw(struct State *state, float dt){
  float accel_roll = atan2f(state->a_y, state->a_z) * RAD_TO_DEG;
  float accel_pitch = atan2f(-state->a_x, sqrtf(state->a_y * state->a_y + state->a_z * state->a_z)) * RAD_TO_DEG;

  if(primed == 0){
    primed = 1;
    state->roll = accel_roll;
    state->pitch = accel_pitch;
    state->yaw = 0.0f;
  } else {
    state->roll = ALPHA * (state->roll + state->g_x * dt) + (1 - ALPHA)*accel_roll;
    state->pitch = ALPHA * (state->pitch + state->g_y * dt) + (1 - ALPHA)*accel_pitch;
    state->yaw = state->yaw + state->g_z * dt;
  }
}
