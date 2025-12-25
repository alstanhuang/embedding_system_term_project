/*
 * imu.c
 *
 *  Created on: Dec 16, 2025
 *      Author: jocer
 */
/* Includes */
#include "gatt_db.h"
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01_gyro.h"
#include "stm32l4s5i_iot01_magneto.h"
#include <stdio.h>
#include <math.h>

/* Parameters */
#define N 100
static uint16_t idx = 0;
static uint16_t filled = 0;
static uint8_t  offset_done = 0;

/* Input Data */
int16_t acc[3];
float gyro[3];
int16_t mag[3];

/* Output Data */
AxesRaw_t x_axes_out = {0, 0, 0};
AxesRaw_t g_axes_out = {0, 0, 0};
AxesRaw_t m_axes_out = {0, 0, 0};


/* Buffers */
int16_t ax_buf[N], ay_buf[N], az_buf[N];
float wx_buf[N], wy_buf[N], wz_buf[N];

/* Offsets and Tracking */
float ax_offset, ay_offset, az_offset;
float wx_offset, wy_offset, wz_offset;

void IMU_Init(void)
{
  ACCELERO_StatusTypeDef acc_status;
  GYRO_StatusTypeDef gyro_status;
  
  /* 1. 清狀態變數與 buffer */
  idx = 0;
  filled = 0;
  offset_done = 0;

  wx_offset = wy_offset = wz_offset = 0.0f;
  ax_offset = ay_offset = az_offset = 0.0f;

  for (int i = 0; i < N; i++) {
    ax_buf[i] = ay_buf[i] = az_buf[i] = 0;
    wx_buf[i] = wy_buf[i] = wz_buf[i] = 0.0f;
  }

  /* 2. 初始化感測器（依你的 BSP 來） */
  acc_status = BSP_ACCELERO_Init();
  if (acc_status == ACCELERO_OK) {
    printf("IMU: Accelerometer init OK\r\n");
  } else {
    printf("IMU: Accelerometer init FAILED (%d)\r\n", acc_status);
  }
  
  gyro_status = BSP_GYRO_Init();
  if (gyro_status == GYRO_OK) {
    printf("IMU: Gyroscope init OK\r\n");
  } else {
    printf("IMU: Gyroscope init FAILED (%d)\r\n", gyro_status);
  }
  BSP_MAGNETO_Init();

  /* 3. 初始化輸出結構 */
  x_axes_out.AXIS_X = x_axes_out.AXIS_Y = x_axes_out.AXIS_Z = 0;
  g_axes_out.AXIS_X = g_axes_out.AXIS_Y = g_axes_out.AXIS_Z = 0;
  m_axes_out.AXIS_X = m_axes_out.AXIS_Y = m_axes_out.AXIS_Z = 0;

}


void Update_Buffer(const int16_t a[3], const float w[3])
{
	ax_buf[idx] = a[0];
	ay_buf[idx] = a[1];
	az_buf[idx] = a[2];
	wx_buf[idx] = w[0];
	wy_buf[idx] = w[1];
	wz_buf[idx] = w[2];
	idx = (idx + 1) % N;
	if(filled < N) ++filled;
}

void Compute_Offsets(void)
{
  if (filled != N || offset_done) return;

  float sum_wx = 0.0f, sum_wy = 0.0f, sum_wz = 0.0f;
  float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
  for (int i = 0; i < N; i++)
  {
    sum_wx += wx_buf[i];
    sum_wy += wy_buf[i];
    sum_wz += wz_buf[i];
    sum_ax += ax_buf[i];
    sum_ay += ay_buf[i];
    sum_az += az_buf[i];
  }

  wx_offset = sum_wx / N;
  wy_offset = sum_wy / N;
  wz_offset = sum_wz / N;
  ax_offset = sum_ax / N;
  ay_offset = sum_ay / N;
  az_offset = sum_az / N;

  offset_done = 1;
}


void Process_Signal(void)
{
  if (!offset_done) return;

  float avg_ax = 0, avg_ay = 0, avg_az = 0;
  float avg_wx = 0, avg_wy = 0, avg_wz = 0;

  // Average last 5 samples for smoothing
  for (int k = 0; k < 5; k++) {
    int i = (idx - 1 - k + N) % N;
    avg_ax += ax_buf[i];
    avg_ay += ay_buf[i];
    avg_az += az_buf[i];
    avg_wx += wx_buf[i];
    avg_wy += wy_buf[i];
    avg_wz += wz_buf[i];
  }
  avg_ax /= 5.0f;
  avg_ay /= 5.0f;
  avg_az /= 5.0f;
  avg_wx /= 5.0f;
  avg_wy /= 5.0f;
  avg_wz /= 5.0f;

  /* Subtract static offset to get linear acceleration */
  x_axes_out.AXIS_X = (int32_t)(avg_ax - ax_offset);
  x_axes_out.AXIS_Y = (int32_t)(avg_ay - ay_offset);
  x_axes_out.AXIS_Z = (int32_t)(avg_az - az_offset);

  g_axes_out.AXIS_X = (int32_t)(avg_wx - wx_offset);
  g_axes_out.AXIS_Y = (int32_t)(avg_wy - wy_offset);
  g_axes_out.AXIS_Z = (int32_t)(avg_wz - wz_offset);
}

void IMU_RunStep()
{
	BSP_ACCELERO_AccGetXYZ(acc);
	BSP_GYRO_GetXYZ(gyro);
	BSP_MAGNETO_GetXYZ(mag);

	// 更新 buffer
	Update_Buffer(acc, gyro);

	// 一旦填滿一次，就計算 offset（只做一次）
	Compute_Offsets();

	if (!offset_done) {
		// Before calibration: output raw values
		x_axes_out.AXIS_X = (int32_t)acc[0];
		x_axes_out.AXIS_Y = (int32_t)acc[1];
		x_axes_out.AXIS_Z = (int32_t)acc[2];
		g_axes_out.AXIS_X = (int32_t)gyro[0];
		g_axes_out.AXIS_Y = (int32_t)gyro[1];
		g_axes_out.AXIS_Z = (int32_t)gyro[2];
		
		m_axes_out.AXIS_X = mag[0];
		m_axes_out.AXIS_Y = mag[1];
		m_axes_out.AXIS_Z = mag[2];
	} else {
		// After calibration: use smoothed + gravity-tracked values
		Process_Signal();
	}
}
