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
//float mx_buf[N], my_buf[N], mz_buf[N];

/* Offsets for w */
float wx_offset, wy_offset, wz_offset;
//float mx_offset, my_offset, mz_offset;

void IMU_Init(void)
{
  ACCELERO_StatusTypeDef acc_status;
  GYRO_StatusTypeDef gyro_status;
  
  /* 1. 清狀態變數與 buffer */
  idx = 0;
  filled = 0;
  offset_done = 0;

  wx_offset = wy_offset = wz_offset = 0.0f;

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

  /* 3. 初始化輸出結構與姿態角 */
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
  for (int i = 0; i < N; i++)
  {
    sum_wx += wx_buf[i];
    sum_wy += wy_buf[i];
    sum_wz += wz_buf[i];
  }

  wx_offset = sum_wx / N;
  wy_offset = sum_wy / N;
  wz_offset = sum_wz / N;

  offset_done = 1;
}


void Process_Signal(void)
{
  if (!offset_done) return;

  long sum_ax = 0, sum_ay = 0, sum_az = 0;
  float sum_wx = 0, sum_wy = 0, sum_wz = 0;

  for (int k = 0; k < 5; k++) {
    int i = (idx - 1 - k + N) % N;
    sum_ax += ax_buf[i];
    sum_ay += ay_buf[i];
    sum_az += az_buf[i];
    sum_wx += wx_buf[i];
    sum_wy += wy_buf[i];
    sum_wz += wz_buf[i];
  }

  x_axes_out.AXIS_X = sum_ax / 5;
  x_axes_out.AXIS_Y = sum_ay / 5;
  x_axes_out.AXIS_Z = sum_az / 5;

  g_axes_out.AXIS_X = (int16_t)(sum_wx / 5.0f - wx_offset);
  g_axes_out.AXIS_Y = (int16_t)(sum_wy / 5.0f - wy_offset);
  g_axes_out.AXIS_Z = (int16_t)(sum_wz / 5.0f - wz_offset);
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

	// Output raw values immediately (before offset calibration is done)
	if (!offset_done) {
		// Before calibration: output raw values
		x_axes_out.AXIS_X = acc[0];
		x_axes_out.AXIS_Y = acc[1];
		x_axes_out.AXIS_Z = acc[2];
		g_axes_out.AXIS_X = (int16_t)gyro[0];
		g_axes_out.AXIS_Y = (int16_t)gyro[1];
		g_axes_out.AXIS_Z = (int16_t)gyro[2];
		
		m_axes_out.AXIS_X = mag[0];
		m_axes_out.AXIS_Y = mag[1];
		m_axes_out.AXIS_Z = mag[2];
	} else {
		// After calibration: use smoothed + offset-corrected values
		Process_Signal();
	}
}

