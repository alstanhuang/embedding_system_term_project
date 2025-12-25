/*
 * imu.h
 *
 *  Created on: Dec 16, 2025
 *      Author: jocer
 */

#ifndef APP_IMU_H_
#define APP_IMU_H_

#include "gatt_db.h"

//void IMU_Init(void);
void IMU_Init(void);
void IMU_RunStep(void);

extern AxesRaw_t x_axes_out;
extern AxesRaw_t g_axes_out;
extern AxesRaw_t m_axes_out;


#endif /* APP_IMU_H_ */

