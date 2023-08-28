/*
 * mpu6050.h
 *
 *  Created on: Aug 23, 2023
 *      Author: johndoe
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"

HAL_StatusTypeDef mpuInit(I2C_HandleTypeDef hi2c);
HAL_StatusTypeDef mpuRead(I2C_HandleTypeDef hi2c);
void mpuCalibrate(I2C_HandleTypeDef hi2c);
void mpuProcessed(void);

typedef struct SensorData
{
	float gX, gY, gZ, aX, aY, aZ;
}SensorData;

extern SensorData sensorRaw;
extern SensorData sensorProcessed;
extern SensorData offset;

#endif /* INC_MPU6050_H_ */
