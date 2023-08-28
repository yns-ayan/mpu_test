#include "mpu6050.h"

uint16_t mpu6050_addr = (104<<1);
SensorData sensorRaw;
SensorData sensorProcessed;
SensorData offset;

HAL_StatusTypeDef mpuInit(I2C_HandleTypeDef hi2c)
{
	HAL_StatusTypeDef ret;
	uint8_t data[3];

	// Check if device is ready to communicate
	ret = HAL_I2C_IsDeviceReady(&hi2c, mpu6050_addr, 3, 5);
	if (ret != HAL_OK) return ret;

	// Configure device
	ret = HAL_I2C_Mem_Read(&hi2c, mpu6050_addr, 0x1A, I2C_MEMADD_SIZE_8BIT, data, 3, 1000);
	if (ret != HAL_OK) return ret;
	// Set DLPF and FSYNC
	data[0] &= ~0x07;
	data[0] |= 0x03;
	// Set range and self test bits for gyro
	data[1] &= ~(0x03<<3);
	data[1] |= 0x18;
	// Set range and self test bits for accelerometer
	data[2] &= ~(0x03<<3);
	data[2] |= 0x18;
	// Write configuration data
	ret = HAL_I2C_Mem_Write(&hi2c, mpu6050_addr, 0x1A, I2C_MEMADD_SIZE_8BIT, data, 3, 1000);
	if (ret != HAL_OK) return ret;

	// Configure interrupt pin
	ret = HAL_I2C_Mem_Read(&hi2c, mpu6050_addr, 0x37, I2C_MEMADD_SIZE_8BIT, data, 2, 1000);
	if (ret != HAL_OK) return ret;
	// Set data for interrupt enable
	data[0] |= 0x14;
	data[1] |= 0x01;
	// Write interrupt configuration
	ret = HAL_I2C_Mem_Write(&hi2c, mpu6050_addr, 0x37, I2C_MEMADD_SIZE_8BIT, data, 2, 1000);
	if (ret != HAL_OK) return ret;
	__NOP();

	// Disable sleep mode
	ret = HAL_I2C_Mem_Read(&hi2c, mpu6050_addr, 0x6B, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
	if (ret != HAL_OK) return ret;
	// Configure sleep mode bits
	data[0] &= ~0x67;
	data[1] = 0;
	// Write sleep configuration
	ret = HAL_I2C_Mem_Write(&hi2c, mpu6050_addr, 0x6B, I2C_MEMADD_SIZE_8BIT, data, 2, 1000);
	return ret;
}

HAL_StatusTypeDef mpuRead(I2C_HandleTypeDef hi2c)
{
	HAL_StatusTypeDef ret;
	uint8_t accelDataRaw[6];
	uint8_t gyroDataRaw[6];

	// Check if device is ready
	ret = HAL_I2C_IsDeviceReady(&hi2c, mpu6050_addr, 3, 5);
	if(ret != HAL_OK) return ret;
	__NOP();
	// Read accelerometer data
	ret = HAL_I2C_Mem_Read(&hi2c, mpu6050_addr, 0x3B, I2C_MEMADD_SIZE_8BIT, accelDataRaw, 6, 1000);
	if(ret != HAL_OK) return ret;
	__NOP();
	// Read gyroscope data
	ret = HAL_I2C_Mem_Read(&hi2c, mpu6050_addr, 0x43, I2C_MEMADD_SIZE_8BIT, gyroDataRaw, 6, 1000);
	if(ret != HAL_OK) return ret;

	// Add raw data to the struct
	sensorRaw.aX = (float)((int16_t)(accelDataRaw[0]<<8 | accelDataRaw[1]));
	sensorRaw.aY = (float)((int16_t)(accelDataRaw[2]<<8 | accelDataRaw[3]));
	sensorRaw.aZ = (float)((int16_t)(accelDataRaw[4]<<8 | accelDataRaw[5]));
	sensorRaw.gX = (float)((int16_t)(gyroDataRaw[0]<<8 | gyroDataRaw[1]));
	sensorRaw.gY = (float)((int16_t)(gyroDataRaw[2]<<8 | gyroDataRaw[3]));
	sensorRaw.gZ = (float)((int16_t)(gyroDataRaw[4]<<8 | gyroDataRaw[5]));

//	*(accelData+2) = (float)(temp*10.0/16384.0f);
//	*(gyroData+2) = (float)(temp/131.072f);

	// Return HAL_OK if everything worked as it should
	return HAL_OK;
}

void mpuProcessed(void)
{
	sensorProcessed.aX = (sensorRaw.aX - offset.aX)*9.81f/2048.0f-0.5f;
	sensorProcessed.aY = (sensorRaw.aY - offset.aY)*9.81f/2048.0f+0.1;
	sensorProcessed.aZ = (sensorRaw.aZ - offset.aZ)*9.81f/2048.0f;
	sensorProcessed.gX = (sensorRaw.gX - offset.gX)/16.4f;
	sensorProcessed.gY = (sensorRaw.gY - offset.gY)/16.4f;
	sensorProcessed.gZ = (sensorRaw.gZ - offset.gZ)/16.4f;
}

void mpuCalibrate(I2C_HandleTypeDef hi2c)
{
  uint8_t rawData[6];
  float gyroBiasSum[3] = {0, 0, 0};
  float accelBiasSum[3] = {0, 0, 0};

  // Take 1000 readings for gyros
  for (int k = 0; k < 1000; ++k) {
    HAL_I2C_Mem_Read(&hi2c, mpu6050_addr, 0x43, I2C_MEMADD_SIZE_8BIT, rawData, 6, 1000);
    gyroBiasSum[0] += (float)((int16_t)(rawData[0] << 8 | rawData[1]));
    gyroBiasSum[1] += (float)((int16_t)(rawData[2] << 8 | rawData[3]));
    gyroBiasSum[2] += (float)((int16_t)(rawData[4] << 8 | rawData[5]));

    HAL_I2C_Mem_Read(&hi2c, mpu6050_addr, 0x3B, I2C_MEMADD_SIZE_8BIT, rawData, 6, 1000);
	accelBiasSum[0] += (float)((int16_t)(rawData[0] << 8 | rawData[1]));
	accelBiasSum[1] += (float)((int16_t)(rawData[2] << 8 | rawData[3]));
	accelBiasSum[2] += (float)((int16_t)(rawData[4] << 8 | rawData[5])-2048);  // Assuming 1 g = 2048 in 16g scale at 16-bit resolution
	HAL_Delay(10); // Wait a bit to simulate real operation
  }

  // Get average bias for gyro
  offset.gX = gyroBiasSum[0] / 1000.0;
  offset.gY= gyroBiasSum[1] / 1000.0;
  offset.gZ = gyroBiasSum[2] / 1000.0;

  // Get average bias for accelerometer
  offset.aX = accelBiasSum[0] / 1000.0;
  offset.aY = accelBiasSum[1] / 1000.0;
  offset.aZ = accelBiasSum[2] / 1000.0;
}
