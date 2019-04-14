/*
 * hal_mpu6050.c
 *
 *  Created on: Feb 19, 2016
 *      Author: Sina Darvishi
 *              Christoph Rackwitz
 */

/**
 * |----------------------------------------------------------------------
 * | Copyright (C) 2016 Sina Darvishi, Christoph Rackwitz
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */

#include <assert.h>
#include "hal_mpu6050.h"

/* Default I2C address */
#define MPU6050_I2C_ADDR			0xD0

/* Who I am register value */
#define MPU6050_I_AM				0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g */
// for regular parts
#define MPU6050_ACCE_SENS_2      ((float) 16384)
#define MPU6050_ACCE_SENS_4      ((float) 8192)
#define MPU6050_ACCE_SENS_8      ((float) 4096)
#define MPU6050_ACCE_SENS_16     ((float) 2048)
// engineering samples have halved acceleration sensitivity, operate at FS_SEL=2 (8g)
// https://forum.sparkfun.com/viewtopic.php?t=30624
// http://www.invensense.com/developers/forum/viewtopic.php?f=3&t=132&hilit=half
#define MPU6050_ACCE_SENS_2_ES   ((float) 8192)
#define MPU6050_ACCE_SENS_4_ES   ((float) 4096)
#define MPU6050_ACCE_SENS_8_ES   ((float) 2048)
#define MPU6050_ACCE_SENS_16_ES  ((float) 1024)

MPU6050_Result MPU6050_Init(I2C_HandleTypeDef *I2Cx, MPU6050 *dev,
		MPU6050_Device DeviceNumber,
		MPU6050_Accelerometer AccelerometerSensitivity,
		MPU6050_Gyroscope GyroscopeSensitivity)
{
	assert(dev != NULL);

	dev->phi2c = I2Cx;

	/* Format I2C address */
	dev->Address = MPU6050_I2C_ADDR | (uint8_t) DeviceNumber;

	/* Check if device is connected */
	if (HAL_I2C_IsDeviceReady(dev->phi2c, dev->Address, 2, 5) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	/* Check who am I */
	uint8_t response;
	if (HAL_I2C_Mem_Read(dev->phi2c, dev->Address, MPU6050_WHO_AM_I, 1, &response, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}
	if (response != MPU6050_I_AM)
	{
		return MPU6050_Result_DeviceInvalid;
	}

	/* Wakeup MPU6050 */
	uint8_t regvalue = 0x00;
	if (HAL_I2C_Mem_Write(dev->phi2c, dev->Address, MPU6050_PWR_MGMT_1, 1, &regvalue, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}
	//------------------

	/* Set sample rate to 1kHz */
	MPU6050_SetDataRate(dev, MPU6050_DataRate_1KHz);

	/* Config accelerometer */
	MPU6050_SetAccelerometer(dev, AccelerometerSensitivity);

	/* Config Gyroscope */
	MPU6050_SetGyroscope(dev, GyroscopeSensitivity);

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_SetDataRate(MPU6050 *dev, MPU6050_DataRate rate)
{
	/* Set data sample rate */
	uint8_t regvalue = rate;
	if (HAL_I2C_Mem_Write(dev->phi2c, dev->Address, MPU6050_SMPLRT_DIV, 1, &regvalue, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_SetDLPF(MPU6050 *dev, uint8_t cfg)
{
	uint8_t regvalue = 0;
	if (HAL_I2C_Mem_Read(dev->phi2c, dev->Address, MPU6050_CONFIG, 1, &regvalue, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	regvalue &= ~0b111;
	regvalue |= (cfg & 0b111);

	if (HAL_I2C_Mem_Write(dev->phi2c, dev->Address, MPU6050_CONFIG, 1, &regvalue, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_SetAccelerometer(MPU6050 *dev, MPU6050_Accelerometer AccelerometerSensitivity)
{
	uint8_t regvalue = 0;

	if (HAL_I2C_Mem_Read(dev->phi2c, dev->Address, MPU6050_ACCEL_CONFIG, 1, &regvalue, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	regvalue &= ~(0b11 << 3);
	regvalue |= (AccelerometerSensitivity & 0b11) << 3;

	if (HAL_I2C_Mem_Write(dev->phi2c, dev->Address, MPU6050_ACCEL_CONFIG, 1, &regvalue, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity)
	{
		case MPU6050_Accelerometer_2G:     dev->Acce_Mult = 1.0f / MPU6050_ACCE_SENS_2; break;
		case MPU6050_Accelerometer_4G:     dev->Acce_Mult = 1.0f / MPU6050_ACCE_SENS_4; break;
		case MPU6050_Accelerometer_8G:     dev->Acce_Mult = 1.0f / MPU6050_ACCE_SENS_8; break;
		case MPU6050_Accelerometer_16G:    dev->Acce_Mult = 1.0f / MPU6050_ACCE_SENS_16; break;
		case MPU6050_Accelerometer_2G_ES:  dev->Acce_Mult = 1.0f / MPU6050_ACCE_SENS_2_ES; break;
		case MPU6050_Accelerometer_4G_ES:  dev->Acce_Mult = 1.0f / MPU6050_ACCE_SENS_4_ES; break;
		case MPU6050_Accelerometer_8G_ES:  dev->Acce_Mult = 1.0f / MPU6050_ACCE_SENS_8_ES; break;
		case MPU6050_Accelerometer_16G_ES: dev->Acce_Mult = 1.0f / MPU6050_ACCE_SENS_16_ES; break;
		default: dev->Acce_Mult = 0; break;
	}

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_SetGyroscope(MPU6050 *dev,
		MPU6050_Gyroscope GyroscopeSensitivity)
{
	uint8_t regvalue = 0;
	if (HAL_I2C_Mem_Read(dev->phi2c, dev->Address, MPU6050_GYRO_CONFIG, 1, &regvalue, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	regvalue &= ~(0b11 << 3);
	regvalue |= (GyroscopeSensitivity & 0b11) << 3;

	if (HAL_I2C_Mem_Write(dev->phi2c, dev->Address, MPU6050_GYRO_CONFIG, 1, &regvalue, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	switch (GyroscopeSensitivity)
	{
	case MPU6050_Gyroscope_250s:
		dev->Gyro_Mult = 1 / MPU6050_GYRO_SENS_250;
		break;
	case MPU6050_Gyroscope_500s:
		dev->Gyro_Mult = 1 / MPU6050_GYRO_SENS_500;
		break;
	case MPU6050_Gyroscope_1000s:
		dev->Gyro_Mult = 1 / MPU6050_GYRO_SENS_1000;
		break;
	case MPU6050_Gyroscope_2000s:
		dev->Gyro_Mult = 1 / MPU6050_GYRO_SENS_2000;
		break;
	default:
		dev->Gyro_Mult = 0;
		break;
	}

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_ReadAccelerometer(MPU6050 *dev)
{
	uint8_t data[6];
	if (HAL_I2C_Mem_Read(dev->phi2c, dev->Address, MPU6050_ACCEL_XOUT_H, 1, data, sizeof(data), 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	/* Format */
	dev->Accelerometer_X = (int16_t) (data[0] << 8 | data[1]);
	dev->Accelerometer_Y = (int16_t) (data[2] << 8 | data[3]);
	dev->Accelerometer_Z = (int16_t) (data[4] << 8 | data[5]);

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_ReadGyroscope(MPU6050 *dev)
{
	uint8_t data[6];
	if (HAL_I2C_Mem_Read(dev->phi2c, dev->Address, MPU6050_GYRO_XOUT_H, 1, data, sizeof(data), 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	/* Format */
	dev->Gyroscope_X = (int16_t) (data[0] << 8 | data[1]);
	dev->Gyroscope_Y = (int16_t) (data[2] << 8 | data[3]);
	dev->Gyroscope_Z = (int16_t) (data[4] << 8 | data[5]);

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_ReadTemperature(MPU6050 *dev)
{
	uint8_t data[2];
	if (HAL_I2C_Mem_Read(dev->phi2c, dev->Address, MPU6050_TEMP_OUT_H, 1, data, sizeof(data), 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	/* Format temperature */
	int16_t temp = (data[0] << 8 | data[1]);
	dev->Temperature = temp / 340.0f + 36.53f;

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_ReadAll(MPU6050 *dev)
{
	uint8_t data[14];
	if (HAL_I2C_Mem_Read(dev->phi2c, dev->Address, MPU6050_ACCEL_XOUT_H, 1, data, sizeof(data), 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	/* Format accelerometer data */
	dev->Accelerometer_X = (int16_t) (data[0] << 8 | data[1]);
	dev->Accelerometer_Y = (int16_t) (data[2] << 8 | data[3]);
	dev->Accelerometer_Z = (int16_t) (data[4] << 8 | data[5]);

	/* Format temperature */
	int16_t temp = (data[6] << 8 | data[7]);
	dev->Temperature = temp / 340.0f + 36.53f;

	/* Format gyroscope data */
	dev->Gyroscope_X = (int16_t) (data[8] << 8 | data[9]);
	dev->Gyroscope_Y = (int16_t) (data[10] << 8 | data[11]);
	dev->Gyroscope_Z = (int16_t) (data[12] << 8 | data[13]);

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_EnableInterrupts(MPU6050 *dev)
{
	uint8_t regvalue;

	/* Enable interrupts for data ready and motion detect */
	regvalue = 0x21; // FIXME: looks wrong, upper bit has no defined function
	if (HAL_I2C_Mem_Write(dev->phi2c, dev->Address, MPU6050_INT_ENABLE, 1, &regvalue, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	/* Clear IRQ flag on any read operation */
	if (HAL_I2C_Mem_Read(dev->phi2c, dev->Address, MPU6050_INT_PIN_CFG, 1, &regvalue, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	regvalue |= 0x10; // INT_RD_CLEAR

	if (HAL_I2C_Mem_Write(dev->phi2c, dev->Address, MPU6050_INT_PIN_CFG, 1, &regvalue, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_DisableInterrupts(MPU6050 *dev)
{
	uint8_t regvalue = 0x00;
	if (HAL_I2C_Mem_Write(dev->phi2c, dev->Address, MPU6050_INT_ENABLE, 1, &regvalue, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_ReadInterrupts(MPU6050 *dev, MPU6050_Interrupt *InterruptsStruct)
{
	InterruptsStruct->Status = 0;

	uint8_t regvalue;
	if (HAL_I2C_Mem_Read(dev->phi2c, dev->Address, MPU6050_INT_STATUS, 1, &regvalue, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	/* Fill value */
	InterruptsStruct->Status = regvalue;

	/* Return OK */
	return MPU6050_Result_Ok;
}
