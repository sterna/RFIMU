/*
 *	mpu6050.h
 *
 *	Created on: 19 mar 2015
 *		Author: Sterna and Påsse
 */
#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>
#include <stdbool.h>
#include "utils.h"
#include <math.h>

//The communication timeout for the MPU6050 I2C
#define MPU_TIMEOUT_MS	100
//The period at which the MPU6050 process is called (in ms)
#define MPU_6050_PROCESS_PERIOD	45

//Defines the fullscale sensitivty of the Gyro (in +/- deg/s)
#define MPU6050_GYRO_SENS_250	(0b00<<3)
#define MPU6050_GYRO_SENS_500	(0b01<<3)
#define MPU6050_GYRO_SENS_1000	(0b10<<3)
#define MPU6050_GYRO_SENS_2000	(0b11<<3)

#define MPU6050_GYRO_FULLSCALE	1000
#define MPU6050_GYRO_SENS_COMMAND	MPU6050_GYRO_SENS_1000
//Defines the fullscale sensitivty of the Accelerometer (in +/- g)
#define MPU6050_ACC_SENS_2		(0b00<<3)
#define MPU6050_ACC_SENS_4		(0b01<<3)
#define MPU6050_ACC_SENS_8		(0b10<<3)
#define MPU6050_ACC_SENS_16		(0b11<<3)
#define MPU6050_ACC_FULLSCALE	4
#define MPU6050_ACC_SENS_COMMAND	MPU6050_ACC_SENS_4

enum
{
	AXIS_X=0,
	AXIS_Y=1,
	AXIS_Z=2,
	AXIS_ROLL=3,
	AXIS_PITCH=4,
	AXIS_YAW=5,
	AXIS_NOF=6,
	AXIS_ALL=255
};

/*
 * Contatins the IMU values for all axis
 */
typedef struct{
	uint16_t accX;
	uint16_t accY;
	uint16_t accZ;
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
}IMUVals_t;

void mpu6050Init();
void mpu6050Process();
void mpu6050SetOffset(uint8_t axis, int16_t val);
void mpu6050ResetAllOffsets();

int16_t mpu6050GetValue(uint8_t axis,bool applyOffset);
int16_t mpu6050GetOffset(uint8_t axis);
void mpu6050GetAllValues(int16_t* out, bool applyOffset);
void mpu6050GetAllValuesStruct(IMUVals_t* out, bool applyOffset);
int16_t mpu6050ConvertAccToG(int16_t val);
int16_t mpu6050ConvertAccToG(int16_t val);

#endif /* MPU6050_H_ */