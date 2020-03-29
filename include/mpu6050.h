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
#define MPU6050_PROCESS_PERIOD	30
#define MPU6050_REPORT_PERIOD	250

//Defines the fullscale sensitivty of the Gyro (in +/- deg/s)
#define MPU6050_GYRO_SENS_250	(0b00<<3)
#define MPU6050_GYRO_SENS_500	(0b01<<3)
#define MPU6050_GYRO_SENS_1000	(0b10<<3)
#define MPU6050_GYRO_SENS_2000	(0b11<<3)

#define MPU6050_GYRO_FULLSCALE	1000
#define MPU6050_GYRO_SENS_COMMAND	MPU6050_GYRO_SENS_1000
#define MPU6050_GYRO_SELF_TEST_ALL	(0b111<<5)
//Defines the fullscale sensitivty of the Accelerometer (in +/- g)
#define MPU6050_ACC_SENS_2		(0b00<<3)
#define MPU6050_ACC_SENS_4		(0b01<<3)
#define MPU6050_ACC_SENS_8		(0b10<<3)
#define MPU6050_ACC_SENS_16		(0b11<<3)
#define MPU6050_ACC_FULLSCALE	4
#define MPU6050_ACC_SENS_COMMAND	MPU6050_ACC_SENS_4
#define MPU6050_ACC_SELF_TEST_ALL	(0b111<<5)

#define MPU6050_ADDR	0xD0
#define MPU6050_ACC_X_H	59

//Defines the index of axis
//Note: I changed the pitch/roll axis, to better suit what I actually have in the board oriented
enum
{
	AXIS_X=0,
	AXIS_Y=1,
	AXIS_Z=2,
	AXIS_PITCH=3,
	AXIS_ROLL=4,
	AXIS_YAW=5,
	AXIS_NOF=6,
	AXIS_ALL=255
};

//Defines the bit of motion events
typedef enum
{
	MTN_EVT_NO_MOTION_ALL=0,
	MTN_EVT_TAP_X=1,
	MTN_EVT_TAP_Y=2,
	MTN_EVT_TAP_Z=3,
	MTN_EVT_TAP_ROLL=4,
	MTN_EVT_TAP_PITCH=5,
	MTN_EVT_TAP_YAW=6,
	MTN_NOF_EVENTS
}motionEvent_t;

/*
 * Contatins the IMU values for all axis
 */
typedef struct{
	int16_t accX;
	int16_t accY;
	int16_t accZ;
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
}IMUVals_t;

typedef struct{
	int32_t accX;
	int32_t accY;
	int32_t accZ;
}accVals_t;

typedef struct{
	int32_t roll;
	int32_t pitch;
	int32_t yaw;
}gyroVals_t;

void mpu6050Init();
void mpu6050Process();
void mpu6050SetOffset(uint8_t axis, int16_t val, bool isG);
void mpu6050ResetAllOffsets();

int16_t mpu6050GetValue(uint8_t axis,bool applyOffset);
int16_t mpu6050GetOffset(uint8_t axis);
void mpu6050GetAllValues(int16_t* out, bool applyOffset);
void mpu6050GetAllValuesStruct(IMUVals_t* out, bool applyOffset);
int16_t mpu6050ConvertAccToG(int16_t val);
int16_t mpu6050ConvertGtoAcc(int16_t mg);
int16_t mpu6050ConvertGyroToDegS(int16_t val);
int16_t mpu6050ConvertDegSToGyro(int16_t degs);


bool mpu6050MotionEventActive(motionEvent_t evt);

void mpu6050TransmitRaw(bool csv);

#endif /* MPU6050_H_ */
