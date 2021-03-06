/*
 *	mpu6050.c
 *
 *	Created on: 19 mar 2015
 *		Author: Sterna and Påsse
 *
 */

#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "misc.h"
#include "mpu6050.h"
#include "time.h"
#include "uart.h"
#include "events.h"


//----------- Internal variables and definitions -------------//

static unsigned char MPU6050Busy=0;
static unsigned char MPU6050RegisterBuffer[14];
volatile unsigned char mpuIsInited=0;
//Raw output vals from the IMU
volatile int16_t IMUVals[AXIS_NOF];
volatile int16_t IMUValsCorrectUnits[AXIS_NOF];
//Raw offsets for the IMU
static int16_t IMUValOffsets[AXIS_NOF];
//Indicates if an axis shall be inverted or not. Set 1 for normal, and -1 for inverted. Order is given in the axis enum
static const int8_t IMUAxisInversions[AXIS_NOF]={1,1,1,-1,1,1};

//Contains the constant offset seen for different values (calibrated against earth gravity)
//Offsets are in milli g and deg/s respectively

static const int16_t IMUFactoryOffsets[AXIS_NOF]={-15,-15,-30,0,0,0};
//const int16_t IMUFactoryOffsets[AXIS_NOF]={-10,0,80,0,0,0};	//Set used for the other unit

//Indicated which motion events are true. Motion events bits are defined in
static eventState_t motionEvents[MTN_NOF_EVENTS];

//Handle a value ring buffer of all IMU data
//Sample window length. The length in time of this data is IMU_BUFFER_LEN*MPU6050_PROCESS_PERIOD
#define IMU_BUFFER_LEN	16
#define IMU_BUFFER_TIME_LEN	(IMU_BUFFER_LEN*MPU6050_PROCESS_PERIOD)
//Ringbuffer with IMU data
static volatile IMUVals_t IMURingBuffer[IMU_BUFFER_LEN];
//Index points to the where newest data is
static volatile uint8_t IMURingBufIndex=0;
//The max amplitude values for each axis in the current ring buffer. The values keep their sign.
//Note that these are not necessarily (and most likely) not from the same sample
static volatile IMUVals_t IMURingBufferMax;
//Holds the index for the currently max sample. Note that this is not actually IMUVals, but only indexes
static volatile IMUVals_t IMURingBufferMaxIndex;
//Low-pass filtered values based on the ringbuffer
static volatile IMUVals_t IMURingBufferLP;
//The number of samples to be used for the low-pass versions of the IMU values. The samples are counted from latest sample
//None of these values must be larger than IMU_BUFFER_LEN
static const uint8_t LPSamplesAcc=12;
static const uint8_t LPSamplesGyro=4;

//Current angle estimation. Unit is whole degrees
static angles_t agEst={0,0,0};

//Counter for impulse events
static volatile uint32_t accImpulsePulseCount=0;

static bool gyroInitCalibrated=false;


//----------- Internal functions -------------//

static void i2CReset();
static void i2cInitHw();
static void i2cWriteRegisterBlocking(uint8_t addr, uint8_t value);
static inline bool isAxis(uint8_t axis);
static bool axisIsAcc(uint8_t axis);
static bool axisIsGyro(uint8_t axis);
int16_t getDataFromStructByAxisIndex(IMUVals_t* str,uint8_t ax);

uint32_t calculateTotalForceVector(int32_t* val);
uint32_t calculateTotalForceVectorStruct(accVals_t* vals);


static void updateIMURingBuffer(IMUVals_t* newdata, bool doCalculations);
void getLatestIMURingBufferData(IMUVals_t* data);
void getIMURingBufferDataByIndex(IMUVals_t* data, uint8_t index, bool fromOldest);

/*
 * Inits the onboard MPU6050
 */
void mpu6050Init()
{
	i2cInitHw();
	for(uint8_t i=0;i<AXIS_NOF;i++)
	{
		IMUValOffsets[i]=0;
		IMUVals[i]=0;
	}
	//Behold all the magic numbers!
	delay_ms(100);
	i2cWriteRegisterBlocking(107,0);	//Enable power (power mgnt1 register)
	delay_us(500);
	i2cWriteRegisterBlocking(25,4);	//Sample rate divider. Sample rate is 1kHz/(1+val)->125Hz
	delay_us(500);
	i2cWriteRegisterBlocking(26,0b00000011);	//Global config, set sync on Gyro YOut
	delay_us(500);

	i2cWriteRegisterBlocking(27,MPU6050_GYRO_SENS_COMMAND);	//Gyroscope config. No self-test and sense mode at +/- 1000 deg/s
	delay_us(500);
	i2cWriteRegisterBlocking(28,MPU6050_ACC_SENS_COMMAND); //Accelermoter config. No self-test and sense mode at +/- 4g
	delay_us(500);
	i2cWriteRegisterBlocking(35,0);	//Fifo setting register, No FIFO enabled
	delay_us(500);
	//Todo: Consider enabling the data read interrupt (Write bit0=1  in register 0x38). Don't bother the other sources
	mpuIsInited=1;

	mpu6050ResetAllOffsets();
	//Clear the ring buffer
	memset(IMURingBuffer,0,sizeof(IMUVals_t)*IMU_BUFFER_LEN);
	memset(&IMURingBufferMax,0,sizeof(IMUVals_t));

	for(uint8_t i=0;i<MTN_NOF_EVENTS;i++)
	{
		eventInit(&motionEvents[i]);
	}
	motionEvents[MTN_EVT_NO_MOTION_ALL].threshold=5;
	//Set all angle motion events threshold to the same value
	for(uint8_t i= MTN_EVT_AG_NEUTRAL;i<=MTN_EVT_AG_THUMB_DOWN;i++)
	{
		motionEvents[i].threshold=2;
	}
	//Todo: Add settings for the rest of the motionEvents, when they are defined

}

/*
 * Parses the buffer received from MPU6050
 *
 * http://image.wikifoundry.com/image/1/gPhN8K8G60yyJ1xIoc7iHg25317/GW500H283
 * Due to the orientation of the chip, X and Y axis are inverted, to comply with ISO8855 (right-hand system)
 * Roll and pitch are inverted
 */
void mpu6050ParseBuffer()
{
	int16_t value=0;
	value=(int16_t)((MPU6050RegisterBuffer[0]<<8)|MPU6050RegisterBuffer[1]);
	IMUVals[AXIS_X]=(value + mpu6050ConvertGtoAcc(IMUFactoryOffsets[AXIS_X])) * IMUAxisInversions[AXIS_X];
	value=(int16_t)((MPU6050RegisterBuffer[2]<<8)|MPU6050RegisterBuffer[3]);
	IMUVals[AXIS_Y]=(value + mpu6050ConvertGtoAcc(IMUFactoryOffsets[AXIS_Y])) * IMUAxisInversions[AXIS_Y];
	value=(int16_t)((MPU6050RegisterBuffer[4]<<8)|MPU6050RegisterBuffer[5]);
	IMUVals[AXIS_Z]=(value + mpu6050ConvertGtoAcc(IMUFactoryOffsets[AXIS_Z])) * IMUAxisInversions[AXIS_Z];

	value=(int16_t)((MPU6050RegisterBuffer[8]<<8)|MPU6050RegisterBuffer[9]);
	IMUVals[AXIS_PITCH]=(value + mpu6050ConvertDegSToGyro(IMUFactoryOffsets[AXIS_PITCH])) * IMUAxisInversions[AXIS_PITCH];
	value=(int16_t)((MPU6050RegisterBuffer[10]<<8)|MPU6050RegisterBuffer[11]);
	IMUVals[AXIS_ROLL]=(value + mpu6050ConvertDegSToGyro(IMUFactoryOffsets[AXIS_ROLL])) * IMUAxisInversions[AXIS_ROLL];
	value=(int16_t)((MPU6050RegisterBuffer[12]<<8)|MPU6050RegisterBuffer[13]);
	IMUVals[AXIS_YAW]=(value + mpu6050ConvertDegSToGyro(IMUFactoryOffsets[AXIS_YAW])) * IMUAxisInversions[AXIS_YAW];

	//If the offset values are reset, use the newest value, per axis
	/*for(uint8_t i=0;i<AXIS_NOF;i++)
	{
		if(!IMUValOffsets[i])
		{
			IMUValOffsets[i]=IMUVals[i];
		}
	}*/
}

/*
 * Set an offset
 * If set to 0, this will use the next sampled value
 */
void mpu6050SetOffset(uint8_t axis, int16_t val, bool isG)
{
	if(!isAxis(axis))
	{
		return;
	}
	if(axis==AXIS_ALL)
	{
		for(uint8_t i=0;i<AXIS_NOF;i++)
		{
			mpu6050SetOffset(i,val,isG);
		}
	}
	if(isG)
	{
		if(axisIsAcc(axis))
		{
			val=mpu6050ConvertGtoAcc(val);
		}
		else if(axisIsGyro(axis))
		{
			val=mpu6050ConvertDegSToGyro(val);
		}
	}
	IMUValOffsets[axis]=val;
}

/*
 * Sets all offsets to the current value.
 * Used as a 0
 * Axis indicates which acc axis shall be set to 1G
 */
void mpu6050SetOffsetsToCurrent(uint8_t axisTo1G)
{
	for(uint8_t i=AXIS_X;i<AXIS_NOF;i++)
	{
		int16_t tmpOffset=0;
		if(i==axisTo1G && axisIsAcc(axisTo1G))
		{
			tmpOffset=mpu6050ConvertGtoAcc(1000);
		}
		mpu6050SetOffset(i,IMUVals[i]+tmpOffset,false);
	}
}

/*
 * Sets all offsets to 0, which causes a sample to the current offsets
 */
void mpu6050ResetAllOffsets()
{
	mpu6050SetOffset(AXIS_ALL, 0,true);
}

/*
 * Returns the value of a certain axis
 * If applyOffset is given as true, the current offset will be applied (subtracted)
 * Returns 0 if the axis does not exist
 */
int16_t mpu6050GetValue(uint8_t axis,bool applyOffset, bool realUnits)
{
	int16_t tmpVal=0;
	if(!isAxis(axis))
	{
		return 0;
	}
	tmpVal=IMUVals[axis];
	if(applyOffset)
	{
		tmpVal-=IMUValOffsets[axis];
	}
	if(realUnits)
	{
		if(axisIsAcc(axis))
		{
			tmpVal=mpu6050ConvertAccToG(tmpVal);
		}
		else
		{
			tmpVal=mpu6050ConvertGyroToDegS(tmpVal);
		}
	}
	return tmpVal;
}

/*
 * Returns the current offset
 * If axis does not exist, 0 will be returned
 */
int16_t mpu6050GetOffset(uint8_t axis)
{
	if(!isAxis(axis))
	{
		return 0;
	}
	return IMUValOffsets[axis];
}

/*
 * Puts all IMU values into the outbuffer. It's up to the caller to verify that the buffer is big enough
 * if applyOffset is given, offsets will be applied (subtracted)
 */
void mpu6050GetAllValues(int16_t* out, bool applyOffset, bool realUnits)
{
	for(uint8_t i=0;i<AXIS_NOF;i++)
	{
		out[i]=mpu6050GetValue(i,applyOffset,realUnits);
	}
}

/*
 * Puts all IMU values into the outbuffer. It's up to the caller to verify that the buffer is big enough
 * if applyOffset is given, offsets will be applied (subtracted)
 */
void mpu6050GetAllValuesStruct(IMUVals_t* out, bool applyOffset, bool realUnits)
{
	out->accX=mpu6050GetValue(AXIS_X,applyOffset,realUnits);
	out->accY=mpu6050GetValue(AXIS_Y,applyOffset,realUnits);
	out->accZ=mpu6050GetValue(AXIS_Z,applyOffset,realUnits);
	out->roll=mpu6050GetValue(AXIS_ROLL,applyOffset,realUnits);
	out->pitch=mpu6050GetValue(AXIS_PITCH,applyOffset,realUnits);
	out->yaw=mpu6050GetValue(AXIS_YAW,applyOffset,realUnits);
}

/*
 * Converts raw acceleration value to milli g
 * valmax = 32767 -> acc = 4G = 4000mG
 * mg per val = sens*1000/valmax
 * mG @ val -> val*sens*1000/valmax
 *
 */
int16_t mpu6050ConvertAccToG(int16_t val)
{
	int32_t tmp=0;
	tmp=(int32_t)((val*MPU6050_ACC_FULLSCALE*1000)/INT16_MAX);
	return (int16_t)tmp;
}

/*
 * Converts milli G into raw acceleration value
 * Useful for setting offset
 */
int16_t mpu6050ConvertGtoAcc(int16_t mg)
{
	int32_t tmp=0;
	tmp=(int32_t)(mg*INT16_MAX/(MPU6050_ACC_FULLSCALE*1000));
	return (int16_t)tmp;
}

/*
 * Converts a raw gyro value from to deg/s
 * valmax = 32767 -> gyro 1000deg/s -> 1000
 * deg/s per val = sens*1000/valmax
 * deg/s @ val -> val*sens*1000/valmax
 */
int16_t mpu6050ConvertGyroToDegS(int16_t val)
{
	int32_t tmp=0;
	tmp=(int32_t)((val*MPU6050_GYRO_FULLSCALE)/INT16_MAX);
	return (int16_t)tmp;
}

/*
 * Converts deg/s into raw gyro value
 */
int16_t mpu6050ConvertDegSToGyro(int16_t degs)
{
	int32_t tmp=0;
	tmp=(int32_t)(degs*INT16_MAX/MPU6050_GYRO_FULLSCALE);
	return (int16_t)tmp;
}

/*
 * Transmits all raw data currently in the input buffer
 */
void mpu6050TransmitRaw(bool csv, bool newline)
{

	IMUVals_t vals;
	mpu6050GetAllValuesStruct(&vals,true,false);
	if(csv)
	{
		int32_t csvValsAcc[8];
		int32_t csvValsGyro[4];
		csvValsAcc[0]=mpu6050ConvertGyroToDegS(vals.roll);//mpu6050ConvertAccToG(vals.accX);
		csvValsAcc[1]=mpu6050ConvertGyroToDegS(vals.pitch);//mpu6050ConvertAccToG(vals.accY);
		csvValsAcc[2]=mpu6050ConvertGyroToDegS(vals.yaw);//mpu6050ConvertAccToG(vals.accZ);
		csvValsAcc[3]=IMURingBufferLP.roll;
		csvValsAcc[4]=IMURingBufferLP.pitch;
		csvValsAcc[5]=IMURingBufferLP.yaw;
		if(mpu6050MotionEventActive(MTN_EVT_NO_MOTION_ALL))
		{
			csvValsAcc[6]=1000;
		}
		else
		{
			csvValsAcc[6]=500;
		}

		uartSendCSV(csvValsAcc,7,newline);	//todo: remove newline here (and also the return)
		return;
		csvValsGyro[0]=mpu6050ConvertGyroToDegS(vals.roll);
		csvValsGyro[1]=mpu6050ConvertGyroToDegS(vals.pitch);
		csvValsGyro[2]=mpu6050ConvertGyroToDegS(vals.yaw);
		if(mpu6050MotionEventActive(MTN_EVT_NO_MOTION_ALL))
		{
			csvValsGyro[3]=1000;
		}
		else
		{
			csvValsGyro[3]=500;
		}
		//uartSendCSV(csvValsGyro,4,newline);
	}
	else
	{
		uartSendKeyVal("AccX",mpu6050ConvertAccToG(vals.accX),false);
		uartSendKeyVal("AccY",mpu6050ConvertAccToG(vals.accY),false);
		uartSendKeyVal("AccZ",mpu6050ConvertAccToG(vals.accZ),false);
		uartSendKeyVal("Yaw",mpu6050ConvertGyroToDegS(vals.yaw),false);
		uartSendKeyVal("Pitch",mpu6050ConvertGyroToDegS(vals.pitch),false);
		uartSendKeyVal("Roll",mpu6050ConvertGyroToDegS(vals.roll),newline);
	}
}

/*
 * Prints out processed data through UART
 */
void mpu6050TransmitDataProcessed(bool csv, bool newline)
{
	int32_t vals[10];
	if(mpu6050MotionEventActive(MTN_EVT_NO_MOTION_ALL))
	{
		vals[0]=1000;
	}
	else
	{
		vals[0]=500;
	}
	vals[1]=20*accImpulsePulseCount;
	uint8_t i=MTN_EVT_AG_NEUTRAL;
	for(i=MTN_EVT_AG_NEUTRAL;i<=MTN_EVT_AG_THUMB_DOWN;i++)
	{
		if(mpu6050MotionEventActive(i))
		{
			break;
		}
	}
	if(i<=MTN_EVT_AG_THUMB_DOWN)
	{
		vals[2]=i*100;
	}
	else
	{
		vals[2]=0;
	}
	vals[3]=10*agEst.roll;
	vals[4]=10*agEst.pitch;
	if(csv)
	{
		uartSendCSV(vals,5,newline);
	}
	else
	{
		uartSendKeyVal("NoMot",vals[0],false);
		uartSendKeyVal("ImpCount",vals[1],false);
		uartSendKeyVal("AgRoll",vals[2],false);
		uartSendKeyVal("AgPitch",vals[3],newline);
		uartSendKeyVal("AccVect",vals[4],newline);
	}

}

//Various filter parameters

//No-motion
//Thresholds used to determine no-motion event
const int16_t noMotionThresholdAcc=40;
const int16_t noMotionThresholdGyro=15;
//If the total accelerometer force vector is close to this value, the device is stationary (or moving veery slowly)
const int32_t notMotionAcc=1000;

//Angle estimation (and gyro)
//The gyro weight for the complementary used for angle estimation
const int32_t gyroAgWeight=90;
const int32_t gyroAgWeightMax=100;
//Times used for gyro re-calibration interval. Calibration happens if we have no-motion for these times.
const uint32_t gyroRecalibrateTimeAg=1500;
const uint32_t gyroRecalibrateTime=5000;

//Impulse parameters
//The holdoff time before we can detect another impulse
const uint32_t impulseHoldoffTime=150;
//The max amplitude must be above this threshold
const int32_t impulseThrehsoldAmplitude=2250;
//The FWHM times must be between these times (in ms)
const uint32_t FWHM_minTime=10;
const uint32_t FWHM_maxTime=40;
//The number of samples required before and after the pulse
const uint32_t minSamplesBefore=1;
const uint32_t minSamplesAfter=1;

//Constants for accelerometer LP-filter
volatile int32_t accNewWeight=15;
const int32_t accNewWeightMax=100;

/*
 * This function will do all the math to estimate various things
 * and if events and conditions are true based on IMU data
 */
void mpu6050CalculateEvents()
{

	//Get data and calculate some basics

	//Calculate the time difference from the last sample. Time gating is done in the caller
	static uint32_t lastCallTime=0;
	static int32_t gyroAgXZLast=0;
	static int32_t gyroAgYZLast=0;

	static uint32_t nextGyroCalibTimeForAg=0;
	int32_t td=0;

	//Current acc vals
	accVals_t accVals;			//Acceleration vals in milliG.
	gyroVals_t gyroVals;		//Gyro vals in deg/s.

	//If lastCallTime is 0, this is the first the function is run. Here we can do inits
	if(!lastCallTime)
	{
		lastCallTime=systemTime;
	}
	td=systemTime-lastCallTime;
	lastCallTime=systemTime;
	{
		IMUVals_t tmp;
		//Includes offset
		getLatestIMURingBufferData(&tmp);
		//mpu6050GetAllValuesStruct(&test,true);
		//mpu6050TransmitRaw(true,true);
		accVals.accX=(int32_t)tmp.accX;
		accVals.accY=(int32_t)tmp.accY;
		accVals.accZ=(int32_t)tmp.accZ;
		gyroVals.roll=(int32_t)tmp.roll;
		gyroVals.yaw=(int32_t)tmp.yaw;
		gyroVals.pitch=(int32_t)tmp.pitch;
	}
	int32_t totalAccVector=calculateTotalForceVectorStruct(&accVals);


	/*
	 * --------------------- No-motion estimation-------------------
	 *
	 * Estimate no-motion (gyro max speed is close to 0 and total force vector of the acc is 1000mg
	 */

	bool noMotionState=false;
	if(((abs(totalAccVector-notMotionAcc)<noMotionThresholdAcc)) &&
			( utilMax(abs(gyroVals.roll),utilMax(abs(gyroVals.pitch),abs(gyroVals.yaw))) < noMotionThresholdGyro))
	{
		noMotionState=true;
	}
	eventStateUpdate(&motionEvents[MTN_EVT_NO_MOTION_ALL],noMotionState);


	/*
	 * ------------------ Angle estimation -----------------------
	 * Estimate angle relative to ground plane
	 * Only roll (XZ) and roll pitch (YZ) angles are estimated, as yaw angle (XY) is tricky to estimate, since there's no acceleration info
	 */

	//Angle estimation based on acceleration reading
	//Create trim accelerations to avoid weird edge-cases with atan and ignore strange angles
	accVals_t accSafe;
	accSafe.accX=IMURingBufferLP.accX;	//accValsLP.accX;
	accSafe.accY=IMURingBufferLP.accY;	//accValsLP.accY;
	accSafe.accZ=IMURingBufferLP.accZ;	//accValsLP.accZ;
	//Todo: Might want to trim these more later
	const int16_t smallestValueXY=150;
	const int16_t smallestValueZ=30;
	//If the absolute angle is too large, set the angle to 180. We will ignore the sign, because -180 and 180 are the same angle
	const int16_t largestAngle=150;
	if(abs(accVals.accX) < smallestValueXY && (abs(accVals.accY) > (notMotionAcc-smallestValueXY)))
	{
		accSafe.accX=0;
	}
	if(abs(accVals.accY)<smallestValueXY && (abs(accVals.accX) > (notMotionAcc-smallestValueXY)))
	{
		accSafe.accY=0;
	}
	if(abs(accVals.accZ)<smallestValueZ)
	{
		accSafe.accZ=smallestValueZ*utilSign(accVals.accZ);
	}
	//The angle towards the ground during tilt around the Y axis (left/right)
	int16_t accAgXZ = (int16_t)(roundf(atan2f((float)accSafe.accX,(float)accSafe.accZ)*180.0/PI));
	if(abs(accAgXZ)>largestAngle)
	{
		accAgXZ=180;
	}
	//The angle towards the ground during tilt around the X axis (front/back)
	int16_t accAgYZ = (int16_t)(roundf(atan2f((float)accSafe.accY,(float)accSafe.accZ)*180.0/PI));
	if(abs(accAgYZ)>largestAngle)
	{
		accAgYZ=180;
	}

	//Calculate angles based on gyro
	//The gyro angles are in milli deg (to avoid floating point and precision losses)
	int32_t gyroAgXZ = gyroAgXZLast + td*IMURingBufferLP.roll;
	gyroAgXZLast=gyroAgXZ;
	int32_t gyroAgYZ = gyroAgYZLast + td*IMURingBufferLP.pitch;
	gyroAgYZLast=gyroAgYZ;
	//Whenever we're not moving, set the last angle to whatever the accelerometer says the last angle, to avoid strange buildup (the gyro is mostly good when moving)
	if(motionEvents[MTN_EVT_NO_MOTION_ALL].active && \
			motionEvents[MTN_EVT_NO_MOTION_ALL].counter >= motionEvents[MTN_EVT_NO_MOTION_ALL].threshold && \
			systemTime > nextGyroCalibTimeForAg)
	{
		nextGyroCalibTimeForAg=systemTime+gyroRecalibrateTimeAg;
		gyroAgXZLast=accAgXZ*1000;
		gyroAgYZLast=accAgYZ*1000;
	}

	//Add the angles from gyro and accelerometer together in a complementary filter
	//The division by 1000 because the time step is in ms, rather than s
	int32_t agXZ = ((gyroAgWeight*gyroAgXZ)/1000 + (gyroAgWeightMax-gyroAgWeight)*accAgXZ)/gyroAgWeightMax;
	int32_t agYZ = ((gyroAgWeight*gyroAgYZ)/1000 + (gyroAgWeightMax-gyroAgWeight)*accAgYZ)/gyroAgWeightMax;
	//Save angles to the state variable
	agEst.roll=agXZ;
	agEst.pitch=agYZ;

	//Transmit angle data for debugging purposes
	//Multiple angle by 10 to get a reasonable scale
	//mpu6050TransmitRaw(true,false);
	/*int32_t agVals1[3];
	int32_t agVals2[3];
	agVals1[0]=10*accAgXZ;
	agVals1[1]=10*gyroAgXZ/1000;
	agVals1[2]=10*agXZ;
	uartSendCSV(agVals1,3,false);
	agVals2[0]=10*accAgYZ;
	agVals2[1]=10*gyroAgYZ/1000;
	agVals2[2]=10*agYZ;
	uartSendCSV(agVals2,3,true);*/

	/*
	 * ----------------- Angle event estimation -------------
	 *
	 * Estimate which of the following event angles is true. Assuming right hand.
	 *
	 * EventName	AG_Roll(X)	AG pitch Y)
	 * Neutral		0			0
	 * UpsideDown	180			180
	 * HandUp		NC			-90
	 * HandDown		NC			90
	 * ThumbUp		90			NC
	 * ThumbDown	-90			NC
	 */

	//The angle event is valid of close enough to the target
	{
		static volatile int16_t agEvtTol=20;
		motionEvent_t eventTrue=0;
		int16_t agRoll=agEst.roll;
		int16_t agPitch=agEst.pitch;
		if(utilValCloseEnoughDual(agRoll,agPitch,0,0,agEvtTol))
		{
			eventTrue=MTN_EVT_AG_NEUTRAL;
		}
		else if(utilValCloseEnoughDual(agRoll,agPitch,180,180,agEvtTol))
		{
			eventTrue=MTN_EVT_AG_UPSIDEDOWN;
		}
		else if(utilValCloseEnough(agPitch,-90,agEvtTol))
		{
			eventTrue=MTN_EVT_AG_HAND_UP;
		}
		else if(utilValCloseEnough(agPitch,90,agEvtTol))
		{
			eventTrue=MTN_EVT_AG_HAND_DOWN;
		}
		else if(utilValCloseEnough(agRoll,90,agEvtTol))
		{
			eventTrue=MTN_EVT_AG_THUMB_UP;
		}
		else if(utilValCloseEnough(agRoll,-90,agEvtTol))
		{
			eventTrue=MTN_EVT_AG_THUMB_DOWN;
		}
		motionEvent_t i=MTN_EVT_AG_NEUTRAL;
		while(i<=MTN_EVT_AG_THUMB_DOWN)
		{
			if(eventTrue==i)
			{
				eventStateUpdate(&motionEvents[i],true);
			}
			else
			{
				eventStateUpdate(&motionEvents[i],false);
			}
			i++;
		}
	}

	/*
	 * ---------------- Impulse estimation -----------------
	 *
	 * Estimate impulse (like a clap, stomp, or tap).
	 * Two types of impluses:
	 * 	Angle impulse (based on gyro) - Didn't work well, and is not very useful. Angle estimation + time gate will solve this input
	 * 	Hit impulse, based on acc
	 *
	 * Hit impulse
	 * Use the same method as the angle impulse with these parameters:
	 * - High acc amplitude on any axis.
	 * - Very short pulse (in the range of 10 to 100 ms).
	 * - Shorter holdoff (say 200ms or so)
	 *
	 * General detection scheme:
	 * 1. Find max sample in window
	 * 2. Search from max sample in reverse to find the start of the pulse
	 * 3. Search from max sample forward to find the end of the pulse
	 * A pulse is considered to be active if the value is above half the value of max
	 *
	 */
	//The next time we shall perform impulse detection (handles the hold-off timer)
	static uint32_t impulseNextAnalyzeTime=0;
	//Only perform analysis when we have not recently found a pulse
	if(systemTime>impulseNextAnalyzeTime)
	{
		//This is because it was annoying to extract the data from the struct into an array
		int32_t maxSample[3]={0,0,0};
		maxSample[0]=IMURingBufferMax.accX;
		maxSample[1]=IMURingBufferMax.accY;
		maxSample[2]=IMURingBufferMax.accZ;
		//Check if any amplitude is greater than the threshold exist, before wasting time looking it (because we already know this)
		if(utilMax(abs(maxSample[0]),utilMax(abs(maxSample[1]),abs(maxSample[2])))>impulseThrehsoldAmplitude)
		{
			uint8_t maxRingBufferIndex[3]={0,0,0};
			maxRingBufferIndex[0]=IMURingBufferMaxIndex.accX;
			maxRingBufferIndex[1]=IMURingBufferMaxIndex.accY;
			maxRingBufferIndex[2]=IMURingBufferMaxIndex.accZ;

			const uint32_t FWHM_minSamples=FWHM_minTime/MPU6050_SAMPLE_PERIOD;
			const uint32_t FWHM_maxSamples=FWHM_maxTime/MPU6050_SAMPLE_PERIOD;

			//Calculate the number of samples that shall have this
			bool pulseFound=false;
			for(uint8_t ax=0;ax<3 && pulseFound==false;ax++)
			{
				//Make sure we only analyze axis which have a high enough max
				if(abs(maxSample[ax])>impulseThrehsoldAmplitude)
				{
					int32_t halfMaxVal=maxSample[ax]/2;
					//Counters for the number of samples for various events
					uint8_t FWHMSamplesAbove=0;
					uint8_t FWHMSamplesBelowBefore=0;
					uint8_t FWHMSamplesBelowAfter=0;
					//Copy data from ringbuffer into a buffer that's easier to work with. Index 0 is the oldest data.
					int16_t dataBuf[IMU_BUFFER_LEN];
					uint8_t tmpRingIndex=IMURingBufIndex;
					uint8_t tmpMaxIndex=0;
					for(uint8_t i=0;i<IMU_BUFFER_LEN;i++)
					{
						tmpRingIndex=utilIncLoopSimple(tmpRingIndex,IMU_BUFFER_LEN-1);
						dataBuf[i]=getDataFromStructByAxisIndex(&IMURingBuffer[tmpRingIndex],ax);
						if(maxRingBufferIndex[ax]==tmpRingIndex)
						{
							tmpMaxIndex=i;
						}
					}

					uint8_t dataIndex=0;
					dataIndex=tmpMaxIndex;
					bool pulseStartOK=false;
					bool pulseEndOK=false;
					bool pulseFail=false;
					//Count backwards
					while(!pulseStartOK && !pulseFail)
					{
						int16_t data=dataBuf[dataIndex];
						bool aboveHM=false;
						if((abs(data) > abs(halfMaxVal)) &&
								(utilSign(data)==utilSign(halfMaxVal)))
						{
							aboveHM=true;
						}
						if(aboveHM)
						{
							FWHMSamplesAbove++;
						}
						else
						{
							FWHMSamplesBelowBefore++;
						}
						if(FWHMSamplesBelowBefore>=minSamplesBefore && FWHMSamplesAbove>=FWHM_minSamples)
						{
							//Success! We found a valid start of pulse
							pulseStartOK=true;
						}
						if(FWHMSamplesAbove>FWHM_maxSamples)
						{
							pulseFail=true;
						}
						if(dataIndex)
						{
							dataIndex--;
						}
						else if(!pulseStartOK)
						{
							pulseFail=true;
						}
					}	//End of pulseStart-detection
					//Count forwards
					dataIndex=tmpMaxIndex+1;
					//Check if we are at the end of window and if this might actuallt be OK
					if(dataIndex>=IMU_BUFFER_LEN)
					{
						if(pulseStartOK && minSamplesAfter==0)
						{
							pulseEndOK=true;
						}
						else
						{
							pulseFail=true;
						}
					}
					while(!pulseEndOK && !pulseFail)
					{
						int16_t data=dataBuf[dataIndex];
						bool aboveHM=false;
						if((abs(data) > abs(halfMaxVal)) &&
								(utilSign(data)==utilSign(halfMaxVal)))
						{
							aboveHM=true;
						}
						if(aboveHM)
						{
							FWHMSamplesAbove++;
						}
						else
						{
							FWHMSamplesBelowAfter++;
						}
						if(FWHMSamplesBelowAfter>=minSamplesAfter && FWHMSamplesAbove>=FWHM_minSamples)
						{
							//Success! We found a valid end of pulse
							pulseEndOK=true;
						}
						if(FWHMSamplesAbove>FWHM_maxSamples)
						{
							pulseFail=true;
						}

						dataIndex++;
						if(dataIndex>=IMU_BUFFER_LEN && !pulseEndOK)
						{
							pulseFail=true;
						}
					}	//End of pulseEnd-detection

					//Check if we found a valid start and end of pulse
					if(!pulseFail && pulseStartOK && pulseEndOK)
					{
						accImpulsePulseCount++;
						pulseFound=true;
						impulseNextAnalyzeTime=systemTime+impulseHoldoffTime;
					}
				}
			}
		}
	}	//End of impulse analysis


	/*
	 *	Todo: Implement gesture/sequence recording and recognition
	 *	This will most likely not work well...
	 *	Sequences are started and ended by no-motion (short periods of no-motion)
	 *	Two modes: Record and detection
	 *	Record:
	 *	1. Sample an average over a "longer" period of time (say 100-200ms) for all sensors (perhaps just use the acc)
	 *	2. Store this average in a sequence (Allow for about 1-2 second sequences)
	 *	Detection:
	 *	1. Perform the same sampling
	 *	2. Compare, at each sample, the current sample to all existing recordings.
	 *	Allow for a success count for each recording, and ignore recordings that are not possible to match
	 */


	//If no motion has been active for a long time, reset the gyro offset to current value
	if(eventGetActiveForMoreThan(&motionEvents[MTN_EVT_NO_MOTION_ALL],gyroRecalibrateTime) && \
			motionEvents[MTN_EVT_NO_MOTION_ALL].counter >= motionEvents[MTN_EVT_NO_MOTION_ALL].threshold)
	{
		eventSetTimerToNow(&motionEvents[MTN_EVT_NO_MOTION_ALL]);
		mpu6050SetOffset(AXIS_ROLL,IMUVals[AXIS_ROLL],false);
		mpu6050SetOffset(AXIS_PITCH,IMUVals[AXIS_PITCH],false);
		mpu6050SetOffset(AXIS_YAW,IMUVals[AXIS_YAW],false);
		gyroInitCalibrated=true;
	}
}


/*
 * Checks one single motion event
 */
bool mpu6050MotionEventActive(motionEvent_t evt)
{
	if (evt>=MTN_NOF_EVENTS)
	{
		return false;
	}
	return (eventGetSate(&motionEvents[evt]));
}

/*
 * Get the currently estimated angles
 */
void mpu6050GetAngles(angles_t* ag)
{
	ag->roll = agEst.roll;
	ag->pitch = agEst.pitch;
	ag->yaw = agEst.yaw;
}


/*
 * The task for for the MPU6050 handling and analysis
 * Call as often as possible (has it's own time gate)
 */
void mpu6050Task()
{
	static uint32_t nextCallTime=0;
	static uint32_t nextSendTime=0;
	static uint8_t processCounter=0;
	if(nextCallTime<systemTime)
	{
		nextCallTime=systemTime+MPU6050_SAMPLE_PERIOD;
		if(mpuIsInited)
		{
			if(!MPU6050Busy)
			{
				//i2CReset();
				MPU6050Busy=1;
				I2C_GenerateSTART(I2C1,ENABLE);
			}
			bool calculate=false;
			processCounter++;
			if(processCounter>=(MPU6050_PROCESS_PERIOD/MPU6050_SAMPLE_PERIOD))
			{
				calculate=true;
				processCounter=0;
			}
			//Update ring buffer with new data
			IMUVals_t tmp;
			mpu6050GetAllValuesStruct(&tmp,true,true);
			updateIMURingBuffer(&tmp,calculate);
			//Do all the signal processing stuff
			if(calculate)
			{
				mpu6050CalculateEvents();
			}
			//Transmit all the values through UART
			//mpu6050TransmitRaw(true,true);
			mpu6050TransmitDataProcessed(true,true);

		}
	}
	if(nextSendTime<systemTime)
	{
		//nextSendTime=systemTime+MPU6050_REPORT_PERIOD;
		//mpu6050TransmitRaw(true);
	}
}

// ---------------- Internal functions --------------- //

/*
 * A version to calculate the total force vector using a struct
 */
uint32_t calculateTotalForceVectorStruct(accVals_t* vals)
{
	int32_t tmp[3];
	tmp[0]=vals->accX;
	tmp[1]=vals->accY;
	tmp[2]=vals->accZ;
	return calculateTotalForceVector(tmp);
}

/*
 * Calculate the total inertial vector (R^2=Rx^2+Ry^2+Rz^2)
 * Result will be in whatever unit the input data is
 */
uint32_t calculateTotalForceVector(int32_t* val)
{
	int32_t totalVal=(val[0]*val[0]) + (val[1]*val[1]) + (val[2]*val[2]);
	return utilSqrtI2I((uint32_t)totalVal);

}

/*
 * Updates the ring buffer with new values
 * Purely dependent on the current state
 * If doCalculations is given, it will perform some additional calculations (find max and some average)
 */
static void updateIMURingBuffer(IMUVals_t* newdata, bool doCalculations)
{
	IMURingBufIndex=utilIncLoopSimple(IMURingBufIndex,IMU_BUFFER_LEN-1);
	memcpy(&IMURingBuffer[IMURingBufIndex],newdata,sizeof(IMUVals_t));

	//Update max amplitude values for each axis
	//Make sure to reset all values before going through
	if(!doCalculations)
	{
		return;
	}
	memset(&IMURingBufferMax,0,sizeof(IMUVals));
	//Uses the acc/gyro vals structs, because they are int32
	accVals_t valsTotTmpAcc={0,0,0};
	gyroVals_t valsTotTmpGyro={0,0,0};
	IMUVals_t* tmpData;
	uint8_t tmpBufInd=IMURingBufIndex;
	for(uint8_t i=0; i<IMU_BUFFER_LEN;i++)
	{
		//Find max values
		if(abs(IMURingBuffer[i].accX)>abs(IMURingBufferMax.accX))
		{
			IMURingBufferMax.accX=IMURingBuffer[i].accX;
			IMURingBufferMaxIndex.accX = i;
		}
		if(abs(IMURingBuffer[i].accY)>abs(IMURingBufferMax.accY))
		{
			IMURingBufferMax.accY=IMURingBuffer[i].accY;
			IMURingBufferMaxIndex.accY = i;
		}
		if(abs(IMURingBuffer[i].accZ)>abs(IMURingBufferMax.accZ))
		{
			IMURingBufferMax.accZ=IMURingBuffer[i].accZ;
			IMURingBufferMaxIndex.accZ = i;
		}
		if(abs(IMURingBuffer[i].roll)>abs(IMURingBufferMax.roll))
		{
			IMURingBufferMax.roll=IMURingBuffer[i].roll;
			IMURingBufferMaxIndex.roll = i;
		}
		if(abs(IMURingBuffer[i].pitch)>abs(IMURingBufferMax.pitch))
		{
			IMURingBufferMax.pitch=IMURingBuffer[i].pitch;
			IMURingBufferMaxIndex.pitch = i;
		}
		if(abs(IMURingBuffer[i].yaw)>abs(IMURingBufferMax.yaw))
		{
			IMURingBufferMax.yaw=IMURingBuffer[i].yaw;
			IMURingBufferMaxIndex.yaw = i;
		}

		//Calculate total for average value
		tmpData=&IMURingBuffer[tmpBufInd];
		if(i<LPSamplesAcc)
		{
			valsTotTmpAcc.accX+=tmpData->accX;
			valsTotTmpAcc.accY+=tmpData->accY;
			valsTotTmpAcc.accZ+=tmpData->accZ;
		}
		if(i<LPSamplesGyro)
		{
			valsTotTmpGyro.roll+=tmpData->roll;
			valsTotTmpGyro.pitch+=tmpData->pitch;
			valsTotTmpGyro.yaw+=tmpData->yaw;
		}
		utilDecLoopSimple(tmpBufInd,IMU_BUFFER_LEN-1);
	}
	//Calculate average
	IMURingBufferLP.accX = valsTotTmpAcc.accX/LPSamplesAcc;
	IMURingBufferLP.accY = valsTotTmpAcc.accY/LPSamplesAcc;
	IMURingBufferLP.accZ = valsTotTmpAcc.accZ/LPSamplesAcc;
	IMURingBufferLP.roll = valsTotTmpGyro.roll/LPSamplesGyro;
	IMURingBufferLP.pitch = valsTotTmpGyro.pitch/LPSamplesGyro;
	IMURingBufferLP.yaw = valsTotTmpGyro.yaw/LPSamplesGyro;
}

/*
 * Returns the latest data in the ring buffer
 */
void getLatestIMURingBufferData(IMUVals_t* data)
{
	getIMURingBufferDataByIndex(data,0,false);
}

/*
 * Returns the data from the IMU ring buffer
 * If fromOldest is given, index is in order from oldest data to newest data. Otherwise, it's in reverse
 */
void getIMURingBufferDataByIndex(IMUVals_t* data, uint8_t index, bool fromOldest)
{
	uint8_t tmpIndex=0;
	int8_t direction=1;
	if(!fromOldest)
	{
		direction=-1;
	}
	else
	{
		index=utilIncLoopSimple(index,IMU_BUFFER_LEN-1);
	}
	//Find right index:
	tmpIndex=utilIncWithDir(IMURingBufIndex,direction,index,0,IMU_BUFFER_LEN-1);
	memcpy(data,&IMURingBuffer[tmpIndex],sizeof(IMUVals_t));
}

int16_t getDataFromStructByAxisIndex(IMUVals_t* str,uint8_t ax)
{
	switch(ax)
	{
		case AXIS_X:
			return str->accX;
			break;
		case AXIS_Y:
			return str->accY;
			break;
		case AXIS_Z:
			return str->accZ;
			break;
		case AXIS_ROLL:
			return str->roll;
			break;
		case AXIS_PITCH:
			return str->pitch;
			break;
		case AXIS_YAW:
			return str->yaw;
			break;
		default:
			return 0;
	}
}

/*
 * Checks if an axis actually is an axis
 */
static inline bool isAxis(uint8_t axis)
{
	if(axis<AXIS_NOF || axis == AXIS_ALL)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
 * Checks if the axis is an accelerometer axis
 */
static bool axisIsAcc(uint8_t axis)
{
	if(axis== AXIS_X || axis == AXIS_Y || axis==AXIS_Z)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
 * Checks if the axis is a fyro axis
 */
static bool axisIsGyro(uint8_t axis)
{
	if(axis==AXIS_ROLL || axis==AXIS_PITCH || axis==AXIS_YAW)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//----------- I2C handling functions ------------------//

static void i2cInitHw()
{
	//Enable clock for I2C module and I/O pins
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1,ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1,DISABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	//GPIO init
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	//I2C init
	I2C_InitTypeDef I2C_InitStruct;
	I2C_StructInit(&I2C_InitStruct);
	I2C_InitStruct.I2C_Ack=I2C_Ack_Enable;
	I2C_InitStruct.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStruct.I2C_ClockSpeed=400000;
	I2C_Init(I2C1,&I2C_InitStruct);


	I2C_ITConfig(I2C1,I2C_IT_EVT | I2C_IT_BUF,ENABLE);
	I2C_Cmd(I2C1,ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel=I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
}

static void i2cWriteRegisterBlocking(uint8_t addr, uint8_t value)
{
	uint8_t retries=3;

	I2C_ITConfig(I2C1,I2C_IT_EVT | I2C_IT_BUF,DISABLE);
	static uint32_t mpuTimeoutTime=0;

	do{
		mpuTimeoutTime=systemTime+MPU_TIMEOUT_MS;
		I2C_GenerateSTART(I2C1,ENABLE);
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT) && mpuTimeoutTime>systemTime);
		I2C_SendData(I2C1,MPU6050_ADDR);
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && mpuTimeoutTime>systemTime);
		I2C_SendData(I2C1,addr);
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED) && mpuTimeoutTime>systemTime);
		I2C_SendData(I2C1,value);
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED) && mpuTimeoutTime>systemTime);
		I2C_GenerateSTOP(I2C1,ENABLE);
		if(mpuTimeoutTime<systemTime)
		{
			i2CReset();
		}
	}while(retries-- && (mpuTimeoutTime<systemTime));

	I2C_ClearITPendingBit(I2C1,I2C_IT_EVT);
	I2C_ClearITPendingBit(I2C1,I2C_IT_BUF);

	I2C_ITConfig(I2C1,I2C_IT_EVT | I2C_IT_BUF,ENABLE);
}


static void i2CReset()
{
	I2C_ITConfig(I2C1,I2C_IT_EVT | I2C_IT_BUF,DISABLE);
	I2C_Cmd(I2C1,DISABLE);
	i2cInitHw();
}


/*
 * I2C irq handler
 */
void I2C1_EV_IRQHandler()
{
	static unsigned char state=1;
	unsigned char lastState=state;
	switch(state)
	{
	case 1:
		if(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
		{
			I2C_SendData(I2C1,MPU6050_ADDR|0);
			I2C_GenerateSTOP(I2C1,DISABLE);
			I2C_GenerateSTART(I2C1,DISABLE);
			state=2;
		}
		break;
	case 2:
		if(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
			I2C_SendData(I2C1,MPU6050_ACC_X_H);	//Start reading from AccX_H register
			state=3;
		}
		break;
	case 3:
		if(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
			I2C_GenerateSTART(I2C1,ENABLE);
			state=4;
		}
		break;
	case 4:
		if(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
		{
			I2C_SendData(I2C1,MPU6050_ADDR|1);
			state=5;
		}
		break;
	case 5:
		if(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		{
			I2C_AcknowledgeConfig(I2C1,ENABLE);
			state=6;
		}
		break;
	case 6://AccX
	case 7:
	case 8://AccY
	case 9:
	case 10://AccZ
	case 11:
	case 12://Temperature
	case 13:
	case 14://GyroX
	case 15:
	case 16://GyroY
	case 17:
	case 18://GyroZ
	case 19:
		if(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			MPU6050RegisterBuffer[state-6]=I2C_ReceiveData(I2C1);
			state++;
			if(state==19)
			{
				I2C_AcknowledgeConfig(I2C1,DISABLE);
				I2C_GenerateSTOP(I2C1,ENABLE);
				state++;
			}

		}
		break;
	case 20:
		//Make sure that NO FUCKING BYTE IS LEFT IN THE BUFFER!
		//If there was interesting data, we will have take care of it already
		while((I2C1->SR1 & I2C_SR1_BTF) || (I2C1->SR1 & I2C_SR1_RXNE))
		{
			I2C_ReceiveData(I2C1);
		}
		mpu6050ParseBuffer();
		state=1;
		MPU6050Busy=0;
		break;
	default:
		state=1;
		break;
	}
	if((lastState==state)&&I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
	{
		//If this ever happens, something went wrong (I guess)?
		state=1;

	}
	I2C_ClearITPendingBit(I2C1,I2C_IT_EVT);
	I2C_ClearITPendingBit(I2C1,I2C_IT_BUF);
}
