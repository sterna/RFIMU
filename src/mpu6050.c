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


//----------- Internal variables -------------//

static unsigned char MPU6050Busy=0;
static unsigned char MPU6050RegisterBuffer[14];
volatile unsigned char mpuIsInited=0;
//Raw output vals from the IMU
volatile int16_t IMUVals[AXIS_NOF];
volatile int16_t IMUValsCorrectUnits[AXIS_NOF];
//Raw offsets for the IMU
static int16_t IMUValOffsets[AXIS_NOF];
//Indicates if an axis shall be inverted or not. Set 1 for normal, and -1 for inverted. Order is given in the axis enum
static const int8_t IMUAxisInversions[AXIS_NOF]={1,1,1,1,1,1};


//----------- Internal functions -------------//

static void i2CReset();
static void i2cInitHw();
static void i2cWriteRegisterBlocking(uint8_t addr, uint8_t value);
static inline bool isAxis(uint8_t axis);

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
	i2cWriteRegisterBlocking(107,0);	//Reset all device registers and enable (power mgnt1 register)
	delay_us(500);
	i2cWriteRegisterBlocking(25,7);	//Sample rate divider. Sample rate is 1kHz/(1+val)->125Hz
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
	IMUVals[AXIS_X]=value*IMUAxisInversions[AXIS_X];
	//value=(int16_t)((MPU6050RegisterBuffer[2]<<8)|MPU6050RegisterBuffer[5]); //Todo: This must be incorrect..
	value=(int16_t)((MPU6050RegisterBuffer[2]<<8)|MPU6050RegisterBuffer[3]);
	IMUVals[AXIS_Y]=value*IMUAxisInversions[AXIS_Y];
	//value=(int16_t)((MPU6050RegisterBuffer[4]<<8)|MPU6050RegisterBuffer[6]);	//Todo: This also
	value=(int16_t)((MPU6050RegisterBuffer[4]<<8)|MPU6050RegisterBuffer[5]);
	IMUVals[AXIS_Z]=value*IMUAxisInversions[AXIS_Z];

	value=(int16_t)((MPU6050RegisterBuffer[8]<<8)|MPU6050RegisterBuffer[9]);
	IMUVals[AXIS_ROLL]=value*IMUAxisInversions[AXIS_ROLL];
	value=(int16_t)((MPU6050RegisterBuffer[10]<<8)|MPU6050RegisterBuffer[11]);
	IMUVals[AXIS_PITCH]=value*IMUAxisInversions[AXIS_PITCH];
	value=(int16_t)((MPU6050RegisterBuffer[12]<<8)|MPU6050RegisterBuffer[13]);
	IMUVals[AXIS_YAW]=value*IMUAxisInversions[AXIS_YAW];

	//If the offset values are reset, use the newest value, per axis
	for(uint8_t i=0;i<AXIS_NOF;i++)
	{
		if(!IMUValOffsets[i])
		{
			IMUValOffsets[i]=IMUVals[i];
		}
	}
}

/*
 * Set an offset
 * If set to 0, this will use the next sampled value
 */
void mpu6050SetOffset(uint8_t axis, int16_t val)
{
	if(!isAxis(axis))
	{
		return;
	}
	if(axis==AXIS_ALL)
	{
		for(uint8_t i=0;i<AXIS_NOF;i++)
		{
			mpu6050SetOffset(i,val);
		}
	}
	IMUValOffsets[axis]=val;
}

/*
 * Sets all offsets to 0, which causes a sample to the current offsets
 */
void mpu6050ResetAllOffsets()
{
	mpu6050SetOffset(AXIS_ALL, 0);
}

/*
 * Returns the value of a certain axis
 * If applyOffset is given as true, the current offset will be applied (subtracted)
 * Returns 0 if the axis does not exist
 */
int16_t mpu6050GetValue(uint8_t axis,bool applyOffset)
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
void mpu6050GetAllValues(int16_t* out, bool applyOffset)
{
	for(uint8_t i=0;i<AXIS_NOF;i++)
	{
		out[i]=mpu6050GetValue(i,applyOffset);
	}
}

/*
 * Puts all IMU values into the outbuffer. It's up to the caller to verify that the buffer is big enough
 * if applyOffset is given, offsets will be applied (subtracted)
 */
void mpu6050GetAllValuesStruct(IMUVals_t* out, bool applyOffset)
{
	out->accX=mpu6050GetValue(AXIS_X,applyOffset);
	out->accY=mpu6050GetValue(AXIS_Y,applyOffset);
	out->accZ=mpu6050GetValue(AXIS_Z,applyOffset);
	out->roll=mpu6050GetValue(AXIS_ROLL,applyOffset);
	out->pitch=mpu6050GetValue(AXIS_PITCH,applyOffset);
	out->yaw=mpu6050GetValue(AXIS_YAW,applyOffset);
}

/*
 * Converts raw acceleration value milli g
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

//Call this function every 50ms or so, to start a measurement.
void mpu6050Process()
{
	static uint32_t nextCallTime=0;
	volatile static IMUVals_t vals;
	if(nextCallTime<systemTime)
	{
		nextCallTime=systemTime+MPU_6050_PROCESS_PERIOD;
		if(mpuIsInited)
		{
			if(!MPU6050Busy)
			{
				//i2CReset();
				MPU6050Busy=1;
				I2C_GenerateSTART(I2C1,ENABLE);
			}
			mpu6050GetAllValuesStruct(&vals,false);
		}
	}
}

static int16_t noMotionThresholdAcc=0;
static int16_t noMotionThresholdGyro=0;

/*
 * This function will do all the math to estimate various things
 * and if events and conditions are true based on IMU data
 */
void mpu6050CalculateEvents()
{

	int16_t accVals[3]={0,0,0};		//Acceleration vals in milliG. No offsets applied
	int16_t gyroVals[3]={0,0,0};	//Gyro vals in deg/s. No offsets applied
	for(uint8_t i=0;i<(AXIS_NOF/2);i++)
	{
		accVals[i]=mpu6050ConvertAccToG(mpu6050GetValue(i,false));
		gyroVals[i]=mpu6050ConvertGyroToDegS(mpu6050GetValue(i+(AXIS_NOF/2),false));
	}
	/*
	 * Todo: Estimate angle relative to ground plane
	 * http://www.instructables.com/id/Accelerometer-Gyro-Tutorial/
	 * Also add a reset functions (maybe based on the offset already used in some way?)
	 */

	/*
	 * Todo: Estimate no-motion (gyro speed is close to 0 and total result of the acc is 1
	 * Some filtering required
	 */

	/*
	 * Todo: Estimate impulse (like a clap or stomp). Look for a sudden peak in any acc data
	 */

	/*
	 *	Todo: Implement gesture/sequence recording and recognition
	 *	Sequences are started and ended by no-motion (short periods of no-motion)
	 *	Two modes: Record and detection
	 *	Record:
	 *	1. Sample an average over a "longer" period of time (say 100-200ms) for all sensors (perhaps just use the acc)
	 *	2. Store this average in a sequence (Allow for about 1-2 second sequences)
	 *	Detection:
	 *	1. Perform the same sampling
	 *	2. Compare, at each sample, the current sample to all existing recordings.
	 *	Allow for a successcount for each recording, and ignore recordings that are not possible to match
	 */
}

/*
 * Calculate the total inertial vector (R^2=Rx^2+Ry^2+Rz^2)
 */
uint32_t calculateTotalForceVector(int16_t val[3])
{
	uint32_t totalVal=(uint32_t)( (int32_t)(val[0]*val[0]) + (int32_t)(val[1]*val[1]) + (int32_t)(val[2]*val[2]) );
	return utilSquareRootRounded(totalVal);
}

// ---------------- Internal functions --------------- //

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
