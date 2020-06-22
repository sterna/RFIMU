

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "time.h"
#include "uart.h"
#include "xprintf.h"
#include "ledPwm.h"
#include "sw.h"
#include "apa102.h"
#include "utils.h"
#include "ledSegment.h"
#include "mpu6050.h"
#include "extFetCtrl.h"
#include "onboardLedCtrl.h"
#include "adc.h"
#include "advancedAnimations.h"

bool poorMansOS();
void poorMansOSRunAll();

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/*
 * Simple interface:
 * 	SW1: Cycle between 4 preset patterns (different colours, speeds etc).
 * 	SW2: Restart pulse and fade (manual beat)
 * 	SW3: Toggle pause
 */

typedef enum
{
	SMODE_BLUE_FADE_YLW_PULSE=0,
	SMODE_CYAN_FADE_YLW_PULSE,
	SMODE_RED_FADE_YLW_PULSE,
	SMODE_YLW_FADE_PURPLE_PULSE,
	SMODE_YLW_FADE_GREEN_PULSE,
	SMODE_CYAN_FADE_NO_PULSE,
	SMODE_YLW_FADE_NO_PULSE,
	SMODE_RED_FADE_NO_PULSE,
	SMODE_WHITE_FADE_PRIDE_PULSE,
	SMODE_RANDOM,
	SMODE_DISCO,
	SMODE_PRIDE_WHEEL,
	SMODE_BATTERY_DISP,
	SMODE_OFF,
	SMODE_NOF_MODES
}simpleModes_t;

static void dummyLedTask();
void displayBattery(uint8_t segment, uint16_t startLED);
void handleApplicationSimple();

#define GLOBAL_SETTING	5
#define GLITTER_TO_PULSE_CHANGE_TIME	5000

#define PULSE_FAST_PIXEL_TIME	1
#define PULSE_NORMAL_PIXEL_TIME	1
#define FADE_FAST_TIME		300
#define FADE_NORMAL_TIME	700

uint8_t segmentArmLeft=0;
uint8_t segmentArmRight=0;
uint8_t segmentHead=0;
uint8_t segmentTail=0;

uint8_t segment1Up=0;	//45 LEDs
uint8_t segment2Down=0;	//44 LEDs
uint8_t segment3Up=0;	//44 LEDs

volatile uint16_t batteryIndicatorStartLed=39;	//

int main(int argc, char* argv[])
{
	SystemCoreClockUpdate();
	timeInit();
	swInit();
	adcInit();
	uart1Init(250000);	//Todo: Upped the baud rate. 250000 is used, because this is what the Arduino serial plotter supports
	//extFetInit();
	onboardLedCtrlInit();
	apa102Init(1,140);
	apa102SetDefaultGlobal(GLOBAL_SETTING);
	apa102UpdateStrip(APA_ALL_STRIPS);
	mpu6050Init();

	while(1)
	{
		poorMansOS();
	}

}	//End of main()


/*
 * Main application to make a (somewhat) simple mode handling
 */
void handleApplicationSimple()
{

	//Ugly program "Full patte!", Which just sets all LEDs to max and then waits
	/*	apa102FillStrip(1,255,255,255,APA_MAX_GLOBAL_SETTING);
	apa102UpdateStrip(APA_ALL_STRIPS);
	while(1)
	{
	}
	 */
	static bool setupDone=false;
	//This is a loop for a simple user interface, with not as much control
	static ledSegmentPulseSetting_t pulse;
	static ledSegmentFadeSetting_t fade;
	static simpleModes_t smode=SMODE_RED_FADE_NO_PULSE;
	static bool isActive=true;
	static bool pulseIsActive=true;
	static uint32_t nextDiscoUpdate=0;
	static uint8_t globalSetting=GLOBAL_SETTING;
	static isPulseMode=false;

	if(!setupDone)
	{
		animLoadLedSegPulseColour(SIMPLE_COL_YELLOW,&pulse,200);
		/*pulse.cycles =0;
		pulse.ledsFadeAfter = 5;
		pulse.ledsFadeBefore = 5;
		pulse.ledsMaxPower = 15;
		pulse.mode = LEDSEG_MODE_LOOP_END;
		pulse.pixelTime = PULSE_NORMAL_PIXEL_TIME;
		pulse.pixelsPerIteration = 2;
		pulse.startDir =1;
		pulse.startLed = 1;*/

		pulse.cycles =0;
		pulse.ledsFadeAfter = 5;
		pulse.ledsFadeBefore = 5;
		pulse.ledsMaxPower = 30;
		pulse.mode = LEDSEG_MODE_GLITTER_BOUNCE;
		pulse.pixelTime = 1000;
		pulse.pixelsPerIteration = 5;
		pulse.startDir =1;
		pulse.startLed = 1;

		animLoadLedSegFadeColour(SIMPLE_COL_BLUE,&fade,50,200);
		fade.cycles =0;
		fade.mode = LEDSEG_MODE_BOUNCE;
		fade.startDir = -1;
		fade.fadeTime = 1500;
		fade.syncGroup=1;
		apa102SetDefaultGlobal(GLOBAL_SETTING);
		//segment1Up=ledSegInitSegment(1,2,15,false, &pulse,&fade);	//Skip the first LED to sync up the pulses
		//segment2Down=ledSegInitSegment(1,16,30,true,&pulse,&fade);
		//segment3Up=ledSegInitSegment(1,31,133,false, &pulse,&fade);
		segment1Up=ledSegInitSegment(1,2,45,false,false,&pulse,&fade);	//Skip the first LED to sync up the pulses
		segment2Down=ledSegInitSegment(1,46,89,true,false,&pulse,&fade);
		segment3Up=ledSegInitSegment(1,90,133,false,false,&pulse,&fade);
		//pulse.startDir=-1;
		//pulse.startLed = 100;
		setupDone=true;
	}

	//Change mode
	if(swGetFallingEdge(1))
	{
		bool fadeAlreadySet=false;
		bool pulseAlreadySet=false;
		pulseIsActive=true;
		//Handles if something needs to be done when changing from a state
		switch(smode)
		{
			case SMODE_DISCO:
			{
				if(isPulseMode)
				{
					pulse.pixelTime=PULSE_NORMAL_PIXEL_TIME;
				}
				else
				{
					pulse.pixelTime=1000;//PULSE_NORMAL_PIXEL_TIME;
				}
				fade.fadeTime=FADE_NORMAL_TIME;
				break;
			}
			case SMODE_PRIDE_WHEEL:
			{
				//Turn of pride-wheel
				animSetPrideWheelState(false);
				break;
			}
			case SMODE_WHITE_FADE_PRIDE_PULSE:
			{
				pulse.rainbowColour=false;
				pulse.ledsMaxPower=30;
				pulse.pixelsPerIteration=5;
				break;
			}
			default:
			{
				//Do nothing for default
			}
		}
		smode++;
		if(smode>=SMODE_NOF_MODES)
		{
			smode=0;
		}
		switch(smode)
		{
			case SMODE_BLUE_FADE_YLW_PULSE:
				animLoadLedSegFadeColour(SIMPLE_COL_BLUE,&fade,25,200);
				animLoadLedSegPulseColour(SIMPLE_COL_YELLOW,&pulse,200);
				break;
			case SMODE_CYAN_FADE_YLW_PULSE:
				animSetModeChange(SIMPLE_COL_GREEN,&fade,LEDSEG_ALL,true,50,200);
				fadeAlreadySet=true;
				pulseIsActive=false;
				animLoadLedSegPulseColour(SIMPLE_COL_YELLOW,&pulse,200);
				break;
			case SMODE_RED_FADE_YLW_PULSE:
				animLoadLedSegFadeBetweenColours(SIMPLE_COL_RED,SIMPLE_COL_BLUE,&fade,150,150);
				animSetModeChange(SIMPLE_COL_NO_CHANGE,&fade,LEDSEG_ALL,false,0,0);
				fadeAlreadySet=true;
				pulseIsActive=false;
				//animLoadLedSegPulseColour(SIMPLE_COL_YELLOW,&pulse,150);
				break;
			case SMODE_YLW_FADE_PURPLE_PULSE:
				animLoadLedSegFadeBetweenColours(SIMPLE_COL_CYAN,SIMPLE_COL_YELLOW,&fade,150,150);
				animSetModeChange(SIMPLE_COL_NO_CHANGE,&fade,LEDSEG_ALL,true,0,0);
				//animLoadLedSegPulseColour(SIMPLE_COL_PURPLE,&pulse,150);
				fadeAlreadySet=true;
				pulseIsActive=false;
				break;
			case SMODE_YLW_FADE_GREEN_PULSE:
				animSetModeChange(SIMPLE_COL_RED,&fade,LEDSEG_ALL,true,25,200);
				fadeAlreadySet=true;
				pulseIsActive=false;
				break;
			case SMODE_CYAN_FADE_NO_PULSE:
				animSetModeChange(SIMPLE_COL_CYAN,&fade,LEDSEG_ALL,false,50,200);
				fadeAlreadySet=true;
				pulseIsActive=false;
				break;
			case SMODE_YLW_FADE_NO_PULSE:
				animSetModeChange(SIMPLE_COL_YELLOW,&fade,LEDSEG_ALL,false,50,200);
				fadeAlreadySet=true;
				pulseIsActive=false;
				break;
			case SMODE_RED_FADE_NO_PULSE:
				animLoadLedSegFadeColour(SIMPLE_COL_RED,&fade,50,150);
				pulseIsActive=false;
				break;
			case SMODE_WHITE_FADE_PRIDE_PULSE:
				animSetModeChange(SIMPLE_COL_WHITE,&fade,LEDSEG_ALL,false,50,150);
				fadeAlreadySet=true;
				pulseIsActive=true;
				pulse.ledsMaxPower=40;
				pulse.pixelsPerIteration=3;
				pulse.rainbowColour=true;
				break;
			case SMODE_DISCO:
				if(pulseIsActive)
				{
					pulse.pixelTime=PULSE_FAST_PIXEL_TIME;
				}
				else
				{
					pulse.pixelTime=500;
				}
				fade.fadeTime=FADE_FAST_TIME;	//The break is omitted by design, since SMODE_DISCO does the same thing as SMODE_RANDOM
			case SMODE_RANDOM:
				animLoadLedSegFadeColour(SIMPLE_COL_RANDOM,&fade,100,200);
				animLoadLedSegPulseColour(SIMPLE_COL_RANDOM,&pulse,200);
				pulseIsActive=true;
				break;
			case SMODE_PRIDE_WHEEL:
				fade.fadeTime=FADE_NORMAL_TIME;
				animSetPrideWheel(&fade,LEDSEG_ALL);
				pulseIsActive=false;
				fadeAlreadySet=true;
				break;
			case SMODE_BATTERY_DISP:
			{
				//Do nothing here
				break;
			}
			case SMODE_OFF:	//turn LEDs off
			{
				fade.r_min=0;
				fade.r_max=0;
				fade.g_min=0;
				fade.g_max=0;
				fade.b_min=0;
				fade.b_max=0;
				pulse.r_max=0;
				pulse.g_max=0;
				pulse.b_max=0;
				break;
			}
			case SMODE_NOF_MODES:	//Should never happen
			{
				smode=0;
				break;
			}
		}

		//Update all segements
		if(!fadeAlreadySet)
		{
			ledSegSetFade(LEDSEG_ALL,&fade);
		}
		if(!pulseAlreadySet)
		{
			ledSegSetPulse(LEDSEG_ALL,&pulse);
			ledSegSetPulseActiveState(LEDSEG_ALL,pulseIsActive);
		}
		if(smode == SMODE_BATTERY_DISP)
		{
			//Pause The other segment (possible arm)
			//Set LEDs in the correct place to the right colours, corresponding to battery level
			ledSegSetPulseActiveState(segment1Up,false);
			ledSegSetFadeActiveState(segment1Up,false);
			displayBattery(segment1Up,batteryIndicatorStartLed);
		}
	}	//End of change mode clause

	if(swGetActiveForMoreThan(1,1000))
	{
		globalSetting++;
		if(globalSetting>APA_MAX_GLOBAL_SETTING/2)
		{
			globalSetting=0;
		}
		apa102SetDefaultGlobal(globalSetting);
	}

	//Generate a pulse (and switch modes for the staff)
	if(swGetRisingEdge(2))
	{
		apa102SetDefaultGlobal(globalSetting*3);
		ledSegRestart(LEDSEG_ALL,true,true);
	}
	if(swGetFallingEdge(2))
	{
		apa102SetDefaultGlobal(globalSetting);
	}
	//Switches between pulse mode and glitter mode
	if(swGetActiveForMoreThan(2,GLITTER_TO_PULSE_CHANGE_TIME))
	{
		if(isPulseMode)
		{
			//Switch to glitter mode
			pulse.mode = LEDSEG_MODE_GLITTER_BOUNCE;
			pulse.cycles =0;
			pulse.ledsFadeAfter = 0;
			pulse.ledsFadeBefore = 5;
			pulse.ledsMaxPower = 150;
			pulse.pixelTime = 2000;
			pulse.pixelsPerIteration = 5;
			pulse.startDir =1;
			pulse.startLed = 1;
			isPulseMode=false;
		}
		else
		{
			//Switch to pulse mode
			pulse.mode=LEDSEG_MODE_LOOP_END;
			pulse.cycles =0;
			pulse.ledsFadeAfter = 10;
			pulse.ledsFadeBefore = 10;
			pulse.ledsMaxPower = 20;
			pulse.pixelTime = PULSE_NORMAL_PIXEL_TIME;
			pulse.pixelsPerIteration = 3;
			pulse.startDir =1;
			pulse.startLed = 1;
			isPulseMode=true;
		}
		ledSegSetFade(LEDSEG_ALL,&fade);
		ledSegSetPulse(LEDSEG_ALL,&pulse);
		ledSegSetPulseActiveState(LEDSEG_ALL,pulseIsActive);
	}
	//Handle special modes
	switch(smode)
	{
		case SMODE_DISCO:
		{
			if(systemTime>nextDiscoUpdate)
			{
				nextDiscoUpdate=systemTime+FADE_FAST_TIME;
				animLoadLedSegFadeColour(SIMPLE_COL_RANDOM,&fade,100,200);
				animLoadLedSegPulseColour(SIMPLE_COL_RANDOM,&pulse,200);
				ledSegSetFade(LEDSEG_ALL,&fade);
				ledSegSetPulse(LEDSEG_ALL,&pulse);
				ledSegSetPulseActiveState(LEDSEG_ALL,pulseIsActive);
			}
			break;
		}
		default:
		{
			//Do nothing for default
			break;
		}
	}
	//End of simple main loop mode handling
}

//The different battery levels in mV
//The first is the lowest battery level
#define NOF_BATTERY_LEVELS	5
#define BATTERY_LEVEL_RED_PWR	250
const uint16_t batteryLevels[NOF_BATTERY_LEVELS] ={3300,3500,3700,3900,4100};
/*
 * Displays the battery state on a given segment with a given start LED (takes a total of 5 LEDs)
 */
void displayBattery(uint8_t segment, uint16_t startLED)
{
	volatile uint16_t voltage=0;
	voltage=adcGetBatVolt();
	for(uint8_t i=0;i<NOF_BATTERY_LEVELS;i++)
	{
		if(voltage>batteryLevels[i])
		{
			ledSegSetLed(segment,startLED+i,0,BATTERY_LEVEL_RED_PWR/2,0);
		}
		else
		{
			ledSegSetLed(segment,startLED+i,BATTERY_LEVEL_RED_PWR,0,0);
		}
	}
}

#define LED_DUMMY_NOF_STATES	4
/*
 * Dummy task to blink LEDs
 */
static void dummyLedTask()
{
	static uint32_t nextCallTime=0;
	static uint8_t state=0;
	//This is a debug thingy, to test the LED
	volatile S_RGB_LED led;
	led.red=500;
	led.green=0;
	led.blue=0;
	if(systemTime>nextCallTime)
	{
		nextCallTime=systemTime+250;
		switch(state)
		{
			case 0:
			{
				led.red=0;
				led.green=0;
				led.blue=0;
				break;
			}
			case 1:
			{
				led.red=500;
				led.green=0;
				led.blue=0;
				break;
			}
			case 2:
			{
				led.red=0;
				led.green=500;
				led.blue=0;
				break;
			}
			case 3:
			{
				led.red=0;
				led.green=0;
				led.blue=500;
				break;
			}
		}
		onboardLedCtrlWriteColours(led);
		state++;
		if(state>=LED_DUMMY_NOF_STATES)
		{
			state=0;
		}
	}
}

static volatile bool mutex=false;
#define OS_NOF_TASKS 6
/*
 * Semi-OS, used for tasks that are not extremely time critical and might take a while to perform
 */
bool poorMansOS()
{
	static uint8_t task=0;
	if(mutex)
	{
		return false;
	}
	mutex=true;
	switch(task)
	{
		case 0:
			handleApplicationSimple();
		break;
		case 1:
			ledSegRunIteration();
		break;
		case 2:
			swDebounceTask();
		break;
		case 3:
			dummyLedTask();
		break;
		case 4:
			mpu6050Task();
		break;
		case 5:
			animTask();
		break;
	}
	task++;
	if(task>=OS_NOF_TASKS)
	{
		task=0;
	}
	mutex=false;
	return true;
}

/*
 * Runs a full iteration of all tasks in the "OS"
 */
void poorMansOSRunAll()
{
	for(uint8_t i=0;i<=OS_NOF_TASKS;i++)
	{
		while(!poorMansOS()){}
	}
}
#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
