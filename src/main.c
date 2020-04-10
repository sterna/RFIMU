

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

bool poorMansOS();
void poorMansOSRunAll();

#define LED_BOARD_SET()		GPIOC->BRR = GPIO_Pin_13
#define LED_BOARD_CLEAR()	GPIOC->BSRR = GPIO_Pin_13
#define LED_BOARD_TOGGLE()	GPIOC->ODR ^= GPIO_Pin_13

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

typedef enum
{
	MODE_NORMAL=0,
	MODE_CHARGE,	//Carebearstare!
	MODE_LOW_POWER,
	MODE_DISCO,
	MODE_STROBE,
	MODE_PRIDE,
	MODE_OFF,
	MODE_NOF_MODES,
	MODE_HANDBLAST1,
	MODE_HANDBLAST2,
	MODE_ANGRY_RED,
	MODE_STROBE_TERRIBLE
}prog_mode_t;

typedef enum
{
	GMODE_SETUP_FADE=0,	//Just a fade on the LEDs, same colour on all of them
	GMODE_SETUP_PULSE,	//Pulses and fades
	GMODE_PAUSE,		//Stops all LEDs as they are
	GMODE_NOF_MODES
}gaurianMode_t;

typedef enum
{
	DISCO_COL_PURPLE=0,
	DISCO_COL_CYAN,
	DISCO_COL_YELLOW,
	DISCO_COL_WHITE,
	DISCO_COL_RED,
	DISCO_COL_GREEN,
	DISCO_COL_BLUE,
	DISCO_COL_RANDOM,
	DISCO_COL_OFF,
	DISCO_COL_NOF_COLOURS
}discoCols_t;

#define DISCO_NOF_COLORS	(DISCO_COL_NOF_COLOURS-2)
/*
 * CH5: legs
 * CH4: right arm
 * CH3: left arm
 * CH2: chest
 * CH1: head
 */
led_fade_setting_t setting_normal={50,150,100,400,100,500,2500,0,0};
led_fade_setting_t setting_charge={200,400,500,800,500,800,750,0,0};
led_fade_setting_t setting_low_power={50,100,100,300,100,300,2500,0,0};
led_fade_setting_t setting_handblast1={50,200,150,600,150,600,250,0,8};
led_fade_setting_t setting_handblast2={100,400,300,900,300,900,250,0,1};
led_fade_setting_t setting_angry_red={50,600,0,0,0,0,750,0,1};

led_fade_setting_t setting_disco[DISCO_NOF_COLORS]=
{
{0,500,0,0,0,500,1000,0,1},	//Purple
{0,0,0,500,0,500,1000,0,1},	//Cyan
{0,600,0,400,0,0,1000,0,1},	//"Yellow"	(Trimmed, a little, since it was very greenish)
{0,500,0,500,0,500,1000,0,1},	//White
{0,500,0,0,0,0,1000,0,1},		//Red
{0,0,0,500,0,0,1000,0,1},		//Green
{0,0,0,0,0,500,1000,0,1}		//Blue
};


led_fade_setting_t setting_strobe[DISCO_NOF_COLORS]=
{
{0,500,0,0,0,500,200,0,1},	//Purple
{0,0,0,500,0,500,200,0,1},	//Cyan
{0,500,0,500,0,0,200,0,1},	//"Yellow"
{0,500,0,500,0,500,200,0,1},	//White
{0,500,0,0,0,0,200,0,1},		//Red
{0,0,0,500,0,0,200,0,1},		//Green
{0,0,0,0,0,500,200,0,1}		//Blue
};

led_fade_setting_t seeting_white_strobe={0,1000,0,1000,0,1000,250,0,0};

#define PRIDE_NOF_COLORS 5
led_fade_setting_t setting_pride[PRIDE_NOF_COLORS]=
{
{20,500,0,0,0,00,1000,0,0},		//Red
{100,600,20,300,0,0,1000,0,0},	//Yellow
{0,0,20,500,0,0,1000,0,0},		//Green
{0,0,0,0,20,500,1000,0,0},		//Blue
{20,500,0,0,20,400,1000,0,0},	//Purple
};

#define PWR_LOW (uint16_t)(250)
#define PWR_MID (uint16_t)(500)
#define PWR_HI (uint16_t)(750)

#define NOF_LEDS 300

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
	SMODE_RANDOM,
	SMODE_DISCO,
	SMODE_BATTERY_DISP,
	SMODE_OFF,
	SMODE_NOF_MODES
}simpleModes_t;

void generateColor(led_fade_setting_t* s);
void loadMode(prog_mode_t mode);
void loadLedSegFadeColour(discoCols_t col,ledSegmentFadeSetting_t* st);
void loadLedSegPulseColour(discoCols_t col,ledSegmentPulseSetting_t* st);
static void dummyLedTask();
void displayBattery(uint8_t channel, uint8_t segment, uint16_t startLED);
void handleApplicationSimple();

#define GLOBAL_SETTING	4
#define UGLY_MODE_CHANGE_TIME	10000

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

static volatile IMUVals_t imuVals;
volatile uint16_t batteryIndicatorStartLed=40;	//There are 156 and 157 LEDs on each side

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
	static bool uglyModeChange=false;
	static uint32_t uglyModeChangeActivateTime=0;
	static uint32_t nextDiscoUpdate=0;
	static uint8_t globalSetting=GLOBAL_SETTING;
	if(!setupDone)
	{
		loadLedSegPulseColour(DISCO_COL_YELLOW,&pulse);
		pulse.cycles =0;
		pulse.ledsFadeAfter = 5;
		pulse.ledsFadeBefore = 5;
		pulse.ledsMaxPower = 15;
		pulse.mode = LEDSEG_MODE_LOOP_END;
		pulse.pixelTime = PULSE_NORMAL_PIXEL_TIME;
		pulse.pixelsPerIteration = 2;
		pulse.startDir =1;
		pulse.startLed = 1;

		loadLedSegFadeColour(DISCO_COL_BLUE,&fade);
		fade.cycles =0;
		fade.mode = LEDSEG_MODE_BOUNCE;
		fade.startDir = -1;
		fade.fadeTime = 700;
		apa102SetDefaultGlobal(4);
		segment1Up=ledSegInitSegment(1,2,45,&pulse,&fade);	//Skip the first LED to sync up the pulses
		segment3Up=ledSegInitSegment(1,90,133,&pulse,&fade);
		pulse.startDir=-1;
		pulse.startLed = 100;
		segment2Down=ledSegInitSegment(1,46,89,&pulse,&fade);
		setupDone=true;
	}

	//Change mode
	if(swGetFallingEdge(1) || uglyModeChange)
	{
		pulseIsActive=true;
		uglyModeChange=false;
		//Handles if something needs to be done when changing from a state
		switch(smode)
		{
		case SMODE_DISCO:
		{
			pulse.pixelTime=PULSE_NORMAL_PIXEL_TIME;
			fade.fadeTime=FADE_NORMAL_TIME;
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
			loadLedSegFadeColour(DISCO_COL_BLUE,&fade);
			loadLedSegPulseColour(DISCO_COL_YELLOW,&pulse);
			break;
		case SMODE_CYAN_FADE_YLW_PULSE:
			loadLedSegFadeColour(DISCO_COL_CYAN,&fade);
			loadLedSegPulseColour(DISCO_COL_YELLOW,&pulse);
			break;
		case SMODE_YLW_FADE_GREEN_PULSE:
			loadLedSegFadeColour(DISCO_COL_YELLOW,&fade);
			loadLedSegPulseColour(DISCO_COL_GREEN,&pulse);
			break;
		case SMODE_RED_FADE_YLW_PULSE:
			loadLedSegFadeColour(DISCO_COL_RED,&fade);
			loadLedSegPulseColour(DISCO_COL_YELLOW,&pulse);
			break;
		case SMODE_YLW_FADE_PURPLE_PULSE:
			loadLedSegFadeColour(DISCO_COL_YELLOW,&fade);
			loadLedSegPulseColour(DISCO_COL_PURPLE,&pulse);
			break;
		case SMODE_CYAN_FADE_NO_PULSE:
			loadLedSegFadeColour(DISCO_COL_CYAN,&fade);
			pulseIsActive=false;
			break;
		case SMODE_YLW_FADE_NO_PULSE:
			loadLedSegFadeColour(DISCO_COL_YELLOW,&fade);
			pulseIsActive=false;
			break;
		case SMODE_RED_FADE_NO_PULSE:
			loadLedSegFadeColour(DISCO_COL_RED,&fade);
			pulseIsActive=false;
			break;
		case SMODE_DISCO:
			pulse.pixelTime=PULSE_FAST_PIXEL_TIME;
			fade.fadeTime=FADE_FAST_TIME;	//The break is omitted by design, since SMODE_DISCO does the same thing as SMODE_RANDOM
		case SMODE_RANDOM:
			loadLedSegFadeColour(DISCO_COL_RANDOM,&fade);
			loadLedSegPulseColour(DISCO_COL_RANDOM,&pulse);
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
		ledSegSetFade(segment1Up,&fade);
		ledSegSetFade(segment2Down,&fade);
		ledSegSetFade(segment3Up,&fade);
		pulse.startDir=1;
		pulse.startLed =1;
		ledSegSetPulse(segment1Up,&pulse);
		ledSegSetPulse(segment3Up,&pulse);
		pulse.startDir=-1;
		pulse.startLed =100;
		ledSegSetPulse(segment2Down,&pulse);
		ledSegSetPulseActiveState(segment1Up,pulseIsActive);
		ledSegSetPulseActiveState(segment2Down,pulseIsActive);
		ledSegSetPulseActiveState(segment3Up,pulseIsActive);

		if(smode == SMODE_BATTERY_DISP)
		{
			//Pause The other segment (possible arm)
			//Set LEDs in the correct place to the right colours, corresponding to battery level
			ledSegSetPulseActiveState(segment1Up,false);
			ledSegSetFadeActiveState(segment1Up,false);
			displayBattery(1,segment1Up,batteryIndicatorStartLed);
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
		apa102SetDefaultGlobal(globalSetting*3);//)APA_MAX_GLOBAL_SETTING);
		ledSegRestart(segment1Up,true,true);
		ledSegRestart(segment2Down,true,true);
		ledSegRestart(segment3Up,true,true);
	}
	if(swGetFallingEdge(2))
	{
		apa102SetDefaultGlobal(globalSetting);
	}
	if(swGetActiveForMoreThan(2,UGLY_MODE_CHANGE_TIME))
	{
		apa102SetDefaultGlobal(globalSetting);
		uglyModeChange=true;
	}
	//Handle special modes
	switch(smode)
	{
		case SMODE_DISCO:
		{
			if(systemTime>nextDiscoUpdate)
			{
				nextDiscoUpdate=systemTime+FADE_FAST_TIME;
				loadLedSegFadeColour(DISCO_COL_RANDOM,&fade);
				loadLedSegPulseColour(DISCO_COL_RANDOM,&pulse);
				ledSegSetFade(segment1Up,&fade);
				ledSegSetFade(segment2Down,&fade);
				ledSegSetFade(segment3Up,&fade);
				pulse.startDir=1;
				pulse.startLed =1;
				ledSegSetPulse(segment1Up,&pulse);
				ledSegSetPulse(segment3Up,&pulse);
				pulse.startDir=-1;
				pulse.startLed =100;
				ledSegSetPulse(segment2Down,&pulse);
				ledSegSetPulseActiveState(segment1Up,pulseIsActive);
				ledSegSetPulseActiveState(segment2Down,pulseIsActive);
				ledSegSetPulseActiveState(segment3Up,pulseIsActive);
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
void displayBattery(uint8_t channel, uint8_t segment, uint16_t startLED)
{
	volatile uint16_t voltage=0;
	voltage=adcGetBatVolt(channel);
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

#define MAX_DIVISOR	4
#define MIN_MAX_DIVISOR	3

/*
 * Load new colours for a given ledFadeSegment
 * Todo: Make sure this doesn't fuck up the fade timing (which it does now)
 */
void loadLedSegFadeColour(discoCols_t col,ledSegmentFadeSetting_t* st)
{
	led_fade_setting_t tmpSet;
	if(col==DISCO_COL_RANDOM)
	{
		tmpSet=setting_disco[(utilRandRange(DISCO_NOF_COLORS-1))];
	}
	else if(col==DISCO_COL_OFF)
	{
		tmpSet.r_min=0;
		tmpSet.g_min=0;
		tmpSet.b_min=0;
		tmpSet.r_max=0;
		tmpSet.g_max=0;
		tmpSet.b_max=0;
	}
	else if(col<DISCO_COL_RANDOM)
	{
		tmpSet = setting_disco[col];
	}
	st->r_max = tmpSet.r_max/MAX_DIVISOR;
	st->r_min = st->r_max/MIN_MAX_DIVISOR;
	st->g_max = tmpSet.g_max/MAX_DIVISOR;
	st->g_min = st->g_max/MIN_MAX_DIVISOR;
	st->b_max = tmpSet.b_max/MAX_DIVISOR;
	st->b_min = st->b_max/MIN_MAX_DIVISOR;
}

/*
 * Load new colours for a given ledFadeSegment
 */
void loadLedSegPulseColour(discoCols_t col,ledSegmentPulseSetting_t* st)
{
	led_fade_setting_t tmpSet;	//Yes, it's correct using a fade setting
	if(col==DISCO_COL_RANDOM)
	{
		tmpSet=setting_disco[(utilRandRange(DISCO_NOF_COLORS-1))];
	}
	else if(col==DISCO_COL_OFF)
	{
		tmpSet.r_max=0;
		tmpSet.g_max=0;
		tmpSet.b_max=0;
	}
	else if(col<DISCO_COL_RANDOM)
	{
		tmpSet = setting_disco[col];
	}
	st->r_max = tmpSet.r_max/4;
	st->g_max = tmpSet.g_max/4;
	st->b_max = tmpSet.b_max/4;
}

#define MODE_HANDLER_CALL_PERIOD	25

//Some switches might have more than one function depending on mode
#define SW_MODE			1
#define SW_COL_UP		2
#define SW_PAUSE		2
#define SW_SETUP_SYNC 	3
#define SW_ONOFF		4
/*
 * The "main" loop of the program. Will read the buttons and change modes accordingly
 * Called from poorManOS
 *
 * General idea for control modes:
 * Modes:
 * 		Set fade colour
 * 			SW1, beat synchronizer, if active. Otherwise, cycle mode.
 * 			SW2, colour fade up (we have a random colour, an off colour, and it will loop)
 * 			SW3, beat synch start/stop.
 * 			SW4, toggle fade on/off
 * 		Set pulse colour
 * 			SW1, beat synchronizer, if active. Otherwise, cycle mode.
 * 			SW2, colour fade up (we have a random colour, an off colour, and it will loop)
 * 			SW3, beat synch start/stop.
 * 			SW4, toggle pulse on/off
 * 		Pause
 * 			SW1, Cycle mode
 * 			SW2, freeze/unfreeze colour
 *
 *	Colours are fetched from the disco_led_fade struct (use max as is, divided by 4. Use min as new max divided by 3)
 *	If colour is set to DISCO_NOF_COLOURS, a random colour will be used
 */
void handleModes()
{
	static uint32_t nextCallTime=0;

	//Since the setting cannot remember the "simple colours" (or it's hard to extract them), we keep track of them here
	static discoCols_t fadeColour=0;
	static discoCols_t pulseColour=0;

	static uint32_t tmpSynchPeriod=1000;
	static uint32_t synchPeriodLastTime=0;
	static bool synchMode=false;
	static gaurianMode_t mode=GMODE_SETUP_FADE;
	static bool isPaused=false;

	if(systemTime<nextCallTime)
	{
		return;
	}
	nextCallTime=systemTime+MODE_HANDLER_CALL_PERIOD;

	//Temp variables used to contain various settings
	ledSegmentFadeSetting_t* fdSet;
	ledSegmentPulseSetting_t* puSet;
	ledSegmentState_t st;
	ledSegGetState(segmentTail,&st);
	fdSet=&(st.confFade);
	puSet=&(st.confPulse);
	//Check if we should change mode
	if(synchMode == false && swGetFallingEdge(SW_MODE))
	{
		mode++;
		if(mode>=GMODE_NOF_MODES)
		{
			mode=0;	//Because 0 will always be the first mode
		}
	}

	//Start of sync mode handling
	if(swGetFallingEdge(SW_SETUP_SYNC))
	{
		//Handle synch mode
		if(synchMode)
		{
			synchMode=false;
		}
		else
		{
			//Start synch mode
			synchMode=true;
			tmpSynchPeriod=0;
			synchPeriodLastTime=0;
		}
	}
	if(synchMode && swGetRisingEdge(SW_MODE))
	{
		//New beat setup
		if(synchPeriodLastTime)
		{
			if(tmpSynchPeriod==0)
			{
				tmpSynchPeriod=systemTime-synchPeriodLastTime;
			}
			else
			{
				tmpSynchPeriod=(tmpSynchPeriod+(systemTime-synchPeriodLastTime))/2;
			}
		}
		synchPeriodLastTime=systemTime;
	}
	//Write colours during sync mode
	if(synchMode && tmpSynchPeriod)
	{
		switch(mode)
		{
		case GMODE_SETUP_FADE:
		{
			fdSet->fadeTime = tmpSynchPeriod;
			ledSegSetFade(segmentTail,fdSet);
			break;
		}
		case GMODE_SETUP_PULSE:
		{
			//Todo: find out what good analogy shall be used for pulse with beat detection
			fdSet->fadeTime = tmpSynchPeriod;
			ledSegSetFade(segmentTail,fdSet);
			break;
		}
		}
		//End synch mode?
		if(swGetFallingEdge(SW_SETUP_SYNC))
		{
			synchMode=false;
		}
	}
	//End of synch mode handling

	//Change colour
	if(swGetFallingEdge(SW_COL_UP))
	{
		switch(mode)
		{
		case GMODE_SETUP_FADE:
		{
			fadeColour=utilIncAndWrapTo0(fadeColour,DISCO_NOF_COLORS);
			loadLedSegFadeColour(fadeColour,fdSet);
			break;
		}
		case GMODE_SETUP_PULSE:
		{
			pulseColour=utilIncAndWrapTo0(pulseColour,DISCO_NOF_COLORS);
			if(pulseColour==DISCO_COL_OFF)
			{
				ledSegSetPulseActiveState(segmentTail,false);
			}
			else
			{
				ledSegSetPulseActiveState(segmentTail,true);	//OK to do every time, since whatever checks it doesn't care about state changes
				loadLedSegPulseColour(pulseColour,puSet);
			}
			break;
		}
		}
	}

	//Pause handling (will just freeze the segments)
	if(mode==GMODE_PAUSE && swGetFallingEdge(SW_PAUSE))
	{
		if(isPaused)
		{
			//Start fade and pulse
			ledSegSetFadeActiveState(segmentTail,true);
			ledSegSetPulseActiveState(segmentTail,true);
			isPaused=false;
		}
		else
		{
			//Stop fade and pulse
			ledSegSetFadeActiveState(segmentTail,false);
			ledSegSetPulseActiveState(segmentTail,false);
			isPaused=true;
		}
	}
}

/*
 * Generate a random color (most likely it looks white...)
 */
#define COLOR_MAX 500
void generateColor(led_fade_setting_t* s)
{
	s->r_max = utilRandRange(COLOR_MAX);
	s->g_max = utilRandRange(COLOR_MAX);
	s->b_max = utilRandRange(COLOR_MAX);
}



void loadMode(prog_mode_t mode)
{
	switch(mode)
	{
	case MODE_NORMAL:
		ledFadeSetup(&setting_normal,0);
		break;
	case MODE_CHARGE:
		ledFadeSetup(&setting_charge,0);
		break;
	case MODE_LOW_POWER:
		ledFadeSetup(&setting_low_power,0);
		break;
	case MODE_DISCO:
		ledFadeSetup(&(setting_disco[0]),0);
		break;
	case MODE_STROBE:
		ledFadeSetup(&(setting_strobe[0]),0);
		break;
	case MODE_PRIDE:
		for (uint8_t i=1;i<=LED_PWM_NOF_CHANNELS;i++)
		{
			ledFadeSetup(&(setting_pride[i-1]),i);
		}
		break;
	case MODE_OFF:
		//Clear all channels
		ledPwmUpdateColours(0,0,0,0);
		ledFadeSetActive(0,false);
		break;
	case MODE_STROBE_TERRIBLE:
	case MODE_NOF_MODES:
	case MODE_HANDBLAST1:
	case MODE_HANDBLAST2:
	case MODE_ANGRY_RED:

		//This should actually never happen (but impressive that gcc notices this)
		break;
	}
}

static volatile bool mutex=false;
#define OS_NOF_TASKS 5
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
