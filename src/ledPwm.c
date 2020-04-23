/*
 *	ledPwm.c
 *
 *	Created on: 6 feb. 2016
 *		Author: Sterna
 */

#if 0
#include "ledPwm.h"

static led_fade_state_t* getLedState(uint8_t ch);

led_fade_state_t ledFadeCh1;
led_fade_state_t ledFadeCh2;
led_fade_state_t ledFadeCh3;
led_fade_state_t ledFadeCh4;
led_fade_state_t ledFadeCh5;

void ledPwmInit()
{
	SystemCoreClockUpdate();
	//Init LED PWM
	GPIO_InitTypeDef GPIO_InitStruct;

	/*
	 * Using the led driver shield:
	 * CH1_R, G, B = A7, B0, B1 -> TIM3_CH2, CH3, CH4
	 * CH2_R, G, B = A1, A2, A3 -> TIM2_CH2, CH3, CH4
	 * CH3_R, G, B = B6, B7, B8 -> TIM4_CH1, CH2, CH3
	 * CH4_R, G, B = B9, A0, A6 -> TIM4_CH4, TIM2_CH1, TIM3_CH1
	 * CH5_R, G, B, W = A10, A8, A11, A9 -> TIM1_CH3, CH1, CH4, CH2 (This should maybe be changed, so that W->R, R->G, G->B, W->B)
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
	GPIO_InitStruct.GPIO_Mode= GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init(GPIOB,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7 | \
			 GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

	//Init timer (TIM4)
	TIM_TimeBaseInitTypeDef TIMBaseInitStruct;
	TIM_OCInitTypeDef TIMOCInitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

	//Setup timer 1, 2, 3 and 4
	TIMBaseInitStruct.TIM_Period = 1000;	//
	TIMBaseInitStruct.TIM_Prescaler = 72;	//Gives about 1Khz
	TIMBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIMBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1,&TIMBaseInitStruct);
	TIM_TimeBaseInit(TIM2,&TIMBaseInitStruct);
	TIM_TimeBaseInit(TIM3,&TIMBaseInitStruct);
	TIM_TimeBaseInit(TIM4,&TIMBaseInitStruct);

	//Same setting for all timer channels
	TIM_OCStructInit(&TIMOCInitStruct);
	TIMOCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIMOCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIMOCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIMOCInitStruct.TIM_Pulse = 0;
	TIM_OC1Init(TIM1,&TIMOCInitStruct);
	TIM_OC2Init(TIM1,&TIMOCInitStruct);
	TIM_OC3Init(TIM1,&TIMOCInitStruct);
	TIM_OC4Init(TIM1,&TIMOCInitStruct);
	TIM_OC1Init(TIM2,&TIMOCInitStruct);
	TIM_OC2Init(TIM2,&TIMOCInitStruct);
	TIM_OC3Init(TIM2,&TIMOCInitStruct);
	TIM_OC4Init(TIM2,&TIMOCInitStruct);
	TIM_OC1Init(TIM3,&TIMOCInitStruct);
	TIM_OC2Init(TIM3,&TIMOCInitStruct);
	TIM_OC3Init(TIM3,&TIMOCInitStruct);
	TIM_OC4Init(TIM3,&TIMOCInitStruct);
	TIM_OC1Init(TIM4,&TIMOCInitStruct);
	TIM_OC2Init(TIM4,&TIMOCInitStruct);
	TIM_OC3Init(TIM4,&TIMOCInitStruct);
	TIM_OC4Init(TIM4,&TIMOCInitStruct);

	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	//Clear all PWMs
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;
	TIM4->CCR4 = 0;

	TIM_Cmd(TIM1,ENABLE);
	TIM_Cmd(TIM2,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
	TIM_Cmd(TIM4,ENABLE);
}


/*
 * Clear the led channel
 * chan=0 clear all channels
 */
void ledPwmClear(uint8_t chan)
{
	ledPwmUpdateColours(0,0,0,chan);

}

//Chan=0 updates all channels
//Value for each colour is 0-1000
void ledPwmUpdateColours(uint16_t r,uint16_t g,uint16_t b, uint8_t chan)
{
	switch(chan)
	{
		case 0:
			for(uint8_t i=1;i<=LED_PWM_NOF_CHANNELS;i++)
			{
				ledPwmUpdateColours(r,g,b,i);
			}
		break;
		case 1:
			R1_PWM=r;
			G1_PWM=g;
			B1_PWM=b;
		break;
		case 2:
			R2_PWM=r;
			G2_PWM=g;
			B2_PWM=b;
		break;
		case 3:
			R3_PWM=r;
			G3_PWM=g;
			B3_PWM=b;
		break;
		case 4:
			R4_PWM=r;
			G4_PWM=g;
			B4_PWM=b;
		break;
		case 5:
			R5_PWM=r;
			G5_PWM=g;
			B5_PWM=b;
		break;
	}
}

/*
 * Bmax=1000,Bmin=200
 * GMax=900,Bmin=200
 * RMax=300,RMin=50
 *
 * BSteps = 800
 * Gstep=700
 * rsteps=250
 *
 * masterSteps: 50
 * Brate=bsteps/masterSteps = 800/50=16
 * Grate=gsteps/masterSteps = 700/50=14
 * Rrate=rsteps/masterSteps = 250/50=5
 *
 * Parametrar: min/max för rgb
 * FadeTime (total tid från min till max)
 * DelayTime (fast på 5 ms eller nåt)
 *
 * FadeSteps = FadeTime/DelayTime
 *
 * interna states: värde av r,g,b
 * min/max för rgb
 * rate för rgb
 * dir (styrs av b bestämmer jag)
 *
 */

//Bad fix to cancel PWM with switch
#include "sw.h"

/*
 * Currently only supports all channels simul
 * This function has no protection
 * cycles states how many complete cycles to do (1 cycle = fade from min->max, next cycle is max->min) if cycles=0, it will not stop
 *
 *
 * r_min=100
 * r_max=200
 * r_diff=100
 * time=10000
 * delay=20
 * master_steps=100
 * r_rate=r_diff/master_steps=1
 */

/*
 * New fade thingy: Call this every fade-delay time
 * This module will keep all channel states and handle all of them at for each iteration
 * Call the "ledSetupChannel" function with a led configuration and a channel number. The led will start immediately
 * There is also a "ledRestartFade"-function, that restarts all active channels
 * This does not support white on ch5 yet
 * If continous fade is needed, set cycles to 0
 * fade_time_ms must be larger than FADE_DELAY_TIME_MS (50 ms)
 */

void ledFadeSetup(led_fade_setting_t* s, uint8_t ch)
{
	if(ch==0)
	{
		while(ch<LED_PWM_NOF_CHANNELS)
		{
			ledFadeSetup(s,++ch);
		}
	}
	led_fade_state_t* st;
	st = getLedState(ch);
	st->conf = *s;	//Todo: This might be the culprit, verify that the correct setting is in the correct channel
	st->cycles = s->cycles;
	uint32_t master_steps=st->conf.fade_time_ms/FADE_DELAY_TIME_MS;
	st->r_rate = (s->r_max-s->r_min)/master_steps;
	if(st->r_rate==0)
	{
		st->r_rate=1;
	}
	st->g_rate = (s->g_max-s->g_min)/master_steps;
	if(st->g_rate==0)
	{
		st->g_rate=1;
	}
	st->b_rate = (s->b_max-s->b_min)/master_steps;
	if(st->b_rate==0)
	{
		st->b_rate=1;
	}
	st->r=s->r_min;
	st->g=s->g_min;
	st->b=s->b_min;
	st->dir = 1;
	//Check if user wants a very large number of cycles
	if((UINT32_MAX/st->cycles)<master_steps)
	{
		st->cycles = 0;
	}
	else
	{
		st->cycles=s->cycles*master_steps;	//Each cycle shall be one half cycle (min->max)
	}
	st->active = true;
}

/*
 * Runs the iteration for a single channel
 * This functions needs to be run at a certain interval
 */
void ledFadeRunIteration(uint8_t ch)
{
	led_fade_state_t* st;
	st = getLedState(ch);
	colour_t compareColor=COL_BLUE;
	if(st->conf.b_max == st->conf.b_min)
	{
		compareColor = COL_GREEN;
		if(st->conf.g_max == st->conf.g_min)
		{
			compareColor = COL_RED;
		}
	}
	if(st->active)
	{
		st->r=utilIncWithDir(st->r,st->dir,st->r_rate,st->conf.r_min,st->conf.r_max);
		st->g=utilIncWithDir(st->g,st->dir,st->g_rate,st->conf.g_min,st->conf.g_max);
		st->b=utilIncWithDir(st->b,st->dir,st->b_rate,st->conf.b_min,st->conf.b_max);

		//Update dir when one existing color is complete
		if(compareColor==COL_BLUE)
		{
			if(st->b>=st->conf.b_max)
			{
				st->dir=-1;
			}
			else if(st->b<=st->conf.b_min)
			{
				st->dir=1;
			}
		}
		else if (compareColor==COL_GREEN)
		{
			if(st->g>=st->conf.g_max)
			{
				st->dir=-1;
			}
			else if(st->g<=st->conf.g_min)
			{
				st->dir=1;
			}
		}
		else	//red
		{
			if(st->r>=st->conf.r_max)
			{
				st->dir=-1;
			}
			else if(st->r<=st->conf.r_min)
			{
				st->dir=1;
			}
		}
		ledPwmUpdateColours(st->r,st->g,st->b,ch);
		//if channel is active and cycles == 0, the channel will never be inactive
		if(st->cycles)
		{
			st->cycles--;
			if(st->cycles==0)
			{
				st->active=false;
				//ledPwmClear(ch);
			}
		}
	}
	else
	{
		ledPwmClear(ch);
	}
}

//Restarts channels from their min val with existing settings
void ledFadeRestart()
{
	led_fade_state_t* st;
	for(uint8_t ch=1;ch<=LED_PWM_NOF_CHANNELS;ch++)
	{
		st = getLedState(ch);
		st->r=st->conf.r_min;
		st->g=st->conf.g_min;
		st->b=st->conf.b_min;
		st->cycles = st->conf.cycles*(st->conf.fade_time_ms/FADE_DELAY_TIME_MS);
		st->dir = 1;
		st->active = true;
	}
}

static led_fade_state_t* getLedState(uint8_t ch)
{
	switch(ch)
	{
	case 1:
		return &ledFadeCh1;
		break;
	case 2:
		return &ledFadeCh2;
		break;
	case 3:
		return &ledFadeCh3;
		break;
	case 4:
		return &ledFadeCh4;
		break;
	case 5:
		return &ledFadeCh5;
		break;
	default:
		return 0;
	}
}

/*
 * Can be used to pause a channel.
 * ch=0 sets the mode on all channels
 */
void ledFadeSetActive(uint8_t ch, bool setting)
{
	if(ch==0)
	{
		while(ch<LED_PWM_NOF_CHANNELS)
		{
			ledFadeSetActive(++ch,setting);
		}
	}
	led_fade_state_t* st;
	st = getLedState(ch);
	st->active=setting;
}

/*
 * If a channel is note active, it means that it's either paused or done fading
 */
bool ledFadeGetState(uint8_t ch)
{
	return getLedState(ch)->active;
}

#endif
