/*
 * servoPR.c
 *
 *  Created on: 18 lut 2022
 *      Author: Daniel
 */
#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "ServoPR.h"

static uint16_t servoPR_PwmPeriod[]={ //13 pozycji
		1000,	//0 stopni
		1417,	//15 stopni
		1833,	//30 stopni
		2250,	//45 stopni
		2667,	//60 stopni
		3083,	//75 stopni
		3500,	//90 stopni
		3916,	//105 stopni
		4333,	//120 stopni
		4750,	//135 stopni
		5167,	//150 stopni
		5583,	//165 stopni
		6000	//180 stopni
};

static void (*servoPR_StatusFunction[])(servoPRStruct *me)={
		servoPR_IdleFuction,
		servoPR_WaitToStabilizeServoFunction,
		servoPR_ErrorFunction
};

servoPRStruct* servoPR_Create(TIM_HandleTypeDef *PWMtimerGen,uint32_t TimChannel){
	servoPRStruct* me=(servoPRStruct*) malloc(sizeof(servoPRStruct));
	if(me!=NULL){
		me->PWMtimerGen=PWMtimerGen;
		me->TimChannel=TimChannel;
		me->status=servoPR_IDLE;
		me->position=0;
		me->timeToStabilizedServo=0;
		servoPR_Init(me,
				servoPR_SetPosition,
				servoPR_GetPosition,
				servoPR_Reset,
				servoPR_IsReady,
				servoPR_TimerToInterrupt,
				servoPR_ControlFunction);
		me->setPosition(me,6);
		HAL_TIM_PWM_Start(me->PWMtimerGen,me->TimChannel);
		HAL_TIM_Base_Start(me->PWMtimerGen);
	}
	return me;
}

void servoPR_Init(servoPRStruct *me,
				uint8_t (*setPosition)(servoPRStruct *me,uint8_t numberPosition),
				uint8_t (*getPosition)(servoPRStruct *me),
				void (*resetServo)(servoPRStruct *me),
				uint8_t (*isReady)(servoPRStruct *me),
				void (*timerToInterrupt)(servoPRStruct *me),
				void (*controlFunction)(servoPRStruct *me)){
	me->setPosition=setPosition;
	me->getPosition=getPosition;
	me->resetServo=resetServo;
	me->isReady=isReady;
	me->timerToInterrupt=timerToInterrupt;
	me->controlFunction=controlFunction;
}
uint8_t servoPR_SetPosition(servoPRStruct *me,uint8_t numberPosition){
	uint8_t maxPosition=sizeof(servoPR_PwmPeriod)/sizeof(uint16_t);
	if(numberPosition>=maxPosition){
		addErrorValue(SERVOPR_OutOfMaxServoPosition);
		return RESET;
	}
	if(me->status==servoPR_IDLE){
		uint8_t prevPosition=me->position;
		if(prevPosition==numberPosition){
			return SET;
		}
		int  difference=prevPosition-numberPosition;
		uint8_t absDifference=abs(difference);
		me->timeToStabilizedServo=absDifference*SERVOPR_TIME_FOR_OPERATING_SPEED;
		me->position=numberPosition;
		__HAL_TIM_SET_COMPARE(me->PWMtimerGen,me->TimChannel,servoPR_PwmPeriod[numberPosition]);
		me->status=servoPR_WAIT_TO_STABILIZE_SERVO;
		return SET;
	}
	return RESET;
}
inline uint8_t servoPR_GetPosition(servoPRStruct *me){
	return me->position;
}
void servoPR_Reset(servoPRStruct *me){
	me->status=servoPR_IDLE;
	me->position=0;
	me->timeToStabilizedServo=0;
	me->setPosition(me,6);
}

void servoPR_TimerToInterrupt(servoPRStruct *me){
	if(me->timeToStabilizedServo>0){
		me->timeToStabilizedServo--;
	}
}
void servoPR_ControlFunction(servoPRStruct *me){
	servoPR_StatusFunction[me->status](me);
}
uint8_t servoPR_IsReady(servoPRStruct *me){
	if(me->status==servoPR_IDLE){
		return SET;
	}else{
		return RESET;
	}
}
void servoPR_IdleFuction(servoPRStruct *me){

}
void servoPR_WaitToStabilizeServoFunction(servoPRStruct *me){
	if(me->timeToStabilizedServo==0){
		me->status=servoPR_IDLE;
	}
}
void servoPR_ErrorFunction(servoPRStruct *me){

}
