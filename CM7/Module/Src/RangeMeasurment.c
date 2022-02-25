/*
 * RangeMeasurment.c
 *
 *  Created on: 21 lut 2022
 *      Author: Daniel
 */
#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "RangeMeasurment.h"

static const volatile  void (*RangeMeasurment_StatusFunction[])(rangeMeasurmentStruct *me)={
		RangeMeasurment_IdleFunction,
		RangeMeasurment_PostitionChangingFunction,
		RangeMeasurment_MeasurmentDistanceFunction,
		RangeMeasurment_EndRangeFunction,
};
rangeMeasurmentStruct* RangeMeasurment_Create(){
	rangeMeasurmentStruct* me=malloc(sizeof(rangeMeasurmentStruct));
	if(me!=NULL){
		me->status=rangeMeasurmentIdle;
		me->Vl53l0xDistance=0;
		me->Vl53l0xLockValue=RESET;
		me->hcSr04Distance=0;
		me->hcSr04LockValue=RESET;
		me->servoPRPosition=0;
		me->time=0;
		me->servoPR=NULL;
		me->HcSr04=NULL;
		me->vl53l0x=NULL;
		RangeMeasurment_Init(me,
				RangeMeasurment_CreateHcSr04,
				RangeMeasurment_CreateVl54l0x,
				RangeMeasurment_CreateServoPR,
				RangeMeasurment_RangeMeasurment,
				RangeMeasurment_GetVl53l0xDistance,
				RangeMeasurment_GetHcSr04Distance,
				RangeMeasurment_GetServoPRPosition,
				RangeMeasurment_TimeInterrupt,
				RangeMeasurment_HcSr04Interrupt,
				RangeMeasurment_Main,
				RangeMeasurment_isRangeMeasurmentEnd);
	}
	return me;
}
void RangeMeasurment_Init(rangeMeasurmentStruct* me,
						void (*createHcSr04)(rangeMeasurmentStruct* me,TIM_HandleTypeDef* timerHcSr04),
						void (*createVl54l0x)(rangeMeasurmentStruct *me,I2C_HandleTypeDef * hi2cVl53l0,GPIO_TypeDef *xshutgpio,uint16_t xshutpin),
						void (*createServoPR)(rangeMeasurmentStruct *me,TIM_HandleTypeDef *PWMtimerGen,uint32_t TimChannel),
						uint8_t (*rangeMeasurment)(rangeMeasurmentStruct *me,uint8_t position),
						uint16_t* (*getVl53l0xDistance)(rangeMeasurmentStruct *me),
						uint32_t* (*getHcSr04Distance)(rangeMeasurmentStruct *me),
						uint8_t* (*getServoPRPosition)(rangeMeasurmentStruct *me),
						void (*timeInterrupt)(rangeMeasurmentStruct *me),
						void (*HcSr04Interrupt)(rangeMeasurmentStruct *me,TIM_HandleTypeDef* htim),
						void (*main)(rangeMeasurmentStruct *me),
						uint8_t (*isRangeMeasurmentEnd)(rangeMeasurmentStruct *me)){
	me->createHcSr04=createHcSr04;
	me->createVl54l0x=createVl54l0x;
	me->createServoPR=createServoPR;
	me->rangeMeasurment=rangeMeasurment;
	me->getVl53l0xDistance=getVl53l0xDistance;
	me->getHcSr04Distance=getHcSr04Distance;
	me->getServoPRPosition=getServoPRPosition;
	me->timeInterrupt=timeInterrupt;
	me->HcSr04Interrupt=HcSr04Interrupt;
	me->main=main;
	me->isRangeMeasurmentEnd=isRangeMeasurmentEnd;
}

void RangeMeasurment_CreateHcSr04(rangeMeasurmentStruct* me,TIM_HandleTypeDef* timerHcSr04){
	me->HcSr04=HcSr04_Create(timerHcSr04);
}
void RangeMeasurment_CreateVl54l0x(rangeMeasurmentStruct *me,I2C_HandleTypeDef * hi2cVl53l0,GPIO_TypeDef *xshutgpio,uint16_t xshutpin){
	me->vl53l0x=vl53l0x_Create(hi2cVl53l0,xshutgpio,xshutpin);
}
void RangeMeasurment_CreateServoPR(rangeMeasurmentStruct *me,TIM_HandleTypeDef *PWMtimerGen,uint32_t TimChannel){
	me->servoPR=servoPR_Create(PWMtimerGen, TimChannel);
}
uint8_t RangeMeasurment_RangeMeasurment(rangeMeasurmentStruct *me,uint8_t position){
	if(me->status==rangeMeasurmentIdle || me->status==rangeMeasurmentEnd){
		if(me->servoPR->isReady(me->servoPR)){
			me->servoPR->setPosition(me->servoPR,position);
			me->Vl53l0xLockValue=RESET;
			me->hcSr04LockValue=RESET;
			me->time=0;
			me->servoPRPosition=position;
			me->status=rangeMeasurmentPostitionChanging;
			return SET;
		}
	}
	return RESET;
}
uint16_t* RangeMeasurment_GetVl53l0xDistance(rangeMeasurmentStruct *me){
	if(me->status==rangeMeasurmentIdle || me->status==rangeMeasurmentEnd){
		me->Vl53l0xLockValue=RESET;
		return &me->Vl53l0xDistance;
	}
	return NULL;
}
uint32_t* RangeMeasurment_GetHcSr04Distance(rangeMeasurmentStruct *me){
	if(me->status==rangeMeasurmentIdle || me->status==rangeMeasurmentEnd){
			me->hcSr04LockValue=RESET;
			return &me->hcSr04Distance;
	}
	return NULL;
}
uint8_t* RangeMeasurment_GetServoPRPosition(rangeMeasurmentStruct *me){
	return &me->servoPRPosition;
}
void RangeMeasurment_TimeInterrupt(rangeMeasurmentStruct *me){
	me->time++;
	me->vl53l0x->increaseTime(me->vl53l0x);
	me->servoPR->timerToInterrupt(me->servoPR);
}
void RangeMeasurment_HcSr04Interrupt(rangeMeasurmentStruct *me,TIM_HandleTypeDef* htim){
	me->HcSr04->htimInterrupt(me->HcSr04,htim);
}
void RangeMeasurment_Main(rangeMeasurmentStruct *me){
	RangeMeasurment_StatusFunction[me->status](me);
	me->servoPR->controlFunction(me->servoPR);
}
uint8_t RangeMeasurment_isRangeMeasurmentEnd(rangeMeasurmentStruct *me){
	if(me->status==rangeMeasurmentIdle || me->status==rangeMeasurmentEnd){
		return SET;
	}
		return RESET;
}
volatile void RangeMeasurment_IdleFunction(rangeMeasurmentStruct *me){}
void RangeMeasurment_PostitionChangingFunction(rangeMeasurmentStruct *me){
	if(me->time>RANGEMEASURMENT_TIME_FOR_ERROR_SERVOPR){
		//TODO dodać error
		me->status=rangeMeasurmentIdle;
		me->time=0;
	}
	if(me->servoPR->isReady(me->servoPR)){
			if(me->HcSr04->isReady && me->vl53l0x->isReady){
				me->vl53l0x->startSingleMeasurment(me->vl53l0x);
				me->HcSr04->startMeasurment(me->HcSr04);
				me->status=rangeMeasurmentDistance;
				me->time=0;
			}else{
				me->HcSr04->getMeasurment(me->HcSr04);
				me->vl53l0x->getDistance(me->vl53l0x);
			}
	}
}
void RangeMeasurment_MeasurmentDistanceFunction(rangeMeasurmentStruct *me){
	if(me->time>RANGEMEASURMENT_TIME_FOR_ERROR_RANGE_MEASURMENT){
			//TODO dodać error
			me->status=rangeMeasurmentIdle;
			me->time=0;
			if(me->hcSr04LockValue==RESET){
				me->hcSr04Distance=0;
				me->hcSr04LockValue=SET;
			}
			if(me->Vl53l0xLockValue==RESET){
				me->Vl53l0xDistance=0;
				me->Vl53l0xLockValue=SET;
			}
	}
		//Pomiar odleglości
	if(me->hcSr04LockValue==RESET){
		uint32_t *pointeHcSr04=me->HcSr04->getMeasurment(me->HcSr04);
		if(pointeHcSr04){
			me->hcSr04Distance=*pointeHcSr04;
			me->hcSr04LockValue=SET;
		}
	}

	if(me->Vl53l0xLockValue==RESET){
		uint16_t *pointerVl53l0x=me->vl53l0x->getDistance(me->vl53l0x);
		if(pointerVl53l0x){
			me->Vl53l0xDistance=*pointerVl53l0x;
			me->Vl53l0xLockValue=SET;
		}
	}
		//Zakonczenie pomiaru
	if(me->hcSr04LockValue==SET && me->Vl53l0xLockValue==SET){
		me->status=rangeMeasurmentEnd;

	}
}
void RangeMeasurment_EndRangeFunction(rangeMeasurmentStruct *me){
	if(me->hcSr04LockValue==RESET && me->Vl53l0xLockValue==RESET){
			me->status=rangeMeasurmentIdle;
	}
}
