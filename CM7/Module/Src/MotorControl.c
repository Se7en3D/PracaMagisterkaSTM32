/*
 * motorControl.c
 *
 *  Created on: 19 lut 2022
 *      Author: Daniel
 */
#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "MotorsControl.h"

motorsControlStruct* MotorsControl_Create(GPIO_TypeDef *outGPIO,
										uint16_t pinOut1,
										uint16_t pinOut2,
										uint16_t pinOut3,
										uint16_t pinOut4,
										TIM_HandleTypeDef *timerControlMotor,
										uint8_t channelEnA,
										uint8_t channelEnB){
	motorsControlStruct* me=malloc(sizeof(motorsControlStruct));
	if(me!=NULL){
		me->outGPIO=outGPIO;
		me->pinOut1=pinOut1;
		me->pinOut2=pinOut2;
		me->pinOut3=pinOut3;
		me->pinOut4=pinOut4;
		me->timerControlMotor=timerControlMotor;
		me->channelEnA=channelEnA;
		me->channelEnB=channelEnB;
		me->timerPeriod=timerControlMotor->Init.Period;
		me->timerCompareForTurn=me->timerPeriod/10;
		MotorsControl_Init(me,
				MotorsControl_DrivingForward,
				MotorsControl_DrivingRight,
				MotorsControl_DrivingLeft,
				MotorsControl_DrivingBackwart,
				MotorsControl_ClockwiseRotation,
				MotorsControl_CountersclockwiseRotation,
				MotorsControl_DrivingReverseRight,
				MotorsControl_DrivingReverseLeft,
				MotorsControl_ResetDriving);
		me->resetDriving(me);
		HAL_TIM_PWM_Start(timerControlMotor, channelEnA);
		HAL_TIM_PWM_Start(timerControlMotor, channelEnB);
		HAL_TIM_Base_Start(timerControlMotor);

	}
	return me;
}
void MotorsControl_Init(motorsControlStruct* me,
						void (*drivingForward)(motorsControlStruct* me),
						void (*drivingRight)(motorsControlStruct* me),
						void (*drivingLeft)(motorsControlStruct* me),
						void (*drivingBackwart)(motorsControlStruct* me),
						void (*clockwiseRotation)(motorsControlStruct* me),
						void (*countersclockwiseRotation)(motorsControlStruct* me),
						void (*drivingReverseRight)(motorsControlStruct* me),
						void (*drivingReverseLeft)(motorsControlStruct* me),
						void (*resetDriving)(motorsControlStruct* me)){
	me->drivingForward=drivingForward;
	me->drivingRight=drivingRight;
	me->drivingLeft=drivingLeft;
	me->drivingBackwart=drivingBackwart;
	me->clockwiseRotation=clockwiseRotation;
	me->countersclockwiseRotation=countersclockwiseRotation;
	me->drivingReverseRight=drivingReverseRight;
	me->drivingReverseLeft=drivingReverseLeft;
	me->resetDriving=resetDriving;
}

void MotorsControl_DrivingForward(motorsControlStruct* me){
	MotorsControl_SetHtimCompare(me,me->timerPeriod,me->timerPeriod);
	MotorsControl_SetOutput(me,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET);
}
void MotorsControl_DrivingRight(motorsControlStruct* me){
	MotorsControl_SetHtimCompare(me,me->timerPeriod,me->timerCompareForTurn);
	MotorsControl_SetOutput(me,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET);

}
void MotorsControl_DrivingLeft(motorsControlStruct* me){
	MotorsControl_SetHtimCompare(me,me->timerCompareForTurn,me->timerPeriod);
	MotorsControl_SetOutput(me,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET);
}
void MotorsControl_DrivingBackwart(motorsControlStruct* me){
	MotorsControl_SetHtimCompare(me,me->timerPeriod,me->timerPeriod);
	MotorsControl_SetOutput(me,GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET);
}
void MotorsControl_ClockwiseRotation(motorsControlStruct* me){
	MotorsControl_SetHtimCompare(me,me->timerPeriod,me->timerPeriod);
	MotorsControl_SetOutput(me,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET);

}
void MotorsControl_CountersclockwiseRotation(motorsControlStruct* me){
	MotorsControl_SetHtimCompare(me,me->timerPeriod,me->timerPeriod);
	MotorsControl_SetOutput(me,GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_RESET);

}
void MotorsControl_DrivingReverseRight(motorsControlStruct* me){
	MotorsControl_SetHtimCompare(me,me->timerPeriod,me->timerCompareForTurn);
	MotorsControl_SetOutput(me,GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET);
}
void MotorsControl_DrivingReverseLeft(motorsControlStruct* me){
	MotorsControl_SetHtimCompare(me,me->timerCompareForTurn,me->timerPeriod);
	MotorsControl_SetOutput(me,GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET);
}
void MotorsControl_ResetDriving(motorsControlStruct* me){
	MotorsControl_SetHtimCompare(me,me->timerPeriod,me->timerPeriod);
	MotorsControl_SetOutput(me,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET);
}
void MotorsControl_SetOutput(motorsControlStruct* me,GPIO_PinState out1,GPIO_PinState out2,GPIO_PinState out3,GPIO_PinState out4){
	GPIO_PinState prevout1=HAL_GPIO_ReadPin(me->outGPIO, me->pinOut1);
	GPIO_PinState prevout2=HAL_GPIO_ReadPin(me->outGPIO, me->pinOut2);
	GPIO_PinState prevout3=HAL_GPIO_ReadPin(me->outGPIO, me->pinOut3);
	GPIO_PinState prevout4=HAL_GPIO_ReadPin(me->outGPIO, me->pinOut4);
	if(prevout1!=out1){
		HAL_GPIO_WritePin(me->outGPIO, me->pinOut1,out1);
	}
	if(prevout2!=out2){
		HAL_GPIO_WritePin(me->outGPIO, me->pinOut2,out2);
	}
	if(prevout3!=out3){
		HAL_GPIO_WritePin(me->outGPIO, me->pinOut3,out3);
	}
	if(prevout4!=out4){
		HAL_GPIO_WritePin(me->outGPIO, me->pinOut4,out4);
	}
}
void MotorsControl_SetHtimCompare(motorsControlStruct* me,uint32_t htimChannel1,uint32_t htimChannel2){
	uint16_t prevCompareChannel1=__HAL_TIM_GET_COMPARE(me->timerControlMotor,TIM_CHANNEL_1);
	uint16_t prevCompareChannel2=__HAL_TIM_GET_COMPARE(me->timerControlMotor,TIM_CHANNEL_2);

	if(prevCompareChannel1!=htimChannel1){
		__HAL_TIM_SET_COMPARE(me->timerControlMotor,TIM_CHANNEL_1,htimChannel1);
	}
	if(prevCompareChannel2!=htimChannel2){
		__HAL_TIM_SET_COMPARE(me->timerControlMotor,TIM_CHANNEL_2,htimChannel2);
	}
}

