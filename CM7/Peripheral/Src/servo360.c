/*
 * servo360.c
 *
 *  Created on: 6 sie 2021
 *      Author: DanielD
 */
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "errorCode.h"
#include "servo360.h"

void servo360Init(TIM_HandleTypeDef *htim,uint16_t pin ,GPIO_TypeDef *port){
	servo360Structure.htim=htim;
	servo360Structure.period=SERVO360_DEFAULT_PERIOD;
	servo360Structure.status=servo360_INITIALIZATION;
	servo360Structure.repeatValue=0;
	__HAL_TIM_SET_COMPARE(servo360Structure.htim,TIM_CHANNEL_1,servo360Structure.period);
	servo360Structure.stateFunction[servo360_INITIALIZATION]=&servo360StatusInitialization;
	servo360Structure.stateFunction[servo360_IDLE]=&servo360StatusIdle;
	servo360Structure.stateFunction[servo360_ROTATE]=&servo360StatusRotate;
	servo360Structure.stateFunction[servo360_WAIT_TO_MEASURMENT]=&servo360StatusWaitToMeasurment;
	servo360Structure.stateFunction[servo360_ERROR_INITIALIZATION]=NULL;
	servo360Structure.limitSwitchServoPin=pin;
	servo360Structure.limitSwitchServoPort=port;
}

void servo360PWMEdit(){
	if(servo360Structure.htim==0){
		return;
	}

	if(servo360Structure.period==SERVO360_DEFAULT_PERIOD){
		return;
	}

	if(servo360Structure.repeatValue<1 ){
		if(servo360Structure.period==SERVO360_DEFAULT_PERIOD){
			//servo360Structure.repeatValue=0;
			return;
		}
		servo360Structure.period=SERVO360_DEFAULT_PERIOD;
		servo360Structure.repeatValue=0;
		__HAL_TIM_SET_COMPARE(servo360Structure.htim,TIM_CHANNEL_1,servo360Structure.period);
		return;
	}

	__HAL_TIM_SET_COMPARE(servo360Structure.htim,TIM_CHANNEL_1,servo360Structure.period);
	servo360Structure.repeatValue--;

}
void servo360PWMDefault(){
	servo360Structure.period=SERVO360_DEFAULT_PERIOD;
	servo360Structure.repeatValue=0;
	__HAL_TIM_SET_COMPARE(servo360Structure.htim,TIM_CHANNEL_1,SERVO360_DEFAULT_PERIOD);
}

void servo360NewDataPWM(uint8_t *framePointer){

	int timer=0;
	if(servo360Structure.repeatValue!=0){
		return;
	}
	while(framePointer[timer]!=0xFE){
		if(timer>50)
			break;

		timer++;
	}

	if(timer>50 || timer<5){
		return;
	}
	uint16_t period=framePointer[2]<<8;
	period|=framePointer[3];

	uint16_t repeat=framePointer[4]<<8;
	repeat|=framePointer[5];
	servo360Structure.repeatValue=repeat;
	servo360Structure.period=period;
	//printf("repeat=%d period=%d %x %x\n",repeat,period,framePointer[2],framePointer[3]);
}

void servo360StatusInitialization(){
	if(servo360Structure.status==servo360_INITIALIZATION){
		GPIO_PinState limitSwitchServoPinStatus=HAL_GPIO_ReadPin(servo360Structure.limitSwitchServoPort, servo360Structure.limitSwitchServoPin);
		if(limitSwitchServoPinStatus==GPIO_PIN_RESET){
			servo360Structure.status=servo360_IDLE;
			servo360PWMDefault();
			servo360SetCurrentPositionByPositionNumber(0);
			servo360SetTargetPosition(0);
			/*servo360Structure.period=4200;
			servo360Structure.repeatValue=36;
			servo360SetCurrentPositionByPositionNumber(SERVO360_MAX_POSITION/2);
			servo360SetTargetPosition(SERVO360_MAX_POSITION/2);*/
		}else{
			if(servo360Structure.repeatValue<1){
				servo360Structure.period=SERVO360_PERIOD_INITIALIZATION;
				servo360Structure.repeatValue=SERVO360_REPEAT_INITIALIZATION;
				servo360Structure.currentPosition++;
				if(servo360Structure.currentPosition>=SERVO360_MAX_REPEAT_TO_ERROR_IN_INITIALIZATION){
					servo360Structure.status=servo360_ERROR_INITIALIZATION;
					servo360PWMDefault();
				}
			}
		}
	}
}
void servo360StatusIdle(){
	if(servo360Structure.targetPosition!=servo360Structure.currentPosition && servo360Structure.status==servo360_IDLE ){
		servo360Structure.status=servo360_ROTATE;
	}
}
void servo360StatusRotate(){
	if(servo360Structure.status==servo360_ROTATE ){
		if(servo360Structure.targetPosition!=servo360Structure.currentPosition ){
			if(servo360Structure.repeatValue<=0){
				uint32_t difference=abs(servo360Structure.targetPosition-servo360Structure.currentPosition)-1;
				if(servo360Structure.targetPosition>servo360Structure.currentPosition){
					servo360Structure.period=SERVO360_PERIOD_ROTATION_RIGHT;
				}else{
					servo360Structure.period=SERVO360_PERIOD_ROTATION_LEFT;
				}
				servo360Structure.currentPosition=servo360Structure.targetPosition;
				servo360Structure.repeatValue=servo360RepeatAngle[difference];
			}
			/*
			if(servo360Structure.repeatValue<=0){
				if(servo360Structure.targetPosition>servo360Structure.currentPosition){
					servo360Structure.period=SERVO360_PERIOD_ROTATION_RIGHT;
					servo360Structure.currentPosition++;
				}else{
					servo360Structure.period=SERVO360_PERIOD_ROTATION_LEFT;
					servo360Structure.currentPosition--;
				}
				if(servo360Structure.currentPosition>SERVO360_MAX_POSITION || servo360Structure.currentPosition<0){
					errorCodePush(SERVO360_CURRENTPOSITION_OUT_OF_RANGE);
				}else{
					servo360Structure.repeatValue=SERVO360_REPEAT_ROTATION;
				}
			}*/
		}else{
			servo360Structure.status=servo360_IDLE;
		}
	}
}
void servo360StatusWaitToMeasurment(){

}
void servo360StateFunctions(){
	if(servo360Structure.stateFunction[servo360Structure.status]!=0){
		(*servo360Structure.stateFunction[servo360Structure.status])();
	}
}

void servo360SetCurrentPositionByPositionNumber(uint16_t number){
	if(number>SERVO360_MAX_POSITION){
		errorCodePush(SERVO360_NO_NUMBER_IN_ARRAY);
	}else{
		servo360Structure.currentPosition=number;
	}
}

void servo360SetTargetPosition(uint16_t position){
	if(position>SERVO360_MAX_POSITION){
		errorCodePush(SERVO360_NO_NUMBER_IN_ARRAY);
	}else{
		servo360Structure.targetPosition=position;
	}
}
