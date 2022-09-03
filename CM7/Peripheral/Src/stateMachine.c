/*
 * stateMachine.c
 *
 *  Created on: 19 lip 2021
 *      Author: Daniel
 */

#include <stdio.h>
#include "errorCode.h"
#include "stateMachine.h"

void stateMachineInit(TIM_HandleTypeDef *htim ,GPIO_TypeDef *GPIOOUT,uint16_t L298NOUT1A,uint16_t L298NOUT2A,uint16_t L298NOUT1B,uint16_t L298NOUT2B){
	drivingStructure.GPIOOUT=GPIOOUT;
	drivingStructure.L298NOUT1A=L298NOUT1A;
	drivingStructure.L298NOUT2A=L298NOUT2A;
	drivingStructure.L298NOUT1B=L298NOUT1B;
	drivingStructure.L298NOUT2B=L298NOUT2B;
	drivingStructure.htim=htim;
	drivingStructure.htimPeriod=htim->Init.Period;
	drivingStructure.htimFullPulse=drivingStructure.htimPeriod+10;
	drivingStructure.htimHalfPulse=drivingStructure.htimPeriod*0.1;

	drivingStructure.previousDrivingStatus=IDLE_DRIVING;
	drivingStructure.drivingStatus=IDLE_DRIVING;

	stateMachineSetHtimCompare(drivingStructure.htimFullPulse,drivingStructure.htimFullPulse);
}

void stateMachineDrivingForward(){
	stateMachineInitNewState();
	drivingStructure.drivingStatus=GO_UP;

	if(drivingStructure.previousDrivingStatus==drivingStructure.drivingStatus)
		return;

	stateMachineSetHtimCompare(drivingStructure.htimFullPulse,drivingStructure.htimFullPulse);
	stateMachineSetOutput(GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET);
}
void stateMachineDrivingLeft(){
	stateMachineInitNewState();
	drivingStructure.drivingStatus=GO_UP_LEFT;
	if(drivingStructure.previousDrivingStatus==drivingStructure.drivingStatus)
		return;

	stateMachineSetHtimCompare(drivingStructure.htimHalfPulse,drivingStructure.htimFullPulse);
	stateMachineSetOutput(GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET);
}
void stateMachineDrivingRight(){
	stateMachineInitNewState();
	drivingStructure.drivingStatus=GO_UP_RIGHT;
	if(drivingStructure.previousDrivingStatus==drivingStructure.drivingStatus)
		return;

	stateMachineSetHtimCompare(drivingStructure.htimFullPulse,drivingStructure.htimHalfPulse);
	stateMachineSetOutput(GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET);
}
void stateMachineDrivingBack(){
	stateMachineInitNewState();
	drivingStructure.drivingStatus=GO_BACK;

	if(drivingStructure.previousDrivingStatus==drivingStructure.drivingStatus)
		return;

	stateMachineSetHtimCompare(drivingStructure.htimFullPulse,drivingStructure.htimFullPulse);
	stateMachineSetOutput(GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET);
}
void stateMachineRotateLeft(){
	stateMachineInitNewState();
	drivingStructure.drivingStatus=ROTATE_LEFT_DRIV;


	if(drivingStructure.previousDrivingStatus==drivingStructure.drivingStatus)
		return;

	stateMachineSetHtimCompare(drivingStructure.htimFullPulse,drivingStructure.htimFullPulse);
	stateMachineSetOutput(GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_SET,GPIO_PIN_RESET);
}
void stateMachineRotateRight(){
	stateMachineInitNewState();
	drivingStructure.drivingStatus=ROTATE_RIGHT_DRIV;

	if(drivingStructure.previousDrivingStatus==drivingStructure.drivingStatus)
		return;

	stateMachineSetHtimCompare(drivingStructure.htimFullPulse,drivingStructure.htimFullPulse);
	stateMachineSetOutput(GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET);
}
void stateMachineRotateBackLeft(){
	stateMachineInitNewState();
	drivingStructure.drivingStatus=GO_UP_RIGHT;
	if(drivingStructure.previousDrivingStatus==drivingStructure.drivingStatus)
		return;

	stateMachineSetHtimCompare(drivingStructure.htimHalfPulse,drivingStructure.htimFullPulse);
	stateMachineSetOutput(GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET);
}
void stateMachineRotateBackRight(){
	stateMachineInitNewState();
	drivingStructure.drivingStatus=GO_UP_LEFT;
	if(drivingStructure.previousDrivingStatus==drivingStructure.drivingStatus)
		return;

	stateMachineSetHtimCompare(drivingStructure.htimFullPulse,drivingStructure.htimHalfPulse);
	stateMachineSetOutput(GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET);
}
void stateMachineStopDriving(){
	stateMachineInitNewState();
	drivingStructure.drivingStatus=STOP_DRIVING;


	if(drivingStructure.previousDrivingStatus==drivingStructure.drivingStatus)
		return;

	stateMachineSetHtimCompare(drivingStructure.htimFullPulse,drivingStructure.htimFullPulse);
	stateMachineSetOutput(GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET);
}
void stateMachineMeasureDistance(){
	stateMachineInitNewState();
	drivingStructure.drivingStatus=STOP_DRIVING;
	drivingStructure.resetTimer=0;
	if(drivingStructure.previousDrivingStatus==drivingStructure.drivingStatus)
		return;

	stateMachineSetHtimCompare(drivingStructure.htimFullPulse,drivingStructure.htimFullPulse);
	stateMachineSetOutput(GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET);
}
void stateMachineMeasureDistanceEnd(){
	drivingStructure.drivingStatus=IDLE_DRIVING;
}

void stateMachineTimeout(){
	drivingStructure.drivingStatus=IDLE_DRIVING;
	drivingStructure.stopManualDriving=RESET;
	stateMachineSetOutput(GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET);
	stateMachineSetHtimCompare(drivingStructure.htimFullPulse,drivingStructure.htimFullPulse);
	errorCodePush(STATEMACHINE_TIMEOUT);
}
void stateMachineResetTimer(){
	drivingStructure.resetTimer=0;
}

void stateMachineSetOutput(GPIO_PinState A1,GPIO_PinState A2,GPIO_PinState B1,GPIO_PinState B2){
	if(drivingStructure.GPIOOUT!=0){
		if(drivingStructure.stopManualDriving==SET){
			return;
		}
		printf("Ustawiono A1=%d A2=%d B1=%d B2=%d\n",(int)A1,(int)A2,(int) B1,(int)B2);
		HAL_GPIO_WritePin(drivingStructure.GPIOOUT, drivingStructure.L298NOUT1A, A1);
		HAL_GPIO_WritePin(drivingStructure.GPIOOUT, drivingStructure.L298NOUT2A, A2);
		HAL_GPIO_WritePin(drivingStructure.GPIOOUT, drivingStructure.L298NOUT1B, B1);
		HAL_GPIO_WritePin(drivingStructure.GPIOOUT, drivingStructure.L298NOUT2B, B2);
		return;
	}
	errorCodePush(STATEMACHINE_GPIO_POINTER_NULL);
}

void stateMachineInitNewState(){
	drivingStructure.previousDrivingStatus=drivingStructure.drivingStatus;
	drivingStructure.resetTimer=1;

}
uint32_t stateMachineGetResetTimer(){
	return drivingStructure.resetTimer;
}

void stateMachineSetHtimCompare(uint16_t htimChannel1,uint16_t htimChannel2){
	if(drivingStructure.htim!=0){
		__HAL_TIM_SET_COMPARE(drivingStructure.htim,TIM_CHANNEL_1,htimChannel1);
		__HAL_TIM_SET_COMPARE(drivingStructure.htim,TIM_CHANNEL_2,htimChannel2);

	}else{
		errorCodePush(STATEMACHINE_HTIM_POINTER_NULL);
	}
}

driving_structure_t stateMachineGetDrivingStructure(){
	return drivingStructure;
}

uint8_t stateMachineDrabingsStatusIsEqual(driving_structure_t *drivingStatus){
	if(drivingStatus->previousDrivingStatus==drivingStatus->drivingStatus){
		return TRUE;
	}else{
		return FALSE;
	}

}

driving_status_t stateMachineGetDrivingStatus(driving_structure_t *drivingStructure){
	return drivingStructure->drivingStatus;
}

void stateMachineSetStopManualDriving(){
	if(drivingStructure.stopManualDriving==RESET){
		stateMachineSetOutput(GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET);
	}
	drivingStructure.stopManualDriving=SET;
}

void stateMachineResetStopManualDriving(){
	if(drivingStructure.stopManualDriving==SET){ //Wymagane żeby ustawić wyjścia silnika po usunięcia przeszkody z czujników IR
		drivingStructure.previousDrivingStatus=IDLE_DRIVING;
		drivingStructure.drivingStatus=IDLE_DRIVING;
	}
	drivingStructure.stopManualDriving=RESET;
}

uint8_t stateMachineIsManualDriving(){
	if(drivingStructure.stopManualDriving==SET){
		return TRUE;
	}else{
		return FALSE;
	}
}


