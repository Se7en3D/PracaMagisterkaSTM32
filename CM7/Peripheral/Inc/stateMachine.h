/*
 * stateMachine.h
 *
 *  Created on: 12 lip 2021
 *      Author: DanielD
 */

#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_
#include "stm32h7xx_hal.h"

typedef enum{
	IDLE_DRIVING=0,
	GO_LEFT,
	GO_RIGHT,
	GO_UP,
	GO_BACK,
	ROTATE_LEFT_DRIV,
	ROTATE_RIGHT_DRIV,
	STOP_DRIVING,
}driving_status_t;



typedef struct{
	driving_status_t previousDrivingStatus;
	driving_status_t drivingStatus;
	GPIO_TypeDef *GPIOOUT;
	uint16_t L298NOUT1A;
	uint16_t L298NOUT2A;
	uint16_t L298NOUT1B;
	uint16_t L298NOUT2B;
	uint32_t resetTimer;
	TIM_HandleTypeDef *htim;
	uint16_t htimPeriod;
	uint16_t htimFullPulse;
	uint16_t htimHalfPulse;
}driving_structure_t;

driving_structure_t drivingStructure;
driving_status_t stateMachineGetDrivingStatus();

void stateMachineInit(TIM_HandleTypeDef *htim ,GPIO_TypeDef *GPIOOUT,uint16_t L298NOUT1A,uint16_t L298NOUT2A,uint16_t L298NOUT1B,uint16_t L298NOUT2B);
void stateMachineDrivingForward();
void stateMachineDrivingLeft();
void stateMachineDrivingRight();
void stateMachineDrivingBack();
void stateMachineRotateLeft();
void stateMachineRotateRight();
void stateMachineStopDriving();
void stateMachineMeasureDistance();
void stateMachineMeasureDistanceEnd();
void stateMachineTimeout();
void stateMachineResetTimer();
void stateMachineSetOutput(GPIO_PinState A1,GPIO_PinState A2,GPIO_PinState B1,GPIO_PinState B2);
void stateMachineInitNewState();
uint32_t stateMachineGetResetTimer();
void stateMachineSetHtimCompare(uint16_t htimChannel1,uint16_t htimChannel2);
driving_structure_t stateMachineGetDrivingStructure();

#endif /* INC_STATEMACHINE_H_ */
