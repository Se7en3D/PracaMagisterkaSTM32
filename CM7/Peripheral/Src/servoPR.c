/*
 * servoPR.c
 *
 *  Created on: 16 lis 2021
 *      Author: Daniel
 */
#include "stm32h7xx_hal.h"
#include "errorCode.h"
#include "servoPR.h"



void servoPRInit(servoPR_GeneralStructure *servoPR,TIM_HandleTypeDef *PWMtimerGen){
	servoPR->PWMtimerGen=PWMtimerGen;

	servoPR->periodTimer=servoPR->PWMtimerGen->Init.Period;
	uint32_t difference=(SERVOPR_MAX_PERIOD_IN_MS)-(SERVOPR_MIN_PERIOD_IN_MS);
	uint32_t stepValue=difference/SERVOPR_SUM_POSITION_SERVO;
	uint32_t pulseValueFor20ms=servoPR->periodTimer;
	for(int i=0;i<SERVOPR_SUM_POSITION_SERVO;i++){
		uint32_t time=SERVOPR_MIN_PERIOD_IN_MS+(stepValue*i);
		servoPR->pulseValue[i]=time*pulseValueFor20ms/(20*SERVOPR_MAX_PERIOD_IN_MS);
	}
}

uint8_t servoPRGetCurrentAngle(servoPR_GeneralStructure *servoPR){
	return 0;
}

void servoPRSetCurrentAngle(servoPR_GeneralStructure *servoPR,uint8_t newAngle){

}

