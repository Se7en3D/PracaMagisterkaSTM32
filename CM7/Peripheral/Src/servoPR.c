/*
 * servoPR.c
 *
 *  Created on: 16 lis 2021
 *      Author: Daniel
 */
#include "stm32h7xx_hal.h"
#include "errorCode.h"
#include "servoPR.h"


/**
 * @brief servoPRFunToStatePointer wzkażnik na funkcję obsługi servo
 */
static const volatile void(*servoPRFunToStatePointer[])(servoPR_GeneralStructure *servoPR)={
	&servoPRIdleFunction,
	&servoPRWaitToStabilizeServoFunction,
	&servoPRWaitToMeasurmentFunction,
	&servoPRErrorFunction
};


void servoPRInit(servoPR_GeneralStructure *servoPR,TIM_HandleTypeDef *PWMtimerGen,uint32_t TimChannel){
	servoPR->PWMtimerGen=PWMtimerGen;
	servoPR->TimChannel=TimChannel;
	servoPR->periodTimFor20ms=servoPR->PWMtimerGen->Init.Period;
	servoPR->targetAngle=90;
	uint32_t difference=(SERVOPR_MAX_PERIOD_IN_MS)-(SERVOPR_MIN_PERIOD_IN_MS);
	uint32_t stepValue=difference/SERVOPR_SUM_POSITION_SERVO;
	uint32_t pulseValueFor20ms=servoPR->periodTimFor20ms;

	for(int i=0;i<SERVOPR_SUM_POSITION_SERVO;i++){
		uint32_t time=SERVOPR_MIN_PERIOD_IN_MS+(stepValue*i);
		servoPR->pulseValue[i]=time*pulseValueFor20ms/(20*SERVOPR_MAX_PERIOD_IN_MS);
	}

	  HAL_TIM_Base_Start_IT(servoPR->PWMtimerGen);
	  HAL_TIM_PWM_Start_IT(servoPR->PWMtimerGen, servoPR->TimChannel);
}

uint8_t servoPRGetCurrentAngle(servoPR_GeneralStructure *servoPR){
	return 0;
}

void servoPRSetCurrentAngle(servoPR_GeneralStructure *servoPR,uint8_t newAngle){
	if(servoPR->PWMtimerGen!=0){
		if(newAngle<=SERVOPR_MAX_ROTATION_ANGLE){
			servoPR->currentAngle=newAngle;

		}else{
			errorCodePush(SERVOPR_NEW_ANGLE_OUT_OFF_LIMIT);
		}

	}else{
		errorCodePush(SERVOPR_NULL_POINTER_ON_TIMER);
	}
}

void servoPRSetTargetAngle(servoPR_GeneralStructure *servoPR,uint8_t newAngle){
	if(servoPR->PWMtimerGen!=0){
		if(newAngle<=SERVOPR_MAX_ROTATION_ANGLE){
			servoPR->targetAngle=newAngle;

		}else{
			errorCodePush(SERVOPR_NEW_ANGLE_OUT_OFF_LIMIT);
		}

	}else{
		errorCodePush(SERVOPR_NULL_POINTER_ON_TIMER);
	}
}
void servoPRIdleFunction(servoPR_GeneralStructure *servoPR){
	if(servoPR->status==servoPR_IDLE && servoPR->blockedStatus==SERVOPR_NON_BLOCKED_VALUE){
		const uint8_t currentAngle=servoPR->currentAngle;
		const uint8_t targetAngle=servoPR->targetAngle;
		if(currentAngle!=targetAngle){
			servoPR->status=servoPR_WAIT_TO_STABILIZE_SERVO;
			servoPR->currentAngle=targetAngle;
				/**
				 * Określnie czasu potrzebnego do stabilizacji
				 * https://www.electronicoscaldas.com/datasheet/MG90S_Tower-Pro.pdf
				 * Operating speed: 0.1s/60 defree(4.8V), 0.08s/60 degree(6V)
				 */
			uint8_t differenceAngle;

			if(currentAngle>targetAngle){
				differenceAngle=currentAngle-targetAngle;
			}else{
				differenceAngle=targetAngle-currentAngle;
			}
				//(czas poruszenia się o X stopni)*(stopień ruchu)/(X stopni)+(Dodatkowa wartość dla pewności że serwo się przemieści)
			servoPR->timeToStabilize=100*differenceAngle/(60)+10;

			servoPRChangePulseFill(servoPR,targetAngle);
			servoPR->time=0;
		}
	}
}
void servoPRWaitToStabilizeServoFunction(servoPR_GeneralStructure *servoPR){
	if(servoPR->status==servoPR_WAIT_TO_STABILIZE_SERVO && servoPR->blockedStatus==SERVOPR_NON_BLOCKED_VALUE){
		const uint8_t timeToStabilize=servoPR->timeToStabilize;
		const uint8_t time=servoPR->time;
		if(time>=timeToStabilize){
			servoPR->timeToStabilize=0;
			servoPR->status=servoPR_WAIT_TO_MEASURMENT;
			servoPR->blockedStatus=1;
		}
	}
}
void servoPRWaitToMeasurmentFunction(servoPR_GeneralStructure *servoPR){
	if(servoPR->status==servoPR_WAIT_TO_MEASURMENT && servoPR->blockedStatus==SERVOPR_NON_BLOCKED_VALUE){
		servoPR->status=servoPR_IDLE;
	}
}
void servoPRErrorFunction(servoPR_GeneralStructure *servoPR){
	if(servoPR->status==servoPR_ERROR){

	}
}
void servoPRExecuteStatusFunction(servoPR_GeneralStructure *servoPR){
	servoPRFunToStatePointer[servoPR->status](servoPR);
}

void servoPRClearBlockedStatus(servoPR_GeneralStructure *servoPR){
	servoPR->blockedStatus=SERVOPR_NON_BLOCKED_VALUE;
}

uint8_t servoPRGetBlockedStatus(servoPR_GeneralStructure *servoPR){
	return servoPR->blockedStatus;
}

void servoPRChangePulseFill(servoPR_GeneralStructure *servoPR,const uint8_t targetAngle){
	uint32_t pulseFor180Degree=((servoPR->periodTimFor20ms+1)*SERVOPR_MAX_PERIOD_IN_MS)/(20*SERVOPR_MULTIPLIER);
	uint32_t pulseFor0Degree=((servoPR->periodTimFor20ms+1)*SERVOPR_MIN_PERIOD_IN_MS)/(20*SERVOPR_MULTIPLIER);
	uint32_t different=pulseFor180Degree-pulseFor0Degree;
	uint32_t pulseForTargetDegree=(different*targetAngle)/SERVOPR_MAX_ROTATION_ANGLE+pulseFor0Degree;
	__HAL_TIM_SET_COMPARE(servoPR->PWMtimerGen,servoPR->TimChannel,(pulseForTargetDegree-1));

}

void servoPRAddTime(servoPR_GeneralStructure *servoPR){
	servoPR->time++;
}

uint8_t servoPRReadyToChangeAngle(servoPR_GeneralStructure *servoPR){
	if(servoPR->currentAngle==servoPR->targetAngle){
		if(servoPR->status==servoPR_IDLE){
			return TRUE;
		}
	}
	return FALSE;
}

