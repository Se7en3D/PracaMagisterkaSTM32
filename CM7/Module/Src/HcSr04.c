/*
 * HcSr04.c
 *
 *  Created on: 23 sty 2022
 *      Author: Daniel
 */
#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "HcSr04.h"

ultrasonicSensorStruct* HcSr04_Create(TIM_HandleTypeDef *htim){
	ultrasonicSensorStruct * me=malloc(sizeof(ultrasonicSensorStruct));
	if(me!=NULL){
		me->status=HCSR04_Idle;
		me->risingEdgeTime=0;
		me->fallingEdgeTime=0;
		me->continousMeasurment=0;
		me->result=0;
		me->htim=htim;
		HcSr04_Init(me,
				HcSr04_AddRisingEdgeTime,
				HcSr04_AddFallingEdgeTime,
				HcSr04_SetContinousMeasurment,
				HcSr04_ResetContinousMeasurment,
				HcSr04_GetMeasurment,
				HcSr04_ResetMeasurment,
				HcSr04_StartMeasurment,
				HcSr04_htimInterrupt);
	}
	return me;
}
void HcSr04_Init(ultrasonicSensorStruct *me,
				void (*addRisingEdgeTime)(ultrasonicSensorStruct *me, uint32_t value),
				void (*addFallingEdgeTime)(ultrasonicSensorStruct *me, uint32_t value),
				void (*setContinousMeasurment)(ultrasonicSensorStruct *me),
				void (*resetContinousMeasurment)(ultrasonicSensorStruct *me),
				uint32_t(*getMeasurment)(ultrasonicSensorStruct *me),
				void (*resetMeasurment)(ultrasonicSensorStruct *me),
				void (*startMeasurment)(ultrasonicSensorStruct *me),
				void (*htimInterrupt)(ultrasonicSensorStruct *me,TIM_HandleTypeDef *htim)){
	me->addRisingEdgeTime=addRisingEdgeTime;
	me->addFallingEdgeTime=addFallingEdgeTime;
	me->setContinousMeasurment=setContinousMeasurment;
	me->resetContinousMeasurment=resetContinousMeasurment;
	me->getMeasurment=getMeasurment;
	me->resetMeasurment=resetMeasurment;
	me->startMeasurment=startMeasurment;
	me->htimInterrupt=htimInterrupt;

}
void HcSr04_AddRisingEdgeTime(ultrasonicSensorStruct *me, uint32_t value){
	if(me->status==HCSR04_Measurment){
		me->risingEdgeTime=value;
	}
}
void HcSr04_AddFallingEdgeTime(ultrasonicSensorStruct *me, uint32_t value){
	if(me->status==HCSR04_Measurment){
		me->fallingEdgeTime=value;
		me->status=HCSR04_CompleteMeasurment;
			/*Reset licznika*/
		HcSr04_HalStop(me->htim);
	}
}
void HcSr04_SetContinousMeasurment(ultrasonicSensorStruct *me){
	me->continousMeasurment=SET;
}
void HcSr04_ResetContinousMeasurment(ultrasonicSensorStruct *me){
	me->continousMeasurment=RESET;
}
uint32_t HcSr04_GetMeasurment(ultrasonicSensorStruct *me){
	if(me->status==HCSR04_CompleteMeasurment){
		me->result=me->fallingEdgeTime-me->risingEdgeTime;
		me->status=HCSR04_Idle;
		if(me->continousMeasurment==SET){
			me->startMeasurment(me);
		}
	}
	return me->result;
}
void HcSr04_ResetMeasurment(ultrasonicSensorStruct *me){
	me->status=HCSR04_Idle;
	me->risingEdgeTime=0;
	me->fallingEdgeTime=0;
	me->continousMeasurment=0;
}
void HcSr04_StartMeasurment(ultrasonicSensorStruct *me){
	if(me->status==HCSR04_Idle){
		me->status=HCSR04_Measurment;
		HcSr04_HalStart(me->htim);
	}
}
void HcSr04_htimInterrupt(ultrasonicSensorStruct *me,TIM_HandleTypeDef *htim){
	if(htim==me->htim){
			uint32_t value;
			if ((htim->Instance->CCMR1 & TIM_CCMR1_CC1S) != 0x00U){
				value=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				me->addRisingEdgeTime(me,value);
			}

			if ((htim->Instance->CCMR1 & TIM_CCMR1_CC2S) != 0x00U){
				value=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				me->addFallingEdgeTime(me,value);
			}
		}
}
void HcSr04_HalStart(TIM_HandleTypeDef *htim){
	HAL_TIM_IC_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
	HAL_TIM_Base_Start_IT(htim);
}
void HcSr04_HalStop(TIM_HandleTypeDef *htim){
	HAL_TIM_Base_Stop_IT(htim);
	HAL_TIM_IC_Stop(htim, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3);
	__HAL_TIM_SET_COUNTER(htim,0);
}
