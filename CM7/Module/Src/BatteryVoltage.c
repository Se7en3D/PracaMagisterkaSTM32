/*
 * BatteryVoltage.c
 *
 *  Created on: 30 sty 2022
 *      Author: Daniel
 */


#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "BatteryVoltage.h"
#include "ErrorValue.h"

adcMeasureStruct *BatteryVoltage_Create(ADC_HandleTypeDef *hadc,GPIO_TypeDef *gpio,uint16_t pin){
	adcMeasureStruct *me=malloc(sizeof(adcMeasureStruct));
	if(me!=NULL){
		me->hadc=hadc;
		me->gpioTypeDefPowerSupplySchematicMeasurment=gpio;
		me->gpioPinPowerSupplySchematicMeasurment=pin;
		me->statusFunction[Battery_Idle]=BatteryVoltage_StatusIdle;
		me->statusFunction[Battery_Stabilize]=BatteryVoltage_StatusStabilize;
		me->statusFunction[Battery_Measurment]=BatteryVoltage_StatusMeasurment;
		me->statusFunction[Battery_CompleteMeasurment]=BatteryVoltage_StatusCompleteMeasurment;
		me->status=Battery_Idle;
		me->continousMeasurment=0;
		BatteryVoltage_Init(me,
				BatteryVoltage_GetValue,
				BatteryVoltage_StartMeasurment,
				BatteryVoltage_AddTime,
				BatteryVoltage_ResetMeasurment,
				BatteryVoltage_SetContinousMeasurment,
				BatteryVoltage_ResetContinousMeasurment,
				BatteryVoltage_AddMeasure);
	}

	return me;
}

void BatteryVoltage_Init(adcMeasureStruct* const me,
		uint32_t* (*getValue)(adcMeasureStruct* const me),
		void (*startMeasurment)(adcMeasureStruct* const me),
		void (*addTime)(adcMeasureStruct* const me),
		void (*resetMeasurment)(adcMeasureStruct* const me),
		void (*setContinousMeasurment)(adcMeasureStruct* const me),
		void (*resetContinousMeasurment)(adcMeasureStruct* const me),
		void (*addMeasure)(adcMeasureStruct* const me,uint32_t value)){
	me->getValue=getValue;
	me->startMeasurment=startMeasurment;
	me->addTime=addTime;
	me->resetMeasurment=resetMeasurment;
	me->setContinousMeasurment=setContinousMeasurment;
	me->resetContinousMeasurment=resetContinousMeasurment;
	me->addMeasure=addMeasure;
}

void BatteryVoltage_StatusIdle(adcMeasureStruct* const me){
	if(me->continousMeasurment){
		me->startMeasurment(me);
	}
}

void BatteryVoltage_StatusStabilize(adcMeasureStruct* const me){
	if(me->time>=BATTERYVOLTAGE_TIME_TO_STABILIZE){
		me->status=Battery_Measurment;
		BatteryVoltage_ResetTime(me);
		HAL_ADC_Start_IT(me->hadc);
		/*Obsługa przetwornika ADC*/
		BatteryVoltage_StartADC(me);
	}
}

void BatteryVoltage_StatusMeasurment(adcMeasureStruct* const me){
	/*if(HAL_ADC_GetState(me->hadc)!=HAL_ADC_STATE_READY){
		HAL_ADC_Start_IT(me->hadc);
	}*/

	if(me->number>=BATTERYVOLTAGE_MAX_MEASURMENTS){
		me->status=Battery_CompleteMeasurment;
		BatteryVoltage_EnGpioOn(me);
			/*Obsługa przetwornika ADC*/
		BatteryVoltage_StopADC(me);

	}

}

void BatteryVoltage_StatusCompleteMeasurment(adcMeasureStruct* const me){

}

uint32_t *BatteryVoltage_GetValue(adcMeasureStruct* const me){
	me->statusFunction[me->status](me);
	if(me->status==Battery_CompleteMeasurment){
		me->status=Battery_Idle;
		BatteryVoltage_CalcMean(me);
		return &me->meanValue;
	}else{
		return NULL;
	}
}

void BatteryVoltage_StartMeasurment(adcMeasureStruct* const me){
	if(me->status==Battery_Idle){
		me->resetMeasurment(me);
		BatteryVoltage_EnGpioOn(me);
		me->status=Battery_Stabilize;
	}

}

void BatteryVoltage_AddTime(adcMeasureStruct* const me){
	me->time++;
}

void BatteryVoltage_ResetTime(adcMeasureStruct* const me){
	me->time=0;
}

void BatteryVoltage_ResetMeasurment(adcMeasureStruct* const me){
	me->status=Battery_Idle;
	BatteryVoltage_ResetTime(me);
	me->meanValue=0;
	me->number=0;
	BatteryVoltage_StopADC(me);
	BatteryVoltage_EnGpioOff(me);
}

void BatteryVoltage_SetContinousMeasurment(adcMeasureStruct* const me){
	me->continousMeasurment=SET;
}

void BatteryVoltage_ResetContinousMeasurment(adcMeasureStruct* const me){
	me->continousMeasurment=RESET;
}

void BatteryVoltage_StartADC(adcMeasureStruct* const me){
	HAL_ADC_Start_IT(me->hadc);
}

void BatteryVoltage_StopADC(adcMeasureStruct* const me){
	HAL_ADC_Stop_IT(me->hadc);
}

void BatteryVoltage_EnGpioOn(adcMeasureStruct* const me){
	HAL_GPIO_WritePin(me->gpioTypeDefPowerSupplySchematicMeasurment, me->gpioPinPowerSupplySchematicMeasurment, GPIO_PIN_SET);
}

void BatteryVoltage_EnGpioOff(adcMeasureStruct* const me){
	HAL_GPIO_WritePin(me->gpioTypeDefPowerSupplySchematicMeasurment, me->gpioPinPowerSupplySchematicMeasurment, GPIO_PIN_RESET);
}

void BatteryVoltage_CalcMean(adcMeasureStruct* const me){
	uint32_t sum=0;
	for(int i=0;i<BATTERYVOLTAGE_MAX_MEASURMENTS;i++){
		sum+=me->measurments[i];
	}
	me->meanValue=sum/BATTERYVOLTAGE_MAX_MEASURMENTS;
}

void BatteryVoltage_AddMeasure(adcMeasureStruct* const me,uint32_t value){
	if(me->status==Battery_Measurment){
		if(me->number>=BATTERYVOLTAGE_MAX_MEASURMENTS){
			BatteryVoltage_StopADC(me);
			return;
		}
		me->measurments[me->number]=value;
		me->number++;
	}
}
