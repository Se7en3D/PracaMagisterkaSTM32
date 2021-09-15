/*
 * adc.c
 *
 *  Created on: 5 sie 2021
 *      Author: DanielD
 */

#include "stm32h7xx_hal.h"
#include "errorCode.h"
#include "adc.h"



void adcInit(ADC_HandleTypeDef *hadc1,GPIO_TypeDef *gpioPowerMeasureOn_OffPort,uint32_t gpioPowerMeasureOn_OffPin){
	adcBaseStructure.hadc=hadc1;
	adcBaseStructure.stage=adcWaitForTurnOnPowerByOptocoupler;
	adcBaseStructure.fun_ptr[0]=&adcTurnOnPowerOptocoupler;
	adcBaseStructure.fun_ptr[1]=&adcWaitForStabilizeVoltage;
	adcBaseStructure.fun_ptr[2]=&adcWaitForConversionValue;
	adcBaseStructure.fun_ptr[3]=&adcReadyToSendData;
	adcBaseStructure.gpioPowerMeasureOn_OffPort=gpioPowerMeasureOn_OffPort;
	adcBaseStructure.gpioPowerMeasureOn_OffPin=gpioPowerMeasureOn_OffPin;
}

void adcSetConversionValue(uint32_t data){
	if(adcBaseStructure.hadc==0){
		errorCodePush(ADC_HADC_NULL_POINTER);
		return;
	}
	adcBaseStructure.value=data;
	adcBaseStructure.readyToSend=ADC_READY_TO_SEND;
}

uint32_t adcTurnOnPowerOptocoupler(){ //Funkcja przygotowująca do załączenia zasilania
	if(adcBaseStructure.time>=ADC_TIME_TO_START_CONVERSION){
		adcBaseStructure.time=0;
		adcBaseStructure.stage=adcWaitoForStabilizeVoltage;
		HAL_GPIO_WritePin(adcBaseStructure.gpioPowerMeasureOn_OffPort, adcBaseStructure.gpioPowerMeasureOn_OffPin, GPIO_PIN_SET);
		return ADC_TRUN_ON_POWER;
	}else{
		return ADC_NOT_REACTION;
	}
}

uint32_t adcWaitForStabilizeVoltage(){ //Funkcja oczekiwania na załączenie przetwornika ADC
	if(adcBaseStructure.time>=ADC_TIME_TO_STABILIZE_VOLTAGE){
		adcBaseStructure.time=0;
		adcBaseStructure.stage=adcMeasureVoltage;
		HAL_ADC_Start_IT(adcBaseStructure.hadc);
		return ADC_START_CONVERSION;
	}else{
		return ADC_NOT_REACTION;
	}
}
uint32_t adcWaitForConversionValue(){
	if(adcBaseStructure.value!=ADC_WRONG_CONV){
		adcBaseStructure.stage=adcSendVoltage;
		return ADC_READY_TO_SEND;
	}else{
		return ADC_NOT_REACTION;
	}

}

uint32_t adcReadyToSendData(){
	if(adcBaseStructure.value==ADC_WRONG_CONV){
		adcBaseStructure.stage=adcWaitForTurnOnPowerByOptocoupler;
		HAL_GPIO_WritePin(adcBaseStructure.gpioPowerMeasureOn_OffPort, adcBaseStructure.gpioPowerMeasureOn_OffPin, GPIO_PIN_RESET);
		adcBaseStructure.time=0;
		return ADC_READY_TO_SEND;
	}else{
		return ADC_TRUN_ON_POWER;
	}

}
uint32_t adcGetValue(){
	if(adcBaseStructure.stage==adcSendVoltage){
		uint32_t tempData=adcBaseStructure.value;
		adcBaseStructure.value=ADC_WRONG_CONV;
		return tempData;
	}else{
		return ADC_WRONG_CONV;
	}
}

void adcAddTime(){
	adcBaseStructure.time++;
}
uint32_t adcGetTime(){
	return adcBaseStructure.time;
}

void adcClearTime(){
	adcBaseStructure.time=0;
}

uint32_t adcStage(){
	return adcBaseStructure.fun_ptr[adcBaseStructure.stage]();
}
