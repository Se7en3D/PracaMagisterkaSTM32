/*
 * hc-sr04.c
 *
 *  Created on: 9 lip 2021
 *      Author: DanielD
 */
#include <stdio.h>
#include "errorCode.h"
#include "hc-sr04.h"

void hcsr04TIMInterruptAdd(hcsr04_t *hcsr04,uint32_t value){
	HC_SR04Status status=hcsr04->status;
	uint8_t risingMeasurment=(status==HC_SR04_MEASURMENT_RISING_EDGE);
	uint8_t fallingMeasurment=(status==HC_SR04_MEASURMENT_FALLING_EDGE);
	if(risingMeasurment || fallingMeasurment){
		if(risingMeasurment){
			hcsr04->risingTime=value;
			hcsr04->status=HC_SR04_MEASURMENT_FALLING_EDGE;
		}

		if(fallingMeasurment){
			hcsr04->fallingTime=value;
			hcsr04->fallingTime=value;
		}
	}
}

void hcsr04CalculateDistance(){
	if(hcsr04Tim2_p->isCalculated==0){
		hcsr04Tim2_p->calculatedValue=(hcsr04Tim2_p->stop - hcsr04Tim2_p->start) / 58.0f;
		hcsr04Tim2_p->isCalculated=1;
	}
}
uint8_t hcsr04IsReadyToSend(){
	if(hcsr04Tim2_p->isCalculated==1){
		return 1;
	}else{
		return 0;
	}
}

float hcsr04GetCelculatedValue(){
	if(hcsr04Tim2_p!=0){
		if(hcsr04Tim2_p->isCalculated==1){
			hcsr04Tim2_p->isCalculated=0;
			hcsr04Tim2_p->timeout=0;
			return hcsr04Tim2_p->calculatedValue;
		}else{
			hcsr04Tim2_p->timeout++;
			return 0.0;
		}

	}else{
		errorCodePush(HCSR04_INIT_ERROR);
		return 0.0;
	}
}

uint32_t hcsr04GetTimeout(){
	return hcsr04Tim2_p->timeout++;;
}
void hcsr04ClearTimeout(){
	hcsr04Tim2_p->timeout=0;
}

void hcsr04ClearMeasurement(){
	hcsr04Tim2_p->isCalculated=0;
	hcsr04Tim2_p->timeout=0;
}
