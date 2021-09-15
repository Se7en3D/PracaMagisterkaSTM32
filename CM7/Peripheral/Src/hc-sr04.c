/*
 * hc-sr04.c
 *
 *  Created on: 9 lip 2021
 *      Author: DanielD
 */
#include <stdio.h>
#include "errorCode.h"
#include "hc-sr04.h"

void hcsr04CompCH1Add(uint32_t value){
	if(hcsr04Tim2_p!=0){
		hcsr04Tim2_p->start=value;
	}else{
		errorCodePush(HCSR04_INIT_ERROR);
	}
}

void hcsr04CompCH2Add(uint32_t value){
	if(hcsr04Tim2_p!=0){
		hcsr04Tim2_p->stop=value;
		hcsr04CalculateDistance();
	}else{
		errorCodePush(HCSR04_INIT_ERROR);
	}
}

void hcsr04CalculateDistance(){
	if(hcsr04Tim2_p->isCalculated==0){
		hcsr04Tim2_p->calculatedValue=(hcsr04Tim2_p->stop - hcsr04Tim2_p->start) / 58.0f;
		hcsr04Tim2_p->isCalculated=1;
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
