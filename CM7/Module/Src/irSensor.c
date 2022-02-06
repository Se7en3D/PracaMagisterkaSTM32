/*
 * irSensor.c
 *
 *  Created on: 31 sty 2022
 *      Author: Daniel
 */
#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "irSensor.h"

irSensorStruct* IrSensor_Create(){
	irSensorStruct *me=malloc(sizeof(irSensorStruct));
	if(me!=NULL){
		me->first=NULL;
		IrSensor_Init(
				me,
				IrSensor_ReadGPIO,
				IrSensor_GetValueGPIO,
				IRSensor_AddGPIO,
				IrSensor_GetSize);
	}
	return me;
}
void IrSensor_Init(irSensorStruct*  const me,
		void (*readStatus)(irSensorStruct*  const me),
		uint32_t (*getValue)(irSensorStruct* const me),
		void (*addGPIO)(irSensorStruct* const me, GPIO_TypeDef *gpio, uint16_t pin),
		uint32_t (*getSize)(irSensorStruct* const me)){
	me->readStatus=readStatus;
	me->getValue=getValue;
	me->addGPIO=addGPIO;
	me->getSize=getSize;
}
void IrSensor_ReadGPIO(irSensorStruct* const me){
	linkListGPIO *linkList=me->first;
	while(linkList!=NULL){
		uint8_t value=HAL_GPIO_ReadPin(linkList->gpio, linkList->pin);
		if(value){
			if(linkList->value<IRSENSOR_MAX_ON_VALUE){
				linkList->value++;
			}
		}else{
			if(linkList->value>0){
				linkList->value--;
			}
		}
		linkList=linkList->next;
	}
}
uint32_t IrSensor_GetValueGPIO(irSensorStruct* const me){
	uint32_t value=0;
	linkListGPIO *linkList=me->first;
	uint32_t i=0;
	while(linkList!=NULL){
		if(linkList->value>=(IRSENSOR_MAX_ON_VALUE/2)){
			value|=(1<<i);
		}
		i++;
		linkList=linkList->next;
	}
	return value;
}
uint32_t IrSensor_GetSize(irSensorStruct* const me){
	uint32_t i=0;
	linkListGPIO *linkList=me->first;
	while(linkList!=NULL){
		i++;
		linkList=linkList->next;
	}
	return i;
}
void IRSensor_AddGPIO(irSensorStruct* const me, GPIO_TypeDef *gpio, uint16_t pin){
	linkListGPIO *next=malloc(sizeof(linkListGPIO));
	next->gpio=gpio;
	next->pin=pin;
	next->value=0;
	next->next=NULL;
	if(me->first==NULL){
		me->first=next;
	}else{
		linkListGPIO *linkList=me->first;
		while(linkList->next!=NULL){
			linkList=linkList->next;
		}
		linkList->next=next;
	}

}
