/*
 * irSensor.c
 *
 *  Created on: 3 sie 2021
 *      Author: DanielD
 */

#include "irSensor.h"
#include "errorCode.h"

void irSensorInitIrPinout(uint8_t number,uint16_t Pin,GPIO_TypeDef *port){
	if(number>MAX_SENSOR_IR){
		errorCodePush(IRSENSOR_NUMBER_OUT_OF_SIZE);
		return;
	}

	irSensor.gpioIrPort[number]=port;
	irSensor.gpioIrPin[number]=Pin;
	irSensor.collision[number]=0;
	irSensor.timer=0;
}

void irSensorReadStatusIrSensor(){
	for(int i=0;i<MAX_SENSOR_IR;i++){

		if(irSensor.gpioIrPort[i]==NULL){
			irSensor.collision[i]=IR_SENSOR_NOT_DETECTED_COLLISION;
			errorCodePush(IRSENSOR_GPIO_POINTER_NULL);
			continue;
		}

		if(IrSensorExist[i]==IR_SENSOR_EXIST){
			irSensor.collision[i]=!HAL_GPIO_ReadPin(irSensor.gpioIrPort[i], irSensor.gpioIrPin[i]);
		}else{
			irSensor.collision[i]=IR_SENSOR_NOT_DETECTED_COLLISION;
		}
	}
}
uint8_t irSensorGetCollision(uint8_t number){
	return irSensor.collision[number];
}
uint8_t *irSensorGetAllCollision(){
	return &irSensor.collision[0];
}
void irSensorAddTime(){
	irSensor.timer++;
}
uint32_t irSensorGetTime(){
	return irSensor.timer;
}
void irSensorClearTime(){
	irSensor.timer=0;
}
