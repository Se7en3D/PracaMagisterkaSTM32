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

		if(irSensor.gpioIrPort[i]==0){
			irSensor.collision[i]=0;
			errorCodePush(IRSENSOR_GPIO_POINTER_NULL);
			continue;
		}
		irSensor.collision[i]=!HAL_GPIO_ReadPin(irSensor.gpioIrPort[i], irSensor.gpioIrPin[i]);

	}

	/*
			 * MASKOWANIE IR_8 BłĄD w POłĄCZENIU ???
			 */
			irSensor.collision[IR_NR8_NUMBER]=0;
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
