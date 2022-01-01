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
	irSensor.timerToSendCollision=0;

}

void irSensorReadStatusIrSensor(){
	if(irSensor.timerToReadGPIO>=IRSENSOR_TIME_TO_READ_GPIO){
		irSensor.timerToReadGPIO=0;
	}else{
		return;
	}
	for(int i=0;i<MAX_SENSOR_IR;i++){
		uint8_t value=0;
		if(irSensor.gpioIrPort[i]==NULL){
			irSensor.collision[i]=IR_SENSOR_NOT_DETECTED_COLLISION;
			errorCodePush(IRSENSOR_GPIO_POINTER_NULL);
			continue;
		}

		if(IrSensorExist[i]==IR_SENSOR_EXIST){
#if IRSENSOR_COLLISION_NEGATION
			value=!HAL_GPIO_ReadPin(irSensor.gpioIrPort[i], irSensor.gpioIrPin[i]);
#else
			value=HAL_GPIO_ReadPin(irSensor.gpioIrPort[i], irSensor.gpioIrPin[i]);
#endif
		irSensorAddSample(&irSensor,i,value);

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
	irSensor.timerToSendCollision++;
	irSensor.timerToReadGPIO++;
}
uint32_t irSensorGetTime(){
	return irSensor.timerToSendCollision;
}
void irSensorClearTime(){
	irSensor.timerToSendCollision=0;
}
void irSensorAddSample(ir_sensor_t *irSensor,const uint8_t sensorId,const uint8_t value){
	uint8_t sumOfSetGPIO=irSensor->sumOfSetGPIO[sensorId];
	if(value){
		if(sumOfSetGPIO<IRSENSOR_MAX_SAMPLE){
			(sumOfSetGPIO)++;
		}
	}else{
		if(sumOfSetGPIO>0){
			(sumOfSetGPIO)--;
		}
	}

	irSensor->sumOfSetGPIO[sensorId]=sumOfSetGPIO;

	if(sumOfSetGPIO>=(IRSENSOR_MAX_SAMPLE/2)){
		irSensor->collision[sensorId]=IR_SENSOR_DETECTED_COLLISION;
	}else{
		irSensor->collision[sensorId]=IR_SENSOR_NOT_DETECTED_COLLISION;
	}
}

