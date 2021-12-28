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

	for(int i=0;i<MAX_SENSOR_IR;i++){
		irSensorResetModeStruct(&irSensor.modeStruct[i]);
	}
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
		//if(i==0){
		//	printf("\n");
		//}
		irSensorAddSample(&irSensor.modeStruct[i],value);
		irSensorCalcCollisionValue(&irSensor,i);
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
void irSensorAddSample(irModeStruct *mode,const uint8_t value){
	uint8_t head=mode->head;
	mode->gpioReadPinArray[head]=value;
	head++;
	if(head>=IRSENSOR_MAX_SAMPLE){
		head=0;
	}
	//if(mode->gpioReadPinArray[head]!=value){
		if(value==IR_SENSOR_DETECTED_COLLISION){
			if(mode->countOffPin>0){
				mode->countOffPin--;
				mode->countOnPin++;
			}
		}else{
			if(mode->countOnPin>0){
				mode->countOffPin++;
				mode->countOnPin--;
			}
		}
	//}
	mode->head=head;
}
void irSensorCalcCollisionValue(ir_sensor_t *irSensor,const uint8_t sensorId){
	/*uint8_t countOnPin=irSensor->modeStruct[sensorId].countOnPin;
	uint8_t countOffPin=irSensor->modeStruct[sensorId].countOffPin;
	if(countOnPin>countOffPin){
		irSensor->collision[sensorId]=IR_SENSOR_DETECTED_COLLISION;
	}else{
		irSensor->collision[sensorId]=IR_SENSOR_NOT_DETECTED_COLLISION;
	}*/
	uint8_t count1=0;
	uint8_t halSize=IRSENSOR_MAX_SAMPLE/2;
	for(int i=0;i<IRSENSOR_MAX_SAMPLE;i++){
		if(irSensor->modeStruct[sensorId].gpioReadPinArray[i]==IR_SENSOR_DETECTED_COLLISION){
			count1++;
		}
	}
	if(count1>halSize){
		irSensor->collision[sensorId]=IR_SENSOR_DETECTED_COLLISION;
	}else{
		irSensor->collision[sensorId]=IR_SENSOR_NOT_DETECTED_COLLISION;
	}
}
void irSensorResetModeStruct(irModeStruct *mode){
	for(int i=0;i<IRSENSOR_MAX_SAMPLE;i++){
		mode->gpioReadPinArray[i]=0;
	}
	mode->countOffPin=IRSENSOR_MAX_SAMPLE;
	mode->countOnPin=0;
	mode->head=0;
}
