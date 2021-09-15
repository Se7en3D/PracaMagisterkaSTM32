/*
 * uartCom.c
 *
 *  Created on: 12 lip 2021
 *      Author: DanielD
 */

#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "uartCom.h"
#include "errorCode.h"
#include "stateMachine.h"

void uartComAddFrame(uint8_t data){
	if(uartCommunication_p!=0){ //Sprawdzenie istnienia wskażnika uartCommunication_p
		uint8_t *position=&uartCommunication_p->positionCommand;
		if(uartCommunication_p->detectedCommand==0){ //Sprawdzenie czy wykonano wcześniej odebraną komendę
			if(uartCommunication_p->positionCommand==0){//Sprawdzenie czy w buforze ramek znajduje się początek ramki
				if(data==0xFF){ //Sprawdzenie czy odebrana ramka to 0xFF(początek transmisji)
					uartCommunication_p->frameCommand[*position]=data;
					*position=*position+1;
				}else{
					return;
				}
			}else{//w buforze znajdują sie już dane
				if(data==0xFE){ //Sprawdzenie czy odebrana ramka to 0xFE(koniec transmisji)
					uartCommunication_p->frameCommand[*position]=data;
					*position=0;
					uartCommunication_p->detectedCommand=1;
				}else{
					if(*position+1>=MAXCOMMENDLENGTH){ //Odebrana ramka jest niepoprawna
						*position=0;
						errorCodePush(UARTCOM_ADDFRAME_INVALID_LENGTH_FRAME);
					}else{
						uartCommunication_p->frameCommand[*position]=data;
						*position=*position+1;
					}
				}
			}
		}else{
			errorCodePush(UARTCOM_ADDFRAME_NOT_EXECUTE_PREVIOUS_FUN);
		}
	}else{
		errorCodePush(UARTCOM_INIT_ERROR);
	}
}

uint8_t* uartComGetFrame(){
	if(uartCommunication_p==0){
		return 0;
	}
	if(uartCommunication_p->detectedCommand==0){
		return 0;
	}else{
		return uartCommunication_p->frameCommand;
	}
}

void uartComClearFrame(){
	if(uartCommunication_p==0){
		return;
	}
	if(uartCommunication_p->detectedCommand==0){
		return;
	}else{
		uartCommunication_p->detectedCommand=0;
		uartCommunication_p->positionCommand=0;
	}
}

void uartComSendDistance(float hc5r04, uint16_t vl5310x){ //float-32bit uint16_t-16bit
	if(uartCommunication_p==0){
		return ;
	}
	uint8_t *tempArrayHc5r04=(uint8_t*)(&hc5r04);
	uartComPush(0xFF); //Początek ramki
	uartComPush(MEASURE_DISTANCE_FUN_RECEIVED);
	uartComPush(tempArrayHc5r04[0]);
	uartComPush(tempArrayHc5r04[1]);
	uartComPush(tempArrayHc5r04[2]);
	uartComPush(tempArrayHc5r04[3]);
	uartComPush(vl5310x & 0xFF);
	uartComPush(vl5310x>>8 & 0xFF);
	uartComPush(0xFE);
	uartComPush(0xFE);
}
void uartComSendErrorCode(uint8_t errorCode){
	if(uartCommunication_p==0){
		return ;
	}
	//uint8_t arrayPrepared[5];
	uartComPush(0xFF);
	uartComPush(ERROR_CODE_FUN);
	uartComPush(errorCode);
	uartComPush(0xFE);
	uartComPush(0xFE);
}

void uartComSendIrSensorStatus(uint8_t *collision,uint16_t size){
	if(uartCommunication_p==0){
		return ;
	}
	uint32_t status=0;
	if(size>32){
		errorCodePush(UARTCOM_MAX_LENGTH_IR_STATUS_SEND);
		return;
	}
	for(int i=0;i<size;i++){
		if(collision[i]>0){
			status|=1<<i;
		}
	}
	uartComPush(0xFF); //Początek ramki
	uartComPush(SEND_IR_SENSOR_STATUS);

	if(size<=8){
		uartComPush((uint8_t) status);
	}else if(size<=16){
		uartComPush((uint8_t) status>>8);
		uartComPush((uint8_t) status);
	}else if(size<=32){
		uartComPush((uint8_t) status>>24);
		uartComPush((uint8_t) status>>16);
		uartComPush((uint8_t) status>>8);
		uartComPush((uint8_t) status);
	}

	uartComPush(0xFE);
	uartComPush(0xFE);

}
void uartComSendAdcBatteryVoltage(uint32_t value){
	if(uartCommunication_p==0){
		return ;
	}

	uartComPush(0xFF);
	uartComPush(SEND_BATTERY_MEASURMENT_VALUE);
	uartComPush(value>>24);
	uartComPush(value>>16);
	uartComPush(value>>8);
	uartComPush(value);
	uartComPush(0xFE);
	uartComPush(0xFE);
}
void uartComSendDistanceServo(float hcSr04, uint16_t vl5310x){
	if(uartCommunication_p==0){
		return ;
	}
	uint8_t *tempArrayHc5r04=(uint8_t*)(&hcSr04);
	uartComPush(0xFF); //Początek ramki
	uartComPush(SEND_MEASUREMENT_FUN);
	uartComPush(uartComSensorFrame.position);
	uartComPush(tempArrayHc5r04[0]);
	uartComPush(tempArrayHc5r04[1]);
	uartComPush(tempArrayHc5r04[2]);
	uartComPush(tempArrayHc5r04[3]);
	uartComPush(vl5310x & 0xFF);
	uartComPush(vl5310x>>8 & 0xFF);
	uartComPush(0xFE);
	uartComPush(0xFE);
}


void uartComSensorInit(TIM_HandleTypeDef *htim){
	uartComSensorFrame.htimServo=htim;
	const uint16_t period=htim->Init.Period+1;
	const uint16_t ms1=period/20;
	uartComBuffer.maxlen=UARTCOM_BUF_MAXLEN;
	uartComBuffer.head=0;
	uartComBuffer.tail=0;
	for(int i=0;i<SERVO_MAX_POSITION;i++){
		uartComSensorFrame.tableDegreesServo[i]=ms1/SERVO_MAX_POSITION*i+ms1-1;
	}
}

void uartComServoNextPosition(){
	driving_structure_t  drivingStructure=stateMachineGetDrivingStructure();
	uint32_t drivingStatus=drivingStructure.drivingStatus;
	printf("status numer=%d pozycja=%d\n ",(int )drivingStatus,(int )uartComSensorFrame.position);
	if(uartComSensorFrame.position<uartComArrayServoMaxMin[drivingStatus][0] || uartComSensorFrame.position>=uartComArrayServoMaxMin[drivingStatus][1]){
		uartComSensorFrame.position=uartComArrayServoMaxMin[drivingStatus][0];
	}

	if(uartComSensorFrame.htimServo==0){
		errorCodePush(UARTCOM_HTIM_POINTER_NULL);
		return;
	}

	__HAL_TIM_SET_COMPARE(uartComSensorFrame.htimServo,TIM_CHANNEL_1,uartComSensorFrame.tableDegreesServo[uartComSensorFrame.position]);
	uartComSensorFrame.position++;


/*
	if(uartComSensorFrame.position>=(SERVO_MAX_POSITION-1)){
		uartComSensorFrame.position=0;
	}else{
		uartComSensorFrame.position++;
	}
	if(uartComSensorFrame.htimServo==0){
		errorCodePush(UARTCOM_HTIM_POINTER_NULL);
	}
	__HAL_TIM_SET_COMPARE(uartComSensorFrame.htimServo,TIM_CHANNEL_1,uartComSensorFrame.tableDegreesServo[uartComSensorFrame.position]);
*/
}

void uartComServoPreviousPosition(){
	driving_structure_t  drivingStructure=stateMachineGetDrivingStructure();
	uint32_t drivingStatus=drivingStructure.drivingStatus;
	printf("status numer=%d pozycja=%d\n ",(int )drivingStatus,(int )uartComSensorFrame.position);
	if(uartComSensorFrame.position<uartComArrayServoMaxMin[drivingStatus][0] || uartComSensorFrame.position>=uartComArrayServoMaxMin[drivingStatus][1]){
		uartComSensorFrame.position=uartComArrayServoMaxMin[drivingStatus][1]-1;
	}

	if(uartComSensorFrame.htimServo==0){
		errorCodePush(UARTCOM_HTIM_POINTER_NULL);
		return;
	}

	__HAL_TIM_SET_COMPARE(uartComSensorFrame.htimServo,TIM_CHANNEL_1,uartComSensorFrame.tableDegreesServo[uartComSensorFrame.position]);
	uartComSensorFrame.position--;
	/*
	if(uartComSensorFrame.position==0){
		uartComSensorFrame.position=SERVO_MAX_POSITION-1;
	}else{
		uartComSensorFrame.position--;
	}
	if(uartComSensorFrame.htimServo==0){
		errorCodePush(UARTCOM_HTIM_POINTER_NULL);
	}
	__HAL_TIM_SET_COMPARE(uartComSensorFrame.htimServo,TIM_CHANNEL_1,uartComSensorFrame.tableDegreesServo[uartComSensorFrame.position]);
*/
}

void uartComServoIsShiftPosition(){
	if(uartComSensorFrame.waitForData==0){
		uartComSensorFrame.waitForData=1;
		uartComServoNextPosition();
	}
}
void uartComServoClearWaitForData(){
	uartComSensorFrame.waitForData=0;
}

void uartComPush(const uint8_t data){
	if(uartComBuffer.maxlen!=UARTCOM_BUF_MAXLEN){
		return;
	}
	int next;
	next = uartComBuffer.head + 1;  // next is where head will point to after this write.
	if (next >= (uartComBuffer.maxlen-1))
			next = 0;

	if (next == uartComBuffer.tail){  // if the head + 1 == tail, circular buffer is full
		uartComBuffer.buffer[uartComBuffer.head] = data;  // Load data and then move
		return ;
	}

	uartComBuffer.buffer[uartComBuffer.head] = data;  // Load data and then move
	uartComBuffer.head = next;             // head to next data offset.
}

uint8_t *uartComGetBufferFirstElementAddress(){
	return &uartComBuffer.buffer[uartComBuffer.tail];
}

uint32_t uartComGetBufferLength(){
	if(uartComBuffer.head==uartComBuffer.tail){
		return 0;
	}

	if(uartComBuffer.head<uartComBuffer.tail){
		uint32_t length=uartComBuffer.maxlen-uartComBuffer.tail-1;
		uartComBuffer.tail=0;
		return length;
	}else{
		uint32_t length=uartComBuffer.head-uartComBuffer.tail;
		uartComBuffer.tail=uartComBuffer.head;
		return length;
	}
}
uint8_t uartComGetMeasureForMeasureDistanceFun(){
	return uartComSensorFrame.measureForMeasureDistanceFun;
}

void uartComAddMeasureForMeasureDistanceFun(){
	uartComSensorFrame.measureForMeasureDistanceFun++;
}
void uartComResetMeasureForMeasureDistanceFun(){
	uartComSensorFrame.measureForMeasureDistanceFun=0;
}

