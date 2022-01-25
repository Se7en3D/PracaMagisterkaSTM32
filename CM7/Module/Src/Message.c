#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "Message.h"


messageStruct *Message_Create(UART_HandleTypeDef *huart){
	messageStruct *me=malloc(sizeof(messageStruct));
	if(me!=NULL && huart!=NULL){
		me->huart=huart;
		Message_Init(me,
				Message_Insert,
				Message_InsertError,
				Message_InsertDistance,
				Message_InsertIrSensor,
				Message_InsertAdcBatteryVoltage,
				Message_SendMessage);
		return me;
	}else{
		return NULL;
	}
}
void Message_Init(messageStruct* const me,
		void(*Insert)(messageStruct* const me,uint8_t data),
		void(*InsertError)(messageStruct* const me,uint8_t data),
		void(*InsertDistance)(messageStruct* const me,float  hcSr04,uint16_t vl53l0x),
		void(*InsertIrSensor)(messageStruct* const me,uint8_t  *collision,uint16_t size),
		void(*InsertAdcBatteryVoltage)(messageStruct* const me,uint32_t value),
		void(*SendMessage)(messageStruct* const me)){
	me->buffer=CircularBuffer_Create();
	me->Insert=Insert;
	me->InsertError=InsertError;
	me->InsertDistance=InsertDistance;
	me->InsertIrSensor=InsertIrSensor;
	me->InsertAdcBatteryVoltage=InsertAdcBatteryVoltage;
	me->SendMessage=SendMessage;
}

void Message_Insert(messageStruct* const me,uint8_t data){
	me->buffer->insert(me->buffer,data);
}
void Message_InsertError(messageStruct* const me,uint8_t data){
	me->buffer->insert(me->buffer,0xFF);
	me->buffer->insert(me->buffer,ERROR_CODE_FUN);
	me->buffer->insert(me->buffer,data);
	me->buffer->insert(me->buffer,0xFE);
	me->buffer->insert(me->buffer,0xFE);
}
void Message_InsertDistance(messageStruct* const me,float  hcSr04,uint16_t vl53l0x){
	uint8_t *tempArrayHc5r04=(uint8_t*)(&hcSr04);
	me->buffer->insert(me->buffer,0xFF);
	me->buffer->insert(me->buffer,MEASURE_DISTANCE_FUN_RECEIVED);
	me->buffer->insert(me->buffer,tempArrayHc5r04[0]);
	me->buffer->insert(me->buffer,tempArrayHc5r04[1]);
	me->buffer->insert(me->buffer,tempArrayHc5r04[2]);
	me->buffer->insert(me->buffer,tempArrayHc5r04[3]);
	me->buffer->insert(me->buffer,vl53l0x & 0xFF);
	me->buffer->insert(me->buffer,vl53l0x>>8 & 0xFF);
	me->buffer->insert(me->buffer,0xFE);
	me->buffer->insert(me->buffer,0xFE);
}
void Message_InsertIrSensor(messageStruct* const me,uint8_t  *collision,uint16_t size){

	uint32_t status=0;
	if(size>32){
			//TODO Error dla przekroczenia wielkości
		return;
	}
	for(int i=0;i<size;i++){
		if(collision[i]>0){
			status|=1<<i;
		}
	}
	me->buffer->insert(me->buffer,0xFF); //Początek ramki
	me->buffer->insert(me->buffer,SEND_IR_SENSOR_STATUS);
	if(status>=0xFF){
		printf("error\n"); //TODO Error dla złego statusu
	}
	if(size<=8){
		me->buffer->insert(me->buffer,(uint8_t) status);
	}else if(size<=16){
		me->buffer->insert(me->buffer,(uint8_t) status>>8);
		me->buffer->insert(me->buffer,(uint8_t) status);
	}else if(size<=32){
		me->buffer->insert(me->buffer,(uint8_t) status>>24);
		me->buffer->insert(me->buffer,(uint8_t) status>>16);
		me->buffer->insert(me->buffer,(uint8_t) status>>8);
		me->buffer->insert(me->buffer,(uint8_t) status);
	}

	me->buffer->insert(me->buffer,0xFE);
	me->buffer->insert(me->buffer,0xFE);
}
void Message_InsertAdcBatteryVoltage(messageStruct* const me,uint32_t value){
	if(value>>24>=0xFF){ //TODO sprawdzić dlaczego niekidy jest wartość FULL
		return;
	}
	me->buffer->insert(me->buffer,0xFF);
	me->buffer->insert(me->buffer,SEND_BATTERY_MEASURMENT_VALUE);
	me->buffer->insert(me->buffer,value>>24);
	me->buffer->insert(me->buffer,value>>16);
	me->buffer->insert(me->buffer,value>>8);
	me->buffer->insert(me->buffer,value);
	me->buffer->insert(me->buffer,0xFE);
	me->buffer->insert(me->buffer,0xFE);
}
void Message_SendMessage(messageStruct* const me){
		DMA_HandleTypeDef *hdma_uart4_tx=me->huart->hdmatx;
	  if(hdma_uart4_tx->State==HAL_DMA_STATE_READY){
		  uint32_t uartBufferLength=me->buffer->getSize(me->buffer);
		  if(uartBufferLength){
			  uint8_t *uartBufferPointer=me->buffer->getDataToDMA(me->buffer);
			  HAL_UART_Transmit_DMA(me->huart, uartBufferPointer, uartBufferLength);
		  }
	  }
}
