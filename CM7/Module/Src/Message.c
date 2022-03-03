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
				Message_InsertDistanceWithPosition,
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
		void(*InsertDistanceWithPosition)(messageStruct* const me,uint32_t  hcSr04,uint16_t vl53l0x,uint8_t position),
		void(*InsertIrSensor)(messageStruct* const me,uint32_t value),
		void(*InsertAdcBatteryVoltage)(messageStruct* const me,uint32_t value),
		void(*SendMessage)(messageStruct* const me)){
	me->buffer=CircularBuffer_Create();
	me->Insert=Insert;
	me->InsertError=InsertError;
	me->InsertDistance=InsertDistance;
	me->InsertDistanceWithPosition=InsertDistanceWithPosition;
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
void Message_InsertDistanceWithPosition(messageStruct* const me,uint32_t  hcSr04,uint16_t vl53l0x,uint8_t position){
	uint8_t *tempArrayHc5r04=(uint8_t*)(&hcSr04);
	me->buffer->insert(me->buffer,0xFF);
	me->buffer->insert(me->buffer,MEASURE_DISTANCE_FOR_PC2);
	me->buffer->insert(me->buffer,position);
	me->buffer->insert(me->buffer,tempArrayHc5r04[0]);
	me->buffer->insert(me->buffer,tempArrayHc5r04[1]);
	me->buffer->insert(me->buffer,tempArrayHc5r04[2]);
	me->buffer->insert(me->buffer,tempArrayHc5r04[3]);
	me->buffer->insert(me->buffer,vl53l0x & 0xFF);
	me->buffer->insert(me->buffer,vl53l0x>>8 & 0xFF);
	me->buffer->insert(me->buffer,0xFE);
	me->buffer->insert(me->buffer,0xFE);
}
void Message_InsertIrSensor(messageStruct* const me,uint32_t value){
	me->buffer->insert(me->buffer,0xFF); //Początek ramki
	me->buffer->insert(me->buffer,SEND_IR_SENSOR_STATUS);
	uint8_t valueAfterMask=(uint8_t)value&(~MESSAGE_IR_MASK);
	me->buffer->insert(me->buffer,valueAfterMask);
	me->buffer->insert(me->buffer,0xFE);
	me->buffer->insert(me->buffer,0xFE);
}
void Message_InsertAdcBatteryVoltage(messageStruct* const me,uint32_t value){
	/*if(value>>24>=0xFF){ //TODO sprawdzić dlaczego niekidy jest wartość FULL
		return;
	}*/
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
			  uint8_t *uartBufferPointer=me->buffer->getDataFromDMA(me->buffer);
			  HAL_UART_Transmit_DMA(me->huart, uartBufferPointer, uartBufferLength);
		  }
	  }
}
