/*
 * CommandDecoder.c
 *
 *  Created on: 27 sty 2022
 *      Author: Daniel
 */

#include <stdio.h>
#include <stdlib.h>
#include "CommandDecoder.h"
#include "ErrorValue.h"


bluetoothDecoderStruct* CommandDecoder_Create(){
	bluetoothDecoderStruct *me=malloc(sizeof(bluetoothDecoderStruct));
	if(me!=NULL){
		CommandDecoder_Init(me,
				CommandDecoder_Insert,
				CommandDecoder_GetFunction,
				CommandDecoder_ClearBuffer,
				CommandDecoder_AddTimeout,
				CommandDecoder_DecodeTheFunction);
		return me;
	}
	return NULL;
}

void CommandDecoder_Init(bluetoothDecoderStruct* const me,
		void(*insert)(bluetoothDecoderStruct* const me, const uint8_t value),
		uint8_t* (*getFunction)(bluetoothDecoderStruct* const me),
		void (*clearBuffer)(bluetoothDecoderStruct* const me),
		void (*addTimeout)(bluetoothDecoderStruct* const me),
		uint8_t (*DecodeTheFunction)(bluetoothDecoderStruct* const me)
		){
	CommandDecoder_ClearBuffer(me);
	me->Insert=insert;
	me->GetFunction=getFunction;
	me->ClearBuffer=clearBuffer;
	me->AddTimeout=addTimeout;
	me->StatusFunction[decoder_Idle]=CommandDecoder_StatusIdle;
	me->StatusFunction[decoder_WaitToEnd]=CommandDecoder_StatusWaitToEnd;
	me->StatusFunction[decoder_EndDecod]=CommandDecoder_StatusEndDecod;
	me->DecodeFunction=DecodeTheFunction;

}

void CommandDecoder_Insert(bluetoothDecoderStruct* const me, const uint8_t value){
	me->StatusFunction[me->status](me,value);
	CommandDecoder_ResetTimeout(me);
}

uint8_t* CommandDecoder_GetFunction(bluetoothDecoderStruct* const me){
	if(me->status==decoder_EndDecod){
		return &me->buffer[COMMANDDECODER_POSITION_FUNCTION_IN_FRAME];
	}
	return NULL;
}
void CommandDecoder_ClearBuffer(bluetoothDecoderStruct* const me){
	me->head=0;
	me->timeout=0;
	me->status=decoder_Idle;
}
void CommandDecoder_AddTimeout(bluetoothDecoderStruct* const me){
	if(me->status!=decoder_Idle){
		me->timeout++;

		if(me->timeout>COMMADNDECODER_TIME_TO_CLEAR_BUFFER){
			addErrorValue(CommandDecoder_Timeout_Decod);
			me->ClearBuffer(me);
		}
	}
}
void CommandDecoder_ResetTimeout(bluetoothDecoderStruct* const me){
	me->timeout=0;
}
void CommandDecoder_StatusIdle(bluetoothDecoderStruct* const me, const uint8_t value){
	if(COMMANDDECODER_START_BYTE==value){
		me->status=decoder_WaitToEnd;
		me->buffer[me->head]=value;
		me->head++;
		CommandDecoder_CheckOutOfHeadValue(me);
	}
}
void CommandDecoder_StatusWaitToEnd(bluetoothDecoderStruct* const me, const uint8_t value){
	me->buffer[me->head]=value;
	if(me->buffer[me->head]==COMMANDDECODER_STOP_BYTE
			&&
			me->buffer[me->head-1]==COMMANDDECODER_STOP_BYTE){
		me->status=decoder_EndDecod;
	}
	me->head++;
	CommandDecoder_CheckOutOfHeadValue(me);
}
void CommandDecoder_StatusEndDecod(bluetoothDecoderStruct* const me, const uint8_t value){
	if(me->status==decoder_EndDecod){
		addErrorValue(CommandDecoder_WaitToHandle);
	}
}
void CommandDecoder_CheckOutOfHeadValue(bluetoothDecoderStruct* const me){
	if(me->head>=COMMANDDECODER_MAX_LENGTH_COMMAND){
		addErrorValue(CommandDecoder_OutOfBuffer);
		me->ClearBuffer(me);
	}
}
uint8_t CommandDecoder_DecodeTheFunction(bluetoothDecoderStruct* const me){
	functionFromPcEnum decodedValue=FUN_NOT_RECOGNIZED;
	uint8_t undecodedValue=*me->GetFunction(me);
	switch(undecodedValue){
	case 0x01:
		decodedValue=RIDE_FORWARD_FUN;
		break;
	case 0x02:
		decodedValue=RIDE_BACKWARD_FUN;
		break;
	case 0x03:
		decodedValue=RIDE_RIGHT_FUN;
		break;
	case 0x04:
		decodedValue=RIDE_LEFT_FUN;
		break;
	case 0x05:
		decodedValue=ROTATE_LEFT;
		break;
	case 0x06:
		decodedValue=ROTATE_RIGHT;
		break;
	case 0x07:
		decodedValue=RESET_DRIVE;
		break;
	case 0x08:
		decodedValue=RIDE_BACKWARD_RIGHT;
		break;
	case 0x09:
		decodedValue=RIDE_BACKWARD_LEFT;
		break;
	case 0x82:
		decodedValue=GET_STATUS_ALL_STRUCTURE;
		break;
	case 0xF0:
		decodedValue=MEASURE_DISTANCE_FOR_PC;
		break;
	default:
		decodedValue=FUN_NOT_RECOGNIZED;
		break;
	}
	return decodedValue;
}
void __attribute__((weak)) addErrorValue(uint8_t value ){

}
