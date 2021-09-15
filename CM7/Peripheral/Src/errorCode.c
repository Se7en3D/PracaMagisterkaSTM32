/*
 * errorCode.c
 *
 *  Created on: 14 lip 2021
 *      Author: DanielD
 */
#include <stdio.h>
#include "errorCode.h"

void errorCodeInit(){
	errorCode_s.maxlen=ERROR_CODE_MAXLEN;
	errorCode_s.head=0;
	errorCode_s.tail=0;
}
void errorCodePush(const uint8_t data){
	if(errorCode_s.maxlen!=ERROR_CODE_MAXLEN){
		return;
	}
	int next;
	next = errorCode_s.head + 1;  // next is where head will point to after this write.
	if (next >= (errorCode_s.maxlen-1))
			next = 0;

	if (next == errorCode_s.tail){  // if the head + 1 == tail, circular buffer is full
		errorCode_s.buffer[errorCode_s.head] = data;  // Load data and then move
		return ;
	}

	errorCode_s.buffer[errorCode_s.head] = data;  // Load data and then move
	errorCode_s.head = next;             // head to next data offset.
}
uint8_t errorCodePop(){
	uint8_t data;
	int next;
	if(errorCode_s.maxlen!=ERROR_CODE_MAXLEN){
		return UNIMPORTANT_ERROR;
	}

	if (errorCode_s.head == errorCode_s.tail){  // if the head == tail, we don't have any data
		return UNIMPORTANT_ERROR;
	}
	next = errorCode_s.tail + 1;  // next is where tail will point to after this read.
	if (next >= errorCode_s.maxlen)
		next = 0;

	data = errorCode_s.buffer[errorCode_s.tail];  // Read data and then move
	errorCode_s.tail = next;              // tail to next offset.
	return data;  // return success to indicate successful push.
}
