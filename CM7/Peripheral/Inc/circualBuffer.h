/*
 * circualBuffer.h
 *
 *  Created on: 5 lis 2021
 *      Author: Daniel
 */

#ifndef INC_CIRCUALBUFFER_H_
#define INC_CIRCUALBUFFER_H_


typedef struct {
    uint8_t buffer[150]; //uint8_t
    int head;
    int tail;
    int maxlen;
} circualBufferStructure;

#endif /* INC_CIRCUALBUFFER_H_ */
