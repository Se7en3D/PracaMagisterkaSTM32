#include <stdio.h>
#include <stdlib.h>
#include "CircularBuffer.h"



CircularBufferStruct *CircularBuffer_Create() {
	CircularBufferStruct* me = (CircularBufferStruct*)malloc(sizeof(CircularBufferStruct));
	if (me != NULL) {
		CircularBuffer_Init(me, CircularBuffer_isFull,
			CircularBuffer_isEmpty,
			CircularBuffer_getSize,
			CircularBuffer_insert,
			CircularBuffer_getData,
			CircularBuffer_show);
	}
	return me;
}
void CircularBuffer_Init(CircularBufferStruct* const me,
						uint32_t(*isFull)(CircularBufferStruct* const me),
						uint32_t(*isEmpty)(CircularBufferStruct* const me),
						uint32_t(*getSize)(CircularBufferStruct* const me),
						void (*insert)(CircularBufferStruct* const me, uint32_t value),
						uint32_t(*getData)(CircularBufferStruct* const me),
						void (*show)(CircularBufferStruct* const me)) {
		/*initialize attributes*/
	if (0 != me->buffer) {
		free(me->buffer);
	}
	me->buffer = malloc(CIRCULARBUFFER_MAX_SIZE_BUFFER * sizeof(uint32_t));
	me->head = 0;
	me->tail = 0;
	me->size = CIRCULARBUFFER_MAX_SIZE_BUFFER;
		/* initialize member function pointers */
	me->isFull = isFull;
	me->isEmpty = isEmpty;
	me->getSize = getSize;
	me->insert = insert;
	me->getData = getData;
	me->show = CircularBuffer_show;
}
uint32_t CircularBuffer_isFull(CircularBufferStruct* const me) {
	uint32_t next = me->head + 1;
	if (next >= CIRCULARBUFFER_MAX_SIZE_BUFFER) {
		next = 0;
	}
	if (me->tail == next) {
		return 1;
	}
	else {
		return 0;
	}
}
uint32_t CircularBuffer_isEmpty(CircularBufferStruct* const me) {
	if (me->tail == me->head) {
		return 1;
	}
	else {
		return 0;
	}
}
uint32_t CircularBuffer_getSize(CircularBufferStruct* const me) {
	uint32_t totalSize;
	uint32_t head=me->head;
	uint32_t tail=me->tail;
	if (head < tail) {
		totalSize = CIRCULARBUFFER_MAX_SIZE_BUFFER - tail;
		totalSize += head;
	}
	else {
		totalSize = head - tail;
	}
	return totalSize;

}
void CircularBuffer_insert(CircularBufferStruct* const me, uint32_t value) {
	if (me->isFull(me)==0) {
		uint32_t next = me->head+1;
		if (next >= CIRCULARBUFFER_MAX_SIZE_BUFFER) {
			next = 0;
		}
		me->buffer[next] = value;
		me->head = next;
	}
}
uint32_t* CircularBuffer_getData(CircularBufferStruct* const me){
	if (me->isEmpty(me) == 0) {
		uint32_t* result = me->buffer[me->tail];
		me->tail++;
		if (me->tail >= CIRCULARBUFFER_MAX_SIZE_BUFFER) {
			me->tail = 0;
		}
		return result;
	}else {
		return NULL;
	}
}
void CircularBuffer_show(CircularBufferStruct* const me) {
	uint32_t* pointerToData;
	while ((pointerToData = me->getData(me))) {
		printf("Dana=%d\n", pointerToData);
	}
}
