#include "Message.h"

typedef struct carModule carModule;

struct carModule{
	messageStruct *outMessage;
	messageStruct *inMessage;
	uint8_t ReceivedInMessageBuff;
};

carModule generalCarModule;
void Car_Create(UART_HandleTypeDef *Message_huart);
void mainFun();
