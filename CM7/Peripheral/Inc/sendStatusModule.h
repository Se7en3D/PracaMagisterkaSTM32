/*
 * sendStatusModule.h
 *
 *  Created on: 20 paź 2021
 *      Author: Daniel
 */

#ifndef INC_SENDSTATUSMODULE_H_
#define INC_SENDSTATUSMODULE_H_


typedef enum{
	sendStructStatusIdle,
	sendAdcStructBaseStatus,
	sendConnectionModuleStructStatus,
	sendErrorCodeStructStatus,
	sendHcsr04StructStatus,
	sendIrsensorStructStatus,
	sedServo360StructStatus,
	sendStatemachineStructStatus,
	sendUartComStructStatus,
	sendVl5310xStructStatus,
}enumStageSendingStructStatus;

typedef struct{
	enumStageSendingStructStatus status;
}structSendingSystemStatus;



#endif /* INC_SENDSTATUSMODULE_H_ */
