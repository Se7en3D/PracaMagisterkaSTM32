/*
 * servoPR.h
 *
 *  Created on: 18 lut 2022
 *      Author: Daniel
 */

#ifndef INC_SERVOPR_H_
#define INC_SERVOPR_H_

#include "ErrorValue.h"

#define SERVOPR_MIN_PERIOD_IN_MS 1000*0.5 /*!<Najmniejsza wartosc wypełnienia potrzebna do obslugi servo podawana w milisekundach*/
#define SERVOPR_MAX_PERIOD_IN_MS 1000*2.5 /*!<Największa wartosc wypełnienia potrzebna do obslugi servo podawana w milisekundach*/
#define SERVOPR_TIME_FOR_OPERATING_SPEED 30 /*!<Prędkość obrotowa potrzebna do ruchu o 15 stopni[ms]*/

typedef enum servoPRStatus servoPRStatus;
typedef struct servoPRStruct servoPRStruct;

enum servoPRStatus{
	servoPR_IDLE=0, /*!< Servo w stanie spoczynku */
	servoPR_WAIT_TO_STABILIZE_SERVO, /*!< Servo w oczekiwania na stabilizacje*/
	servoPR_ERROR, /*!< Wykruto błąd w działaniu servo*/
};


struct servoPRStruct{
	TIM_HandleTypeDef *PWMtimerGen; /*!< Uchwyt do struktury TIM_HandleTypeDef*/
	uint32_t TimChannel;/*!<Kanał sygnału PWM*/
	servoPRStatus status;
	uint8_t position;
	uint32_t timeToStabilizedServo;
	uint8_t (*setPosition)(servoPRStruct *me,uint8_t numberPosition);
	uint8_t (*getPosition)(servoPRStruct *me);
	void (*resetServo)(servoPRStruct *me);
	uint8_t (*isReady)(servoPRStruct *me);
	void (*timerToInterrupt)(servoPRStruct *me);
	void (*controlFunction)(servoPRStruct *me);
};

servoPRStruct* servoPR_Create(TIM_HandleTypeDef *PWMtimerGen,uint32_t TimChannel);
void servoPR_Init(servoPRStruct *me,
				uint8_t (*setPosition)(servoPRStruct *me,uint8_t numberPosition),
				uint8_t (*getPosition)(servoPRStruct *me),
				void (*resetServo)(servoPRStruct *me),
				uint8_t (*isReady)(servoPRStruct *me),
				void (*timerToInterrupt)(servoPRStruct *me),
				void (*controlFunction)(servoPRStruct *me));
uint8_t servoPR_SetPosition(servoPRStruct *me,uint8_t numberPosition);
uint8_t servoPR_GetPosition(servoPRStruct *me);
void servoPR_Reset(servoPRStruct *me);
uint8_t servoPR_IsReady(servoPRStruct *me);
void servoPR_TimerToInterrupt(servoPRStruct *me);
void servoPR_ControlFunction(servoPRStruct *me);
void servoPR_IdleFuction(servoPRStruct *me);
void servoPR_WaitToStabilizeServoFunction(servoPRStruct *me);
void servoPR_ErrorFunction(servoPRStruct *me);

#endif /* INC_SERVOPR_H_ */
