/*
 * irSensor.h
 *
 *  Created on: 3 sie 2021
 *      Author: DanielD
 */

#ifndef INC_IRSENSOR_H_
#define INC_IRSENSOR_H_

#include "stm32h7xx_hal.h"

#define MAX_SENSOR_IR 8
#define IR_NR1_NUMBER 0
#define IR_NR2_NUMBER 1
#define IR_NR3_NUMBER 2
#define IR_NR4_NUMBER 3
#define IR_NR5_NUMBER 4
#define IR_NR6_NUMBER 5
#define IR_NR7_NUMBER 6
#define IR_NR8_NUMBER 7
#define IR_SENSOR_DOES_NOT_EXIST 0
#define IR_SENSOR_NOT_DETECTED_COLLISION 0
#define IR_SENSOR_DETECTED_COLLISION 1
#define IR_SENSOR_EXIST 1
#define TIME_TO_SEND_STATUS_IR 500 //czas(ms) do wysłania statusu od sensorów podczerwieni
#define IRSENSOR_TIME_TO_READ_GPIO 10
#define	IRSENSOR_COLLISION_NEGATION 0
#define IRSENSOR_MAX_SAMPLE 5 //Maksymalna wielkość tablicy pobierającej stany wejścia pinu potrzebna do określania dominanty

volatile static int IrSensorExist[MAX_SENSOR_IR]={
		IR_SENSOR_EXIST,
		IR_SENSOR_EXIST,
		IR_SENSOR_EXIST,
		IR_SENSOR_DOES_NOT_EXIST,
		IR_SENSOR_DOES_NOT_EXIST,
		IR_SENSOR_EXIST,
		IR_SENSOR_EXIST,
		IR_SENSOR_DOES_NOT_EXIST
};


typedef struct{
	GPIO_TypeDef *gpioIrPort[MAX_SENSOR_IR];
	uint16_t gpioIrPin[MAX_SENSOR_IR];
	uint8_t collision[MAX_SENSOR_IR];
	uint32_t timerToSendCollision;
	uint32_t timerToReadGPIO;
	uint8_t sumOfSetGPIO[MAX_SENSOR_IR];
}ir_sensor_t;



ir_sensor_t irSensor;

void irSensorInitIrPinout(uint8_t number,uint16_t Pin,GPIO_TypeDef *port);
void irSensorReadStatusIrSensor();
uint8_t irSensorGetCollision(uint8_t number);
uint8_t *irSensorGetAllCollision();
void irSensorAddTime();
uint32_t irSensorGetTime();
void irSensorClearTime();
void irSensorAddSample(ir_sensor_t *irSensor,const uint8_t sensorId,const uint8_t value);



#endif /* INC_IRSENSOR_H_ */
