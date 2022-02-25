/*
 * RangeMeasurment.h
 *
 *  Created on: 21 lut 2022
 *      Author: Daniel
 */

#ifndef INC_RANGEMEASURMENT_H_
#define INC_RANGEMEASURMENT_H_
#include "Vl53l0x.h"
#include "ServoPR.h"
#include "HcSr04.h"

#define RANGEMEASURMENT_TIME_FOR_ERROR_SERVOPR 1000 /*!<Czas po ktorym nastepuje wykrycie problemu z serwomechanizmem[ms]*/
#define RANGEMEASURMENT_TIME_FOR_ERROR_RANGE_MEASURMENT 300 /*!<Czas po ktorym nastepuje wykrycie problemu z serwomechanizmem[ms]*/
typedef enum rangeMeasurmentStatus rangeMeasurmentStatus;
typedef struct rangeMeasurmentStruct rangeMeasurmentStruct;
enum rangeMeasurmentStatus{
	rangeMeasurmentIdle,
	rangeMeasurmentPostitionChanging,
	rangeMeasurmentDistance,
	rangeMeasurmentEnd,
};

struct rangeMeasurmentStruct{
		//Lokalne zmienne
	rangeMeasurmentStatus status;
	uint16_t Vl53l0xDistance;
	uint8_t Vl53l0xLockValue;
	uint32_t hcSr04Distance;
	uint8_t	hcSr04LockValue;
	uint8_t servoPRPosition;
	uint32_t time;
		//Moduły
	servoPRStruct* servoPR;
	ultrasonicSensorStruct *HcSr04;
	StructVl53l0x *vl53l0x;
		//Funkcje
	void (*createHcSr04)(rangeMeasurmentStruct* me,TIM_HandleTypeDef* timerHcSr04);
	void (*createVl54l0x)(rangeMeasurmentStruct *me,I2C_HandleTypeDef * hi2cVl53l0,GPIO_TypeDef *xshutgpio,uint16_t xshutpin);
	void (*createServoPR)(rangeMeasurmentStruct *me,TIM_HandleTypeDef *PWMtimerGen,uint32_t TimChannel);
	uint8_t (*rangeMeasurment)(rangeMeasurmentStruct *me,uint8_t position);
	uint16_t* (*getVl53l0xDistance)(rangeMeasurmentStruct *me);
	uint32_t* (*getHcSr04Distance)(rangeMeasurmentStruct *me);
	uint8_t* (*getServoPRPosition)(rangeMeasurmentStruct *me);
	void (*timeInterrupt)(rangeMeasurmentStruct *me);
	void (*HcSr04Interrupt)(rangeMeasurmentStruct *me,TIM_HandleTypeDef* htim);
	void (*main)(rangeMeasurmentStruct *me);
	uint8_t (*isRangeMeasurmentEnd)(rangeMeasurmentStruct *me);
};

rangeMeasurmentStruct* RangeMeasurment_Create();
void RangeMeasurment_Init(rangeMeasurmentStruct* me,
						void (*createHcSr04)(rangeMeasurmentStruct* me,TIM_HandleTypeDef* timerHcSr04),
						void (*createVl54l0x)(rangeMeasurmentStruct *me,I2C_HandleTypeDef * hi2cVl53l0,GPIO_TypeDef *xshutgpio,uint16_t xshutpin),
						void (*createServoPR)(rangeMeasurmentStruct *me,TIM_HandleTypeDef *PWMtimerGen,uint32_t TimChannel),
						uint8_t (*rangeMeasurment)(rangeMeasurmentStruct *me,uint8_t position),
						uint16_t* (*getVl53l0xDistance)(rangeMeasurmentStruct *me),
						uint32_t* (*getHcSr04Distance)(rangeMeasurmentStruct *me),
						uint8_t* (*getServoPRPosition)(rangeMeasurmentStruct *me),
						void (*timeInterrupt)(rangeMeasurmentStruct *me),
						void (*HcSr04Interrupt)(rangeMeasurmentStruct *me,TIM_HandleTypeDef* htim),
						void (*main)(rangeMeasurmentStruct *me),
						uint8_t (*isRangeMeasurmentEnd)(rangeMeasurmentStruct *me));

void RangeMeasurment_CreateHcSr04(rangeMeasurmentStruct* me,TIM_HandleTypeDef* timerHcSr04);
void RangeMeasurment_CreateVl54l0x(rangeMeasurmentStruct *me,I2C_HandleTypeDef * hi2cVl53l0,GPIO_TypeDef *xshutgpio,uint16_t xshutpin);
void RangeMeasurment_CreateServoPR(rangeMeasurmentStruct *me,TIM_HandleTypeDef *PWMtimerGen,uint32_t TimChannel);
uint8_t RangeMeasurment_RangeMeasurment(rangeMeasurmentStruct *me,uint8_t position);
uint16_t* RangeMeasurment_GetVl53l0xDistance(rangeMeasurmentStruct *me);
uint32_t* RangeMeasurment_GetHcSr04Distance(rangeMeasurmentStruct *me);
uint8_t* RangeMeasurment_GetServoPRPosition(rangeMeasurmentStruct *me);
void RangeMeasurment_TimeInterrupt(rangeMeasurmentStruct *me);
void RangeMeasurment_HcSr04Interrupt(rangeMeasurmentStruct *me,TIM_HandleTypeDef* htim);
void RangeMeasurment_Main(rangeMeasurmentStruct *me);
uint8_t RangeMeasurment_isRangeMeasurmentEnd(rangeMeasurmentStruct *me);
void RangeMeasurment_IdleFunction(rangeMeasurmentStruct *me);
void RangeMeasurment_PostitionChangingFunction(rangeMeasurmentStruct *me);
void RangeMeasurment_MeasurmentDistanceFunction(rangeMeasurmentStruct *me);
void RangeMeasurment_EndRangeFunction(rangeMeasurmentStruct *me);
#endif /* INC_RANGEMEASURMENT_H_ */
