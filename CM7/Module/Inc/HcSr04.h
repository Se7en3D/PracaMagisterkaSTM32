/*
 * HcSr04.h
 *
 *  Created on: 23 sty 2022
 *      Author: Daniel
 */

#ifndef INC_HCSR04_H_
#define INC_HCSR04_H_

typedef struct ultrasonicSensorStruct ultrasonicSensorStruct;
typedef enum ultrasonicSensorStatus ultrasonicSensorStatus;

enum ultrasonicSensorStatus{
	HCSR04_Idle,
	HCSR04_Measurment,
	HCSR04_CompleteMeasurment,
};

struct ultrasonicSensorStruct{
	TIM_HandleTypeDef *htim;
	ultrasonicSensorStatus status;
	uint32_t risingEdgeTime;
	uint32_t fallingEdgeTime;
	uint8_t continousMeasurment;
	uint32_t result;
	void (*addRisingEdgeTime)(ultrasonicSensorStruct *me, uint32_t value);
	void (*addFallingEdgeTime)(ultrasonicSensorStruct *me, uint32_t value);
	void (*setContinousMeasurment)(ultrasonicSensorStruct *me);
	void (*resetContinousMeasurment)(ultrasonicSensorStruct *me);
	uint32_t(*getMeasurment)(ultrasonicSensorStruct *me);
	void (*resetMeasurment)(ultrasonicSensorStruct *me);
	void (*startMeasurment)(ultrasonicSensorStruct *me);
	void (*htimInterrupt)(ultrasonicSensorStruct *me,TIM_HandleTypeDef *htim);
};


ultrasonicSensorStruct* HcSr04_Create(TIM_HandleTypeDef *htim);
void HcSr04_Init(ultrasonicSensorStruct *me,
				void (*addRisingEdgeTime)(ultrasonicSensorStruct *me, uint32_t value),
				void (*addFallingEdgeTime)(ultrasonicSensorStruct *me, uint32_t value),
				void (*setContinousMeasurment)(ultrasonicSensorStruct *me),
				void (*resetContinousMeasurment)(ultrasonicSensorStruct *me),
				uint32_t(*getMeasurment)(ultrasonicSensorStruct *me),
				void (*resetMeasurment)(ultrasonicSensorStruct *me),
				void (*startMeasurment)(ultrasonicSensorStruct *me),
				void (*htimInterrupt)(ultrasonicSensorStruct *me,TIM_HandleTypeDef *htim));
void HcSr04_AddRisingEdgeTime(ultrasonicSensorStruct *me, uint32_t value);
void HcSr04_AddFallingEdgeTime(ultrasonicSensorStruct *me, uint32_t value);
void HcSr04_SetContinousMeasurment(ultrasonicSensorStruct *me);
void HcSr04_ResetContinousMeasurment(ultrasonicSensorStruct *me);
uint32_t HcSr04_GetMeasurment(ultrasonicSensorStruct *me);
void HcSr04_ResetMeasurment(ultrasonicSensorStruct *me);
void HcSr04_StartMeasurment(ultrasonicSensorStruct *me);
void HcSr04_htimInterrupt(ultrasonicSensorStruct *me,TIM_HandleTypeDef *htim);
void HcSr04_HalStart(TIM_HandleTypeDef *htim);
void HcSr04_HalStop(TIM_HandleTypeDef *htim);
#endif /* INC_HCSR04_H_ */
