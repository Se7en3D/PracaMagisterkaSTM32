/*
 * BatteryVoltage.h
 *
 *  Created on: 30 sty 2022
 *      Author: Daniel
 */

#ifndef INC_BATTERYVOLTAGE_H_
#define INC_BATTERYVOLTAGE_H_

#define BATTERYVOLTAGE_MAX_MEASURMENTS 10 /*!<Ilosc pomiarow do sredniej arytmetycznej*/
#define BATTERYVOLTAGE_TIME_TO_STABILIZE 50 /*!<Czas potrzebny do stabilizacji napięcia*/
typedef struct adcMeasureStruct adcMeasureStruct;
typedef enum adcMeasureStatus adcMeasureStatus;

enum adcMeasureStatus{
	Battery_Idle,
	Battery_Stabilize,
	Battery_Measurment,
	Battery_CompleteMeasurment
};

struct adcMeasureStruct{
	ADC_HandleTypeDef *hadc;
	uint32_t measurments[BATTERYVOLTAGE_MAX_MEASURMENTS];
	adcMeasureStatus status;
	uint32_t time;
	GPIO_TypeDef *gpioTypeDefPowerSupplySchematicMeasurment;
	uint16_t gpioPinPowerSupplySchematicMeasurment;
	uint32_t meanValue;
	uint8_t continousMeasurment;
	uint8_t number;
	void (*statusFunction[4])(adcMeasureStruct* const me);
	uint32_t* (*getValue)(adcMeasureStruct* const me);
	void (*startMeasurment)(adcMeasureStruct* const me);
	void (*addTime)(adcMeasureStruct* const me);
	void (*resetMeasurment)(adcMeasureStruct* const me);
	void (*setContinousMeasurment)(adcMeasureStruct* const me);
	void (*resetContinousMeasurment)(adcMeasureStruct* const me);
	void (*addMeasure)(adcMeasureStruct* const me,uint32_t value);
};


adcMeasureStruct *BatteryVoltage_Create(ADC_HandleTypeDef *hadc,GPIO_TypeDef *gpio,uint16_t pin);
void BatteryVoltage_Init(adcMeasureStruct* const me,
		uint32_t* (*getValue)(adcMeasureStruct* const me),
		void (*startMeasurment)(adcMeasureStruct* const me),
		void (*addTime)(adcMeasureStruct* const me),
		void (*resetMeasurment)(adcMeasureStruct* const me),
		void (*setContinousMeasurment)(adcMeasureStruct* const me),
		void (*resetContinousMeasurment)(adcMeasureStruct* const me),
		void (*addMeasure)(adcMeasureStruct* const me,uint32_t value));
void BatteryVoltage_StatusIdle(adcMeasureStruct* const me);
void BatteryVoltage_StatusStabilize(adcMeasureStruct* const me);
void BatteryVoltage_StatusMeasurment(adcMeasureStruct* const me);
void BatteryVoltage_StatusCompleteMeasurment(adcMeasureStruct* const me);
uint32_t *BatteryVoltage_GetValue(adcMeasureStruct* const me);
void BatteryVoltage_StartMeasurment(adcMeasureStruct* const me);
void BatteryVoltage_AddTime(adcMeasureStruct* const me);
void BatteryVoltage_ResetTime(adcMeasureStruct* const me);
void BatteryVoltage_ResetMeasurment(adcMeasureStruct* const me);
void BatteryVoltage_SetContinousMeasurment(adcMeasureStruct* const me);
void BatteryVoltage_ResetContinousMeasurment(adcMeasureStruct* const me);
void BatteryVoltage_StartADC(adcMeasureStruct* const me);
void BatteryVoltage_StopADC(adcMeasureStruct* const me);
void BatteryVoltage_EnGpioOn(adcMeasureStruct* const me);
void BatteryVoltage_EnGpioOff(adcMeasureStruct* const me);
void BatteryVoltage_CalcMean(adcMeasureStruct* const me);
void BatteryVoltage_AddMeasure(adcMeasureStruct* const me,uint32_t value);

#endif /* INC_BATTERYVOLTAGE_H_ */
