/*
 * adc.h
 *
 *  Created on: 5 sie 2021
 *      Author: DanielD
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#define ADC_TIME_TO_START_CONVERSION 5000 //Czas do natępnej konwersji (ms)
#define ADC_TIME_TO_STABILIZE_VOLTAGE 25
#define ADC_WRONG_CONV 0xFFFFFFFF
#define ADC_NOT_REACTION 0
#define ADC_TRUN_ON_POWER 1
#define ADC_START_CONVERSION 2
#define ADC_READY_TO_SEND 3

typedef enum{
	adcWaitForTurnOnPowerByOptocoupler=0,
	adcWaitoForStabilizeVoltage,
	adcMeasureVoltage,
	adcSendVoltage

}adc_Measurment_Battery_stage;

typedef struct{
	ADC_HandleTypeDef *hadc;
	uint32_t value;
	uint32_t time;
	uint32_t readyToSend;
	adc_Measurment_Battery_stage stage;
	uint32_t(*fun_ptr[4])(void);
	GPIO_TypeDef *gpioPowerMeasureOn_OffPort;
	uint32_t gpioPowerMeasureOn_OffPin;
}adc_Base_structure_t;

volatile adc_Base_structure_t adcBaseStructure;


void adcInit(ADC_HandleTypeDef *hadc1,GPIO_TypeDef *gpioPowerMeasureOn_OffPort,uint32_t gpioPowerMeasureOn_OffPin);
void adcSetConversionValue(uint32_t data);
uint32_t adcTurnOnPowerOptocoupler();
uint32_t adcWaitForStabilizeVoltage();
uint32_t adcWaitForConversionValue();
uint32_t adcReadyToSendData();
uint32_t adcGetValue();
void adcAddTime();
uint32_t adcGetTime();
void adcClearTime();
uint32_t adcStage();


#endif /* INC_ADC_H_ */
