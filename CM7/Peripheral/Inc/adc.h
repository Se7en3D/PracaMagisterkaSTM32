/**
  ******************************************************************************
  * @file    adc.h
  * @author  Daniel Dunak
  * @brief   Plik nagłówkowy odpowiedzialny za moduł związany z odczytem napięcia akumulatora
  *
  ******************************************************************************
  */

#ifndef INC_ADC_H_
#define INC_ADC_H_
/**
  * @brief Stała czasowa definująca interwał konwersji przetwornika ADC [ms]
  */
#define ADC_TIME_TO_START_CONVERSION 5000
/**
  * @brief Stała czasowa definująca czas potrzebny do stabilizacji napięcia na układzie pomiarowym [ms]
  */
#define ADC_TIME_TO_STABILIZE_VOLTAGE 25
/**
  * @brief Stała definująca wartość braku konwersji przetwornika ADC
  */
#define ADC_WRONG_CONV 0xFFFFFFFF
/**
  * @brief Stała definująca brak reakcji
  */
#define ADC_NOT_REACTION 0
/**
  * @brief Stała definująca załączenie napięcia na układzie pomiarowym
  */
#define ADC_TURN_ON_POWER 1
/**
  * @brief Stała definująca rozpoczęcie konwersji przetwornika ADC
  */
#define ADC_START_CONVERSION 2
/**
  * @brief Stała czasowa definująca gotowość do pobrania wartości z przetwornika
  */
#define ADC_READY_TO_SEND 3
#define ADC_MAX_COUNT_MEASUREMENT 5 /*!<Ilość pomiarów napięcia na przetworniku potrzebne do uśrednienia wartości*/

/*! Stany układu pomiarowego */
typedef enum{
	adcWaitForTurnOnPowerByOptocoupler=0, /*!< Układ pomiarowy jest wyłączony. Oczekiwanie na właczenie zasilania  */
	adcWaitoForStabilizeVoltage,/*!< Oczekiwanie na stabilizację napięcia */
	adcMeasureVoltage,/*!< Pomiar napięcia*/
	adcSendVoltage/*!< Gotowość do wysłania danych */

}adc_Measurment_Battery_stage;

/**
  * @brief Struktura adc_Base_structure_t
  */
typedef struct{
	ADC_HandleTypeDef *hadc; /*!< Uchwyt do struktury ADC_HandleTypeDef*/
	uint32_t value; /*!< Wartośc pobrana ze układu ADC */
	uint32_t time; /*!< Licznik czasu*/
	uint32_t readyToSend; /*!<Wartość bitowa określająca gotowość do wysłania danych */
	adc_Measurment_Battery_stage stage; /*!< Stan układu pomiarowego*/
	uint32_t(*fun_ptr[4])(void); /*!< Wzkażnik na funkcję*/
	GPIO_TypeDef *gpioPowerMeasureOn_OffPort; /*!< Wzkaznik na port odpowiedzialny za wyłączenie/włączenie zasilania */
	uint32_t gpioPowerMeasureOn_OffPin; /*!< Numer pinu odpowiedzialnego za wyłączenie/włączenie zasilania */
	uint8_t countMeasurement;/*!<Ilość wykonanych pomiarów adc w jednym cyklu */
	uint32_t meanValue;/*!<Zmienna przechowująca zsumowane dane do uśrednienia*/
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
