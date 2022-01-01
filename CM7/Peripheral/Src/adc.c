/**
  ******************************************************************************
  * @file    adc.c
  * @author  Daniel Dunak
  * @brief   Plik zródłowy odpowiedzialny za moduł związany z odczytem napięcia akumulatora
  *
  ******************************************************************************
  */
#include "stm32h7xx_hal.h"
#include "errorCode.h"
#include "adc.h"


/**
  * @brief	Inicjaliazcja struktury ADC_HandleTypeDef do wartości domyślnych
  * @param	hadc1 wskaźnik na strukturę ADC_HandleTypeDef zawierającą
  * 		informację na temat pomiaru napięcia oraz obecnego stanu przetwornika
  * @param	gpioPowerMeasureOn_OffPort Wskaźnik  struktury GPIO_TypeDef zawierającą
  * 		definiecję portu odpowiedzialnego za włączenie/wyłączenie
  * 		zasilania do pomiaru napięcia
  * @param 	gpioPowerMeasureOn_OffPin Zmienna zawierająca numer pinu
  * 		odpowiedzialnego za włączenie/wyłączenie zasilania do pomiaru napięcia
  * @retval None
  */
void adcInit(ADC_HandleTypeDef *hadc1,GPIO_TypeDef *gpioPowerMeasureOn_OffPort,uint32_t gpioPowerMeasureOn_OffPin){
	adcBaseStructure.hadc=hadc1;
	adcBaseStructure.stage=adcWaitForTurnOnPowerByOptocoupler;
	adcBaseStructure.fun_ptr[0]=&adcTurnOnPowerOptocoupler;
	adcBaseStructure.fun_ptr[1]=&adcWaitForStabilizeVoltage;
	adcBaseStructure.fun_ptr[2]=&adcWaitForConversionValue;
	adcBaseStructure.fun_ptr[3]=&adcReadyToSendData;
	adcBaseStructure.gpioPowerMeasureOn_OffPort=gpioPowerMeasureOn_OffPort;
	adcBaseStructure.gpioPowerMeasureOn_OffPin=gpioPowerMeasureOn_OffPin;
}

/**
  * @brief  Zapisanie pomiaru napięcia do struktury adc_Base_structure_t
  * @param  data Zmienna zawierająca pomiar przetwonika ADC
  * @retval None
  */
void adcSetConversionValue(uint32_t data){
	if(adcBaseStructure.hadc==0){
		errorCodePush(ADC_HADC_NULL_POINTER);
		return;
	}

	if(adcBaseStructure.countMeasurement==0){
		adcBaseStructure.meanValue=0;
	}

	if(adcBaseStructure.countMeasurement<ADC_MAX_COUNT_MEASUREMENT){
		adcBaseStructure.meanValue+=data;
		adcBaseStructure.countMeasurement++;
	}

	if(adcBaseStructure.countMeasurement>=ADC_MAX_COUNT_MEASUREMENT){
		adcBaseStructure.value=adcBaseStructure.meanValue/ADC_MAX_COUNT_MEASUREMENT;
		adcBaseStructure.readyToSend=ADC_READY_TO_SEND;
	}else{
		HAL_ADC_Start_IT(adcBaseStructure.hadc);
	}

}

/**
  * @brief  Sprawdzenie potrzeby załączenia zasilania układu pomiaru napięcia.
  * 		W momecnie przekroczenie wartości zmiennej time w strukturze
  * 		adc_Base_structure_t o  wartość stałej ADC_TIME_TO_START_CONVERSION
  * @retval ADC_TURN_ON_POWER informująca o potrzebie załączenia zasilania.
  * 		ADC_NOT_REACTION brak reakcji
  */
uint32_t adcTurnOnPowerOptocoupler(){ //Funkcja przygotowująca do załączenia zasilania
	if(adcBaseStructure.time>=ADC_TIME_TO_START_CONVERSION){
		adcBaseStructure.time=0;
		adcBaseStructure.stage=adcWaitoForStabilizeVoltage;
		HAL_GPIO_WritePin(adcBaseStructure.gpioPowerMeasureOn_OffPort, adcBaseStructure.gpioPowerMeasureOn_OffPin, GPIO_PIN_SET);
		return ADC_TURN_ON_POWER;
	}else{
		return ADC_NOT_REACTION;
	}
}

/**
  * @brief  Oczekiwanie na stabilizację napięcia na układzie pomiarowym po
  * 		jego załączeniu
  * @retval ADC_START_CONVERSION załączenie pomiaru napięcia.
  * 		ADC_NOT_REACTION brak reakcji
  */
uint32_t adcWaitForStabilizeVoltage(){ //Funkcja oczekiwania na załączenie przetwornika ADC
	if(adcBaseStructure.time>=ADC_TIME_TO_STABILIZE_VOLTAGE){
		adcBaseStructure.time=0;
		adcBaseStructure.stage=adcMeasureVoltage;
		HAL_ADC_Start_IT(adcBaseStructure.hadc);
		return ADC_START_CONVERSION;
	}else{
		return ADC_NOT_REACTION;
	}
}

/**
  * @brief  Oczkiwanie na zakończenie pomiaru napiecia
  * @retval ADC_READY_TO_SEND gotowość do wysłania danych po pomiarze napięcia.
  * 		ADC_NOT_REACTION brak reakcji
  */
uint32_t adcWaitForConversionValue(){
	if(adcBaseStructure.value!=ADC_WRONG_CONV){
		adcBaseStructure.stage=adcSendVoltage;
		return ADC_READY_TO_SEND;
	}else{
		return ADC_NOT_REACTION;
	}

}

/**
  * @brief  Sprawdzenie potrzeby załączenia zasilania układu pomiaru napięcia.
  * 		W momecnie przekroczenie wartości zmiennej time w strukturze
  * 		adc_Base_structure_t o  wartość stałej ADC_TIME_TO_START_CONVERSION
  * @retval ADC_TURN_ON_POWER informująca o potrzebie załączenia zasilania.
  * 		ADC_NOT_REACTION brak reakcji
  */
uint32_t adcReadyToSendData(){
	if(adcBaseStructure.value==ADC_WRONG_CONV){
		adcBaseStructure.stage=adcWaitForTurnOnPowerByOptocoupler;
		HAL_GPIO_WritePin(adcBaseStructure.gpioPowerMeasureOn_OffPort, adcBaseStructure.gpioPowerMeasureOn_OffPin, GPIO_PIN_RESET);
		adcBaseStructure.time=0;
		return ADC_READY_TO_SEND;
	}else{
		return ADC_TURN_ON_POWER;
	}

}

/**
  * @brief  Zwraza odczytaną wartości z przetwronika jeśli struktura adc_Base_structure_t
  * 		znajduje sie w stanie adcSendVoltage
  * @retval wartośc odczytana z przetowrnika albo stała
  * 		ADC_WRONG_CONV informująca o braku danych
  */
uint32_t adcGetValue(){
	if(adcBaseStructure.stage==adcSendVoltage){
		uint32_t tempData=adcBaseStructure.value;
		adcBaseStructure.value=ADC_WRONG_CONV;
		adcReadyToSendData();
		return tempData;
	}else{
		return ADC_WRONG_CONV;
	}
}

/**
  * @brief  Inkrementuje zmienna time w strukturze adc_Base_structure_t
  * @retval None
  */
void adcAddTime(){
	adcBaseStructure.time++;
}

/**
  * @brief  Zwraca wartość zmiennej time znajdującej się w strukturze adc_Base_structure_t
  * @retval Wartość zmiennej time
  */
uint32_t adcGetTime(){
	return adcBaseStructure.time;
}

/**
  * @brief  Czyszczenie zmiennej time w strukturze adc_Base_structure_t
  * @retval None
  */
void adcClearTime(){
	adcBaseStructure.time=0;
}

/**
  * @brief  Wywołanie funkcji z tablicy wskażników funkcji zależnej od stanu zmiennej stage
  * @retval Zwrócenie wartości ze wskażnika na funkcję
  */
uint32_t adcStage(){
	return adcBaseStructure.fun_ptr[adcBaseStructure.stage]();
}
