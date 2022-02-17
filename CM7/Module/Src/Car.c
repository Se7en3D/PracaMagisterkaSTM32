#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "Car.h"


void Car_Create(UART_HandleTypeDef *Message_huart,
		TIM_HandleTypeDef* timer1ms,
		TIM_HandleTypeDef* timerHcSr04,
		ADC_HandleTypeDef* adcBattery,
		GPIO_TypeDef* gpioBattery,
		uint16_t pinBattery,
		I2C_HandleTypeDef * hi2cVl53l0,
		GPIO_TypeDef *xshutgpio,
		uint16_t xshutpin){
	generalCarModule.inMessage=CommandDecoder_Create();
	generalCarModule.outMessage=Message_Create(Message_huart);
	generalCarModule.batteryVoltage=BatteryVoltage_Create(adcBattery, gpioBattery, pinBattery);
	generalCarModule.irSensor=IrSensor_Create();
	generalCarModule.HcSr04=HcSr04_Create(timerHcSr04);
	generalCarModule.timer1ms=timer1ms;
	generalCarModule.vl53l0x=vl53l0x_Create(hi2cVl53l0,xshutgpio,xshutpin);
	//generalCarModule.batteryVoltage->setContinousMeasurment(generalCarModule.batteryVoltage);
	/*HAL Start Per*/
	HAL_UART_Receive_IT(Message_huart,&generalCarModule.ReceivedInMessageBuff,1);
	HAL_TIM_Base_Start_IT(timer1ms);
}

void mainFun(){
					/*Odbieranie komend z komputera PC*/
	uint8_t *pointerToCommunicationFunction=generalCarModule.inMessage->GetFunction(generalCarModule.inMessage);
	if(pointerToCommunicationFunction){
		printf("Odebrano ramke o funkcji=%x\n",*pointerToCommunicationFunction);
		generalCarModule.inMessage->ClearBuffer(generalCarModule.inMessage);
	}
					/*Obsługa modułu ppomiarowego napięcia zasilania bateryjnego*/
	if(generalCarModule.timerMeasureBatteryVoltage>=CAR_TIME_TO_SEND_BATTERY_VOLTAGE){
		generalCarModule.batteryVoltage->startMeasurment(generalCarModule.batteryVoltage);
		generalCarModule.timerMeasureBatteryVoltage=0;


		//TODO usunąc czujnik HCsr04 stąd
		/*uint32_t HcSr04Result=vl53l0x_ReadDistance(generalCarModule.vl53l0x);
		printf("Dystans=%d\n",(int)HcSr04Result);*/
		generalCarModule.vl53l0x->startSingleMeasurment(generalCarModule.vl53l0x);


	}
	uint32_t *pointerToValueFromAdc=generalCarModule.batteryVoltage->getValue(generalCarModule.batteryVoltage);
	if(pointerToValueFromAdc){
		generalCarModule.outMessage->InsertAdcBatteryVoltage(generalCarModule.outMessage,*pointerToValueFromAdc);
	}
					/*Obsługa modułu sensora IR*/
	generalCarModule.irSensor->readStatus(generalCarModule.irSensor);
	if(generalCarModule.timerToSendIrSensorStatus>=CAR_TIME_TO_SEND_STATUS_IRSENSOR){
		uint32_t IrSensorValue=generalCarModule.irSensor->getValue(generalCarModule.irSensor);
		generalCarModule.outMessage->InsertIrSensor(generalCarModule.outMessage,IrSensorValue);
		generalCarModule.timerToSendIrSensorStatus=0;
	}
		/*Obsługa modułu vl53lx0		 */
	uint16_t *vl53lx0_Distance=generalCarModule.vl53l0x->getDistance(generalCarModule.vl53l0x);
	if(vl53lx0_Distance){
		printf("vl53l0x dystans=%d\n",(int)(*vl53lx0_Distance));
	}
					/*Testowanie czujnika odległości HcSr04 */
	generalCarModule.HcSr04->startMeasurment(generalCarModule.HcSr04);
					/*Wysłanie danych z bufora nadajnika*/
	generalCarModule.outMessage->SendMessage(generalCarModule.outMessage);
}
void Car_AddIrSensor(GPIO_TypeDef *gpio, uint16_t pin){
	generalCarModule.irSensor->addGPIO(generalCarModule.irSensor,gpio,pin);
}

int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  //TODO Dodać komendę do wysyłania odpowiedzi poprzez Bluetooth
  //Message_Insert(generalCarModule.outMessage,ch);
  generalCarModule.outMessage->Insert(generalCarModule.outMessage,ch);
  return 1;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if(htim==generalCarModule.timer1ms){
		generalCarModule.inMessage->AddTimeout(generalCarModule.inMessage);
		generalCarModule.batteryVoltage->addTime(generalCarModule.batteryVoltage);
		generalCarModule.timerMeasureBatteryVoltage++;
		generalCarModule.timerToSendIrSensorStatus++;
		generalCarModule.vl53l0x->increaseTime(generalCarModule.vl53l0x);
		/*
		irSensorAddTime();
		adcAddTime();
		connectionModuleaddTimeout(&measurmentStructure);
		servoPRAddTime(&servoPRGeneralStructure);*/
	}
	generalCarModule.HcSr04->htimInterrupt(generalCarModule.HcSr04,htim);
	/*

	if(htim==&htim13){

	}
	if(htim==&htim17){
		stateMachineTimeout();
		HAL_TIM_Base_Stop(&htim17);
	}
	if(htim==&htim2){
		if ((htim->Instance->CCMR1 & TIM_CCMR1_CC1S) != 0x00U){
			hcsr04CompCH1Add(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
		}

		if ((htim->Instance->CCMR1 & TIM_CCMR1_CC2S) != 0x00U){
			hcsr04CompCH2Add(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2));
		}
	}*/
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
	if(UartHandle==generalCarModule.outMessage->huart){
		generalCarModule.inMessage->Insert(generalCarModule.inMessage,generalCarModule.ReceivedInMessageBuff);
		HAL_UART_Receive_IT(generalCarModule.outMessage->huart,&generalCarModule.ReceivedInMessageBuff,1);
	}
}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c){

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc==generalCarModule.batteryVoltage->hadc){
		uint32_t value=HAL_ADC_GetValue(hadc);
		generalCarModule.batteryVoltage->addMeasure(generalCarModule.batteryVoltage,value);
	}
}

void addErrorValue(uint8_t value ){
	generalCarModule.outMessage->Insert(generalCarModule.outMessage,value);
}
