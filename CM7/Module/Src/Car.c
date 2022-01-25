#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "Car.h"


void Car_Create(UART_HandleTypeDef *Message_huart){
	generalCarModule.inMessage=Message_Create(Message_huart);
	generalCarModule.outMessage=Message_Create(Message_huart);

	/*HAL Start Per*/
	HAL_UART_Receive_IT(Message_huart,&generalCarModule.ReceivedInMessageBuff,1);
}

void mainFun(){
	generalCarModule.outMessage->SendMessage(generalCarModule.inMessage);
}


int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  //TODO Dodać komendę do wysyłania odpowiedzi poprzez Bluetooth
  generalCarModule.outMessage->InsertError(generalCarModule.outMessage,ch);
  return 1;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
	if(UartHandle==generalCarModule.inMessage->huart){
		generalCarModule.inMessage->Insert(generalCarModule.inMessage,generalCarModule.ReceivedInMessageBuff);
		HAL_UART_Receive_IT(generalCarModule.inMessage->huart,&generalCarModule.ReceivedInMessageBuff,1);
	}
}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c){

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

}
