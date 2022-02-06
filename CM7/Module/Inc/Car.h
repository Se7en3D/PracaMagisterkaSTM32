#include "Message.h"
#include "CommandDecoder.h"
#include "BatteryVoltage.h"
#include "irSensor.h"
#include "HcSr04.h"

#define CAR_TIME_TO_SEND_BATTERY_VOLTAGE 1000 /*!<Czas po jakim załączny jest pomiar napięcia[ms]*/
#define CAR_TIME_TO_SEND_STATUS_IRSENSOR 250 /*!<Czas po jakim nalezy wyslac status czujnikow IR*/
typedef struct carModule carModule;

struct carModule{
	messageStruct *outMessage;
	bluetoothDecoderStruct *inMessage;
	adcMeasureStruct *batteryVoltage;
	irSensorStruct *irSensor;
	ultrasonicSensorStruct *HcSr04;
	uint8_t ReceivedInMessageBuff;
	TIM_HandleTypeDef* timer1ms;
	uint32_t timerMeasureBatteryVoltage;
	uint32_t timerToSendIrSensorStatus;
};

carModule generalCarModule;
void Car_Create(UART_HandleTypeDef *Message_huart,
		TIM_HandleTypeDef* timer1ms,
		TIM_HandleTypeDef* timerHcSr04,
		ADC_HandleTypeDef* adcBattery,
		GPIO_TypeDef* gpioBattery,
		uint16_t pinBattery);
void mainFun();
void Car_AddIrSensor(GPIO_TypeDef *gpio, uint16_t pin);
