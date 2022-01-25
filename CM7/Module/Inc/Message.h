#include "CircularBuffer.h"

#define SEND_MEASUREMENT_FUN 125
#define ERROR_CODE_FUN 126
#define SEND_IR_SENSOR_STATUS 127
#define SEND_BATTERY_MEASURMENT_VALUE 128
#define CALIBRATION_PWM_DATA 129
#define MEASURE_DISTANCE_FUN 240
#define MEASURE_DISTANCE_FUN_RECEIVED 241
#define NO_DEFINED_FUN	250

typedef struct messageStruct messageStruct;
struct messageStruct{
	CircularBufferStruct *buffer;
	UART_HandleTypeDef *huart;
	void(*Insert)(messageStruct* const me,uint8_t data);
	void(*InsertError)(messageStruct* const me,uint8_t data);
	void(*InsertDistance)(messageStruct* const me,float  hcSr04,uint16_t vl53l0x);
	void(*InsertIrSensor)(messageStruct* const me,uint8_t  *collision,uint16_t size);
	void(*InsertAdcBatteryVoltage)(messageStruct* const me,uint32_t value);
	void(*SendMessage)(messageStruct* const me);
};

messageStruct *Message_Create(UART_HandleTypeDef *huart);
void Message_Init(messageStruct* const me,
		void(*Insert)(messageStruct* const me,uint8_t data),
		void(*InsertError)(messageStruct* const me,uint8_t data),
		void(*InsertDistance)(messageStruct* const me,float  hcSr04,uint16_t vl53l0x),
		void(*InsertIrSensor)(messageStruct* const me,uint8_t  *collision,uint16_t size),
		void(*InsertAdcBatteryVoltage)(messageStruct* const me,uint32_t value),
		void(*SendMessage)(messageStruct* const me));

void Message_Insert(messageStruct* const me,uint8_t data);
void Message_InsertError(messageStruct* const me,uint8_t data);
void Message_InsertDistance(messageStruct* const me,float  hcSr04,uint16_t vl53l0x);
void Message_InsertIrSensor(messageStruct* const me,uint8_t  *collision,uint16_t size);
void Message_InsertAdcBatteryVoltage(messageStruct* const me,uint32_t value);
void Message_SendMessage(messageStruct* const me);
