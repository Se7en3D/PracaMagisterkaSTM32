#include "CircularBuffer.h"

#define MEASURE_DISTANCE_FOR_PC2  125
#define ERROR_CODE_FUN 126
#define SEND_IR_SENSOR_STATUS 127
#define SEND_BATTERY_MEASURMENT_VALUE 128
#define CALIBRATION_PWM_DATA 129
#define SEND_DRIVE_STATUS 131
#define MEASURE_DISTANCE_FUN 240
#define MEASURE_DISTANCE_FUN_RECEIVED 241
#define NO_DEFINED_FUN	250
#define MESSAGE_IR_MASK 0b10011000 /*!<Maska do usuwanie nieużywanych modułów IR z ramki wysyłanej do aplikacji sterujacej*/
typedef struct messageStruct messageStruct;
struct messageStruct{
	CircularBufferStruct *buffer;
	UART_HandleTypeDef *huart;
	void(*Insert)(messageStruct* const me,uint8_t data);
	void(*InsertError)(messageStruct* const me,uint8_t data);
	void(*InsertDistance)(messageStruct* const me,uint32_t  hcSr04,uint16_t vl53l0x);
	void(*InsertDistanceWithPosition)(messageStruct* const me,uint32_t  hcSr04,uint16_t vl53l0x,uint8_t position);
	void(*InsertIrSensor)(messageStruct* const me,uint32_t value);
	void(*InsertAdcBatteryVoltage)(messageStruct* const me,uint32_t value);
	void(*SendMessage)(messageStruct* const me);
	void(*SendDriveStatus)(messageStruct* const me,uint8_t status);
};

messageStruct *Message_Create(UART_HandleTypeDef *huart);
void Message_Init(messageStruct* const me,
		void(*Insert)(messageStruct* const me,uint8_t data),
		void(*InsertError)(messageStruct* const me,uint8_t data),
		void(*InsertDistance)(messageStruct* const me,uint32_t  hcSr04,uint16_t vl53l0x),
		void(*InsertDistanceWithPosition)(messageStruct* const me,uint32_t  hcSr04,uint16_t vl53l0x,uint8_t position),
		void(*InsertIrSensor)(messageStruct* const me,uint32_t value),
		void(*InsertAdcBatteryVoltage)(messageStruct* const me,uint32_t value),
		void(*SendMessage)(messageStruct* const me),
		void(*SendDriveStatus)(messageStruct* const me,uint8_t status));

void Message_Insert(messageStruct* const me,uint8_t data);
void Message_InsertError(messageStruct* const me,uint8_t data);
void Message_InsertDistance(messageStruct* const me,uint32_t  hcSr04,uint16_t vl53l0x);
void Message_InsertDistanceWithPosition(messageStruct* const me,uint32_t  hcSr04,uint16_t vl53l0x,uint8_t position);
void Message_InsertIrSensor(messageStruct* const me,uint32_t value);
void Message_InsertAdcBatteryVoltage(messageStruct* const me,uint32_t value);
void Message_SendMessage(messageStruct* const me);
void Message_SendDriveStatus(messageStruct* const me,uint8_t status);
