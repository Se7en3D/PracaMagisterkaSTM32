/*
 * irSensor.h
 *
 *  Created on: 31 sty 2022
 *      Author: Daniel
 */

#ifndef INC_IRSENSOR_H_
#define INC_IRSENSOR_H_

#define IRSENSOR_MAX_SIZE_LINK_LIST 32 /*!<Maksymalna wielkość link listy*/
#define IRSENSOR_MAX_ON_VALUE 10 /*!<Maksymalna wielkosc tablicy zawierajacej stan GPIO*/

typedef struct linkListGPIO linkListGPIO;
typedef struct irSensorStruct irSensorStruct;

struct linkListGPIO{
	GPIO_TypeDef *gpio;
	uint16_t pin;
	uint32_t value;
	linkListGPIO *next;
};

struct irSensorStruct{
	linkListGPIO *first;
	void (*readStatus)(irSensorStruct*  const me);
	uint32_t (*getValue)(irSensorStruct* const me);
	void (*addGPIO)(irSensorStruct* const me, GPIO_TypeDef *gpio, uint16_t pin);
	uint32_t (*getSize)(irSensorStruct* const me);
};

irSensorStruct* IrSensor_Create();
void IrSensor_Init(irSensorStruct*  const me,
		void (*readStatus)(irSensorStruct*  const me),
		uint32_t (*getValue)(irSensorStruct* const me),
		void (*addGPIO)(irSensorStruct* const me, GPIO_TypeDef *gpio, uint16_t pin),
		uint32_t (*getSize)(irSensorStruct* const me));
void IrSensor_ReadGPIO(irSensorStruct* const me);
uint32_t IrSensor_GetValueGPIO(irSensorStruct* const me);
uint32_t IrSensor_GetSize(irSensorStruct* const me);
void IRSensor_AddGPIO(irSensorStruct* const me, GPIO_TypeDef *gpio, uint16_t pin);
#endif /* INC_IRSENSOR_H_ */
