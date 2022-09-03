/*
 * motorsControl.h
 *
 *  Created on: 19 lut 2022
 *      Author: Daniel
 */

#ifndef INC_MOTORSCONTROL_H_
#define INC_MOTORSCONTROL_H_

typedef struct motorsControlStruct motorsControlStruct;

struct motorsControlStruct{
	GPIO_TypeDef *outGPIO;
	uint16_t pinOut1; //L298NOUT1A
	uint16_t pinOut2; //L298NOUT2A
	uint16_t pinOut3; //L298NOUT1B
	uint16_t pinOut4; //L298NOUT2B
	TIM_HandleTypeDef *timerControlMotor;
	uint32_t timerPeriod;
	uint32_t timerCompareForTurn;
	uint8_t channelEnA;
	uint8_t channelEnB;
	void (*drivingForward)(motorsControlStruct* me);
	void (*drivingRight)(motorsControlStruct* me);
	void (*drivingLeft)(motorsControlStruct* me);
	void (*drivingBackwart)(motorsControlStruct* me);
	void (*clockwiseRotation)(motorsControlStruct* me);
	void (*countersclockwiseRotation)(motorsControlStruct* me);
	void (*drivingReverseRight)(motorsControlStruct* me);
	void (*drivingReverseLeft)(motorsControlStruct* me);
	void (*resetDriving)(motorsControlStruct* me);
};

motorsControlStruct* MotorsControl_Create(GPIO_TypeDef *outGPIO,
										uint16_t pinOut1,
										uint16_t pinOut2,
										uint16_t pinOut3,
										uint16_t pinOut4,
										TIM_HandleTypeDef *timerControlMotor,
										uint8_t channelEnA,
										uint8_t channelEnB);
void MotorsControl_Init(motorsControlStruct* me,
						void (*drivingForward)(motorsControlStruct* me),
						void (*drivingRight)(motorsControlStruct* me),
						void (*drivingLeft)(motorsControlStruct* me),
						void (*drivingBackwart)(motorsControlStruct* me),
						void (*clockwiseRotation)(motorsControlStruct* me),
						void (*countersclockwiseRotation)(motorsControlStruct* me),
						void (*drivingReverseRight)(motorsControlStruct* me),
						void (*drivingReverseLeft)(motorsControlStruct* me),
						void (*resetDriving)(motorsControlStruct* me));

void MotorsControl_DrivingForward(motorsControlStruct* me);
void MotorsControl_DrivingRight(motorsControlStruct* me);
void MotorsControl_DrivingLeft(motorsControlStruct* me);
void MotorsControl_DrivingBackwart(motorsControlStruct* me);
void MotorsControl_ClockwiseRotation(motorsControlStruct* me);
void MotorsControl_CountersclockwiseRotation(motorsControlStruct* me);
void MotorsControl_DrivingReverseRight(motorsControlStruct* me);
void MotorsControl_DrivingReverseLeft(motorsControlStruct* me);
void MotorsControl_ResetDriving(motorsControlStruct* me);
void MotorsControl_SetOutput(motorsControlStruct* me,GPIO_PinState A1,GPIO_PinState A2,GPIO_PinState B1,GPIO_PinState B2);
void MotorsControl_SetHtimCompare(motorsControlStruct* me,uint32_t htimChannel1,uint32_t htimChannel2);

#endif /* INC_MOTORSCONTROL_H_ */
