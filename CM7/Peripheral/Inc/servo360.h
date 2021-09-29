/*
 * servo360.h
 *
 *  Created on: 6 sie 2021
 *      Author: DanielD
 */

#ifndef INC_SERVO360_H_
#define INC_SERVO360_H_


#define SERVO360_DEFAULT_PERIOD 3600 //Domyśle wypełnienie PWM
#define SERVO360_STATE_SIZE 6	//Ilość statusów znajdujących się w servo360
#define SERVO360_MAX_POSITION 13 //maksymalna ilość pozycji
#define SERVO360_MAX_ANGLE 180 //maksymalny kąt poruszania się serwomechanizmu
#define SERVO360_PERIOD_INITIALIZATION 3000 //wypełnienie PWM podczas inicjalizacji
#define SERVO360_REPEAT_INITIALIZATION 3	//powtórzenie wypełnienia do kolejnego sprawdzenia
#define SERVO360_MAX_REPEAT_TO_ERROR_IN_INITIALIZATION 20 //powtórzenia sekwencji inicjalizującej po której ma nastapić błąd
#define SERVO360_PERIOD_ROTATION_RIGHT 4200 //wypełnienie podczas obrotu w lewo
#define SERVO360_PERIOD_ROTATION_LEFT 3012 // wpełnienie podczas obrotu w prawo
#define SERVO360_REPEAT_ROTATION 8 //powtórzenie podczas oboru
#define SERVO360_DELAY_TO_CHANGE_PERIOD 3 //opóznienie ustawienia nowego wypełnienia

volatile static uint8_t servo360RepeatAngle[]={
		9, //Powtórzenie dla obrotu o 15 stopni
		14,//Powtórzenie dla obrotu o 30 stopni
		18,//Powtórzenie dla obrotu o 45 stopni
		24,//Powtórzenie dla obrotu o 60 stopni
		26,//Powtórzenie dla obrotu o 75 stopni
		33,//Powtórzenie dla obrotu o 90 stopni
		36,//Powtórzenie dla obrotu o 105 stopni
		39,//Powtórzenie dla obrotu o 120 stopni
		43,//Powtórzenie dla obrotu o 135 stopni
		48,//Powtórzenie dla obrotu o 150 stopni
		52,//Powtórzenie dla obrotu o 165 stopni
		55,//Powtórzenie dla obrotu o 180 stopni
};
typedef enum{
	servo360_PositionNone=-1,
	servo360_Position0,
	servo360_Position1,
	servo360_Position2,
	servo360_Position3,
	servo360_Position4,
	servo360_Position5,
	servo360_Position6,
	servo360_Position7,
	servo360_Position8,
	servo360_Position9,
	servo360_Position10,
	servo360_Position11,
	servo360_Position12
}servo360_Position;

typedef enum{
	servo360_INITIALIZATION=0,
	servo360_IDLE,
	servo360_ROTATE,
	servo360_WAIT_TO_STABILIZE_SERVO,
	servo360_WAIT_TO_MEASURMENT,
	servo360_ERROR_INITIALIZATION,
}servo360_state;


typedef struct{
	TIM_HandleTypeDef *htim;
	uint32_t targetPosition;
	uint32_t repeatValue;
	uint32_t period;
	servo360_state status;
	uint16_t limitSwitchServoPin;
	int currentPosition;
	GPIO_TypeDef *limitSwitchServoPort;
	uint8_t delayToChangePeriod;
	 void (*stateFunction[SERVO360_STATE_SIZE])(void);
}servo360_Base_Structure;

servo360_Base_Structure servo360Structure;
void servo360Init(TIM_HandleTypeDef *htim,uint16_t pin ,GPIO_TypeDef *port);
void servo360PWMEdit();
void servo360NewDataPWM(uint8_t *framePointer);
void servo360StatusInitialization();
void servo360StatusIdle();
void servo360StatusRotate();
void servo360StatusWaitToMeasurment();
void servo360PWMDefault();
void servo360StateFunctions();
void servo360SetCurrentPositionByPositionNumber(uint16_t number);
void servo360SetTargetPosition(servo360_Position position);
void servo360NextPosition();
void servo360PrevPosition();
#endif /* INC_SERVO360_H_ */
