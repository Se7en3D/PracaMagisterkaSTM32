/**
  ******************************************************************************
  * @file    servo.h
  * @author  Daniel Dunak
  * @brief   Plik nagłówkowy odpowiedzialny za kontrole servo motora o rotacji pozycyjnej zaleznej od wypelnienia PWM
  *
  ******************************************************************************
  */

#ifndef INC_SERVOPR_H_
#define INC_SERVOPR_H_

#define SERVOPR_MULTIPLIER 1000 /*!<Wartość o jaką należy pomnożyć stałe SERVOPR_MIN_PERIOD_IN_MS i SERVOPR_MAX_PERIOD_IN_MS*/
#define SERVOPR_MAX_ROTATION_ANGLE 180 /*!<Maksymalny kąt obrotu serwo motora */
#define SERVOPR_SUM_POSITION_SERVO 13 /*!<Ilość pozycji*/
#define SERVOPR_MIN_PERIOD_IN_MS SERVOPR_MULTIPLIER*0.5 /*!<Najmniejsza wartosc wypełnienia potrzebna do obslugi servo podawana w milisekundach*/
#define SERVOPR_MAX_PERIOD_IN_MS SERVOPR_MULTIPLIER*2.5 /*!<Największa wartosc wypełnienia potrzebna do obslugi servo podawana w milisekundach*/



/**
  * @brief  servoPR obecny status servo motoru
  */
typedef enum{
	servoPR_INITIALIZATION=0, /*!< Servo w stanie inicjalizacji */
	servoPR_IDLE, /*!< Servo w stanie spoczynku */
	servoPR_WAIT_TO_STABILIZE_SERVO, /*!< Servo w oczekiwania na stabilizacje*/
	servoPR_WAIT_TO_MEASURMENT, /*!< Servo w oczekiwaniu na pomiar odległości*/
	servoPR_ERROR_, /*!< Wykruto błąd w działaniu servo*/
}servoPR_state;

/**
  * @brief servoPR_GeneralStructure glowna struktura odpowiedzialna za kontrole servo
  */
typedef struct{
	TIM_HandleTypeDef *PWMtimerGen; /*!< Uchwyt do struktury TIM_HandleTypeDef*/
	servoPR_state status; /*!<Obecny status servo*/
	uint8_t currentAngle; /*!<Obecny kat servo zakres od 0 do 180 */
	uint16_t periodTimer;/*!<Okres PWM dla 20ms*/
	uint16_t pulseValue[SERVOPR_SUM_POSITION_SERVO];/*!<Wartość wypełnienia dla poszczególnych katow obrotu*/
}servoPR_GeneralStructure;

servoPR_GeneralStructure servoPRGeneralStructure;

void servoPRInit(servoPR_GeneralStructure *servoPR,TIM_HandleTypeDef *PWMtimerGen);
uint8_t servoPRGetCurrentAngle(servoPR_GeneralStructure *servoPR);
void servoPRSetCurrentAngle(servoPR_GeneralStructure *servoPR,uint8_t newAngle);

#endif /* INC_SERVOPR_H_ */
