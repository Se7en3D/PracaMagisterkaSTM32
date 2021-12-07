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
#define SERVOPR_SUM_POSITION_SERVO 12 /*!<Ilość pozycji*/
#define SERVOPR_MIN_PERIOD_IN_MS SERVOPR_MULTIPLIER*0.5 /*!<Najmniejsza wartosc wypełnienia potrzebna do obslugi servo podawana w milisekundach*/
#define SERVOPR_MAX_PERIOD_IN_MS SERVOPR_MULTIPLIER*2.5 /*!<Największa wartosc wypełnienia potrzebna do obslugi servo podawana w milisekundach*/
#define SERVOPR_NON_BLOCKED_VALUE 0 /*<Wartość określająca niezablokowaną wartość zmiennej blockedStatus*/

/**
  * @brief  servoPR obecny status servo motoru
  */
typedef enum{
	servoPR_IDLE=0, /*!< Servo w stanie spoczynku */
	servoPR_WAIT_TO_STABILIZE_SERVO, /*!< Servo w oczekiwania na stabilizacje*/
	servoPR_WAIT_TO_MEASURMENT, /*!< Servo w oczekiwaniu na pomiar odległości*/
	servoPR_ERROR, /*!< Wykruto błąd w działaniu servo*/
}servoPR_state;


typedef enum{
	servoPRPosition0=15,
	servoPRPosition1=30,
	servoPRPosition2=45,
	servoPRPosition3=60,
	servoPRPosition4=75,
	servoPRPosition5=90,
	servoPRPosition6=105,
	servoPRPosition7=120,
	servoPRPosition8=135,
	servoPRPosition9=150,
	servoPRPosition10=165,
	servoPRPosition11=180,
}servoPR_Position;
/**
  * @brief servoPR_GeneralStructure glowna struktura odpowiedzialna za kontrole servo
  */
typedef struct{
	TIM_HandleTypeDef *PWMtimerGen; /*!< Uchwyt do struktury TIM_HandleTypeDef*/
	uint32_t TimChannel;/*!<Kanał sygnału PWM*/
	servoPR_state status; /*!<Obecny status servo*/
	uint8_t currentAngle; /*!<Obecny kat servo zakres od 0 do 180 */
	uint8_t targetAngle; /*!<Docelowy kąt servo*/
	uint8_t blockedStatus;/*!<Blokada statusu gdy wartość jest większa od zera*/
	uint16_t periodTimFor20ms;/*!<Okres PWM dla 20ms*/
	uint16_t timeToStabilize;/*!<Czas potrzebny do stabilizacji servo*/
	uint16_t time;/*!<Zmienna okreslająca upłynięty czas*/
	uint16_t pulseValue[SERVOPR_SUM_POSITION_SERVO];/*!<Wartość wypełnienia dla poszczególnych katow obrotu*/
}servoPR_GeneralStructure;

servoPR_GeneralStructure servoPRGeneralStructure;

void servoPRInit(servoPR_GeneralStructure *servoPR,TIM_HandleTypeDef *PWMtimerGen,uint32_t TimChannel);
uint8_t servoPRGetCurrentAngle(servoPR_GeneralStructure *servoPR);
void servoPRSetCurrentAngle(servoPR_GeneralStructure *servoPR,uint8_t newAngle);
void servoPRSetTargetAngle(servoPR_GeneralStructure *servoPR,uint8_t newAngle);
void servoPRIdleFunction();
void servoPRWaitToStabilizeServoFunction();
void servoPRWaitToMeasurmentFunction();
void servoPRErrorFunction();
void servoPRExecuteStatusFunction(servoPR_GeneralStructure *servoPR);
void servoPRClearBlockedStatus(servoPR_GeneralStructure *servoPR);
uint8_t servoPRGetBlockedStatus(servoPR_GeneralStructure *servoPR);
void servoPRChangePulseFill(servoPR_GeneralStructure *servoPR,const uint8_t targetAngle);
void servoPRAddTime(servoPR_GeneralStructure *servoPR);
uint8_t servoPRReadyToChangeAngle(servoPR_GeneralStructure *servoPR);
#endif /* INC_SERVOPR_H_ */
