/*
 * uartCom.h
 *
 *  Created on: 12 lip 2021
 *      Author: DanielD
 */

#ifndef INC_UARTCOM_H_
#define INC_UARTCOM_H_

#define SERVO_MAX_POSITION 13 //Maksymalna ilość pozycji serwomechanizmu
#define UARTCOM_BUF_MAXLEN 500 //Maksymalna wielkość bufora nadajnika STM32->PC


#define MAXCOMMENDLENGTH 20 //Maksymalna dłygość komendy
#define FUNCTIONPOSITION 1 //Pozycja w ramce komunikacyjnej PC-STM32 na której znajduje się bit Funkcji
#define SIZE_BUFFOR_RECEIVED 1 //wielkośc buffora do obioru danych od *huart
#define MAX_MEASURMENT_SENSOR 20 //Ilość pomiorów wykonywana po odebraniu funkcji

	//Definicje funkcji pomiedzy PC<->STM32
#define RIDE_FORWARD_FUN 1
#define RIDE_BACKWARD_FUN 2
#define RIDE_RIGHT_FUN 3
#define RIDE_LEFT_FUN 4
#define ROTATE_LEFT 5
#define ROTATE_RIGHT 6
#define STOP_FUN 7
#define SEND_MEASUREMENT_FUN 125
#define ERROR_CODE_FUN 126
#define SEND_IR_SENSOR_STATUS 127
#define SEND_BATTERY_MEASURMENT_VALUE 128
#define CALIBRATION_PWM_DATA 129
#define MEASURE_DISTANCE_FUN 240
#define MEASURE_DISTANCE_FUN_RECEIVED 241
#define NO_DEFINED_FUN	250



volatile static uint16_t uartComArrayServoMaxMin[][2]={ //Tabela wartości minimalnych i maksymalnych pozycji serwomechanizmu
		{0,13}, //{min,max}  IDLE_DRIVING
		{0,5}, //GO_LEFT
		{7,13},//GO_RIGHT
		{3,8}, //GO_UP
		{0,13}, //GO_BACK
		{0,3}, //ROTATE_LEFT_DRIV
		{9,13}, //ROTATE_RIGHT_DRIV
		{0,13} //STOP_DRIVING
};
typedef struct{
	UART_HandleTypeDef *huart; //Wzkażnik do uartu pozwalającego na komunikacje z PC
	uint8_t bufforReceived[SIZE_BUFFOR_RECEIVED];//  tablica wykorzystywana jako buffor do obioru danych od *huart
	uint8_t frameCommand[MAXCOMMENDLENGTH]; //tablica przechowująca odebraną komendę
	uint8_t detectedCommand; //flaga oznaczająca odebraną komendę (0- brak komendy 1- odebrano komendę i oczekuję na wykonanie)
	uint8_t positionCommand; //aktualna pozycja zapisu danych do tablicy frameCommand
}uartCommunication_t; //Struktura komunikacji pomiedzy PC->STM32


typedef struct{
	TIM_HandleTypeDef *htimServo; //Wskażnik do timera odpowiedzialnego za kontrole nad seromechanizmem
	uint16_t tableDegreesServo[SERVO_MAX_POSITION]; //tablica zawierająca wypełnienie *htimServo potrzebne do zmiany kąta samgeo serwomechanizmu
	uint16_t position; //aktualna pozycja serwoMechanizmy
	uint8_t waitForData; //Oczekiwanie na odebranie danych pomiarowych z czujnika ultradzwiękowego i laserowego
	uint8_t measureForMeasureDistanceFun;
}uartSensorFrame_t;// struktura serwomechanizmu

typedef struct {
    uint8_t buffer[UARTCOM_BUF_MAXLEN]; //Bufor nadajnika STM32-PC
    int head; //pozycja kolejnego elementu do zapisania danych
    int tail; //pozycja ostanio odebranego elementu tablicy
    int maxlen;//maksymalna wielkość tablicy
} uartCom_bbuf_t; // struktura bufora kołowego do wysyłania danych STM32->PC


uartCommunication_t *uartCommunication_p;
uartSensorFrame_t uartComSensorFrame;
uartCom_bbuf_t uartComBuffer;

/**
  * @brief Initialize the UART mode according to the specified
  * @param data UART handle.
  * @retval HAL status
  */
void uartComAddFrame(uint8_t data);
/**
  * @brief Initialize the UART mode according to the specified
  * @param data UART handle.
  * @retval HAL status
  */
uint8_t* uartComGetFrame();
void uartComClearFrame();
void uartComSendDistance(float hcSr04, uint16_t vl5310x);
void uartComSendDistanceServo(float hcSr04, uint16_t vl5310x);
void uartComSendErrorCode(uint8_t errorCode);
void uartComSendIrSensorStatus(uint8_t *collision,uint16_t size);
void uartComSendAdcBatteryVoltage(uint32_t value);
void uartComSensorInit(TIM_HandleTypeDef *htim);
void uartComServoNextPosition();
void uartComServoPreviousPosition();
void uartComServoIsShiftPosition();
void uartComServoClearWaitForData();
void uartComPush(const uint8_t data);
uint8_t *uartComGetBufferFirstElementAddress();
uint32_t uartComGetBufferLength();
uint8_t uartComGetMeasureForMeasureDistanceFun();
void uartComAddMeasureForMeasureDistanceFun();
void uartComResetMeasureForMeasureDistanceFun();

#endif /* INC_UARTCOM_H_ */
