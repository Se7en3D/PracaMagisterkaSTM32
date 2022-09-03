#include <IrSensor.h>
#include "Message.h"
#include "CommandDecoder.h"
#include "BatteryVoltage.h"
#include "MotorsControl.h"
#include "RangeMeasurment.h"

#define CAR_TIME_TO_SEND_BATTERY_VOLTAGE 1000 /*!<Czas po jakim załączny jest pomiar napięcia[ms]*/
#define CAR_TIME_TO_SEND_STATUS_IRSENSOR 250 /*!<Czas po jakim nalezy wyslac status czujnikow IR*/
#define driveFunctionLength sizeof(driveFunction)/sizeof(driveFunction[0])
#define CAR_TIME_TO_RESET_DRIVE 300 /*!<Czas po jakim należy zresetować kierunek jazdy*/
#define CAR_MAX_MEASURE_DISTANCE_FOR_PC 10 /*!<Ilość pomiarów testowych po odebraniu funkcji MEASURE_DISTANCE_FUN*/
typedef struct carModule carModule;
typedef struct carAutoDrive carAutoDrive;
typedef enum carModuleControlStatus carModuleControlStatus;
typedef enum carModuleModeDriving carModuleModeDriving;

enum carModuleModeDriving{
	modeDrivingManual,
	modeDrivingManualWithCollisionDetect,
	modeDrivingManualWithAutoEvasionObstacle,
};

enum carModuleControlStatus{
	controlIdle,
	controlChangePosition,
	controlWaitForRangeMeasurment,
	controlPrepareMeasurmentDistancePos0,
	controlPrepareMeasurmentDistancePos1,
	controlPrepareMeasurmentDistancePos2,
	controlPrepareMeasurmentDistancePos3,
	controlPrepareMeasurmentDistancePos4,
	controlPrepareMeasurmentDistancePos5,
	controlPrepareMeasurmentDistancePos6,
	controlPrepareMeasurmentDistancePos7,
	controlPrepareMeasurmentDistancePos8,
	controlPrepareMeasurmentDistancePos9,
	controlPrepareMeasurmentDistancePos10,
	controlPrepareMeasurmentDistancePos11,
	controlPrepareMeasurmentDistancePos12,
	controlWaitForMeasurmentDistance,
	controlPickALargerDistanceBetweenPos0AndPos12,
	controlWaitForDistanceisEqualThatPos6,
	controlSetTimeTo100ms,
	controlWaitForClearIrSensorNo1,
	controlStopDrive,
	controlDriveForward,
	controlDriveBackward,
	controlDriveRight,
	controlDriveLeft,
	controlDriveClockwiseRotation,
	controlDriveCountersclockwiseRotation,
	controlWaitUntilTheDistanceIsGreater,
	controlWaitToStopDrive,
	controlDriveByPickPosition,
	controlDriveBackwardRight,
	controlDriveMachineStop,
	controlFindMinimalPosition,
	controlPrepareMeasurmentDistanceFromMinimalPosition,
	controlSetSmallPositionToPos6,
	controlWaitUntilTheDistanceIsGreaterThat20,
	controlDriveBackwardLeft

};
struct carAutoDrive{
	uint32_t HcSr04Distance[13];
	uint16_t Vl53l0xDistance[13];
	//carModuleControlStatus controlStatus;
	uint32_t positionStateMachineControlCar;
	uint8_t selectedLargerPosition;
	uint8_t selectedSmallerPosition;
	uint32_t time;
};
struct carModule{
		//Lokalne zmienne
	carAutoDrive autoDriveVariable;
	carModuleModeDriving modeDriving;
	uint8_t ReceivedInMessageBuff;
	TIM_HandleTypeDef* timer1ms;
	uint32_t timeToMeasureBatteryVoltage;
	uint32_t timeToSendIrSensorStatus;
	uint32_t timeToResetMotorControl;
	functionFromPcEnum driveStatus;
	uint8_t measureDistanceForPcCount;
		//Moduły
	messageStruct *outMessage;
	bluetoothDecoderStruct *inMessage;
	adcMeasureStruct *batteryVoltage;
	rangeMeasurmentStruct *rangeMeasurmentControl;
	irSensorStruct *irSensor;
	motorsControlStruct *motorControl;
		//Funkcje
	void (*createOutMessage)(carModule *me,UART_HandleTypeDef *Message_huart);
	void (*createInMessage)(carModule *me);
	void (*createBatteryVoltage)(carModule *me,ADC_HandleTypeDef* adcBattery,GPIO_TypeDef* gpioBattery,uint16_t pinBattery);
	void (*createServoPR)(carModule *me,TIM_HandleTypeDef *PWMtimerGen,uint32_t TimChannel);
	void (*createHcSr04)(carModule *me,TIM_HandleTypeDef* timerHcSr04);
	void (*createVl53l0x)(carModule *me,I2C_HandleTypeDef * hi2cVl53l0,GPIO_TypeDef *xshutgpio,uint16_t xshutpin);
	void (*createIrSensor)(carModule *me);
	void (*createMotorControl)(carModule *me,GPIO_TypeDef *outGPIO,uint16_t pinOut1,uint16_t pinOut2,uint16_t pinOut3,uint16_t pinOut4,TIM_HandleTypeDef *timerControlMotor,uint8_t channelEnA,uint8_t channelEnB);
};

carModule* CarTestModule;

carModule* Car_Create(TIM_HandleTypeDef* timer1ms);
void Car_Init(carModule *me,
			void (*createOutMessage)(carModule *me,UART_HandleTypeDef *Message_huart),
			void (*createInMessage)(carModule *me),
			void (*createBatteryVoltage)(carModule *me,ADC_HandleTypeDef* adcBattery,GPIO_TypeDef* gpioBattery,uint16_t pinBattery),
			void (*createServoPR)(carModule *me,TIM_HandleTypeDef *PWMtimerGen,uint32_t TimChannel),
			void (*createHcSr04)(carModule *me,TIM_HandleTypeDef* timerHcSr04),
			void (*createVl54l0x)(carModule *me,I2C_HandleTypeDef * hi2cVl53l0,GPIO_TypeDef *xshutgpio,uint16_t xshutpin),
			void (*createIrSensor)(carModule *me),
			void (*createMotorControl)(carModule *me,GPIO_TypeDef *outGPIO,uint16_t pinOut1,uint16_t pinOut2,uint16_t pinOut3,uint16_t pinOut4,TIM_HandleTypeDef *timerControlMotor,uint8_t channelEnA,uint8_t channelEnB));
void Car_CreateOutMessage(carModule *me,UART_HandleTypeDef *Message_huart);
void Car_CreateInMessage(carModule *me);
void Car_CreateBatteryVoltage(carModule *me,ADC_HandleTypeDef* adcBattery,GPIO_TypeDef* gpioBattery,uint16_t pinBattery);
void Car_CreateServoPR(carModule *me,TIM_HandleTypeDef *PWMtimerGen,uint32_t TimChannel);
void Car_CreateHcSr04(carModule *me,TIM_HandleTypeDef* timerHcSr04);
void Car_CreateVl54l0x(carModule *me,I2C_HandleTypeDef * hi2cVl53l0,GPIO_TypeDef *xshutgpio,uint16_t xshutpin);
void Car_CreateIrSensor(carModule *me);
void Car_createMotorControl(carModule *me,GPIO_TypeDef *outGPIO,uint16_t pinOut1,uint16_t pinOut2,uint16_t pinOut3,uint16_t pinOut4,TIM_HandleTypeDef *timerControlMotor,uint8_t channelEnA,uint8_t channelEnB);
void Car_mainFun(carModule *me);
void Car_NotRecognized(carModule *me);
void Car_DrivingForward(carModule *me);
void Car_DrivingBackwart(carModule *me);
void Car_DrivingRight(carModule *me);
void Car_DrivingLeft(carModule *me);
void Car_CountersclockwiseRotation(carModule *me);
void Car_ClockwiseRotation(carModule *me);
void Car_ResetDriving(carModule *me);
void Car_DrivingReverseRight(carModule *me);
void Car_DrivingReverseLeft(carModule *me);
void Car_GetStatusFromAllStruct(carModule *me);
void Car_MeasureDistanceForPc(carModule *me);

void Car_ControlIdle(carModule *me);
void Car_ControlChangePosition(carModule *me);
void Car_ControlWaitForRangeMeasurment(carModule *me);
void Car_ControlPrepareMeasurmentDistancePos0(carModule *me);
void Car_ControlPrepareMeasurmentDistancePos1(carModule *me);
void Car_ControlPrepareMeasurmentDistancePos2(carModule *me);
void Car_ControlPrepareMeasurmentDistancePos3(carModule *me);
void Car_ControlPrepareMeasurmentDistancePos4(carModule *me);
void Car_ControlPrepareMeasurmentDistancePos5(carModule *me);
void Car_ControlPrepareMeasurmentDistancePos6(carModule *me);
void Car_ControlPrepareMeasurmentDistancePos7(carModule *me);
void Car_ControlPrepareMeasurmentDistancePos8(carModule *me);
void Car_ControlPrepareMeasurmentDistancePos9(carModule *me);
void Car_ControlPrepareMeasurmentDistancePos10(carModule *me);
void Car_ControlPrepareMeasurmentDistancePos11(carModule *me);
void Car_ControlPrepareMeasurmentDistancePos12(carModule *me);
void Car_ControlWaitForMeasurmentDistance(carModule *me);
void Car_ControlPickALargerDistanceBetweenPos0AndPos12(carModule *me);
void Car_ControlWaitForDistanceEqualThatPos6(carModule *me);
void Car_ControlSetTimeTo100ms(carModule *me);
void Car_controlWaitForClearIrSensorNo1(carModule *me);
void Car_ControlStopDrive(carModule *me);
void Car_ControlDriveForward(carModule *me);
void Car_ControlDriveBackward(carModule *me);
void Car_ControlDriveRight(carModule *me);
void Car_ControlDriveLeft(carModule *me);
void Car_ControlDriveClockwiseRotation(carModule *me);
void Car_ControlDriveCountersclockwiseRotation(carModule *me);
void Car_ControlWaitUntilTheDistanceIsGreater(carModule *me);
void Car_ControlWaitToStopDrive(carModule *me);
void Car_ControlDriveByPickPosition(carModule *me);
void Car_ControlDriveBackwardRight(carModule *me);
void Car_ControlDriveMachineStop(carModule *me);
void Car_ControlFindMinimalPosition(carModule *me);
void Car_ControlPrepareMeasurmentDistanceFromMinimalPosition(carModule *me);
void Car_ControlSetSmallPositionToPos6(carModule *me);
void Car_ControlWaitUntilTheDistanceIsGreaterThat20(carModule *me);
void Car_ControlDriveBackwardLeft(carModule *me);

void Car_NextPositionStateMachineControlCar(carAutoDrive *me);
void Car_PrevPositionStateMachineControlCar(carAutoDrive *me);
uint32_t Car_AutoDriveGetAutoPosition(carAutoDrive *me,uint8_t irSensorValue,functionFromPcEnum
);
void Car_AddIrSensor(carModule *me,GPIO_TypeDef *gpio, uint16_t pin);

