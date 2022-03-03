#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "Car.h"

static const volatile uint8_t Car_ServoPRPosition[][5]={
		{6,'\0'},	/*IDLE_DRIVING*/
		{5,6,7,'\0'},	//GO_UP
		{6,'\0'},	//GO_BACK
		{1,2,3,4,'\0'},	//GO_UP_RIGHT
		{10,9,8,7,'\0'}, //GO_UP_LEFT
		{12,11,10,'\0'},	//ROTATE_LEFT_DRIV
		{0,1,2,'\0'}, //ROTATE_RIGHT_DRIV
		{6,'\0'}, //STOP_DRIVING
		{6,'\0'}, //GO_BACK_RIGHT
		{6,'\0'}, //GO_BACK_LEFT
};

static const volatile  void (*driveFunction[])(carModule *me)={
		Car_NotRecognized,
		Car_DrivingForward,
		Car_DrivingBackwart,
		Car_DrivingRight,
		Car_DrivingLeft,
		Car_CountersclockwiseRotation,
		Car_ClockwiseRotation,
		Car_ResetDriving,
		Car_DrivingReverseRight,
		Car_DrivingReverseLeft,
		Car_GetStatusFromAllStruct,
		Car_MeasureDistanceForPc
};
static const volatile void (*controlFunction[])(carModule *me)={
		Car_ControlIdle,
		Car_ControlChangePosition,
		Car_ControlWaitForRangeMeasurment,
		Car_ControlPrepareMeasurmentDistancePos0,
		Car_ControlPrepareMeasurmentDistancePos1,
		Car_ControlPrepareMeasurmentDistancePos2,
		Car_ControlPrepareMeasurmentDistancePos3,
		Car_ControlPrepareMeasurmentDistancePos4,
		Car_ControlPrepareMeasurmentDistancePos5,
		Car_ControlPrepareMeasurmentDistancePos6,
		Car_ControlPrepareMeasurmentDistancePos7,
		Car_ControlPrepareMeasurmentDistancePos8,
		Car_ControlPrepareMeasurmentDistancePos9,
		Car_ControlPrepareMeasurmentDistancePos10,
		Car_ControlPrepareMeasurmentDistancePos11,
		Car_ControlPrepareMeasurmentDistancePos12,
		Car_ControlWaitForMeasurmentDistance,
		Car_ControlPickALargerDistanceBetweenPos0AndPos12,
		Car_ControlWaitForDistanceEqualThatPos6,
		Car_ControlSetTimeTo100ms,
		Car_controlWaitForClearIrSensorNo1,
		Car_ControlStopDrive,
		Car_ControlDriveForward,
		Car_ControlDriveBackward,
		Car_ControlDriveRight,
		Car_ControlDriveLeft,
		Car_ControlDriveClockwiseRotation,
		Car_ControlDriveCountersclockwiseRotation,
		Car_ControlWaitUntilTheDistanceIsGreater,
		Car_ControlWaitToStopDrive
};
static const volatile carModuleControlStatus stateMachineControlCar[]={
		controlIdle,
		controlChangePosition,
		controlWaitForRangeMeasurment,
		controlIdle,
		controlStopDrive,
		controlPrepareMeasurmentDistancePos3,
		controlWaitForMeasurmentDistance,
		controlDriveBackward,
		controlWaitUntilTheDistanceIsGreater,
		controlDriveClockwiseRotation,
		controlSetTimeTo100ms,
		controlWaitToStopDrive,
		controlStopDrive,
		controlIdle,
		controlStopDrive,
		controlPrepareMeasurmentDistancePos6,
		controlWaitForMeasurmentDistance,
		controlDriveBackward,
		controlWaitUntilTheDistanceIsGreater,
		controlStopDrive,
		controlPrepareMeasurmentDistancePos0,
		controlWaitForMeasurmentDistance,
		controlPrepareMeasurmentDistancePos12,
		controlWaitForMeasurmentDistance,
		controlPrepareMeasurmentDistancePos6,
		controlWaitForMeasurmentDistance,
		controlPickALargerDistanceBetweenPos0AndPos12,
		controlWaitForDistanceisEqualThatPos6,
		controlStopDrive,
		controlIdle,
		controlStopDrive,
		controlPrepareMeasurmentDistancePos9,
		controlWaitForMeasurmentDistance,
		controlDriveBackward,
		controlWaitUntilTheDistanceIsGreater,
		controlDriveCountersclockwiseRotation,
		controlSetTimeTo100ms,
		controlWaitToStopDrive,
		controlStopDrive,
		controlIdle,
		controlStopDrive,
		controlSetTimeTo100ms,
		controlDriveForward,
		controlWaitToStopDrive,
		controlSetTimeTo100ms,
		controlDriveRight,
		controlWaitToStopDrive,
		controlIdle,
		controlStopDrive,
		controlSetTimeTo100ms,
		controlDriveForward,
		controlWaitToStopDrive,
		controlSetTimeTo100ms,
		controlDriveLeft,
		controlWaitToStopDrive,
		controlIdle,
		controlStopDrive,
		controlSetTimeTo100ms,
		controlDriveForward,
		controlWaitToStopDrive,
		controlSetTimeTo100ms,
		controlWaitToStopDrive,
		controlPrepareMeasurmentDistancePos0,
		controlWaitForMeasurmentDistance,
		controlPrepareMeasurmentDistancePos12,
		controlWaitForMeasurmentDistance,
		controlPickALargerDistanceBetweenPos0AndPos12,
		controlSetTimeTo100ms,
		controlWaitToStopDrive,
		controlIdle,
};

carModule* Car_Create(TIM_HandleTypeDef* timer1ms){
	carModule* me=malloc(sizeof(carModule));
	if(me!=NULL){
		me->autoDriveVariable.positionStateMachineControlCar=0;
		me->autoDriveVariable.time=0;
		me->ReceivedInMessageBuff=0;
		me->timer1ms=timer1ms;
		me->timerMeasureBatteryVoltage=0;
		me->timerToSendIrSensorStatus=0;
		me->driveStatus=RESET_DRIVE;
		me->measureDistanceForPcCount=0;
		me->rangeMeasurmentControl=RangeMeasurment_Create();
		me->modeDriving=modeDrivingManual;
		Car_Init(me,
				Car_CreateOutMessage,
				Car_CreateInMessage,
				Car_CreateBatteryVoltage,
				Car_CreateServoPR,
				Car_CreateHcSr04,
				Car_CreateVl54l0x,
				Car_CreateIrSensor,
				Car_createMotorControl);
	}
	return me;
}
void Car_Init(carModule *me,
			void (*createOutMessage)(carModule *me,UART_HandleTypeDef *Message_huart),
			void (*createInMessage)(carModule *me),
			void (*createBatteryVoltage)(carModule *me,ADC_HandleTypeDef* adcBattery,GPIO_TypeDef* gpioBattery,uint16_t pinBattery),
			void (*createServoPR)(carModule *me,TIM_HandleTypeDef *PWMtimerGen,uint32_t TimChannel),
			void (*createHcSr04)(carModule *me,TIM_HandleTypeDef* timerHcSr04),
			void (*createVl54l0x)(carModule *me,I2C_HandleTypeDef * hi2cVl53l0,GPIO_TypeDef *xshutgpio,uint16_t xshutpin),
			void (*createIrSensor)(carModule *me),
			void (*createMotorControl)(carModule *me,GPIO_TypeDef *outGPIO,uint16_t pinOut1,uint16_t pinOut2,uint16_t pinOut3,uint16_t pinOut4,TIM_HandleTypeDef *timerControlMotor,uint8_t channelEnA,uint8_t channelEnB)){
	me->createOutMessage=createOutMessage;
	me->createInMessage=createInMessage;
	me->createBatteryVoltage=createBatteryVoltage;
	me->createServoPR=createServoPR;
	me->createHcSr04=createHcSr04;
	me->createVl54l0x=createVl54l0x;
	me->createIrSensor=createIrSensor;
	me->createMotorControl=createMotorControl;
}
void Car_CreateOutMessage(carModule *me,UART_HandleTypeDef *Message_huart){
	me->outMessage=Message_Create(Message_huart);
	HAL_UART_Receive_IT(Message_huart,&me->ReceivedInMessageBuff,1);
}
void Car_CreateInMessage(carModule *me){
	me->inMessage=CommandDecoder_Create();
}
void Car_CreateBatteryVoltage(carModule *me,ADC_HandleTypeDef* adcBattery,GPIO_TypeDef* gpioBattery,uint16_t pinBattery){
	me->batteryVoltage=BatteryVoltage_Create(adcBattery, gpioBattery, pinBattery);
}
void Car_CreateServoPR(carModule *me,TIM_HandleTypeDef *PWMtimerGen,uint32_t TimChannel){
	me->rangeMeasurmentControl->createServoPR(me->rangeMeasurmentControl,PWMtimerGen, TimChannel);
}
void Car_CreateHcSr04(carModule *me,TIM_HandleTypeDef* timerHcSr04){
	me->rangeMeasurmentControl->createHcSr04(me->rangeMeasurmentControl,timerHcSr04);
}
void Car_CreateVl54l0x(carModule *me,I2C_HandleTypeDef * hi2cVl53l0,GPIO_TypeDef *xshutgpio,uint16_t xshutpin){
	me->rangeMeasurmentControl->createVl54l0x(me->rangeMeasurmentControl,hi2cVl53l0,xshutgpio,xshutpin);
}
void Car_CreateIrSensor(carModule *me){
	me->irSensor=IrSensor_Create();
}
void Car_createMotorControl(carModule *me,GPIO_TypeDef *outGPIO,uint16_t pinOut1,uint16_t pinOut2,uint16_t pinOut3,uint16_t pinOut4,TIM_HandleTypeDef *timerControlMotor,uint8_t channelEnA,uint8_t channelEnB){
	me->motorControl=MotorsControl_Create(outGPIO, pinOut1, pinOut2, pinOut3, pinOut4, timerControlMotor, channelEnA, channelEnB);
}
void Car_mainFun(carModule *me){

					/*Odbieranie komend z komputera PC*/
	uint8_t *pointerToCommunicationFunction=me->inMessage->GetFunction(me->inMessage);
	if(pointerToCommunicationFunction){
		functionFromPcEnum decodeStatus=(uint8_t) me->inMessage->DecodeTheFunction(me->inMessage);
		if(decodeStatus<(driveFunctionLength)){
			//driveFunction[decodeValue](me);
			CarTestModule->timeToResetMotorControl=0;
			if(me->driveStatus!=decodeStatus){
				Car_ResetDriving(me);
			}
			me->driveStatus=decodeStatus;
		}else{
			addErrorValue(CAR_OutOdSizeFunctionSevices);
		}
		me->inMessage->ClearBuffer(me->inMessage);
	}
					/*Wywołanie funkcji obsługi kierunku jazdy*/
	driveFunction[me->driveStatus](me);
	if(me->timeToResetMotorControl>=CAR_TIME_TO_RESET_DRIVE){
		Car_ResetDriving(me);
		CarTestModule->motorControl->resetDriving(CarTestModule->motorControl);
	}
				/*Obsługa modułu ppomiarowego napięcia zasilania bateryjnego*/
	if(me->timerMeasureBatteryVoltage>=CAR_TIME_TO_SEND_BATTERY_VOLTAGE){
		me->batteryVoltage->startMeasurment(me->batteryVoltage);
		me->timerMeasureBatteryVoltage=0;
	}

	uint32_t *pointerToValueFromAdc=me->batteryVoltage->getValue(me->batteryVoltage);
	if(pointerToValueFromAdc){
		me->outMessage->InsertAdcBatteryVoltage(me->outMessage,*pointerToValueFromAdc);
	}
					/*Obsługa modułu sensora IR*/
	me->irSensor->readStatus(me->irSensor);
	if(me->timerToSendIrSensorStatus>=CAR_TIME_TO_SEND_STATUS_IRSENSOR){
		uint32_t IrSensorValue=me->irSensor->getValue(me->irSensor);
		me->outMessage->InsertIrSensor(me->outMessage,IrSensorValue);
		me->timerToSendIrSensorStatus=0;
	}
					/*Obsługa main z modułu kontrolujace pomiar odleglości */
	me->rangeMeasurmentControl->main(me->rangeMeasurmentControl);
					/*Wysłanie danych z bufora nadajnika*/
	me->outMessage->SendMessage(me->outMessage);
}

				/*
				 * Funkcje kontroli jazdy
				 */
void Car_NotRecognized(carModule *me){
	printf("Nie rozpoznano funkcji\n");
	me->driveStatus=RESET_DRIVE;
	me->motorControl->resetDriving(me->motorControl);
}
void Car_DrivingForward(carModule *me){
	uint32_t positionStateMachineControlerCar=me->autoDriveVariable.positionStateMachineControlCar;
	if(positionStateMachineControlerCar<=3){
		uint8_t irSensorValue=me->irSensor->getValue(me->irSensor);
		uint32_t position=Car_AutoDriveGetAutoPosition(&me->autoDriveVariable,irSensorValue,me->driveStatus);
		if(position){
			me->autoDriveVariable.positionStateMachineControlCar=position;
		}else{
			me->motorControl->drivingForward(me->motorControl);
		}
	}
	controlFunction[stateMachineControlCar[me->autoDriveVariable.positionStateMachineControlCar]](me);
}
void Car_DrivingBackwart(carModule *me){
	uint32_t positionStateMachineControlerCar=me->autoDriveVariable.positionStateMachineControlCar;
		if(positionStateMachineControlerCar<=3){
			uint8_t irSensorValue=me->irSensor->getValue(me->irSensor);
			uint32_t position=Car_AutoDriveGetAutoPosition(&me->autoDriveVariable,irSensorValue,me->driveStatus);
			if(position){
				me->autoDriveVariable.positionStateMachineControlCar=position;
			}else{
				me->motorControl->drivingBackwart(me->motorControl);
			}
		}
		controlFunction[stateMachineControlCar[me->autoDriveVariable.positionStateMachineControlCar]](me);
}
void Car_DrivingRight(carModule *me){
	me->motorControl->drivingRight(me->motorControl);
}
void Car_DrivingLeft(carModule *me){
	me->motorControl->drivingLeft(me->motorControl);
}
void Car_CountersclockwiseRotation(carModule *me){
	me->motorControl->clockwiseRotation(me->motorControl);
}
void Car_ClockwiseRotation(carModule *me){
	me->motorControl->countersclockwiseRotation(me->motorControl);
}
void Car_ResetDriving(carModule *me){
	me->driveStatus=RESET_DRIVE;
	me->autoDriveVariable.positionStateMachineControlCar=0;
	if(me->measureDistanceForPcCount>0){
		addErrorValue(CAR_MeasureDistanceForPcCountIsNoZero);
		me->measureDistanceForPcCount=0;
	}
	me->motorControl->resetDriving(me->motorControl);
}
void Car_DrivingReverseRight(carModule *me){
	me->motorControl->drivingReverseRight(me->motorControl);
}
void Car_DrivingReverseLeft(carModule *me){
	me->motorControl->drivingReverseLeft(me->motorControl);
}
void Car_GetStatusFromAllStruct(carModule *me){
	printf("Wysłanie statusu\n");
}
void Car_MeasureDistanceForPc(carModule *me){
	me->timeToResetMotorControl=0;
	if(me->measureDistanceForPcCount>=CAR_MAX_MEASURE_DISTANCE_FOR_PC){
		me->measureDistanceForPcCount=0;
		Car_ResetDriving(me);
	}else{
		if(me->rangeMeasurmentControl->isRangeMeasurmentEnd(me->rangeMeasurmentControl)){
			uint16_t value1=*me->rangeMeasurmentControl->getVl53l0xDistance(me->rangeMeasurmentControl);
			uint32_t value2=*me->rangeMeasurmentControl->getHcSr04Distance(me->rangeMeasurmentControl);
			//uint8_t pos=*me->rangeMeasurmentControl->getServoPRPosition(me->rangeMeasurmentControl);
			me->outMessage->InsertDistance(me->outMessage,value2,value1);
			me->measureDistanceForPcCount++;
		}
		me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,6);

	}


}
				/*
				 * Funkcje kontroli jazdy automatycznej, pomiaru odległości itp.
				 */
void Car_ControlIdle(carModule *me){

}
void Car_ControlChangePosition(carModule *me){
	uint8_t pos=*me->rangeMeasurmentControl->getServoPRPosition(me->rangeMeasurmentControl);
	uint8_t i=0;
	while(Car_ServoPRPosition[me->driveStatus][i]!='\0'){
		if(Car_ServoPRPosition[me->driveStatus][i]==pos){
					break;
		}
		i++;
	}
	i++;
	if(Car_ServoPRPosition[me->driveStatus][i]=='\0'){
		i=0;
	}

	if(Car_ServoPRPosition[me->driveStatus][i-1]=='\0'){
		i=0;
	}

	if(CarTestModule->rangeMeasurmentControl->rangeMeasurment(CarTestModule->rangeMeasurmentControl,Car_ServoPRPosition[me->driveStatus][i])){
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}

}
void Car_ControlWaitForRangeMeasurment(carModule *me){
	if(me->rangeMeasurmentControl->isRangeMeasurmentEnd(me->rangeMeasurmentControl)){
		uint16_t value1=*me->rangeMeasurmentControl->getVl53l0xDistance(me->rangeMeasurmentControl);
		uint32_t value2=*me->rangeMeasurmentControl->getHcSr04Distance(me->rangeMeasurmentControl);
		uint8_t pos=*me->rangeMeasurmentControl->getServoPRPosition(me->rangeMeasurmentControl);
		/*printf("v1Vl=%d v2Hc=%d pos=%d \n ",(int) value1,(int)value2,(int)pos);*/
		me->outMessage->InsertDistanceWithPosition(me->outMessage,value2,value1,pos);

		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}

}
void Car_ControlPrepareMeasurmentDistancePos0(carModule *me){
	if(me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,0)){
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}
void Car_ControlPrepareMeasurmentDistancePos1(carModule *me){
	if(me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,1)){
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}
void Car_ControlPrepareMeasurmentDistancePos2(carModule *me){
	if(me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,2)){
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}
void Car_ControlPrepareMeasurmentDistancePos3(carModule *me){
	if(me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,3)){
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}
void Car_ControlPrepareMeasurmentDistancePos4(carModule *me){
	if(me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,4)){
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}
void Car_ControlPrepareMeasurmentDistancePos5(carModule *me){
	if(me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,5)){
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}
void Car_ControlPrepareMeasurmentDistancePos6(carModule *me){
	if(me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,6)){
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}
void Car_ControlPrepareMeasurmentDistancePos7(carModule *me){
	if(me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,7)){
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}
void Car_ControlPrepareMeasurmentDistancePos8(carModule *me){
	if(me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,8)){
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}
void Car_ControlPrepareMeasurmentDistancePos9(carModule *me){
	if(me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,9)){
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}
void Car_ControlPrepareMeasurmentDistancePos10(carModule *me){
	if(me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,10)){
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}
void Car_ControlPrepareMeasurmentDistancePos11(carModule *me){
	if(me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,11)){
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}
void Car_ControlPrepareMeasurmentDistancePos12(carModule *me){
	if(me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,12)){
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}
void Car_ControlWaitForMeasurmentDistance(carModule *me){
	if(me->rangeMeasurmentControl->isRangeMeasurmentEnd(me->rangeMeasurmentControl)){
		uint8_t servoPosition=*me->rangeMeasurmentControl->getServoPRPosition(me->rangeMeasurmentControl);
		me->autoDriveVariable.HcSr04Distance[servoPosition]=*me->rangeMeasurmentControl->getHcSr04Distance(me->rangeMeasurmentControl);
		me->autoDriveVariable.Vl53l0xDistance[servoPosition]=*me->rangeMeasurmentControl->getVl53l0xDistance(me->rangeMeasurmentControl);
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}
void Car_ControlPickALargerDistanceBetweenPos0AndPos12(carModule *me){
	uint32_t sumDistancePos0=me->autoDriveVariable.HcSr04Distance[0]+me->autoDriveVariable.Vl53l0xDistance[0];
	uint32_t sumDistancePos12=me->autoDriveVariable.HcSr04Distance[12]+me->autoDriveVariable.Vl53l0xDistance[12];
	if(sumDistancePos0>sumDistancePos12){
		if(me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,12)){
			me->motorControl->countersclockwiseRotation(me->motorControl);
		}
	}else{
		if(me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,0)){
			me->motorControl->clockwiseRotation(me->motorControl);
		}
	}
	Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);

}
void Car_ControlWaitForDistanceEqualThatPos6(carModule *me){
	uint8_t servoPosition=*me->rangeMeasurmentControl->getServoPRPosition(me->rangeMeasurmentControl);
	if(me->rangeMeasurmentControl->isRangeMeasurmentEnd(me->rangeMeasurmentControl)){
		uint32_t distance=*me->rangeMeasurmentControl->getVl53l0xDistance(me->rangeMeasurmentControl);
		uint32_t distanceLarger=me->autoDriveVariable.Vl53l0xDistance[6]*1.25;
		uint32_t distanceSmaller=me->autoDriveVariable.Vl53l0xDistance[6]*0.75;
		printf("pomiar=%d por=%d\n",distance,me->autoDriveVariable.Vl53l0xDistance[6]);
		if(distance>=distanceSmaller && distance<=distanceLarger){
			Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
			return;
		}
	}
	me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,servoPosition);
}
void Car_ControlSetTimeTo100ms(carModule *me){
	me->autoDriveVariable.time=100;
	Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
}
void Car_controlWaitForClearIrSensorNo1(carModule *me){
	uint32_t irSensorObstacle=me->irSensor->getValue(me->irSensor);
	if((irSensorObstacle&0b00000010)==0){
		me->motorControl->resetDriving(me->motorControl);
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}
void Car_ControlStopDrive(carModule *me){
	me->motorControl->resetDriving(me->motorControl);
	Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
}
void Car_ControlDriveForward(carModule *me){
	me->motorControl->drivingForward(me->motorControl);
	Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
}
void Car_ControlDriveBackward(carModule *me){
	me->motorControl->drivingBackwart(me->motorControl);
	Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
}
void Car_ControlDriveRight(carModule *me){
	me->motorControl->drivingRight(me->motorControl);
	Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
}
void Car_ControlDriveLeft(carModule *me){
	me->motorControl->drivingLeft(me->motorControl);
	Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
}
void Car_ControlDriveClockwiseRotation(carModule *me){
	me->motorControl->clockwiseRotation(me->motorControl);
	Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
}
void Car_ControlDriveCountersclockwiseRotation(carModule *me){
	me->motorControl->countersclockwiseRotation(me->motorControl);
	Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
}
void Car_ControlWaitUntilTheDistanceIsGreater(carModule *me){
	uint8_t servoPosition=*me->rangeMeasurmentControl->getServoPRPosition(me->rangeMeasurmentControl);
	if(me->rangeMeasurmentControl->isRangeMeasurmentEnd(me->rangeMeasurmentControl)){
		uint32_t distance=*me->rangeMeasurmentControl->getVl53l0xDistance(me->rangeMeasurmentControl);
		if(distance>100){
			Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
		}else{
			me->rangeMeasurmentControl->rangeMeasurment(me->rangeMeasurmentControl,servoPosition);
		}
	}
}
void Car_ControlWaitToStopDrive(carModule *me){
	if(me->autoDriveVariable.time==0){
		Car_NextPositionStateMachineControlCar(&me->autoDriveVariable);
	}
}

void Car_AddIrSensor(carModule *me,GPIO_TypeDef *gpio, uint16_t pin){
	me->irSensor->addGPIO(me->irSensor,gpio,pin);
}
void Car_NextPositionStateMachineControlCar(carAutoDrive *me){
	uint32_t next=me->positionStateMachineControlCar+1;
	uint32_t sizeOfStateMachine=sizeof(stateMachineControlCar)/sizeof(stateMachineControlCar[0]);
	if(next>=sizeOfStateMachine){
		addErrorValue(CAR_OutOfSizeStateMachine);
	}
	if(stateMachineControlCar[next]==controlIdle){
		next=0;
	}
	me->positionStateMachineControlCar=next;
	//me->controlStatus=stateMachineControlCar[next];
}
uint32_t Car_AutoDriveGetAutoPosition(carAutoDrive *me,uint8_t irSensorValue,functionFromPcEnum driveStatus){
	uint32_t returnNumber=0;
	uint8_t irSensorAfterMasking=0;
	switch(driveStatus){
		case RIDE_FORWARD_FUN:
			irSensorAfterMasking=irSensorValue&0b00000111;
			switch(irSensorAfterMasking){
				case 0b00000001:
					returnNumber=4;
					break;
				case 0b00000010:
					returnNumber=14;
					break;
				case 0b00000011:
					returnNumber=4;
					break;
				case 0b00000100:
					returnNumber=30;
					break;
				case 0b00000101:
					returnNumber=4;
					break;
				case 0b00000110:
					returnNumber=30;
					break;
				case 0b00000111:
					returnNumber=4;
					break;
			}
			break;
		case RIDE_BACKWARD_FUN:
			irSensorAfterMasking=irSensorValue&0b01100000;
			switch(irSensorAfterMasking){
				case 0b00100000:
					break;
					returnNumber=40;
				case 0b01000000:
					returnNumber=48;
					break;
				case 0b01100000:
					returnNumber=56;
					break;
			}
			break;
		case RIDE_RIGHT_FUN:
			irSensorAfterMasking=irSensorValue&0b00000111;
			break;
		case RIDE_LEFT_FUN:
			break;
		case ROTATE_LEFT:
			break;
		case ROTATE_RIGHT:
			break;
		case RIDE_BACKWARD_RIGHT:
			break;
		case RIDE_BACKWARD_LEFT:
			break;
		default:
			break;
	}
	return returnNumber;
}

						/*FUNKCJE OBSLUGI BLEDOW ORAZ PRZERWAN*/
int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }
  CarTestModule->outMessage->Insert(CarTestModule->outMessage,ch);
  return 1;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if(htim==CarTestModule->timer1ms){
		CarTestModule->timeToResetMotorControl++;
		CarTestModule->timerMeasureBatteryVoltage++;
		CarTestModule->timerToSendIrSensorStatus++;
		CarTestModule->inMessage->AddTimeout(CarTestModule->inMessage);
		CarTestModule->batteryVoltage->addTime(CarTestModule->batteryVoltage);
		CarTestModule->rangeMeasurmentControl->timeInterrupt(CarTestModule->rangeMeasurmentControl);
		if(CarTestModule->autoDriveVariable.time>0){
			CarTestModule->autoDriveVariable.time--;
		}
	}
	CarTestModule->rangeMeasurmentControl->HcSr04Interrupt(CarTestModule->rangeMeasurmentControl,htim);

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
	if(UartHandle==CarTestModule->outMessage->huart){
		CarTestModule->inMessage->Insert(CarTestModule->inMessage,CarTestModule->ReceivedInMessageBuff);
		HAL_UART_Receive_IT(CarTestModule->outMessage->huart,&CarTestModule->ReceivedInMessageBuff,1);
	}
}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c){

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc==CarTestModule->batteryVoltage->hadc){
		uint32_t value=HAL_ADC_GetValue(hadc);
		CarTestModule->batteryVoltage->addMeasure(CarTestModule->batteryVoltage,value);
	}
}

void addErrorValue(uint8_t value ){
	CarTestModule->outMessage->InsertError(CarTestModule->outMessage,value);
}
