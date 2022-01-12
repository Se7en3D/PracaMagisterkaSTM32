/*
 * connectionModule.h
 *
 *  Created on: 25 wrz 2021
 *      Author: Daniel
 */


/**
  * @brief Stała  określająca wielkość pierwszego wymiaru tablicy tabPositionByDriving
  */
#define drivingStatusSizeFirstDimension 10
/**
  * @brief Stała  określająca wielkość drugiego wymiaru tablicy tabPositionByDriving
  */
#define drivingStatusSizeSecondDimension 5
/**
  * @brief Stała czasowa określająca po jakim czasie nalezy wysłąć pomiary odległości
  * 		jeśli któryś z czujników nie bedzie odpowiadał[ms]
  */
#define TIMEOUT_TO_SEND 1000
#define measurmentStatusFunctionArreySize 3 //?? co z tym zrobić ??
#define connectionModuleLockDistance 1
#define connectionModuleUnlockDistance 0

#include "servoPR.h"

typedef enum{
	connectionModuleMeasurment_Idle=0,
	connectionModuleMeasurment_Wait,
	connectionModuleMeasurment_SendDistance,
}connectionModuleMeasurmentStatus;

typedef struct{
	float distanceHcSr04;
	uint16_t distanceVl5310x;
	uint8_t isDistanceHcSr04Lock;
	uint8_t isDistanceVl5310xLock;
	int timeout;
	connectionModuleMeasurmentStatus measurmentStatus;

}measurmentStructure_t;

measurmentStructure_t measurmentStructure;
typedef struct{
	uint8_t numberForTabPositionByDriving;
	int adder;
}connectionBetweenServo360AndStateMachine_t;

connectionBetweenServo360AndStateMachine_t  connectionBetweenServo360AndStateMachine;
void connectionModuleDecodeMessage(connectionBetweenServo360AndStateMachine_t *servoAndStateMachine ,driving_structure_t *drivingStructure,servoPR_GeneralStructure * servoPR);
void connectionModuleDrivingStatusWithPositionServo(connectionBetweenServo360AndStateMachine_t *servoAndStateMachine ,driving_structure_t *drivingStructure,servoPR_GeneralStructure * servoPR);
void connectionModuleaddTimeout(measurmentStructure_t *measureS);
void connectionModuleMeasureDistance(servoPR_GeneralStructure * servoPR,measurmentStructure_t *measureS);
void connectrionModuleFunctionMeasurmentIdle(servoPR_GeneralStructure * servoPR,measurmentStructure_t *measureS);
void connectionModuleFunctionMeasurmentWait(servoPR_GeneralStructure * servoPR,measurmentStructure_t *measureS);
void connectionModuleFunctionMeasurmentSend(servoPR_GeneralStructure * servoPR,measurmentStructure_t *measureS);
void connectionModuleClearMeasurmentStatus(measurmentStructure_t *measurmentStructure);
uint8_t connectionModuleCheckCollisionByIrSensor(const uint8_t function);
