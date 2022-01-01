/*
 * connectionModule.c
 *
 *  Created on: 25 wrz 2021
 *      Author: Daniel
 */
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include "servoPR.h"
#include "uartCom.h"
#include "errorCode.h"
#include "stateMachine.h"
#include "connectionModule.h"
#include "hc-sr04.h"
#include "vl5310x.h"

static const volatile servoPR_Position connectionMovementAngleArray[drivingStatusSizeFirstDimension][drivingStatusSizeSecondDimension]={
		{servoPRPosition5,'\0'},	//IDLE_DRIVING
		{servoPRPosition10,servoPRPosition9,servoPRPosition8,servoPRPosition7,'\0'}, //GO_UP_LEFT
		{servoPRPosition1,servoPRPosition2,servoPRPosition3,servoPRPosition4,'\0'},	//GO_UP_RIGHT
		{servoPRPosition4,servoPRPosition5,servoPRPosition6,'\0'},	//GO_UP
		{servoPRPosition5,'\0'},	//GO_BACK
		{servoPRPosition11,servoPRPosition10,servoPRPosition9,'\0'},	//ROTATE_LEFT_DRIV
		{servoPRPosition0,servoPRPosition1,servoPRPosition2,'\0'}, //ROTATE_RIGHT_DRIV
		{servoPRPosition5,'\0'}, //STOP_DRIVING
		{servoPRPosition5,'\0'}, //GO_BACK_LEFT
		{servoPRPosition5,'\0'}, //GO_BACK_RIGHT
};

static void (*measurmentStatusFunction[measurmentStatusFunctionArreySize])(servoPR_GeneralStructure*,measurmentStructure_t*)={
		&connectrionModuleFunctionMeasurmentIdle,
		&connectionModuleFunctionMeasurmentWait,
		&connectionModuleFunctionMeasurmentSend,
};

void connectionModuleDrivingStatusWithPositionServo(connectionBetweenServo360AndStateMachine_t *servoAndStateMachine,driving_structure_t *drivingStructure,servoPR_GeneralStructure * servoPR){
	if(servoPRReadyToChangeAngle(servoPR)){
		uint8_t positionFirstDimension=stateMachineGetDrivingStatus(drivingStructure);
		uint8_t positionSecondDimension=servoAndStateMachine->numberForTabPositionByDriving;
		if(stateMachineDrabingsStatusIsEqual(drivingStructure)){


			if(positionSecondDimension==0){
				servoAndStateMachine->adder=1;
			}

			positionSecondDimension+=servoAndStateMachine->adder;

				//Ostatni element tablicy
			if(connectionMovementAngleArray[positionFirstDimension][positionSecondDimension]=='\0'){
				servoAndStateMachine->adder=-1;
				positionSecondDimension+=servoAndStateMachine->adder;
			}

			if(positionSecondDimension>=drivingStatusSizeSecondDimension){
				positionSecondDimension=0;
			}

		}else{
			positionSecondDimension=0;
			connectionModuleClearMeasurmentStatus(&measurmentStructure);

		}
		servoPRSetTargetAngle(servoPR, connectionMovementAngleArray[positionFirstDimension][positionSecondDimension]);
		servoAndStateMachine->numberForTabPositionByDriving=positionSecondDimension;

	}
	/*
	if(servo360Structure.status==servo360_IDLE){ //Tylko gdy status serva jest IDLE
		if(drivingStatus!=connectionBetweenServo360AndStateMachine.prevDrivingStatus){ //Sprawdzenie czy wcześniejszy status jest identyczny
			connectionBetweenServo360AndStateMachine.numberForTabPositionByDriving=0;
		}

		servo360_Position positionFromtable=tabPositionByDriving[drivingStatus][connectionBetweenServo360AndStateMachine.numberForTabPositionByDriving];
		if(positionFromtable==servo360_PositionNone){
			connectionBetweenServo360AndStateMachine.numberForTabPositionByDriving=0;
		}else{
			//printf("P=%d\n",positionFromtable);
			servo360SetTargetPosition(positionFromtable);
			connectionBetweenServo360AndStateMachine.numberForTabPositionByDriving++;
		}
		connectionBetweenServo360AndStateMachine.prevDrivingStatus=drivingStatus;
	}*/
}

void connectionModuleaddTimeout(measurmentStructure_t *measureS){
	if(measureS->measurmentStatus==connectionModuleMeasurment_Wait){
		measureS->timeout++;
	}else{
		measureS->timeout=0;
	}

}
void connectionModuleMeasureDistance(servoPR_GeneralStructure * servoPR,measurmentStructure_t *measureS){
		measurmentStatusFunction[measureS->measurmentStatus](servoPR,measureS);
}

void connectrionModuleFunctionMeasurmentIdle(servoPR_GeneralStructure * servoPR,measurmentStructure_t *measureS){
	if(servoPRGetBlockedStatus(servoPR)){
		hcsr04ClearMeasurement();
		vl53l0xClearInterruptFlag();
		measureS->timeout=0;
		measureS->measurmentStatus=connectionModuleMeasurment_Wait;
	}
}

void connectionModuleFunctionMeasurmentWait(servoPR_GeneralStructure * servoPR,measurmentStructure_t *measureS){
	if(measureS->timeout>=TIMEOUT_TO_SEND){
		errorCodePush(CONNECTIONMODULE_TIMEOUT_DISTANCE_MEASUREMENT);
		measureS->measurmentStatus=connectionModuleMeasurment_SendDistance;
	}else{
		uint8_t lockStatusFromHcSr04=(measureS->isDistanceHcSr04Lock==connectionModuleLockDistance);
		uint8_t lockStatusFromVl5310x=(measureS->isDistanceVl5310xLock==connectionModuleLockDistance);

		if(hcsr04IsReadyToSend() && !lockStatusFromHcSr04){
			measureS->distanceHcSr04=hcsr04GetCelculatedValue();
			measureS->isDistanceHcSr04Lock=connectionModuleLockDistance;
			lockStatusFromHcSr04=(measureS->isDistanceHcSr04Lock==connectionModuleLockDistance);
		}
		if(!vl53l0xIsReadyToSend()){
			vl53l0xReadRangeContinuousMillimeters();
		}
		if(vl53l0xIsReadyToSend() && !lockStatusFromVl5310x){
			measureS->distanceVl5310x=vl53l0xReadRangeContinuousMillimeters();
			measureS->isDistanceVl5310xLock=connectionModuleLockDistance;
			lockStatusFromVl5310x=(measureS->isDistanceVl5310xLock==connectionModuleLockDistance);
		}


		if(lockStatusFromHcSr04 && lockStatusFromVl5310x){
			measureS->measurmentStatus=connectionModuleMeasurment_SendDistance;
		}
	}
}

void connectionModuleFunctionMeasurmentSend(servoPR_GeneralStructure * servoPR,measurmentStructure_t *measureS){
	uint16_t distanceVl5310x=0;
	float distanceHcSr04=0;
	uint8_t lockStatusFromHcSr04=(measureS->isDistanceHcSr04Lock==connectionModuleLockDistance);
	uint8_t lockStatusFromVl5310x=(measureS->isDistanceVl5310xLock==connectionModuleLockDistance);
	uint16_t position=servoPR->currentAngle/15;
	if(lockStatusFromHcSr04){
		distanceHcSr04=measureS->distanceHcSr04;
		measureS->isDistanceHcSr04Lock=connectionModuleUnlockDistance;
	}

	if(lockStatusFromVl5310x){
		distanceVl5310x=measureS->distanceVl5310x;
		measureS->isDistanceVl5310xLock=connectionModuleUnlockDistance;
	}
	servoPRClearBlockedStatus(servoPR);
	uartComSendDistanceServo(distanceHcSr04, distanceVl5310x,position);
	measureS->measurmentStatus=connectionModuleMeasurment_Idle;
}

void connectionModuleClearMeasurmentStatus(measurmentStructure_t *measurmentStructure){
	measurmentStructure->distanceHcSr04=0;
	measurmentStructure->distanceVl5310x=0;
	measurmentStructure->isDistanceHcSr04Lock=connectionModuleUnlockDistance;
	measurmentStructure->isDistanceVl5310xLock=connectionModuleUnlockDistance;
	measurmentStructure->timeout=0;
	measurmentStructure->measurmentStatus=connectionModuleMeasurment_Idle;
}

