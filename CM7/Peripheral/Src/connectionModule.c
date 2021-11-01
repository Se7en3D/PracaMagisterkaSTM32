/*
 * connectionModule.c
 *
 *  Created on: 25 wrz 2021
 *      Author: Daniel
 */
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include "uartCom.h"
#include "errorCode.h"
#include "servo360.h"
#include "stateMachine.h"
#include "connectionModule.h"
#include "hc-sr04.h"
#include "vl5310x.h"

static void (*measurmentStatusFunction[measurmentStatusFunctionArreySize])(servo360_Base_Structure*,measurmentStructure_t*)={
		&connectrionModuleFunctionMeasurmentIdle,
		&connectionModuleFunctionMeasurmentWait,
		&connectionModuleFunctionMeasurmentSend,
};

void connectionModuleStateMachineWithServo360(driving_status_t drivingStatus){
	if(drivingStatus>=drivingStatusSize){ //Wyjście poza zakres tablicy dodać error
		return;
	}
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
	}
}

void connectionModuleaddTimeout(measurmentStructure_t *measureS){
	if(measureS->measurmentStatus==connectionModuleMeasurment_Wait){
		measureS->timeout++;
	}else{
		measureS->timeout=0;
	}

}
void connectionModuleMeasureDistance(servo360_Base_Structure * servo360S,measurmentStructure_t *measureS){
		measurmentStatusFunction[measureS->measurmentStatus](servo360S,measureS);
}

void connectrionModuleFunctionMeasurmentIdle(servo360_Base_Structure * servo360S,measurmentStructure_t *measureS){
	servo360_state servo360Status=servo360S->status;
	if(servo360Status==servo360_WAIT_TO_MEASURMENT){
		hcsr04ClearMeasurement();
		vl53l0xClearInterruptFlag();
		measureS->timeout=0;
		measureS->measurmentStatus=connectionModuleMeasurment_Wait;
	}
}

void connectionModuleFunctionMeasurmentWait(servo360_Base_Structure * servo360S,measurmentStructure_t *measureS){
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
		vl53l0xReadRangeContinuousMillimeters();
		if(vl53l0xIsReadyToSend() && !lockStatusFromVl5310x){
			measureS->distanceVl5310x=vl53l0xReadRangeContinuousMillimeters();
			measureS->isDistanceVl5310xLock=connectionModuleLockDistance;
			lockStatusFromVl5310x=(measureS->isDistanceVl5310xLock==connectionModuleLockDistance);
		}


		if(lockStatusFromHcSr04 && lockStatusFromVl5310x){
			measureS->measurmentStatus=connectionModuleMeasurment_SendDistance;
		}
	}

	/*	if(measureS->timeout>=TIMEOUT_TO_SEND){
		errorCodePush(CONNECTIONMODULE_TIMEOUT_DISTANCE_MEASUREMENT);
		measureS->distanceVl5310x=vl53l0xReadRangeContinuousMillimeters();
		uartComSensorFrame.position=servo360Structure.currentPosition;
		uartComSendDistanceServo(measureS->distanceHcSr04, measureS->distanceVl5310x);
		measureS->timeout=0;
		measureS->distanceVl5310x=0;
		measureS->distanceHcSr04=0;
		servo36GoToIdleFromMeasurment();
		measureS->measurmentStatus=connectionModuleMeasurment_Idle;
	}else{
		measureS->distanceHcSr04=hcsr04GetCelculatedValue();
		  if(measureS->distanceHcSr04!=0.0){
			  measureS->distanceVl5310x=vl53l0xReadRangeContinuousMillimeters();
			  uartComSensorFrame.position=servo360Structure.currentPosition;
			  uartComSendDistanceServo(measureS->distanceHcSr04, measureS->distanceVl5310x);
			  measureS->timeout=0;
			  measureS->distanceVl5310x=0;
			  measureS->distanceHcSr04=0;
			  servo36GoToIdleFromMeasurment();
			  measureS->measurmentStatus=connectionModuleMeasurment_Idle;
		  }
	}*/
}

void connectionModuleFunctionMeasurmentSend(servo360_Base_Structure * servo360S,measurmentStructure_t *measureS){
	uint16_t distanceVl5310x=0;
	float distanceHcSr04=0;
	uint8_t lockStatusFromHcSr04=(measureS->isDistanceHcSr04Lock==connectionModuleLockDistance);
	uint8_t lockStatusFromVl5310x=(measureS->distanceVl5310x==connectionModuleLockDistance);
	uint16_t position=servo360S->currentPosition;
	if(lockStatusFromHcSr04){
		distanceHcSr04=measureS->distanceHcSr04;
		measureS->isDistanceHcSr04Lock=connectionModuleUnlockDistance;
	}

	if(lockStatusFromVl5310x){
		distanceVl5310x=measureS->distanceVl5310x;
		measureS->isDistanceVl5310xLock=connectionModuleUnlockDistance;
	}

	servo36GoToIdleFromMeasurment();
	uartComSendDistanceServo(distanceHcSr04, distanceVl5310x,position);
	measureS->measurmentStatus=connectionModuleMeasurment_Idle;
}

