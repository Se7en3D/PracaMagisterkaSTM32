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
		&connectionModuleFunctionMeasurmentWait
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
	if(measureS->distanceHcSr04==0.0){
		measureS->timeout++;
	}else{
		measureS->timeout=0;
	}

}
void connectionModuleMeasureDistance(servo360_Base_Structure * servo360S,measurmentStructure_t *measureS){
	if(servo360S->status==servo360_WAIT_TO_MEASURMENT){
		measurmentStatusFunction[measureS->measurmentStatus](servo360S,measureS);
	}
}

void connectrionModuleFunctionMeasurmentIdle(servo360_Base_Structure * servo360S,measurmentStructure_t *measureS){
	if(measureS->measurmentStatus==connectionModuleMeasurmentIDLE){
		hcsr04ClearMeasurement();
		vl53l0xReadRangeContinuousMillimeters();
		measureS->timeout=0;
		measureS->measurmentStatus=connectionModuleMeasurmentWAIT;
	}
}

void connectionModuleFunctionMeasurmentWait(servo360_Base_Structure * servo360S,measurmentStructure_t *measureS){
	if(measureS->timeout>=TIMEOUT_TO_SEND){
		errorCodePush(CONNECTIONMODULE_TIMEOUT_DISTANCE_MEASUREMENT);
		measureS->distanceVl5310x=vl53l0xReadRangeContinuousMillimeters();
		uartComSensorFrame.position=servo360Structure.currentPosition;
		uartComSendDistanceServo(measureS->distanceHcSr04, measureS->distanceVl5310x);
		measureS->timeout=0;
		measureS->distanceVl5310x=0;
		measureS->distanceHcSr04=0;
		servo36GoToIdleFromMeasurment();
		measureS->measurmentStatus=connectionModuleMeasurmentIDLE;
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
			  measureS->measurmentStatus=connectionModuleMeasurmentIDLE;
		  }
	}
}

