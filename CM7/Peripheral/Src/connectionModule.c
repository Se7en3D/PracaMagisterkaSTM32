/*
 * connectionModule.c
 *
 *  Created on: 25 wrz 2021
 *      Author: Daniel
 */
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include "uartCom.h"
#include "servo360.h"
#include "stateMachine.h"
#include "connectionModule.h"


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
			servo360SetTargetPosition(positionFromtable);
			printf("P=%d\n",positionFromtable);
			connectionBetweenServo360AndStateMachine.numberForTabPositionByDriving++;
		}
		connectionBetweenServo360AndStateMachine.prevDrivingStatus=drivingStatus;
	}
}
