/*
 * connectionModule.h
 *
 *  Created on: 25 wrz 2021
 *      Author: Daniel
 */

#ifndef INC_CONNECTIONMODULE_H_
#define INC_CONNECTIONMODULE_H_

#define drivingStatusSize 10
#define TIMEOUT_TO_SEND 100 //ilość czasu po której należy wysłać pomiar (jednostka ms)


static volatile servo360_Position tabPositionByDriving[drivingStatusSize][5]={
		{servo360_PositionNone}, //IDLE_DRIVING
		{servo360_Position8,servo360_Position9,servo360_Position10,servo360_PositionNone}, //GO_UP_LEFT
		{servo360_Position2,servo360_Position3,servo360_Position4,servo360_PositionNone}, //GO_UP_RIGHT
		{servo360_Position5,servo360_Position6,servo360_Position7,servo360_PositionNone}, //GO_UP
		{servo360_Position6,servo360_PositionNone},//GO_BACK
		{servo360_Position10,servo360_Position11,servo360_Position12,servo360_PositionNone}, //ROTATE_LEFT_DRIV
		{servo360_Position0,servo360_Position1,servo360_Position2,servo360_PositionNone}, //ROTATE_RIGHT_DRIV
		{servo360_Position6,servo360_PositionNone}, //STOP_DRIVING
		{servo360_Position6,servo360_PositionNone}, //GO_BACK_LEFT
		{servo360_Position6,servo360_PositionNone}, //GO_BACK_RIGHT
};

typedef struct{
	float distanceHcSr04;
	uint16_t distanceVl5310x;
	int timeout;
}measurmentStructure_t;

measurmentStructure_t measurmentStructure;
typedef struct{
	driving_status_t prevDrivingStatus;
	int numberForTabPositionByDriving;
}connectionBetweenServo360AndStateMachine_t;

connectionBetweenServo360AndStateMachine_t  connectionBetweenServo360AndStateMachine;
void connectionModuleStateMachineWithServo360(driving_status_t drivingStatus);
void connectionModuleaddTimeout(measurmentStructure_t *measureS);
void connectionModuleMeasureDistance(servo360_Base_Structure * servo360S,measurmentStructure_t *measureS);
#endif /* INC_CONNECTIONMODULE_H_ */
