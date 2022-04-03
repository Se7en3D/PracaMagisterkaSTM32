/*
 * ErrorValue.h
 *
 *  Created on: 27 sty 2022
 *      Author: Daniel
 */
#define CommandDecoder_OutOfBuffer 1
#define CommandDecoder_WaitToHandle 2
#define CommandDecoder_Timeout_Decod 3
#define VL5310X_IncorrectModelID 4
#define VL5310X_IncorrectBudgetUs 5
#define VL5310X_IncorrectPeriodVcSelPeriodPrerange 6
#define VL5310X_IncorrectPeriodVcSelPeriodFinalrange 7
#define VL5310X_IncorrectTypeVcselperiodType 8
#define VL53L0X_NullXShulPointer 9
#define CAR_OutOdSizeFunctionSevices 10
#define HCSR04_StatusNoIdle 11
#define RANGEMEASURMENT_TimeoutPositionChangingFunction 12
#define RANGEMEASURMENT_TimeoutMeasurmentDistanceFunction 13
#define RANGEMEASURMENT_NoSetLockHcSr04 14
#define RANGEMEASURMENT_NoSetLockVl53l0x 15
#define SERVOPR_OutOfMaxServoPosition 16
#define VL5310X_ErrorGetSpadInfo 17
#define VL5310x_ErrorPerformSingleRefCalibration 18
#define VL5310x_TimeoutGetSpadInfo 19
#define VL5310X_OutOfTypeVcSelPeriodType 20
#define VL5310X_ErrorEndCodeTimeout 21
#define VL5310X_StatusNoIdle 22
#define CAR_OutOfSizeStateMachine 23
#define CAR_TimeoutResetMotorControl 24
#define CAR_MeasureDistanceForPcCountIsNoZero 25
#define CAR_SizeStateMachineIsZero 26




extern void addErrorValue(uint8_t value);



