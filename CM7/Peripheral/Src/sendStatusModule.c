/*
 * sendStatusModule.c
 *
 *  Created on: 20 paź 2021
 *      Author: Daniel
 */
#include "stm32h7xx_hal.h"
#include "adc.h"
#include "errorCode.h"
#include "hc-sr04.h"
#include "irSensor.h"
#include "stateMachine.h"
#include "uartCom.h"
#include "vl5310x.h"
#include "sendStatusModule.h"
#include "connectionModule.h"

