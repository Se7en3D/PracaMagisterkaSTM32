/*
 * vl53l0x.c
 *
 *  Created on: 7 lut 2022
 *      Author: Daniel
 */
#include <stdio.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "Vl53l0x.h"

static uint16_t* (*statusFunction[])(StructVl53l0x* me)={
		vl53l0x_FunctionNoInitialized,
		vl53l0x_FunctionInitializedError,
		vl53l0x_FunctionIdle,
		vl53l0x_FunctionMeasurmentPreparation,
		vl53l0x_FunctionWaitUntilStartBitHasBeenCleared,
		vl53l0x_FunctionWaitingForTheInterruptFlagToBeSet,
		vl53l0x_FunctionTimeout
};

StructVl53l0x* vl53l0x_Create(I2C_HandleTypeDef * hi2c,GPIO_TypeDef *xshutgpio,uint16_t xshutpin){
	StructVl53l0x* me=malloc(sizeof(StructVl53l0x));
	if(me!=NULL){
		me->instance=malloc(sizeof(StructRegVl53l0x));
		me->instance->address=0x52;
		me->instance->last_status=0;
		me->instance->tcc=0;
		me->instance->msrc=0;
		me->instance->dss=0;
		me->instance->pre_range=0;
		me->instance->final_range=0;
		me->instance->pre_range_vcsel_period_pclks=0;
		me->instance->final_range_vcsel_period_pclks=0;
		me->instance->msrc_dss_tcc_mclks=0;
		me->instance->pre_range_mclks=0;
		me->instance->final_range_mclks=0;
		me->instance->msrc_dss_tcc_us=0;
		me->instance->pre_range_us=0;
		me->instance->final_range_us=0;
		me->instance->io_timeout=0;
		me->instance->did_timeout=0;
		me->instance->timeout_start_ms=0;
		me->instance->stop_variable=0;
		me->instance->measurement_timing_budget_us=0;
		me->instance->continuousMode=0;

		me->status=vl53l0x_NoInitialized;
		me->hi2c=hi2c;
		me->xshutPin=xshutpin;
		me->xshutPort=xshutgpio;
		me->gpio1Pin=0;
		me->gpio1Port=NULL;
		me->distance=0;
		me->time=0;

		vl53l0x_StructInit(me,
				vl53l0x_ReadDistance,
				vl53l0x_SetSignalRateLimit,
				vl53l0x_GetSignalRateLimit,
				vl53l0x_SetMeasurementTimingBudget,
				vl53l0x_GetMeasurementTimingBudget,
				vl53l0x_SetVcselPulsePeriod,
				vl53l0x_GetVcselPulsePeriod,
				vl53l0x_StartContinuous,
				vl53l0x_StopContinuous,
				vl53l0x_IncreaseTime,
				vl53l0x_StartSingleMeasurment,
				vl53l0x_IsReady);

		if(vl53l0x_SensorInit(me)){
			vl53l0x_GetVcselPulsePeriod(me,VcselPeriodPreRange);
		}else{
			me->status=vl53l0x_InitializedError;
		}

		return me;
	}
	return NULL;
}
void vl53l0x_StructInit(StructVl53l0x* me,
						uint16_t* (*getDistance)(StructVl53l0x *me),
						uint8_t (*setSignalRateLimit)(StructVl53l0x* me,float limit_Mcps),
						float (*getSignalRateLimit)(StructVl53l0x* me),
						uint8_t (*setMeasurementTimingBudget)(StructVl53l0x* me,uint32_t budget_us),
						uint32_t (*getMeasurementTimingBudget)(StructVl53l0x* me),
						uint8_t (*setVcselPulsePeriod)(StructVl53l0x* me,vcselPeriodType type, uint8_t period_pclks),
						uint8_t (*getVcselPulsePeriod)(StructVl53l0x* me,vcselPeriodType type),
						void (*startContinuous)(StructVl53l0x* me,uint32_t period_ms),
						void (*stopContinuous)(StructVl53l0x* me),
						void (*increaseTime)(StructVl53l0x* me),
						void (*startSingleMeasurment)(StructVl53l0x* me),
						uint8_t (*isReady)(StructVl53l0x* me)){
	me->getDistance=getDistance;
	me->setSignalRateLimit=setSignalRateLimit;
	me->getSignalRateLimit=getSignalRateLimit;
	me->setMeasurementTimingBudget=setMeasurementTimingBudget;
	me->getMeasurementTimingBudget=getMeasurementTimingBudget;
	me->setVcselPulsePeriod=setVcselPulsePeriod;
	me->getVcselPulsePeriod=getVcselPulsePeriod;
	me->startContinous=startContinuous;
	me->stopContinuous=stopContinuous;
	me->increaseTime=increaseTime;
	me->startSingleMeasurment=startSingleMeasurment;
	me->isReady=isReady;
}
uint8_t vl53l0x_SensorInit(StructVl53l0x *me){
		uint8_t tempValue=0;
	 // check model ID register (value specified in datasheet)
		uint8_t Id=vl53l0x_ReadReg(me,IDENTIFICATION_MODEL_ID);
		  if (Id != 0xEE) {
			  addErrorValue(VL5310X_IncorrectModelID);
			  return 0;
		  }
		  if(me->xshutPort!=NULL){
			  HAL_GPIO_WritePin(me->xshutPort, me->xshutPin,GPIO_PIN_RESET);
			  HAL_Delay(20);
			  HAL_GPIO_WritePin(me->xshutPort, me->xshutPin,GPIO_PIN_SET);
			  HAL_Delay(5);
		  }else{
			  addErrorValue(VL5310X_IncorrectTypeVcselperiodType);
		  }

		  // VL53L0X_DataInit() begin

		  // "Set I2C standard mode"
		  vl53l0x_WriteReg(me,0x88, 0x00);

		  vl53l0x_WriteReg(me,0x80, 0x01);
		  vl53l0x_WriteReg(me,0xFF, 0x01);
		  vl53l0x_WriteReg(me,0x00, 0x00);
		  me->instance->stop_variable = vl53l0x_ReadReg(me,0x91);
		  vl53l0x_WriteReg(me,0x00, 0x01);
		  vl53l0x_WriteReg(me,0xFF, 0x00);
		  vl53l0x_WriteReg(me,0x80, 0x00);

		  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
		  vl53l0x_WriteReg(me,MSRC_CONFIG_CONTROL, vl53l0x_ReadReg(me,MSRC_CONFIG_CONTROL) | 0x12);

		  // set final range signal rate limit to 0.25 MCPS (million counts per second)
		  vl53l0x_SetSignalRateLimit(me,0.25);

		  vl53l0x_WriteReg(me,SYSTEM_SEQUENCE_CONFIG, 0xFF);

		  // VL53L0X_DataInit() end

		  // VL53L0X_StaticInit() begin

		  uint8_t spad_count;
		  uint8_t spad_type_is_aperture;
		  if (!vl53l0x_GetSpadInfo(me,&spad_count, &spad_type_is_aperture)) {
			  addErrorValue(VL5310X_ErrorGetSpadInfo);
			  return 0;
		  }

		  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
		  // the API, but the same data seems to be more easily readable from
		  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
		  uint8_t ref_spad_map[6];
		  vl53l0x_ReadMulti(me,GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

		  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

		  vl53l0x_WriteReg(me,0xFF, 0x01);
		  vl53l0x_WriteReg(me,DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
		  vl53l0x_WriteReg(me,DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
		  vl53l0x_WriteReg(me,0xFF, 0x00);
		  vl53l0x_WriteReg(me,GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

		  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
		  uint8_t spads_enabled = 0;

		  for (uint8_t i = 0; i < 48; i++){
				if (i < first_spad_to_enable || spads_enabled == spad_count){
				  // This bit is lower than the first one that should be enabled, or
				  // (reference_spad_count) bits have already been enabled, so zero this bit
				  ref_spad_map[i / 8] &= ~(1 << (i % 8));
				}
				else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
				{
				  spads_enabled++;
				}
		  }

		  vl53l0x_WriteMulti(me,GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

		  // -- VL53L0X_set_reference_spads() end

		  // -- VL53L0X_load_tuning_settings() begin
		  // DefaultTuningSettings from vl53l0x_tuning.h

		  vl53l0x_WriteReg(me,0xFF, 0x01);
		  vl53l0x_WriteReg(me,0x00, 0x00);

		  vl53l0x_WriteReg(me,0xFF, 0x00);
		  vl53l0x_WriteReg(me,0x09, 0x00);
		  vl53l0x_WriteReg(me,0x10, 0x00);
		  vl53l0x_WriteReg(me,0x11, 0x00);

		  vl53l0x_WriteReg(me,0x24, 0x01);
		  vl53l0x_WriteReg(me,0x25, 0xFF);
		  vl53l0x_WriteReg(me,0x75, 0x00);

		  vl53l0x_WriteReg(me,0xFF, 0x01);
		  vl53l0x_WriteReg(me,0x4E, 0x2C);
		  vl53l0x_WriteReg(me,0x48, 0x00);
		  vl53l0x_WriteReg(me,0x30, 0x20);

		  vl53l0x_WriteReg(me,0xFF, 0x00);
		  vl53l0x_WriteReg(me,0x30, 0x09);
		  vl53l0x_WriteReg(me,0x54, 0x00);
		  vl53l0x_WriteReg(me,0x31, 0x04);
		  vl53l0x_WriteReg(me,0x32, 0x03);
		  vl53l0x_WriteReg(me,0x40, 0x83);
		  vl53l0x_WriteReg(me,0x46, 0x25);
		  vl53l0x_WriteReg(me,0x60, 0x00);
		  vl53l0x_WriteReg(me,0x27, 0x00);
		  vl53l0x_WriteReg(me,0x50, 0x06);
		  vl53l0x_WriteReg(me,0x51, 0x00);
		  vl53l0x_WriteReg(me,0x52, 0x96);
		  vl53l0x_WriteReg(me,0x56, 0x08);
		  vl53l0x_WriteReg(me,0x57, 0x30);
		  vl53l0x_WriteReg(me,0x61, 0x00);
		  vl53l0x_WriteReg(me,0x62, 0x00);
		  vl53l0x_WriteReg(me,0x64, 0x00);
		  vl53l0x_WriteReg(me,0x65, 0x00);
		  vl53l0x_WriteReg(me,0x66, 0xA0);

		  vl53l0x_WriteReg(me,0xFF, 0x01);
		  vl53l0x_WriteReg(me,0x22, 0x32);
		  vl53l0x_WriteReg(me,0x47, 0x14);
		  vl53l0x_WriteReg(me,0x49, 0xFF);
		  vl53l0x_WriteReg(me,0x4A, 0x00);

		  vl53l0x_WriteReg(me,0xFF, 0x00);
		  vl53l0x_WriteReg(me,0x7A, 0x0A);
		  vl53l0x_WriteReg(me,0x7B, 0x00);
		  vl53l0x_WriteReg(me,0x78, 0x21);

		  vl53l0x_WriteReg(me,0xFF, 0x01);
		  vl53l0x_WriteReg(me,0x23, 0x34);
		  vl53l0x_WriteReg(me,0x42, 0x00);
		  vl53l0x_WriteReg(me,0x44, 0xFF);
		  vl53l0x_WriteReg(me,0x45, 0x26);
		  vl53l0x_WriteReg(me,0x46, 0x05);
		  vl53l0x_WriteReg(me,0x40, 0x40);
		  vl53l0x_WriteReg(me,0x0E, 0x06);
		  vl53l0x_WriteReg(me,0x20, 0x1A);
		  vl53l0x_WriteReg(me,0x43, 0x40);

		  vl53l0x_WriteReg(me,0xFF, 0x00);
		  vl53l0x_WriteReg(me,0x34, 0x03);
		  vl53l0x_WriteReg(me,0x35, 0x44);

		  vl53l0x_WriteReg(me,0xFF, 0x01);
		  vl53l0x_WriteReg(me,0x31, 0x04);
		  vl53l0x_WriteReg(me,0x4B, 0x09);
		  vl53l0x_WriteReg(me,0x4C, 0x05);
		  vl53l0x_WriteReg(me,0x4D, 0x04);

		  vl53l0x_WriteReg(me,0xFF, 0x00);
		  vl53l0x_WriteReg(me,0x44, 0x00);
		  vl53l0x_WriteReg(me,0x45, 0x20);
		  vl53l0x_WriteReg(me,0x47, 0x08);
		  vl53l0x_WriteReg(me,0x48, 0x28);
		  vl53l0x_WriteReg(me,0x67, 0x00);
		  vl53l0x_WriteReg(me,0x70, 0x04);
		  vl53l0x_WriteReg(me,0x71, 0x01);
		  vl53l0x_WriteReg(me,0x72, 0xFE);
		  vl53l0x_WriteReg(me,0x76, 0x00);
		  vl53l0x_WriteReg(me,0x77, 0x00);

		  vl53l0x_WriteReg(me,0xFF, 0x01);
		  vl53l0x_WriteReg(me,0x0D, 0x01);

		  vl53l0x_WriteReg(me,0xFF, 0x00);
		  vl53l0x_WriteReg(me,0x80, 0x01);
		  vl53l0x_WriteReg(me,0x01, 0xF8);

		  vl53l0x_WriteReg(me,0xFF, 0x01);
		  vl53l0x_WriteReg(me,0x8E, 0x01);
		  vl53l0x_WriteReg(me,0x00, 0x01);
		  vl53l0x_WriteReg(me,0xFF, 0x00);
		  vl53l0x_WriteReg(me,0x80, 0x00);

		  // -- VL53L0X_load_tuning_settings() end

		  // "Set interrupt config to new sample ready"
		  // -- VL53L0X_SetGpioConfig() begin

		  vl53l0x_WriteReg(me,SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
		  tempValue=(vl53l0x_ReadReg(me,GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
		  vl53l0x_WriteReg(me,GPIO_HV_MUX_ACTIVE_HIGH, tempValue); // active low
		  vl53l0x_WriteReg(me,SYSTEM_INTERRUPT_CLEAR, 0x01);

		  // -- VL53L0X_SetGpioConfig() end

		  me->instance->measurement_timing_budget_us = vl53l0x_GetMeasurementTimingBudget(me);

		  // "Disable MSRC and TCC by default"
		  // MSRC = Minimum Signal Rate Check
		  // TCC = Target CentreCheck
		  // -- VL53L0X_SetSequenceStepEnable() begin

		  vl53l0x_WriteReg(me,SYSTEM_SEQUENCE_CONFIG, 0xE8);

		  // -- VL53L0X_SetSequenceStepEnable() end

		  // "Recalculate timing budget"
		  vl53l0x_SetMeasurementTimingBudget(me,me->instance->measurement_timing_budget_us);

		  // VL53L0X_StaticInit() end

		  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

		  // -- VL53L0X_perform_vhv_calibration() begin

		  vl53l0x_WriteReg(me,SYSTEM_SEQUENCE_CONFIG, 0x01);
		  if (!vl53l0x_PerformSingleRefCalibration(me,0x40)) {
			  addErrorValue(VL5310x_ErrorPerformSingleRefCalibration);
			  return 0;
		  }

		  // -- VL53L0X_perform_vhv_calibration() end

		  // -- VL53L0X_perform_phase_calibration() begin

		  vl53l0x_WriteReg(me,SYSTEM_SEQUENCE_CONFIG, 0x02);
		  if (!vl53l0x_PerformSingleRefCalibration(me,0x00)) {
			  addErrorValue(VL5310x_ErrorPerformSingleRefCalibration);
			  return 0;
		  }

		  // -- VL53L0X_perform_phase_calibration() end

		  // "restore the previous Sequence Config"
		  vl53l0x_WriteReg(me,SYSTEM_SEQUENCE_CONFIG, 0xE8);

		  // VL53L0X_PerformRefCalibration() end
		  me->status=vl53l0x_Idle;
		  return 1;
}


void vl53l0x_WriteReg(StructVl53l0x *me,uint8_t reg, uint8_t value){
	HAL_I2C_Mem_Write(me->hi2c, me->instance->address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 10);
}

void vl53l0x_WriteReg16Bit(StructVl53l0x* me,uint8_t reg, uint16_t value){
	uint8_t tempArray[2];
	tempArray[0]=((value >> 8) & 0xFF);
	tempArray[1]=value & 0xFF;
	HAL_I2C_Mem_Write(me->hi2c, me->instance->address, reg, I2C_MEMADD_SIZE_8BIT, tempArray, 2, 10);

}

void vl53l0x_WriteReg32Bit(StructVl53l0x* me,uint8_t reg, uint32_t value){
	uint8_t tempArray[4];
	tempArray[0]=((value >> 24) & 0xFF);
	tempArray[1]=((value >> 16) & 0xFF);
	tempArray[2]=(value >>  8) & 0xFF;
	tempArray[3]=value & 0xFF;
	HAL_I2C_Mem_Write(me->hi2c, me->instance->address, reg, I2C_MEMADD_SIZE_8BIT, tempArray, 4, 10);

}

uint8_t vl53l0x_ReadReg(StructVl53l0x* me,uint8_t reg){
	uint8_t value;
	HAL_I2C_Mem_Read(me->hi2c, me->instance->address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1);
	return value;
}


uint16_t vl53l0x_ReadReg16Bit(StructVl53l0x* me,uint8_t reg){
	uint8_t tempArray[2];
	uint16_t value;

	HAL_I2C_Mem_Read(me->hi2c, me->instance->address, reg, I2C_MEMADD_SIZE_8BIT, tempArray, 2, 1);


	value  = (uint16_t)tempArray[0]<< 8; // value high byte
	value |=           tempArray[1];      // value low byte

	return value;
}
uint32_t vl53l0x_ReadReg32Bit(StructVl53l0x* me,uint8_t reg){
	uint8_t tempArray[4];
		uint32_t value;

		HAL_I2C_Mem_Read(me->hi2c, me->instance->address, reg, I2C_MEMADD_SIZE_8BIT, tempArray, 4, 1);


		value  = (uint32_t)tempArray[0]<< 24; // value high byte
		value |= (uint32_t)tempArray[1]<< 16;
		value |=(uint32_t)tempArray[2]<< 8;
		value |=tempArray[3]; // value low byte

		return value;
}
void vl53l0x_WriteMulti(StructVl53l0x* me,uint8_t reg, uint8_t * src, uint8_t count){
	HAL_I2C_Mem_Write(me->hi2c, me->instance->address, reg, I2C_MEMADD_SIZE_8BIT, src, count, 10);
}
void vl53l0x_ReadMulti(StructVl53l0x* me,uint8_t reg, uint8_t * dst, uint8_t count){
	HAL_I2C_Mem_Read(me->hi2c, me->instance->address, reg, I2C_MEMADD_SIZE_8BIT, dst, count, 1);
}

uint8_t vl53l0x_SetSignalRateLimit(StructVl53l0x* me,float limit_Mcps){
	  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return 0; }

	  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
	  vl53l0x_WriteReg16Bit(me,FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
	  return 1;
}
float vl53l0x_GetSignalRateLimit(StructVl53l0x* me){
	return (float)vl53l0x_ReadReg16Bit(me,FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

uint8_t vl53l0x_SetMeasurementTimingBudget(StructVl53l0x* me,uint32_t budget_us){


	  uint16_t const StartOverhead     = 1910;
	  uint16_t const EndOverhead        = 960;
	  uint16_t const MsrcOverhead       = 660;
	  uint16_t const TccOverhead        = 590;
	  uint16_t const DssOverhead        = 690;
	  uint16_t const PreRangeOverhead   = 660;
	  uint16_t const FinalRangeOverhead = 550;

	  uint32_t const MinTimingBudget = 20000;

	  if (budget_us < MinTimingBudget) {
		  addErrorValue(VL5310X_IncorrectBudgetUs);
		  return 0;
	  }


	  uint32_t used_budget_us = StartOverhead + EndOverhead;

	  vl53l0x_GetSequenceStepEnables(me);
	  vl53l0x_GetSequenceStepTimeouts(me);

	  if (me->instance->tcc){
	    used_budget_us += (me->instance->msrc_dss_tcc_us + TccOverhead);
	  }

	  if (me->instance->dss){
	    used_budget_us += 2 * (me->instance->msrc_dss_tcc_us + DssOverhead);
	  }
	  else if (me->instance->msrc){
	    used_budget_us += (me->instance->msrc_dss_tcc_us + MsrcOverhead);
	  }

	  if (me->instance->pre_range){
	    used_budget_us += (me->instance->pre_range_us + PreRangeOverhead);
	  }

	  if (me->instance->final_range){
	    used_budget_us += FinalRangeOverhead;

	    // "Note that the final range timeout is determined by the timing
	    // budget and the sum of all other timeouts within the sequence.
	    // If there is no room for the final range timeout, then an error
	    // will be set. Otherwise the remaining time will be applied to
	    // the final range."

	    if (used_budget_us > budget_us){
	      // "Requested timeout too big."
	      return 0;
	    }

	    uint32_t final_range_timeout_us = budget_us - used_budget_us;

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

	    // "For the final range timeout, the pre-range timeout
	    //  must be added. To do this both final and pre-range
	    //  timeouts must be expressed in macro periods MClks
	    //  because they have different vcsel periods."

	    uint32_t final_range_timeout_mclks =
	      vl53l0x_TimeoutMicrosecondsToMclks(final_range_timeout_us,
	    		  me->instance->final_range_vcsel_period_pclks);

	    if (me->instance->pre_range){
	      final_range_timeout_mclks += me->instance->pre_range_mclks;
	    }

	    vl53l0x_WriteReg16Bit(me,FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
	      vl53l0x_EncodeTimeout(final_range_timeout_mclks));

	    // set_sequence_step_timeout() end

	    me->instance->measurement_timing_budget_us = budget_us; // store for internal reuse
	  }
	  return 1;
}
uint32_t vl53l0x_GetMeasurementTimingBudget(StructVl53l0x* me){

	  uint16_t const StartOverhead     = 1910;
	  uint16_t const EndOverhead        = 960;
	  uint16_t const MsrcOverhead       = 660;
	  uint16_t const TccOverhead        = 590;
	  uint16_t const DssOverhead        = 690;
	  uint16_t const PreRangeOverhead   = 660;
	  uint16_t const FinalRangeOverhead = 550;

	  // "Start and end overhead times always present"
	  uint32_t budget_us = StartOverhead + EndOverhead;

	  vl53l0x_GetSequenceStepEnables(me);
	  vl53l0x_GetSequenceStepTimeouts(me);

	  if (me->instance->tcc){
	    budget_us += (me->instance->msrc_dss_tcc_us + TccOverhead);
	  }

	  if (me->instance->dss){
	    budget_us += 2 * (me->instance->msrc_dss_tcc_us + DssOverhead);
	  }
	  else if (me->instance->msrc){
	    budget_us += (me->instance->msrc_dss_tcc_us + MsrcOverhead);
	  }

	  if (me->instance->pre_range){
	    budget_us += (me->instance->pre_range_us + PreRangeOverhead);
	  }

	  if (me->instance->final_range){
	    budget_us += (me->instance->final_range_us + FinalRangeOverhead);
	  }

	  me->instance->measurement_timing_budget_us = budget_us; // store for internal reuse
	  return budget_us;
}

uint8_t vl53l0x_SetVcselPulsePeriod(StructVl53l0x* me,vcselPeriodType type, uint8_t period_pclks){
	uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

	  vl53l0x_GetSequenceStepEnables(me);
	  vl53l0x_GetSequenceStepTimeouts(me);

	  // "Apply specific settings for the requested clock period"
	  // "Re-calculate and apply timeouts, in macro periods"

	  // "When the VCSEL period for the pre or final range is changed,
	  // the corresponding timeout must be read from the device using
	  // the current VCSEL period, then the new VCSEL period can be
	  // applied. The timeout then must be written back to the device
	  // using the new VCSEL period.
	  //
	  // For the MSRC timeout, the same applies - this timeout being
	  // dependant on the pre-range vcsel period."


	  if (type == VcselPeriodPreRange){
	    // "Set phase check limits"
	    switch (period_pclks){
	      case 12:
	        vl53l0x_WriteReg(me,PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
	        break;

	      case 14:
	        vl53l0x_WriteReg(me,PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
	        break;

	      case 16:
	        vl53l0x_WriteReg(me,PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
	        break;

	      case 18:
	        vl53l0x_WriteReg(me,PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
	        break;

	      default:

	    	 addErrorValue(VL5310X_IncorrectPeriodVcSelPeriodPrerange);
	        return 0;
	    }
	    vl53l0x_WriteReg(me,PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

	    // apply new VCSEL period
	    vl53l0x_WriteReg(me,PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

	    // update timeouts

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

	    uint16_t new_pre_range_timeout_mclks=vl53l0x_TimeoutMicrosecondsToMclks(me->instance->pre_range_us, period_pclks);

	    vl53l0x_WriteReg16Bit(me,PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,vl53l0x_EncodeTimeout(new_pre_range_timeout_mclks));

	    // set_sequence_step_timeout() end

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

	    uint16_t new_msrc_timeout_mclks =vl53l0x_TimeoutMicrosecondsToMclks(me->instance->msrc_dss_tcc_us, period_pclks);

	    vl53l0x_WriteReg(me,MSRC_CONFIG_TIMEOUT_MACROP,(new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

	    // set_sequence_step_timeout() end
	  }
	  else if (type ==VcselPeriodFinalRange){
	    switch (period_pclks){
	      case 8:
	        vl53l0x_WriteReg(me,FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
	        vl53l0x_WriteReg(me,FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	        vl53l0x_WriteReg(me,GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
	        vl53l0x_WriteReg(me,ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
	        vl53l0x_WriteReg(me,0xFF, 0x01);
	        vl53l0x_WriteReg(me,ALGO_PHASECAL_LIM, 0x30);
	        vl53l0x_WriteReg(me,0xFF, 0x00);
	        break;

	      case 10:
	    	  vl53l0x_WriteReg(me,FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
	    	  vl53l0x_WriteReg(me,FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	    	  vl53l0x_WriteReg(me,GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
	    	  vl53l0x_WriteReg(me,ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
	    	  vl53l0x_WriteReg(me,0xFF, 0x01);
	        vl53l0x_WriteReg(me,ALGO_PHASECAL_LIM, 0x20);
	        vl53l0x_WriteReg(me,0xFF, 0x00);
	        break;

	      case 12:
	    	  vl53l0x_WriteReg(me,FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
	    	  vl53l0x_WriteReg(me,FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	    	  vl53l0x_WriteReg(me,GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
	    	  vl53l0x_WriteReg(me,ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
	    	  vl53l0x_WriteReg(me,0xFF, 0x01);
	    	  vl53l0x_WriteReg(me,ALGO_PHASECAL_LIM, 0x20);
	    	  vl53l0x_WriteReg(me,0xFF, 0x00);
	        break;

	      case 14:
	    	  vl53l0x_WriteReg(me,FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
	    	  vl53l0x_WriteReg(me,FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	    	  vl53l0x_WriteReg(me,GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
	    	  vl53l0x_WriteReg(me,ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
	    	  vl53l0x_WriteReg(me,0xFF, 0x01);
	    	  vl53l0x_WriteReg(me,ALGO_PHASECAL_LIM, 0x20);
	    	  vl53l0x_WriteReg(me,0xFF, 0x00);
	        break;

	      default:
	        // invalid period
	    	  addErrorValue(VL5310X_IncorrectPeriodVcSelPeriodFinalrange);
	        return 0;
	    }

	    // apply new VCSEL period
	    vl53l0x_WriteReg(me,FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

	    // update timeouts

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

	    // "For the final range timeout, the pre-range timeout
	    //  must be added. To do this both final and pre-range
	    //  timeouts must be expressed in macro periods MClks
	    //  because they have different vcsel periods."

	    uint16_t new_final_range_timeout_mclks = vl53l0x_TimeoutMicrosecondsToMclks(me->instance->final_range_us, period_pclks);

	    if (me->instance->pre_range){
	      new_final_range_timeout_mclks += me->instance->pre_range_mclks;
	    }

	    vl53l0x_WriteReg16Bit(me,FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,vl53l0x_EncodeTimeout(new_final_range_timeout_mclks));

	    // set_sequence_step_timeout end
	  }else{
	    // invalid type
		  addErrorValue(VL5310X_IncorrectTypeVcselperiodType);

		  return 0;
	  }

	  // "Finally, the timing budget must be re-applied"

	  vl53l0x_SetMeasurementTimingBudget(me,me->instance->measurement_timing_budget_us);

	  // "Perform the phase calibration. This is needed after changing on vcsel period."
	  // VL53L0X_perform_phase_calibration() begin

	  uint8_t sequence_config = vl53l0x_ReadReg(me,SYSTEM_SEQUENCE_CONFIG);
	  vl53l0x_WriteReg(me,SYSTEM_SEQUENCE_CONFIG, 0x02);
	  vl53l0x_PerformSingleRefCalibration(me,0x0);
	  vl53l0x_WriteReg(me,SYSTEM_SEQUENCE_CONFIG, sequence_config);

	  // VL53L0X_perform_phase_calibration() end

	  return 1;
}
uint8_t vl53l0x_GetVcselPulsePeriod(StructVl53l0x* me,vcselPeriodType type){
	if (type == VcselPeriodPreRange){
		return decodeVcselPeriod(vl53l0x_ReadReg(me,PRE_RANGE_CONFIG_VCSEL_PERIOD));
	}else if (type == VcselPeriodFinalRange){
		return decodeVcselPeriod(vl53l0x_ReadReg(me,FINAL_RANGE_CONFIG_VCSEL_PERIOD));
	}else {
		addErrorValue(VL5310X_OutOfTypeVcSelPeriodType);
		return 255;
	}
}

void vl53l0x_StartContinuous(StructVl53l0x* me,uint32_t period_ms){
	vl53l0x_WriteReg(me,0x80, 0x01);
	vl53l0x_WriteReg(me,0xFF, 0x01);
	vl53l0x_WriteReg(me,0x00, 0x00);
	vl53l0x_WriteReg(me,0x91, me->instance->stop_variable);
	vl53l0x_WriteReg(me,0x00, 0x01);
	vl53l0x_WriteReg(me,0xFF, 0x00);
	vl53l0x_WriteReg(me,0x80, 0x00);

	  if (period_ms != 0){
	    // continuous timed mode

	    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

	    uint16_t osc_calibrate_val = vl53l0x_ReadReg16Bit(me,OSC_CALIBRATE_VAL);

	    if (osc_calibrate_val != 0){
	      period_ms *= osc_calibrate_val;
	    }

	    vl53l0x_WriteReg32Bit(me,SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

	    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

	    vl53l0x_WriteReg(me,SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
	  }else{
	    // continuous back-to-back mode
	    vl53l0x_WriteReg(me,SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
	  }
	  me->instance->continuousMode=SET;
}
void vl53l0x_StopContinuous(StructVl53l0x* me){
	  vl53l0x_WriteReg(me,SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

	  vl53l0x_WriteReg(me,0xFF, 0x01);
	  vl53l0x_WriteReg(me,0x00, 0x00);
	  vl53l0x_WriteReg(me,0x91, 0x00);
	  vl53l0x_WriteReg(me,0x00, 0x01);
	  vl53l0x_WriteReg(me,0xFF, 0x00);
	  me->instance->continuousMode=RESET;
}

void vl53l0x_ClearInterruptFlag(StructVl53l0x* me){
	vl53l0x_WriteReg(me,SYSTEM_INTERRUPT_CLEAR, 0x01);
}

uint8_t vl53l0x_TimeoutOccurred(StructVl53l0x* me){
	  uint8_t tmp = me->instance->did_timeout;
	  me->instance->did_timeout = 0;
	  return tmp;
}


uint8_t vl53l0x_GetSpadInfo(StructVl53l0x* me,uint8_t * count, uint8_t * type_is_aperture){
	uint8_t tmp;

	  vl53l0x_WriteReg(me,0x80, 0x01);
	  vl53l0x_WriteReg(me,0xFF, 0x01);
	  vl53l0x_WriteReg(me,0x00, 0x00);

	  vl53l0x_WriteReg(me,0xFF, 0x06);
	  vl53l0x_WriteReg(me,0x83, vl53l0x_ReadReg(me,0x83) | 0x04);
	  vl53l0x_WriteReg(me,0xFF, 0x07);
	  vl53l0x_WriteReg(me,0x81, 0x01);

	  vl53l0x_WriteReg(me,0x80, 0x01);

	  vl53l0x_WriteReg(me,0x94, 0x6b);
	  vl53l0x_WriteReg(me,0x83, 0x00);
	  HAL_Delay(100);
	  int i=2550;
	  while (vl53l0x_ReadReg(me,0x83) == 0x00){
	    if (i<1) {
	    	//errorCodePush(VL5310X_TIMEOUT_GETSPADINFO);
	    	return 0;
	    }
	    --i;
	  }
	  vl53l0x_WriteReg(me,0x83, 0x01);
	  tmp = vl53l0x_ReadReg(me,0x92);

	  *count = tmp & 0x7f;
	  *type_is_aperture = (tmp >> 7) & 0x01;

	  vl53l0x_WriteReg(me,0x81, 0x00);
	  vl53l0x_WriteReg(me,0xFF, 0x06);
	  vl53l0x_WriteReg(me,0x83, vl53l0x_ReadReg(me,0x83)  & ~0x04);
	  vl53l0x_WriteReg(me,0xFF, 0x01);
	  vl53l0x_WriteReg(me,0x00, 0x01);

	  vl53l0x_WriteReg(me,0xFF, 0x00);
	  vl53l0x_WriteReg(me,0x80, 0x00);

	  return 1;
}

void vl53l0x_GetSequenceStepEnables(StructVl53l0x* me){
	uint8_t sequence_config = vl53l0x_ReadReg(me,SYSTEM_SEQUENCE_CONFIG);

	  me->instance->tcc          = (sequence_config >> 4) & 0x1;
	  me->instance->dss          = (sequence_config >> 3) & 0x1;
	  me->instance->msrc         = (sequence_config >> 2) & 0x1;
	  me->instance->pre_range    = (sequence_config >> 6) & 0x1;
	  me->instance->final_range  = (sequence_config >> 7) & 0x1;
}
void vl53l0x_GetSequenceStepTimeouts(StructVl53l0x* me){
	me->instance->pre_range_vcsel_period_pclks = vl53l0x_GetVcselPulsePeriod(me,VcselPeriodPreRange);
	me->instance->msrc_dss_tcc_mclks = vl53l0x_ReadReg(me,MSRC_CONFIG_TIMEOUT_MACROP) + 1;
	me->instance->msrc_dss_tcc_us =vl53l0x_TimeoutMclksToMicroseconds(me->instance->msrc_dss_tcc_mclks,me->instance->pre_range_vcsel_period_pclks);
	me->instance->pre_range_mclks = vl53l0x_DecodeTimeout(vl53l0x_ReadReg16Bit(me,PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
	me->instance->pre_range_us =vl53l0x_TimeoutMclksToMicroseconds(me->instance->pre_range_mclks,me->instance->pre_range_vcsel_period_pclks);
	me->instance->final_range_vcsel_period_pclks = vl53l0x_GetVcselPulsePeriod(me,VcselPeriodFinalRange);
	me->instance->final_range_mclks = vl53l0x_DecodeTimeout(vl53l0x_ReadReg16Bit(me,FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

	if (me->instance->pre_range){
		me->instance->final_range_mclks -= me->instance->pre_range_mclks;
	}

	me->instance->final_range_us =vl53l0x_TimeoutMclksToMicroseconds(me->instance->final_range_mclks,me->instance->final_range_vcsel_period_pclks);

}

uint8_t vl53l0x_PerformSingleRefCalibration(StructVl53l0x* me,uint8_t vhv_init_byte){
	vl53l0x_WriteReg(me,SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

	  HAL_Delay(250);
	  int i=2550;
	  while ((vl53l0x_ReadReg(me,RESULT_INTERRUPT_STATUS) & 0x07) == 0){
	    if (i<1) { return 0; }
	    --i;
	  }

	  vl53l0x_WriteReg(me,SYSTEM_INTERRUPT_CLEAR, 0x01);

	  vl53l0x_WriteReg(me,SYSRANGE_START, 0x00);

	  return 1;
}

uint16_t* vl53l0x_ReadDistance(StructVl53l0x* me){
	return statusFunction[me->status](me);
}
uint16_t vl53l0x_DecodeTimeout(uint16_t reg_val){
	 return (uint16_t)((reg_val & 0x00FF) <<(uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}
uint16_t vl53l0x_EncodeTimeout(uint32_t timeout_mclks){
	  // format: "(LSByte * 2^MSByte) + 1"

	  uint32_t ls_byte = 0;
	  uint16_t ms_byte = 0;

	  if (timeout_mclks > 0){
	    ls_byte = timeout_mclks - 1;

	    while ((ls_byte & 0xFFFFFF00) > 0){
	      ls_byte >>= 1;
	      ms_byte++;
	    }

	    return (ms_byte << 8) | (ls_byte & 0xFF);
	  }
	  else {
		  addErrorValue(VL5310X_ErrorEndCodeTimeout);
		  return 0;
	  }
}
uint32_t vl53l0x_TimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks){
	uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

	  return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}
uint32_t vl53l0x_TimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks){
	uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

	  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}
void vl53l0x_IncreaseTime(StructVl53l0x* me){
	me->time++;
}
void vl53l0x_ResetTime(StructVl53l0x* me){
	me->time=0;
}

uint16_t* vl53l0x_FunctionNoInitialized(StructVl53l0x* me){
	if(vl53l0x_SensorInit(me)){
		vl53l0x_GetVcselPulsePeriod(me,VcselPeriodPreRange);
	}else{
		me->status=vl53l0x_InitializedError;
	}
	return NULL;
}
uint16_t* vl53l0x_FunctionInitializedError(StructVl53l0x* me){
	return NULL;
}
uint16_t* vl53l0x_FunctionIdle(StructVl53l0x* me){
	if(me->instance->continuousMode==SET){
		me->status=vl53l0x_WaitingForTheInterruptFlagToBeSet;
		vl53l0x_ResetTime(me);
	}
	return NULL;
}
uint16_t* vl53l0x_FunctionMeasurmentPreparation(StructVl53l0x* me){
	vl53l0x_WriteReg(me,0x80, 0x01);
	vl53l0x_WriteReg(me,0xFF, 0x01);
	vl53l0x_WriteReg(me,0x00, 0x00);
	vl53l0x_WriteReg(me,0x91, me->instance->stop_variable);
	vl53l0x_WriteReg(me,0x00, 0x01);
	vl53l0x_WriteReg(me,0xFF, 0x00);
	vl53l0x_WriteReg(me,0x80, 0x00);
	vl53l0x_WriteReg(me,SYSRANGE_START, 0x01);
	me->status=vl53l0x_WaitUntilStartBitHasBeenCleared;
	vl53l0x_ResetTime(me);
	return NULL;
}
uint16_t* vl53l0x_FunctionWaitUntilStartBitHasBeenCleared(StructVl53l0x* me){

	if(me->time>=VL53L0x_MAX_TIME_FOR_MEARUMENT){
		me->status=vl53l0x_Timeout;
		return NULL;
	}
	if((vl53l0x_ReadReg(me,SYSRANGE_START) & 0x01)==0){
		me->status=vl53l0x_WaitingForTheInterruptFlagToBeSet;
		vl53l0x_ResetTime(me);
	}
	return NULL;
}
uint16_t* vl53l0x_FunctionWaitingForTheInterruptFlagToBeSet(StructVl53l0x* me){
	if(me->time>=VL53L0x_MAX_TIME_FOR_MEARUMENT){
			me->status=vl53l0x_Timeout;
			return NULL;
		}
	if((vl53l0x_ReadReg(me,RESULT_INTERRUPT_STATUS) & 0x07)!= 0){
		me->distance=vl53l0x_ReadReg16Bit(me,RESULT_RANGE_STATUS + 10);
		vl53l0x_ClearInterruptFlag(me);
		me->status=vl53l0x_Idle;
		return &me->distance;
	}
	return NULL;
}
uint16_t* vl53l0x_FunctionTimeout(StructVl53l0x* me){
	me->status=vl53l0x_Idle;
	return NULL;
}
void vl53l0x_StartSingleMeasurment(StructVl53l0x* me){
	if((me->status==vl53l0x_Idle)&& me->instance->continuousMode==RESET){
		me->status=vl53l0x_MeasurmentPreparation;
		vl53l0x_ResetTime(me);
	}else{
		addErrorValue(VL5310X_StatusNoIdle);
	}
}
uint8_t vl53l0x_IsReady(StructVl53l0x* me){
	if(me->status==vl53l0x_Idle && me->instance->continuousMode==RESET){
		return SET;
	}else{
		return RESET;
	}
}
