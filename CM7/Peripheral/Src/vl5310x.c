/*
 * vl5310x.h
 *
 *  Created on: 13 lip 2021
 *      Author: Daniel
 */
#include <stdio.h>
#include "stm32h7xx_hal.h"
#include <vl5310x.h>
#include "errorCode.h"

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)
void vl53l0xSetI2c(I2C_HandleTypeDef * hi2c) {
	vl53l0xReg_s.hi2c = hi2c;
}

I2C_HandleTypeDef * vl53l0xGetI2c() {
	return vl53l0xReg_s.hi2c;
}

uint8_t vl53l0xGetAddress() {
	return vl53l0xReg_s.address;
}

void vl53l0xSetAddress(uint8_t new_addr){
	  //writeReg(I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
	  vl53l0xReg_s.address = new_addr;
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
uint8_t vl53l0xInit(){
	  // check model ID register (value specified in datasheet)
	  if (vl53l0xReadReg(IDENTIFICATION_MODEL_ID) != 0xEE) {
		  errorCodePush(VL5310X_INCORRECT_MODEL_ID);
		  return 0;
	  }
	  if(vl53l0xReg_s.xshutPort!=NULL){
		  HAL_GPIO_WritePin(vl53l0xReg_s.xshutPort, vl53l0xReg_s.xshutPin,GPIO_PIN_RESET);
		  HAL_Delay(20);
		  HAL_GPIO_WritePin(vl53l0xReg_s.xshutPort, vl53l0xReg_s.xshutPin,GPIO_PIN_SET);
		  HAL_Delay(5);
	  }else{
		  errorCodePush(VL53L0X_NULL_XSHUL_POINTER);
	  }

	  // VL53L0X_DataInit() begin

	  // "Set I2C standard mode"
	  vl53l0xWriteReg(0x88, 0x00);

	  vl53l0xWriteReg(0x80, 0x01);
	  vl53l0xWriteReg(0xFF, 0x01);
	  vl53l0xWriteReg(0x00, 0x00);
	  vl53l0xReg_s.stop_variable = vl53l0xReadReg(0x91);
	  vl53l0xWriteReg(0x00, 0x01);
	  vl53l0xWriteReg(0xFF, 0x00);
	  vl53l0xWriteReg(0x80, 0x00);

	  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	  vl53l0xWriteReg(MSRC_CONFIG_CONTROL, vl53l0xReadReg(MSRC_CONFIG_CONTROL) | 0x12);

	  // set final range signal rate limit to 0.25 MCPS (million counts per second)
	  vl53l0xSetSignalRateLimit(0.25);

	  vl53l0xWriteReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

	  // VL53L0X_DataInit() end

	  // VL53L0X_StaticInit() begin

	  uint8_t spad_count;
	  uint8_t spad_type_is_aperture;
	  if (!vl53l0xGetSpadInfo(&spad_count, &spad_type_is_aperture)) {
		  return 0;
	  }

	  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
	  // the API, but the same data seems to be more easily readable from
	  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	  uint8_t ref_spad_map[6];
	  vl53l0xReadMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

	  vl53l0xWriteReg(0xFF, 0x01);
	  vl53l0xWriteReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	  vl53l0xWriteReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	  vl53l0xWriteReg(0xFF, 0x00);
	  vl53l0xWriteReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

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

	  vl53l0xWriteMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	  // -- VL53L0X_set_reference_spads() end

	  // -- VL53L0X_load_tuning_settings() begin
	  // DefaultTuningSettings from vl53l0x_tuning.h

	  vl53l0xWriteReg(0xFF, 0x01);
	  vl53l0xWriteReg(0x00, 0x00);

	  vl53l0xWriteReg(0xFF, 0x00);
	  vl53l0xWriteReg(0x09, 0x00);
	  vl53l0xWriteReg(0x10, 0x00);
	  vl53l0xWriteReg(0x11, 0x00);

	  vl53l0xWriteReg(0x24, 0x01);
	  vl53l0xWriteReg(0x25, 0xFF);
	  vl53l0xWriteReg(0x75, 0x00);

	  vl53l0xWriteReg(0xFF, 0x01);
	  vl53l0xWriteReg(0x4E, 0x2C);
	  vl53l0xWriteReg(0x48, 0x00);
	  vl53l0xWriteReg(0x30, 0x20);

	  vl53l0xWriteReg(0xFF, 0x00);
	  vl53l0xWriteReg(0x30, 0x09);
	  vl53l0xWriteReg(0x54, 0x00);
	  vl53l0xWriteReg(0x31, 0x04);
	  vl53l0xWriteReg(0x32, 0x03);
	  vl53l0xWriteReg(0x40, 0x83);
	  vl53l0xWriteReg(0x46, 0x25);
	  vl53l0xWriteReg(0x60, 0x00);
	  vl53l0xWriteReg(0x27, 0x00);
	  vl53l0xWriteReg(0x50, 0x06);
	  vl53l0xWriteReg(0x51, 0x00);
	  vl53l0xWriteReg(0x52, 0x96);
	  vl53l0xWriteReg(0x56, 0x08);
	  vl53l0xWriteReg(0x57, 0x30);
	  vl53l0xWriteReg(0x61, 0x00);
	  vl53l0xWriteReg(0x62, 0x00);
	  vl53l0xWriteReg(0x64, 0x00);
	  vl53l0xWriteReg(0x65, 0x00);
	  vl53l0xWriteReg(0x66, 0xA0);

	  vl53l0xWriteReg(0xFF, 0x01);
	  vl53l0xWriteReg(0x22, 0x32);
	  vl53l0xWriteReg(0x47, 0x14);
	  vl53l0xWriteReg(0x49, 0xFF);
	  vl53l0xWriteReg(0x4A, 0x00);

	  vl53l0xWriteReg(0xFF, 0x00);
	  vl53l0xWriteReg(0x7A, 0x0A);
	  vl53l0xWriteReg(0x7B, 0x00);
	  vl53l0xWriteReg(0x78, 0x21);

	  vl53l0xWriteReg(0xFF, 0x01);
	  vl53l0xWriteReg(0x23, 0x34);
	  vl53l0xWriteReg(0x42, 0x00);
	  vl53l0xWriteReg(0x44, 0xFF);
	  vl53l0xWriteReg(0x45, 0x26);
	  vl53l0xWriteReg(0x46, 0x05);
	  vl53l0xWriteReg(0x40, 0x40);
	  vl53l0xWriteReg(0x0E, 0x06);
	  vl53l0xWriteReg(0x20, 0x1A);
	  vl53l0xWriteReg(0x43, 0x40);

	  vl53l0xWriteReg(0xFF, 0x00);
	  vl53l0xWriteReg(0x34, 0x03);
	  vl53l0xWriteReg(0x35, 0x44);

	  vl53l0xWriteReg(0xFF, 0x01);
	  vl53l0xWriteReg(0x31, 0x04);
	  vl53l0xWriteReg(0x4B, 0x09);
	  vl53l0xWriteReg(0x4C, 0x05);
	  vl53l0xWriteReg(0x4D, 0x04);

	  vl53l0xWriteReg(0xFF, 0x00);
	  vl53l0xWriteReg(0x44, 0x00);
	  vl53l0xWriteReg(0x45, 0x20);
	  vl53l0xWriteReg(0x47, 0x08);
	  vl53l0xWriteReg(0x48, 0x28);
	  vl53l0xWriteReg(0x67, 0x00);
	  vl53l0xWriteReg(0x70, 0x04);
	  vl53l0xWriteReg(0x71, 0x01);
	  vl53l0xWriteReg(0x72, 0xFE);
	  vl53l0xWriteReg(0x76, 0x00);
	  vl53l0xWriteReg(0x77, 0x00);

	  vl53l0xWriteReg(0xFF, 0x01);
	  vl53l0xWriteReg(0x0D, 0x01);

	  vl53l0xWriteReg(0xFF, 0x00);
	  vl53l0xWriteReg(0x80, 0x01);
	  vl53l0xWriteReg(0x01, 0xF8);

	  vl53l0xWriteReg(0xFF, 0x01);
	  vl53l0xWriteReg(0x8E, 0x01);
	  vl53l0xWriteReg(0x00, 0x01);
	  vl53l0xWriteReg(0xFF, 0x00);
	  vl53l0xWriteReg(0x80, 0x00);

	  // -- VL53L0X_load_tuning_settings() end

	  // "Set interrupt config to new sample ready"
	  // -- VL53L0X_SetGpioConfig() begin

	  vl53l0xWriteReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
	  vl53l0xWriteReg(GPIO_HV_MUX_ACTIVE_HIGH, vl53l0xReadReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
	  vl53l0xWriteReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

	  // -- VL53L0X_SetGpioConfig() end

	  vl53l0xReg_s.measurement_timing_budget_us = vl53l0xGetMeasurementTimingBudget();

	  // "Disable MSRC and TCC by default"
	  // MSRC = Minimum Signal Rate Check
	  // TCC = Target CentreCheck
	  // -- VL53L0X_SetSequenceStepEnable() begin

	  vl53l0xWriteReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

	  // -- VL53L0X_SetSequenceStepEnable() end

	  // "Recalculate timing budget"
	  vl53l0xSetMeasurementTimingBudget(vl53l0xReg_s.measurement_timing_budget_us);

	  // VL53L0X_StaticInit() end

	  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

	  // -- VL53L0X_perform_vhv_calibration() begin

	  vl53l0xWriteReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
	  if (!vl53l0xPerformSingleRefCalibration(0x40)) { return 0; }

	  // -- VL53L0X_perform_vhv_calibration() end

	  // -- VL53L0X_perform_phase_calibration() begin

	  vl53l0xWriteReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
	  if (!vl53l0xPerformSingleRefCalibration(0x00)) { return 0; }

	  // -- VL53L0X_perform_phase_calibration() end

	  // "restore the previous Sequence Config"
	  vl53l0xWriteReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

	  // VL53L0X_PerformRefCalibration() end

	  return 1;
}

// Write an 8-bit register
void vl53l0xWriteReg(uint8_t reg, uint8_t value){
	HAL_I2C_Mem_Write(vl53l0xReg_s.hi2c, vl53l0xReg_s.address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 10);
}

// Write a 16-bit register
void vl53l0xWriteReg16Bit(uint8_t reg, uint16_t value){
	uint8_t tempArray[2];
	tempArray[0]=((value >> 8) & 0xFF);
	tempArray[1]=value & 0xFF;
	HAL_I2C_Mem_Write(vl53l0xReg_s.hi2c, vl53l0xReg_s.address, reg, I2C_MEMADD_SIZE_8BIT, tempArray, 2, 10);
}

// Write a 32-bit register
void vl53l0xWriteReg32Bit(uint8_t reg, uint32_t value){
	uint8_t tempArray[4];
	tempArray[0]=((value >> 24) & 0xFF);
	tempArray[1]=((value >> 16) & 0xFF);
	tempArray[2]=(value >>  8) & 0xFF;
	tempArray[3]=value & 0xFF;
	HAL_I2C_Mem_Write(vl53l0xReg_s.hi2c, vl53l0xReg_s.address, reg, I2C_MEMADD_SIZE_8BIT, tempArray, 4, 10);
}

// Read an 8-bit register
uint8_t vl53l0xReadReg(uint8_t reg){
	uint8_t value;
	HAL_I2C_Mem_Read(vl53l0xReg_s.hi2c, vl53l0xReg_s.address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1);
	return value;
}

// Read a 16-bit register
uint16_t vl53l0xReadReg16Bit(uint8_t reg){
	uint8_t tempArray[2];
	uint16_t value;

	HAL_I2C_Mem_Read(vl53l0xReg_s.hi2c, vl53l0xReg_s.address, reg, I2C_MEMADD_SIZE_8BIT, tempArray, 2, 1);


	value  = (uint16_t)tempArray[0]<< 8; // value high byte
	value |=           tempArray[1];      // value low byte

	return value;
}

// Read a 32-bit register
uint32_t vl53l0xReadReg32Bit(uint8_t reg){
	uint8_t tempArray[4];
	uint32_t value;

	HAL_I2C_Mem_Read(vl53l0xReg_s.hi2c, vl53l0xReg_s.address, reg, I2C_MEMADD_SIZE_8BIT, tempArray, 4, 1);


	value  = (uint32_t)tempArray[0]<< 24; // value high byte
	value |= (uint32_t)tempArray[1]<< 16;
	value |=(uint32_t)tempArray[2]<< 8;
	value |=tempArray[3]; // value low byte

	return value;
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void vl53l0xWriteMulti(uint8_t reg, uint8_t * src, uint8_t count){
	HAL_I2C_Mem_Write(vl53l0xReg_s.hi2c, vl53l0xReg_s.address, reg, I2C_MEMADD_SIZE_8BIT, src, count, 10);
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void vl53l0xReadMulti(uint8_t reg, uint8_t * dst, uint8_t count){
	HAL_I2C_Mem_Read(vl53l0xReg_s.hi2c, vl53l0xReg_s.address, reg, I2C_MEMADD_SIZE_8BIT, dst, count, 1);
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
uint8_t vl53l0xSetSignalRateLimit(float limit_Mcps){
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return 0; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  vl53l0xWriteReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
  return 1;
}

// Get the return signal rate limit check value in MCPS
float vl53l0xGetSignalRateLimit()
{
  return (float)vl53l0xReadReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
uint8_t vl53l0xSetMeasurementTimingBudget(uint32_t budget_us){


  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) {
	  errorCodePush(VL5310X_INCORRECT_BUDGET_US);
	  return 0;
  }


  uint32_t used_budget_us = StartOverhead + EndOverhead;

  vl53l0xGetSequenceStepEnables();
  vl53l0xGetSequenceStepTimeouts();

  if (vl53l0xReg_s.tcc){
    used_budget_us += (vl53l0xReg_s.msrc_dss_tcc_us + TccOverhead);
  }

  if (vl53l0xReg_s.dss){
    used_budget_us += 2 * (vl53l0xReg_s.msrc_dss_tcc_us + DssOverhead);
  }
  else if (vl53l0xReg_s.msrc){
    used_budget_us += (vl53l0xReg_s.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (vl53l0xReg_s.pre_range){
    used_budget_us += (vl53l0xReg_s.pre_range_us + PreRangeOverhead);
  }

  if (vl53l0xReg_s.final_range){
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
      vl53l0xTimeoutMicrosecondsToMclks(final_range_timeout_us,
    		  vl53l0xReg_s.final_range_vcsel_period_pclks);

    if (vl53l0xReg_s.pre_range){
      final_range_timeout_mclks += vl53l0xReg_s.pre_range_mclks;
    }

    vl53l0xWriteReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      vl53l0xEncodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    vl53l0xReg_s.measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return 1;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t vl53l0xGetMeasurementTimingBudget(){

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  vl53l0xGetSequenceStepEnables();
  vl53l0xGetSequenceStepTimeouts();

  if (vl53l0xReg_s.tcc){
    budget_us += (vl53l0xReg_s.msrc_dss_tcc_us + TccOverhead);
  }

  if (vl53l0xReg_s.dss){
    budget_us += 2 * (vl53l0xReg_s.msrc_dss_tcc_us + DssOverhead);
  }
  else if (vl53l0xReg_s.msrc){
    budget_us += (vl53l0xReg_s.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (vl53l0xReg_s.pre_range){
    budget_us += (vl53l0xReg_s.pre_range_us + PreRangeOverhead);
  }

  if (vl53l0xReg_s.final_range){
    budget_us += (vl53l0xReg_s.final_range_us + FinalRangeOverhead);
  }

  vl53l0xReg_s.measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
uint8_t vl53l0xSetVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks){
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  vl53l0xGetSequenceStepEnables();
  vl53l0xGetSequenceStepTimeouts();

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
        vl53l0xWriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
        vl53l0xWriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
        vl53l0xWriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
        vl53l0xWriteReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
    	  errorCodePush(VL5310X_INCORRECT_PERIOD_VCSELPERIODPRERANGE);
        return 0;
    }
    vl53l0xWriteReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    vl53l0xWriteReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks=vl53l0xTimeoutMicrosecondsToMclks(vl53l0xReg_s.pre_range_us, period_pclks);

    vl53l0xWriteReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,vl53l0xEncodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =vl53l0xTimeoutMicrosecondsToMclks(vl53l0xReg_s.msrc_dss_tcc_us, period_pclks);

    vl53l0xWriteReg(MSRC_CONFIG_TIMEOUT_MACROP,(new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type ==VcselPeriodFinalRange){
    switch (period_pclks){
      case 8:
        vl53l0xWriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        vl53l0xWriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        vl53l0xWriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        vl53l0xWriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        vl53l0xWriteReg(0xFF, 0x01);
        vl53l0xWriteReg(ALGO_PHASECAL_LIM, 0x30);
        vl53l0xWriteReg(0xFF, 0x00);
        break;

      case 10:
        vl53l0xWriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        vl53l0xWriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        vl53l0xWriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        vl53l0xWriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        vl53l0xWriteReg(0xFF, 0x01);
        vl53l0xWriteReg(ALGO_PHASECAL_LIM, 0x20);
        vl53l0xWriteReg(0xFF, 0x00);
        break;

      case 12:
        vl53l0xWriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        vl53l0xWriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        vl53l0xWriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        vl53l0xWriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        vl53l0xWriteReg(0xFF, 0x01);
        vl53l0xWriteReg(ALGO_PHASECAL_LIM, 0x20);
        vl53l0xWriteReg(0xFF, 0x00);
        break;

      case 14:
        vl53l0xWriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        vl53l0xWriteReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        vl53l0xWriteReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        vl53l0xWriteReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        vl53l0xWriteReg(0xFF, 0x01);
        vl53l0xWriteReg(ALGO_PHASECAL_LIM, 0x20);
        vl53l0xWriteReg(0xFF, 0x00);
        break;

      default:
        // invalid period
    	  errorCodePush(VL5310X_INCORRECT_PERIOD_VCSELPERIODFINALRANGE);
        return 0;
    }

    // apply new VCSEL period
    vl53l0xWriteReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks = vl53l0xTimeoutMicrosecondsToMclks(vl53l0xReg_s.final_range_us, period_pclks);

    if (vl53l0xReg_s.pre_range){
      new_final_range_timeout_mclks += vl53l0xReg_s.pre_range_mclks;
    }

    vl53l0xWriteReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,vl53l0xEncodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }else{
    // invalid type
	  errorCodePush(VL5310X_INCORRECT_TYPE_VCSELPERIODTYPE);
    return 0;
  }

  // "Finally, the timing budget must be re-applied"

  vl53l0xSetMeasurementTimingBudget(vl53l0xReg_s.measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8_t sequence_config = vl53l0xReadReg(SYSTEM_SEQUENCE_CONFIG);
  vl53l0xWriteReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  vl53l0xPerformSingleRefCalibration(0x0);
  vl53l0xWriteReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return 1;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t vl53l0xGetVcselPulsePeriod(vcselPeriodType type){
	if (type == VcselPeriodPreRange){
		return decodeVcselPeriod(vl53l0xReadReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
	}else if (type == VcselPeriodFinalRange){
		return decodeVcselPeriod(vl53l0xReadReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
	}else {
		return 255;
	}
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void vl53l0xStartContinuous(uint32_t period_ms){
  vl53l0xWriteReg(0x80, 0x01);
  vl53l0xWriteReg(0xFF, 0x01);
  vl53l0xWriteReg(0x00, 0x00);
  vl53l0xWriteReg(0x91, vl53l0xReg_s.stop_variable);
  vl53l0xWriteReg(0x00, 0x01);
  vl53l0xWriteReg(0xFF, 0x00);
  vl53l0xWriteReg(0x80, 0x00);

  if (period_ms != 0){
    // continuous timed mode

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

    uint16_t osc_calibrate_val = vl53l0xReadReg16Bit(OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0){
      period_ms *= osc_calibrate_val;
    }

    vl53l0xWriteReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

    vl53l0xWriteReg(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }else{
    // continuous back-to-back mode
    vl53l0xWriteReg(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void vl53l0xStopContinuous(){
  vl53l0xWriteReg(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

  vl53l0xWriteReg(0xFF, 0x01);
  vl53l0xWriteReg(0x00, 0x00);
  vl53l0xWriteReg(0x91, 0x00);
  vl53l0xWriteReg(0x00, 0x01);
  vl53l0xWriteReg(0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t vl53l0xReadRangeContinuousMillimeters(){
	int i=200;
	while ((vl53l0xReadReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0){
		if (i<1){
			vl53l0xReg_s.did_timeout = 1;
			errorCodePush(VL5310X_TIMEOUT_READRANGECONTINUOUSMILLIMETERS);
			return 65535;
		}
		--i;
	}

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  uint16_t range = vl53l0xReadReg16Bit(RESULT_RANGE_STATUS + 10);

  vl53l0xWriteReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t vl53l0xReadRangeSingleMillimeters(){
  vl53l0xWriteReg(0x80, 0x01);
  vl53l0xWriteReg(0xFF, 0x01);
  vl53l0xWriteReg(0x00, 0x00);
  vl53l0xWriteReg(0x91, vl53l0xReg_s.stop_variable);
  vl53l0xWriteReg(0x00, 0x01);
  vl53l0xWriteReg(0xFF, 0x00);
  vl53l0xWriteReg(0x80, 0x00);

  vl53l0xWriteReg(SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"
  	  int i=2550;
  	  while (vl53l0xReadReg(SYSRANGE_START) & 0x01){
  		  if (i<1){
  			  vl53l0xReg_s.did_timeout = 1;
  			  errorCodePush(VL5310X_TIMEOUT_READRANGESINGLEMILLIMETERS);
  			  return 65535;
  		  }
  	  }

  return vl53l0xReadRangeContinuousMillimeters();
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
uint8_t vl53l0xTimeoutOccurred(){
  uint8_t tmp = vl53l0xReg_s.did_timeout;
  vl53l0xReg_s.did_timeout = 0;
  return tmp;
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
uint8_t vl53l0xGetSpadInfo(uint8_t * count, uint8_t * type_is_aperture){
  uint8_t tmp;

  vl53l0xWriteReg(0x80, 0x01);
  vl53l0xWriteReg(0xFF, 0x01);
  vl53l0xWriteReg(0x00, 0x00);

  vl53l0xWriteReg(0xFF, 0x06);
  vl53l0xWriteReg(0x83, vl53l0xReadReg(0x83) | 0x04);
  vl53l0xWriteReg(0xFF, 0x07);
  vl53l0xWriteReg(0x81, 0x01);

  vl53l0xWriteReg(0x80, 0x01);

  vl53l0xWriteReg(0x94, 0x6b);
  vl53l0xWriteReg(0x83, 0x00);
  HAL_Delay(100);
  int i=2550;
  while (vl53l0xReadReg(0x83) == 0x00){
    if (i<1) {
    	errorCodePush(VL5310X_TIMEOUT_GETSPADINFO);
    	return 0;
    }
    --i;
  }
  vl53l0xWriteReg(0x83, 0x01);
  tmp = vl53l0xReadReg(0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  vl53l0xWriteReg(0x81, 0x00);
  vl53l0xWriteReg(0xFF, 0x06);
  vl53l0xWriteReg(0x83, vl53l0xReadReg(0x83)  & ~0x04);
  vl53l0xWriteReg(0xFF, 0x01);
  vl53l0xWriteReg(0x00, 0x01);

  vl53l0xWriteReg(0xFF, 0x00);
  vl53l0xWriteReg(0x80, 0x00);

  return 1;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void vl53l0xGetSequenceStepEnables(){
  uint8_t sequence_config = vl53l0xReadReg(SYSTEM_SEQUENCE_CONFIG);

  vl53l0xReg_s.tcc          = (sequence_config >> 4) & 0x1;
  vl53l0xReg_s.dss          = (sequence_config >> 3) & 0x1;
  vl53l0xReg_s.msrc         = (sequence_config >> 2) & 0x1;
  vl53l0xReg_s.pre_range    = (sequence_config >> 6) & 0x1;
  vl53l0xReg_s.final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void vl53l0xGetSequenceStepTimeouts(){
	vl53l0xReg_s.pre_range_vcsel_period_pclks = vl53l0xGetVcselPulsePeriod(VcselPeriodPreRange);

	vl53l0xReg_s.msrc_dss_tcc_mclks = vl53l0xReadReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
	vl53l0xReg_s.msrc_dss_tcc_us =vl53l0xTimeoutMclksToMicroseconds(vl53l0xReg_s.msrc_dss_tcc_mclks,vl53l0xReg_s.pre_range_vcsel_period_pclks);

  vl53l0xReg_s.pre_range_mclks = vl53l0xDecodeTimeout(vl53l0xReadReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  vl53l0xReg_s.pre_range_us =vl53l0xTimeoutMclksToMicroseconds(vl53l0xReg_s.pre_range_mclks,vl53l0xReg_s.pre_range_vcsel_period_pclks);

  vl53l0xReg_s.final_range_vcsel_period_pclks = vl53l0xGetVcselPulsePeriod(VcselPeriodFinalRange);

  vl53l0xReg_s.final_range_mclks = vl53l0xDecodeTimeout(vl53l0xReadReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (vl53l0xReg_s.pre_range){
	  vl53l0xReg_s.final_range_mclks -= vl53l0xReg_s.pre_range_mclks;
  }

  vl53l0xReg_s.final_range_us =vl53l0xTimeoutMclksToMicroseconds(vl53l0xReg_s.final_range_mclks,vl53l0xReg_s.final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t vl53l0xDecodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<(uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
uint16_t vl53l0xEncodeTimeout(uint32_t timeout_mclks)
{
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
	  errorCodePush(VL5310X_ERROR_ENCODETIMEOUT);
	  return 0;
  }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t vl53l0xTimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t vl53l0xTimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks){
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


// based on VL53L0X_perform_single_ref_calibration()
uint8_t vl53l0xPerformSingleRefCalibration(uint8_t vhv_init_byte){
  vl53l0xWriteReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  HAL_Delay(250);
  int i=2550;
  while ((vl53l0xReadReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0){
    if (i<1) { return 0; }
    --i;
  }

  vl53l0xWriteReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  vl53l0xWriteReg(SYSRANGE_START, 0x00);

  return 1;
}

void vl53l0xSetXshut(GPIO_TypeDef *xshutPort,uint16_t xshutPin){
	vl53l0xReg_s.xshutPort=xshutPort;
	vl53l0xReg_s.xshutPin=xshutPin;
}
void vl53l0xSetGpio(GPIO_TypeDef *gpio1Port,uint16_t gpio1Pin){
	vl53l0xReg_s.gpio1Port=gpio1Port;
	vl53l0xReg_s.gpio1Pin=gpio1Pin;

}
