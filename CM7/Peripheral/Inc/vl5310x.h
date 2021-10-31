/*
 * vl5310x.h
 *
 *  Created on: 13 lip 2021
 *      Author: Daniel
 */

#ifndef INC_VL5310X_H_
#define INC_VL5310X_H_
#define      SYSRANGE_START                               0x00

#define      SYSTEM_THRESH_HIGH                           0x0C
#define      SYSTEM_THRESH_LOW                            0x0E

#define      SYSTEM_SEQUENCE_CONFIG                       0x01
#define      SYSTEM_RANGE_CONFIG                          0x09
#define      SYSTEM_INTERMEASUREMENT_PERIOD               0x04

#define      SYSTEM_INTERRUPT_CONFIG_GPIO                 0x0A

#define     GPIO_HV_MUX_ACTIVE_HIGH                      0x84

#define      SYSTEM_INTERRUPT_CLEAR                       0x0B

#define      RESULT_INTERRUPT_STATUS                      0x13
#define      RESULT_RANGE_STATUS                          0x14

#define      RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN        0xBC
#define      RESULT_CORE_RANGING_TOTAL_EVENTS_RTN         0xC0
#define      RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF        0xD0
#define      RESULT_CORE_RANGING_TOTAL_EVENTS_REF         0xD4
#define      RESULT_PEAK_SIGNAL_RATE_REF                  0xB6

#define      ALGO_PART_TO_PART_RANGE_OFFSET_MM            0x28

#define      I2C_SLAVE_DEVICE_ADDRESS                     0x8A

#define      MSRC_CONFIG_CONTROL                          0x60

#define      PRE_RANGE_CONFIG_MIN_SNR                     0x27
#define      PRE_RANGE_CONFIG_VALID_PHASE_LOW             0x56
#define      PRE_RANGE_CONFIG_VALID_PHASE_HIGH            0x57
#define      PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT           0x64

#define      FINAL_RANGE_CONFIG_MIN_SNR                   0x67
#define      FINAL_RANGE_CONFIG_VALID_PHASE_LOW           0x47
#define      FINAL_RANGE_CONFIG_VALID_PHASE_HIGH          0x48
#define      FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT  0x44

#define      PRE_RANGE_CONFIG_SIGMA_THRESH_HI             0x61
#define      PRE_RANGE_CONFIG_SIGMA_THRESH_LO             0x62

#define      PRE_RANGE_CONFIG_VCSEL_PERIOD                0x50
#define      PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI           0x51
#define      PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO           0x52

#define      SYSTEM_HISTOGRAM_BIN                         0x81
#define      HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT        0x33
#define      HISTOGRAM_CONFIG_READOUT_CTRL                0x55

#define      FINAL_RANGE_CONFIG_VCSEL_PERIOD              0x70
#define      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI         0x71
#define      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO         0x72
#define      CROSSTALK_COMPENSATION_PEAK_RATE_MCPS        0x20

#define      MSRC_CONFIG_TIMEOUT_MACROP                   0x46
#define      SOFT_RESET_GO2_SOFT_RESET_N                  0xBF
#define      IDENTIFICATION_MODEL_ID                      0xC0
#define      IDENTIFICATION_REVISION_ID                   0xC2

#define      OSC_CALIBRATE_VAL                            0xF8

#define      GLOBAL_CONFIG_VCSEL_WIDTH                    0x32
#define      GLOBAL_CONFIG_SPAD_ENABLES_REF_0             0xB0
#define      GLOBAL_CONFIG_SPAD_ENABLES_REF_1             0xB1
#define      GLOBAL_CONFIG_SPAD_ENABLES_REF_2             0xB2
#define      GLOBAL_CONFIG_SPAD_ENABLES_REF_3             0xB3
#define      GLOBAL_CONFIG_SPAD_ENABLES_REF_4             0xB4
#define      GLOBAL_CONFIG_SPAD_ENABLES_REF_5             0xB5
#define      GLOBAL_CONFIG_REF_EN_START_SELECT            0xB6
#define      DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD          0x4E
#define      DYNAMIC_SPAD_REF_EN_START_OFFSET             0x4F
#define      POWER_MANAGEMENT_GO1_POWER_FORCE             0x80

#define      VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV            0x89
#define      ALGO_PHASECAL_LIM                            0x30
#define      ALGO_PHASECAL_CONFIG_TIMEOUT                 0x30

typedef enum {
	vl53l0xReadRangeContinuous_Idle,
	vl53l0xReadRangeContinuous_WaitToReturn07FromResultInterruptStatus,
	vl53l0xReadRangeContinuous_WaitToReadData,
}vl53l0xReadRangeContinuousStatusEnum;
typedef struct{
	uint8_t address;
	uint8_t last_status; // status of last I2C transmission
	I2C_HandleTypeDef *hi2c;
	uint8_t tcc;
	uint8_t msrc;
	uint8_t dss;
	uint8_t pre_range;
	uint8_t final_range;
    uint16_t pre_range_vcsel_period_pclks;
    uint16_t final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks;
    uint16_t pre_range_mclks;
	uint16_t final_range_mclks;
    uint32_t msrc_dss_tcc_us;
    uint32_t pre_range_us;
    uint32_t final_range_us;
    uint16_t io_timeout;
    uint8_t did_timeout;
    uint16_t timeout_start_ms;
    uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
    uint32_t measurement_timing_budget_us;
    uint16_t xshutPin;
    GPIO_TypeDef *xshutPort;
    uint16_t gpio1Pin;
    GPIO_TypeDef *gpio1Port;
    uint16_t distance;
    vl53l0xReadRangeContinuousStatusEnum vl53l0xReadRangeContinuousStatus;
}vl53l0x_reg_t;

typedef enum {
	VcselPeriodPreRange,
	VcselPeriodFinalRange
}vcselPeriodType;

vl53l0x_reg_t vl53l0xReg_s;

void vl53l0xSetI2c(I2C_HandleTypeDef * hi2c);
I2C_HandleTypeDef * vl53l0xGetI2c();

void vl53l0xSetAddress(uint8_t new_addr);
uint8_t vl53l0xGetAddress();

uint8_t vl53l0xInit();

void vl53l0xWriteReg(uint8_t reg, uint8_t value);
void vl53l0xWriteReg16Bit(uint8_t reg, uint16_t value);
void vl53l0xWriteReg32Bit(uint8_t reg, uint32_t value);
uint8_t vl53l0xReadReg(uint8_t reg);
uint16_t vl53l0xReadReg16Bit(uint8_t reg);
uint32_t vl53l0xReadReg32Bit(uint8_t reg);

void vl53l0xWriteMulti(uint8_t reg, uint8_t * src, uint8_t count);
void vl53l0xReadMulti(uint8_t reg, uint8_t * dst, uint8_t count);

uint8_t vl53l0xSetSignalRateLimit(float limit_Mcps);
float vl53l0xGetSignalRateLimit();

uint8_t vl53l0xSetMeasurementTimingBudget(uint32_t budget_us);
uint32_t vl53l0xGetMeasurementTimingBudget();

uint8_t vl53l0xSetVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);
uint8_t vl53l0xGetVcselPulsePeriod(vcselPeriodType type);

void vl53l0xStartContinuous(uint32_t period_ms);
void vl53l0xStopContinuous();
uint16_t vl53l0xReadRangeContinuousMillimeters();
uint16_t vl53l0xReadRangeSingleMillimeters();

inline void vl53l0xSetTimeoutDrivingStatus(uint16_t timeout) { vl53l0xReg_s.io_timeout = timeout; }
inline uint16_t vl53l0xGetTimeout() { return vl53l0xReg_s.io_timeout; }
uint8_t vl53l0xTimeoutOccurred();


uint8_t vl53l0xGetSpadInfo(uint8_t * count, uint8_t * type_is_aperture);

void vl53l0xGetSequenceStepEnables();
void vl53l0xGetSequenceStepTimeouts();

uint8_t vl53l0xPerformSingleRefCalibration(uint8_t vhv_init_byte);

uint16_t vl53l0xDecodeTimeout(uint16_t value);
uint16_t vl53l0xEncodeTimeout(uint32_t timeout_mclks);
uint32_t vl53l0xTimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
uint32_t vl53l0xTimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

void vl53l0xSetXshut(GPIO_TypeDef *xshutPort,uint16_t xshutPin);
void vl53l0xSetGpio(GPIO_TypeDef *gpio1Port,uint16_t gpio1Pin);
#endif /* INC_VL5310X_H_ */
