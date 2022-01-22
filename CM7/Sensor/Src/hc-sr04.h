/*
 * hc-sr04.h
 *
 *  Created on: 9 lip 2021
 *      Author: DanielD
 */

typedef enum{
	HC_SR04_IDLE,
	HC_SR04_MEASURMENT_RISING_EDGE,
	HC_SR04_MEASURMENT_FALLING_EDGE,
	HC_SR04_END_MEASURMENT,
	HC_SR04_TIMEOUT,
}HC_SR04Status;

typedef struct {
	HC_SR04Status status;
	uint32_t risingTime;
	uint32_t fallingTime;
	float calculatedValue;
	uint32_t timeout;
} hcsr04_t;

hcsr04_t *hcsr04Tim2_p;
void hcsr04TIMInterruptAdd(hcsr04_t *hcsr04,uint32_t value);
void hcsr04CalculateDistance();
uint8_t hcsr04IsReadyToSend();
float hcsr04GetCelculatedValue();
uint32_t hcsr04GetTimeout();
void hcsr04ClearTimeout();
void hcsr04ClearMeasurement();
#endif /* INC_HC_SR04_H_ */
