/*
 * hc-sr04.h
 *
 *  Created on: 9 lip 2021
 *      Author: DanielD
 */

#ifndef INC_HC_SR04_H_
#define INC_HC_SR04_H_
typedef struct {
	uint32_t start;
	uint32_t stop;
	uint8_t isCalculated;
	float calculatedValue;
	uint32_t timeout;
} hcsr04_t;

hcsr04_t *hcsr04Tim2_p;
void hcsr04CompCH1Add(uint32_t value);
void hcsr04CompCH2Add(uint32_t value);
void hcsr04CalculateDistance();
uint8_t hcsr04IsReadyToSend();
float hcsr04GetCelculatedValue();
uint32_t hcsr04GetTimeout();
void hcsr04ClearTimeout();
void hcsr04ClearMeasurement();
#endif /* INC_HC_SR04_H_ */
