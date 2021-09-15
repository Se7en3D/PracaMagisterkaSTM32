/*
 * flash.h
 *
 *  Created on: 4 sie 2021
 *      Author: DanielD
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#define FLASH_READY 1
#define FLASH_BUSY_SECTOR 2
typedef struct{
	uint32_t *baseAddress;
	uint32_t *lastAddress;
	uint32_t *saveDataAddress;
	uint32_t *readDataAddress;
	uint8_t flashRowIndex;
	uint8_t flashReady;
	uint32_t readPositionServo;
}flash_base_structure;

flash_base_structure flashBaseStructure;

void flashInit(uint32_t FlashBankBase,uint32_t FlashSector,uint32_t flashSectorSize);
uint32_t flashSerachSaveAndReadAddress();
void flashClearMemory(uint32_t FlashBankBase,uint32_t FlashSector,uint32_t flashSectorSize);
void flashWriteMemory(uint32_t data);
void flashToString();
#endif /* INC_FLASH_H_ */
