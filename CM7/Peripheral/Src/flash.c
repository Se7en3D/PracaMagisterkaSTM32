/*
 * flash.c
 *
 *  Created on: 4 sie 2021
 *      Author: DanielD
 */
#include "stm32h7xx_hal.h"
#include "errorCode.h"
#include "flash.h"
#include <stdio.h>

void flashInit(uint32_t FlashBankBase,uint32_t FlashSector,uint32_t flashSectorSize){
	flashBaseStructure.flashRowIndex=FLASH_NB_32BITWORD_IN_FLASHWORD;
	flashBaseStructure.baseAddress=(uint32_t *)(FlashBankBase+FlashSector*flashSectorSize);
	flashBaseStructure.lastAddress=(uint32_t *)(FlashBankBase+FlashSector*flashSectorSize*2);
	// uint32_t resultSearch=flashSerachSaveAndReadAddress();
	/*switch(resultSearch){
	case FLASH_READY:
		flashBaseStructure.flashReady=1;
	case FLASH_BUSY_SECTOR:

	default:
		flashBaseStructure.flashReady=0;
	}*/
}


uint32_t flashSerachSaveAndReadAddress(){
	if(flashBaseStructure.baseAddress==0 || flashBaseStructure.lastAddress==0 || flashBaseStructure.flashRowIndex==0){
		errorCodePush(FLASH_BASE_STRUCT_NULL);
		return 0;
	}

	uint32_t *tempPointer=flashBaseStructure.baseAddress;
	if(*tempPointer==0xFFFFFFFF){
		flashBaseStructure.saveDataAddress=tempPointer;
		flashBaseStructure.readDataAddress=(uint32_t *)0;
		return FLASH_READY;
	}else{
		tempPointer+=flashBaseStructure.flashRowIndex;
	}
	//uint32_t timestamp==(flashBaseStructure.lastAddress-flashBaseStructure.baseAddress)/flashBaseStructure.flashRowIndex;
	while(tempPointer<flashBaseStructure.lastAddress){
		if(*tempPointer==0xFFFFFFFF){
			flashBaseStructure.saveDataAddress= tempPointer;
			flashBaseStructure.readDataAddress=(tempPointer-flashBaseStructure.flashRowIndex);
			flashBaseStructure.readPositionServo=*flashBaseStructure.readDataAddress;
			return FLASH_READY;
		}
		tempPointer+=flashBaseStructure.flashRowIndex;
	}
	flashBaseStructure.readPositionServo=*(tempPointer-flashBaseStructure.flashRowIndex);
	return FLASH_BUSY_SECTOR;

}

void flashClearMemory(uint32_t FlashBankBase,uint32_t FlashSector,uint32_t flashSectorSize){
	if(HAL_FLASH_Unlock()==HAL_ERROR){
		errorCodePush(FLASH_UNLOCK_ERROR);
		return;
	}
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.Banks=FlashBankBase;
	EraseInitStruct.Sector=FlashSector;
	EraseInitStruct.NbSectors=1;
	EraseInitStruct.TypeErase=FLASH_TYPEERASE_SECTORS;

	uint32_t  errorStatus = 0;
	HAL_Delay(100);
	if(HAL_FLASHEx_Erase(&EraseInitStruct,&errorStatus)==HAL_ERROR){
		HAL_FLASH_Lock();
		errorCodePush(FLASH_ERASE_ERROR);
		return;
	}
	HAL_FLASH_Lock();

	flashBaseStructure.saveDataAddress=flashBaseStructure.baseAddress;
	flashBaseStructure.readDataAddress=0;

}

void flashWriteMemory(uint32_t data){
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,(uint32_t) flashBaseStructure.saveDataAddress,(uint32_t) &data);
	HAL_FLASH_Lock();
}

void flashToString(){
	printf("flashBaseStructure[baseAddress=%p lastAddress=%p saveDataAddress=%p  readDataAddress=%p flashRowIndex=%d flashReady=%d]\n",
			flashBaseStructure.baseAddress,flashBaseStructure.lastAddress,flashBaseStructure.saveDataAddress,flashBaseStructure.readDataAddress,flashBaseStructure.flashRowIndex,flashBaseStructure.flashReady);
}
