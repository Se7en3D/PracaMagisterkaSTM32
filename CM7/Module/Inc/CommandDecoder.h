/*
 * CommandDecoder.h
 *
 *  Created on: 27 sty 2022
 *      Author: Daniel
 */

#ifndef INC_COMMANDDECODER_H_
#define INC_COMMANDDECODER_H_
#define COMMANDDECODER_MAX_LENGTH_COMMAND 10 /*!<Maksymalna wielkosc odbieranej komendy*/
#define COMMADNDECODER_TIME_TO_CLEAR_BUFFER 50 /*!<Czas po którym należy oczyścić bufor [ms] */
#define COMMANDDECODER_START_BYTE 0xFF/*!<Bajt początku ramki  */
#define COMMANDDECODER_STOP_BYTE 0xFE/*!<Bajt konca ramki*/
#define COMMANDDECODER_POSITION_FUNCTION_IN_FRAME 2/*!<Pozycja bufora który przechowuje funkcję przesyłanej komendy*/
typedef struct bluetoothDecoderStruct bluetoothDecoderStruct;
typedef enum decoderStatus decoderStatus;

enum decoderStatus{
	decoder_Idle,
	decoder_WaitToEnd,
	decoder_EndDecod,
};

struct bluetoothDecoderStruct{
	uint8_t buffer[COMMANDDECODER_MAX_LENGTH_COMMAND];
	uint32_t head;
	uint32_t timeout;
	decoderStatus status;
	void(*Insert)(bluetoothDecoderStruct* const me, const uint8_t value);
	uint8_t* (*GetFunction)(bluetoothDecoderStruct* const me);
	void (*ClearBuffer)(bluetoothDecoderStruct* const me);
	void (*AddTimeout)(bluetoothDecoderStruct* const me);
	void (*StatusFunction[3])(bluetoothDecoderStruct* const me, const uint8_t value);
};

bluetoothDecoderStruct* CommandDecoder_Create();
void CommandDecoder_Init(bluetoothDecoderStruct* const me,
		void(*insert)(bluetoothDecoderStruct* const me, const uint8_t value),
		uint8_t* (*getFunction)(bluetoothDecoderStruct* const me),
		void (*clearBuffer)(bluetoothDecoderStruct* const me),
		void (*addTimeout)(bluetoothDecoderStruct* const me)
		);
void CommandDecoder_Insert(bluetoothDecoderStruct* const me, const uint8_t value);
uint8_t* CommandDecoder_GetFunction(bluetoothDecoderStruct* const me);
void CommandDecoder_ClearBuffer(bluetoothDecoderStruct* const me);
void CommandDecoder_AddTimeout(bluetoothDecoderStruct* const me);
void CommandDecoder_ResetTimeout(bluetoothDecoderStruct* const me);
void CommandDecoder_StatusIdle(bluetoothDecoderStruct* const me, const uint8_t value);
void CommandDecoder_StatusWaitToEnd(bluetoothDecoderStruct* const me, const uint8_t value);
void CommandDecoder_StatusEndDecod(bluetoothDecoderStruct* const me, const uint8_t value);
void CommandDecoder_CheckOutOfHeadValue(bluetoothDecoderStruct* const me);
extern void addErrorValue(uint8_t value);
#endif /* INC_COMMANDDECODER_H_ */
