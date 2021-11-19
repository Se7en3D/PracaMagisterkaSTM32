/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <vl5310x.h>
#include "hc-sr04.h"
#include "uartCom.h"
#include "errorCode.h"
#include "stateMachine.h"
#include "irSensor.h"
#include "adc.h"
#include "servo360.h"
#include "connectionModule.h"
#include "servoPR.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

uint8_t bufor;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;
ADC_HandleTypeDef hadc1;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM7_Init(void);
static void MX_ADC1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  //HAL_UART_Transmit(&huart4, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  uartComPush(ch);
  return 1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */
	//circ_bbuf_init(CIRC_BBUF_UART4RX);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  MX_TIM17_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  	  /*Moduł inicjalizacji modułów*/
  errorCodeInit();
  hcsr04Tim2_p=&(hcsr04_t){0,0,0,0.0,0};
  uartCommunication_p=&(uartCommunication_t){&huart4,{0},{0},0,0};
  stateMachineInit(&htim12,GPIOE,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5);
  uartComSensorInit(&htim13);
  servoPRInit(&servoPRGeneralStructure,&htim2);
  	  /*Inizjalizacja peryfreriów przy pomocy biblioteki HAL */
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim17);
  HAL_UART_Receive_IT(&huart4,&uartCommunication_p->bufforReceived[0],SIZE_BUFFOR_RECEIVED);
  HAL_TIM_Base_Start(&htim12);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim13);
  HAL_TIM_PWM_Start_IT(&htim13, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim7);
	  /*inicjalizacja pozostałych modłów przy pomocy własnych bibliotek*/
  vl53l0xSetAddress(0x52);
  vl53l0xSetI2c(&hi2c1);
  vl53l0xSetXshut(VL53L0X_XSHUT_GPIO_Port, VL53L0X_XSHUT_Pin);
  vl53l0xSetGpio(VL53L0X_GPIO1_GPIO_Port, VL53L0X_GPIO1_Pin);
  vl53l0xInit();
  vl53l0xGetVcselPulsePeriod(VcselPeriodPreRange);
  vl53l0xStartContinuous(0);

  	  /*inicjalizacja potrów od czujników odległości na podczerwień */
  irSensorInitIrPinout(IR_NR1_NUMBER,IR_NR_1_Pin,IR_NR_1_GPIO_Port);
  irSensorInitIrPinout(IR_NR2_NUMBER,IR_NR_2_Pin,IR_NR_2_GPIO_Port);
  irSensorInitIrPinout(IR_NR3_NUMBER,IR_NR_3_Pin,IR_NR_3_GPIO_Port);
  irSensorInitIrPinout(IR_NR4_NUMBER,IR_NR_4_Pin,IR_NR_4_GPIO_Port);
  irSensorInitIrPinout(IR_NR5_NUMBER,IR_NR_5_Pin,IR_NR_5_GPIO_Port);
  irSensorInitIrPinout(IR_NR6_NUMBER,IR_NR_6_Pin,IR_NR_6_GPIO_Port);
  irSensorInitIrPinout(IR_NR7_NUMBER,IR_NR_7_Pin,IR_NR_7_GPIO_Port);
  irSensorInitIrPinout(IR_NR8_NUMBER,IR_NR_8_Pin,IR_NR_8_GPIO_Port);

  adcInit(&hadc1,OPTOCOUPLER_POWER_ON_GPIO_Port,OPTOCOUPLER_POWER_ON_Pin);
  servo360Init(&htim13,LIMIT_SWITCH_SERVO_Pin,LIMIT_SWITCH_SERVO_GPIO_Port);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	  irSensorReadStatusIrSensor(); //Pobranie statusu portów czujników IR

	  uint8_t* framepointer=uartComGetFrame();
	  if(framepointer!=0){
		  switch(framepointer[FUNCTIONPOSITION]){
		  case RIDE_FORWARD_FUN:
			  stateMachineDrivingForward();
			  break;
		  case RIDE_BACKWARD_FUN:
			  stateMachineDrivingBack();
			  break;
		  case RIDE_RIGHT_FUN:
			  stateMachineDrivingRight();
			  break;
		  case RIDE_LEFT_FUN:
			  stateMachineDrivingLeft();
			  break;
		  case ROTATE_LEFT:
			  stateMachineRotateLeft();
			  break;
		  case ROTATE_RIGHT:
			  stateMachineRotateRight();
			  break;
		  case RIDE_BACKWARD_RIGHT:
			  stateMachineRotateBackRight();
			  break;
		  case RIDE_BACKWARD_LEFT:
			  stateMachineRotateBackLeft();
			  break;
		  case STOP_FUN:
			  stateMachineStopDriving();
			  break;
		  case MEASURE_DISTANCE_FUN: //Wykonanie pomiaru odległości Brak w najnowszej wersji

			  break;
		  case CALIBRATION_PWM_DATA:
			  servo360NewDataPWM(framepointer);
			  break;
		  case UNIMPORTANT_ERROR: //Brak żadnych komend
			  break;
		  default:
				  //JAKIŚ ERROR?
				 stateMachineStopDriving();
			  break;
		  }
		  uartComClearFrame();
	  }
	  connectionModuleStateMachineWithServo360(drivingStructure.drivingStatus);
	  connectionModuleMeasureDistance(&servo360Structure,&measurmentStructure);

	  //driving_status_t drivingStatusTemp=stateMachineGetDrivingStructure().drivingStatus;
	  /*if(drivingStructure.drivingStatus!=IDLE_DRIVING){
		  if(drivingStructure.drivingStatus!=STOP_DRIVING){ //Pomiar pracy podczas jazdy
			  //if(uartComSensorFrame.waitForData){
				  float distanceUltra=hcsr04GetCelculatedValue();
				  if(distanceUltra!=0.0){
					  uint16_t distancevl5310x=vl53l0xReadRangeContinuousMillimeters();
					  uartComSendDistanceServo(distanceUltra, distancevl5310x);
					  //uartComServoClearWaitForData();
				  }
			  //}
			  //uartComServoIsShiftPosition(); //Próba przemieszenia serwomechanizmu do następnej pozycji

			  if(uartComGetMeasureForMeasureDistanceFun()!=0){
				  uartComResetMeasureForMeasureDistanceFun();
			  }
		  }else{ //Pomiar dystansu na polecenie specjalne
				float distanceUltra=hcsr04GetCelculatedValue();
				if(distanceUltra!=0.0){
					uint16_t distancevl5310x=vl53l0xReadRangeContinuousMillimeters();
					uartComSendDistance(distanceUltra, distancevl5310x);
					uartComAddMeasureForMeasureDistanceFun();
				}
			  if(uartComGetMeasureForMeasureDistanceFun()>MAX_MEASURMENT_SENSOR){
				  uartComResetMeasureForMeasureDistanceFun();
				  stateMachineMeasureDistanceEnd();
			  }
		  }

	  }*/
	  	 //Reset licznika od powtórzenia polecenia kierunku jazdy
	  if(stateMachineGetResetTimer()){
		  stateMachineResetTimer();
		  if(HAL_TIM_Base_GetState(&htim17)==HAL_TIM_STATE_READY){ //Sprawdzenie czy timer został zastopowany i wymaga ponownego uruchomienia
			  HAL_TIM_Base_Start(&htim17);
		  }else{
			  __HAL_TIM_SET_COUNTER(&htim17, 0);
		  }
	  }


	  //switch()
	  //Sprawdzenie czy należy uruchomić pomiar ADC
	  /*if(adcGetTime()>=ADC_TIME_TO_START_CONVERSION){
		  HAL_ADC_Start_IT(&hadc1);
		  adcClearTime();
	  }

	  //uint32_t convValue=adcGetConversionValue();
	  if(adcReadyToSendData()==ADC_READY_TO_SEND){
		  uint32_t value=adcGetConversionValue();
		  uartComSendAdcBatteryVoltage(value);
	  }*/

	  switch(adcStage()){
	 // case ADC_START_CONVERSION:
		//  HAL_ADC_Start_IT(&hadc1);
		//  break;
	  case ADC_READY_TO_SEND:
		  uartComSendAdcBatteryVoltage(adcGetValue());
		  break;
	  default:
		  break;
	  }


	  servo360StateFunctions();


	  	  //Sprawdzenie czy znajdują się jakieś błedy do wysłania
	  uint8_t errorCode=errorCodePop();
	  if(errorCode!=UNIMPORTANT_ERROR){
		 uartComSendErrorCode(errorCode);
	  }

	  	  //Sprawdzenie czy DMA od komunikacji PC<->STM32 jest już dostepne do wyłsania kolejnych pakietów danych
	  if(hdma_uart4_tx.State==HAL_DMA_STATE_READY){
		  uint8_t *uartBufferPointer=uartComGetBufferFirstElementAddress();
		  uint32_t uartBufferLength=uartComGetBufferLength();
		  if(uartBufferLength!=0){
			  HAL_UART_Transmit_DMA(&huart4, uartBufferPointer, uartBufferLength);
		  }
	  }
	  if(irSensorGetTime()>TIME_TO_SEND_STATUS_IR){
		  uartComSendIrSensorStatus(irSensorGetAllCollision(),MAX_SENSOR_IR);
		  irSensorClearTime();
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* UART4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* TIM17_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM17_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM17_IRQn);
  /* TIM8_UP_TIM13_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 239;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 23;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 2399;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 5999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 99;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 47999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3599;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 2399;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 34999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, L298NOUT1A_Pin|L298NOUT2A_Pin|L298NOUT1B_Pin|L298NOUT1BE5_Pin
                          |OPTOCOUPLER_POWER_ON_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VL53L0X_XSHUT_GPIO_Port, VL53L0X_XSHUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : L298NOUT1A_Pin L298NOUT2A_Pin L298NOUT1B_Pin L298NOUT1BE5_Pin
                           OPTOCOUPLER_POWER_ON_Pin LD2_Pin */
  GPIO_InitStruct.Pin = L298NOUT1A_Pin|L298NOUT2A_Pin|L298NOUT1B_Pin|L298NOUT1BE5_Pin
                          |OPTOCOUPLER_POWER_ON_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_NR_1_Pin IR_NR_2_Pin */
  GPIO_InitStruct.Pin = IR_NR_1_Pin|IR_NR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_NR_3_Pin IR_NR_4_Pin IR_NR_5_Pin */
  GPIO_InitStruct.Pin = IR_NR_3_Pin|IR_NR_4_Pin|IR_NR_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_NR_6_Pin IR_NR_7_Pin IR_NR_8_Pin VL53L0X_GPIO1_Pin */
  GPIO_InitStruct.Pin = IR_NR_6_Pin|IR_NR_7_Pin|IR_NR_8_Pin|VL53L0X_GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LIMIT_SWITCH_SERVO_Pin */
  GPIO_InitStruct.Pin = LIMIT_SWITCH_SERVO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LIMIT_SWITCH_SERVO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VL53L0X_XSHUT_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VL53L0X_XSHUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if(htim==&htim7){
		irSensorAddTime();
		adcAddTime();
		connectionModuleaddTimeout(&measurmentStructure);
	}
	if(htim==&htim13){
		servo360PWMEdit();
	}
	if(htim==&htim17){
		stateMachineTimeout();
		HAL_TIM_Base_Stop(&htim17);
	}
	if(htim==&htim2){
		if ((htim->Instance->CCMR1 & TIM_CCMR1_CC1S) != 0x00U){
			hcsr04CompCH1Add(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
		}

		if ((htim->Instance->CCMR1 & TIM_CCMR1_CC2S) != 0x00U){
			hcsr04CompCH2Add(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2));
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
	if(UartHandle==&huart4){
		uint8_t *pointer=UartHandle->pRxBuffPtr-1;
		int i=&uartCommunication_p->bufforReceived[SIZE_BUFFOR_RECEIVED-1]-pointer;
		while((pointer+i) <= &uartCommunication_p->bufforReceived[SIZE_BUFFOR_RECEIVED-1] && i<SIZE_BUFFOR_RECEIVED){
			uartComAddFrame(uartCommunication_p->bufforReceived[i]);
			i++;

		}
		HAL_UART_Receive_IT(&huart4,&uartCommunication_p->bufforReceived[0],SIZE_BUFFOR_RECEIVED);
	}
}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c){
	printf(" 0x%X",bufor);
	//HAL_I2C_Master_Receive_IT (&hi2c1, VL53L0X_ADDRESS, &bufor, 1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc==&hadc1){
		uint32_t result = HAL_ADC_GetValue(&hadc1);
		adcSetConversionValue(result);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
