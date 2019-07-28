/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lora.h"
#include "INA219.h"
#include "zenith_can_lib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VARELA_BRD_ID 1
#define ALVES_BRD_ID 2
#define COMIN_BRD_ID 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
Z_CAN_Package can_rx_pkg;
Z_CAN_Package can_tx_pkg;

LoRaClass loraRX, loraTX;
int32_t packetSize;
int32_t counter = 0;
uint8_t RxData[128];
uint32_t RxDataLength;
/*
typedef struct {
	//INA ina[8];
	float ext;
	float inter;
	float curr;
	float volt;
}LoRaPacket;
*/

//INA lorapacket[8];

typedef struct {
	//INA ina[8];
	float v_bat;
	float v_charger;
	float v_p1;
	float v_p2;

//	float i_bat;
//	float i_charger;
//	float i_p1;
//	float i_p2;
}LoRaPacket;

LoRaPacket lorapacket, lorapacketRX;

typedef struct {
	uint32_t a;
	int32_t b;
	char c;
	int8_t d;
	void* e;
}TestStruct;

TestStruct t1, t2;

float temp;

int i = 0, j = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void atualizaCAN(Z_CAN_Package can_rx_pkg, LoRaPacket* lorapacket);
float tempADC(ADC_HandleTypeDef* phadc, float R0);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  // OBSERCACOES
  // Falta mudar a frequencia de um dos radios na funcao begin()
  // Falta verificar como a interupcao (DIO0) funciona

  //Inicializacao LoRa RX
  loraRX.setPins(SS_COMM_RX_GPIO_Port, SS_COMM_RX_Pin, RESET_COMM_RX_GPIO_Port, RESET_COMM_RX_Pin, EXTI15_10_IRQn);
  loraRX.setSPI(&hspi2);
  loraRX.setTIM(&htim3);

  if (!loraRX.begin(433.123E6)) {
   	  //printf("Starting LoRa RX failed!\n");
	  //memcpy(consoleData, "Starting LoRa RX failed!\n", 26);
	  //HAL_UART_Transmit(&huart2, consoleData, 26, 100);
   	  while (1);
  }

  loraRX.setSignalBandwidth(125E3);
  loraRX.setSpreadingFactor(11);
  loraRX.enableCrc();
  loraRX.receive();
  //printf("LoRa RX started\n");
  //memcpy(consoleData, "LoRa RX started\n", 17);
  //HAL_UART_Transmit(&huart2, consoleData, 17, 100);

  //Inicializacao LoRa TX
  loraTX.setPins(SS_COMM_TX_GPIO_Port, SS_COMM_TX_Pin, RESET_COMM_TX_GPIO_Port, RESET_COMM_TX_Pin, EXTI9_5_IRQn);
  loraTX.setSPI(&hspi2);
  loraTX.setTIM(&htim3);

  if (!loraTX.begin(433.123E6)) {
     printf("Starting LoRa RX failed!\n");
     while (1);
  }
  loraTX.setTxPower(20, 1);
  loraTX.setSignalBandwidth(125E3);
  loraTX.setSpreadingFactor(11);
  loraTX.enableCrc();
  printf("LoRa TX started\n");

  // Initializing CAN Filter
  filterConfigCAN(&hcan1);
  HAL_CAN_Start(&hcan1);

  uint32_t lorareset = 0;
  memcpy(&lorapacket, "To vivo", sizeof(lorapacket));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //Receber comando
	  if(i++ == 1 || RxData[4] == 'I'){
		  HAL_Delay(1000);
	  	  loraRX.readPacket(&RxData, 10);
	  	  i = 0;
	  }
	  if( RxData[2] == 'M' && RxData[3] == 'M'){
		  if(RxData[4] == 'I'){					//INAs
			  HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
			  loraTX.sendPacket(&lorapacket, sizeof(lorapacket));
			  loraRX.reset();
		  }
		  else if(RxData[4] == 'D'){			//DEPLOY
			  HAL_GPIO_WritePin(DEPLOY_GPIO_Port, DEPLOY_Pin, GPIO_PIN_SET);
			  HAL_Delay(4000);
			  HAL_GPIO_WritePin(DEPLOY_GPIO_Port, DEPLOY_Pin, GPIO_PIN_RESET);
		  }
		  else if(RxData[4] == 'E'){			//ESTABILIZACAO

		  }
		  else if(RxData[4] == 'A'){			//APONTAMENTO

		  }
	  }

	  lorareset++;
	  if(lorareset > 10){
		  loraTX.reset();
		  loraRX.reset();
		  lorareset = 0;
	  }

	  continue;

	  //packetSize = loraRX.parsePacket();
	  //loraRX.readPacket(&RxData, 100);
	  //if(RxData[0] != 0)
		  //lorapacket.volt = 10.6;
	  //loraTX.sendPacket(&lorapacket, sizeof(LoRaPacket));


	  temp = 0;
	  for(int k = 0; k < 5; k++)
	  	temp += tempADC(&hadc1, 4000);
	  temp /= 5;
	  //lorapacket.ext = temp;

	  //Receive CAN Message
	  for(int k = 0; k < 100; k++){
		  can_rx_pkg = readCanMessages(&hcan1);
	  	  atualizaCAN(can_rx_pkg, &lorapacket);
	  }

	  continue;
	  if(isEqual(can_rx_pkg, NULL_MSG)){				//Invalid Message
		  continue;
	  }

	  if(can_rx_pkg.identifier == 1){
		  float f;
	  	  memcpy(&f, can_rx_pkg.data, sizeof(float));
	  	  //lorapacket.inter = f;

	  }

	  if(can_rx_pkg.identifier == 3){					// Bateria
		  float f1, f2;
		  memcpy(&f1, can_rx_pkg.data, sizeof(float));
		  //lorapacket.curr = f1;
		  //lorapacket.i_bat = f1;
		  memcpy(&f2, can_rx_pkg.data + sizeof(float), sizeof(float));
		  //lorapacket.volt = f2;
		  lorapacket.v_bat = f2;
	  }
	  if(can_rx_pkg.identifier == 2){					// Charger
		  float f1, f2;
		  memcpy(&f1, can_rx_pkg.data, sizeof(float));
		  //lorapacket.curr = f1;
		  //lorapacket.i_charger = f1;
		  memcpy(&f2, can_rx_pkg.data + sizeof(float), sizeof(float));
		  //lorapacket.volt = f2;
		  lorapacket.v_charger = f2;
	  }
	  if(can_rx_pkg.identifier == 8){					// Painel 1
		  float f1, f2;
		  memcpy(&f1, can_rx_pkg.data, sizeof(float));
		  //lorapacket.curr = f1;
		  //lorapacket.i_p1 = f1;
		  memcpy(&f2, can_rx_pkg.data + sizeof(float), sizeof(float));
		  //lorapacket.volt = f2;
		  if(f2 < 3.3)
			  f2 = f2*10;
		  lorapacket.v_p1 = f2;
	  }
	  if(can_rx_pkg.identifier == 7){					// Painel 2
		  float f1, f2;
		  memcpy(&f1, can_rx_pkg.data, sizeof(float));
		  //lorapacket.curr = f1;
		  //lorapacket.i_p2 = f1;
		  memcpy(&f2, can_rx_pkg.data + sizeof(float), sizeof(float));
		  //lorapacket.volt = f2;
		  if(f2 < 3.3)
		  		f2 = f2*10;
		  lorapacket.v_p2 = f2;
	  }
	  continue;
	  /*
	  else if(can_rx_pkg.identifier == VARELA_BRD_ID){	//Varela Message
		  uint8_t ina_id = can_rx_pkg.data[6];

		  if(can_rx_pkg.data[7] == 1){						//shuntvoltage
			  memcpy(&(lorapacket.ina[ina_id].shuntvoltage), can_rx_pkg.data, sizeof(float));
		  }
		  else if(can_rx_pkg.data[7] == 2){					//busvoltage
			  memcpy(&(lorapacket.ina[ina_id].busvoltage), can_rx_pkg.data, sizeof(float));
		  }
		  else if(can_rx_pkg.data[7] == 3){					//current_mA
			  memcpy(&(lorapacket.ina[ina_id].current_mA), can_rx_pkg.data, sizeof(float));
		  }
		  else if(can_rx_pkg.data[7] == 4){					//power_mW
			  memcpy(&(lorapacket.ina[ina_id].power_mW), can_rx_pkg.data, sizeof(float));
		  }
		  else if(can_rx_pkg.data[7] == 5){					//loadvoltage
			  memcpy(&(lorapacket.ina[ina_id].loadvoltage), can_rx_pkg.data, sizeof(float));
		  }

	  }
	  else if(can_rx_pkg.identifier == ALVES_BRD_ID){	//Alves Message

	  }
	  else if(can_rx_pkg.identifier == COMIN_BRD_ID){	//Comin Message

	  }
	  */


	  //Teste LoRa com funcoes proprias
	  uint8_t teste[32] = "Teste";
	  loraTX.sendPacket(teste, sizeof(teste));
	  loraRX.readPacket(RxData, sizeof(RxData));

	  t1.a = counter;
	  t1.b = -75;
	  t1.c = '#';
	  t1.d = 128;
	  t1.e = RxData;

	  loraTX.sendPacket(&t1, sizeof(t1));
	  loraRX.readPacket(&t2, sizeof(t2));

	  //Transmissao de pacote
	  printf("Sending packet: %d\n", counter);

	  // send packet
	  loraTX.beginPacket();
	  loraTX.print("alo ");
	  loraTX.print(counter);
	  loraTX.print("\n");
	  loraTX.print("LF working");
	  loraTX.endPacket();

	  counter++;

	  //Recepcao de pacote
	  packetSize = loraRX.parsePacket();

	  if (packetSize) {
	  	printf("Recieved LoRa Packet:\n");
	  }

	  while (loraRX.available()) {
	  	//printf("%c", (char)loraRX.read());
		RxData[RxDataLength++] = (char)loraRX.read();
	  }
	  HAL_Delay(1500);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
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
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RESET_COMM_RX_Pin|SS_COMM_RX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_COMM_TX_GPIO_Port, RESET_COMM_TX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SS_COMM_TX_GPIO_Port, SS_COMM_TX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEPLOY_GPIO_Port, DEPLOY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RESET_COMM_RX_Pin SS_COMM_RX_Pin */
  GPIO_InitStruct.Pin = RESET_COMM_RX_Pin|SS_COMM_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_RX_Pin */
  GPIO_InitStruct.Pin = DIO0_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TEST_LED_Pin RESET_COMM_TX_Pin */
  GPIO_InitStruct.Pin = TEST_LED_Pin|RESET_COMM_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SS_COMM_TX_Pin */
  GPIO_InitStruct.Pin = SS_COMM_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SS_COMM_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_COMM_TX_Pin */
  GPIO_InitStruct.Pin = DIO0_COMM_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_COMM_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DEPLOY_Pin */
  GPIO_InitStruct.Pin = DEPLOY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEPLOY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	if(GPIO_Pin == DIO0_COMM_TX_Pin){
		loraTX.onDio0Rise();
	}
	if(GPIO_Pin == DIO0_RX_Pin){
		loraRX.onDio0Rise();
	}
}

void atualizaCAN(Z_CAN_Package can_rx_pkg, LoRaPacket* lorapacket){
	  if(isEqual(can_rx_pkg, NULL_MSG)){				//Invalid Message
		  return;
	  }

	  if(can_rx_pkg.identifier == 1){
		  float f;
	  	  memcpy(&f, can_rx_pkg.data, sizeof(float));
	  	  //lorapacket.inter = f;

	  }

	  if(can_rx_pkg.identifier == 3){					// Bateria
		  float f1, f2;
		  memcpy(&f1, can_rx_pkg.data, sizeof(float));
		  //lorapacket.curr = f1;
		  //lorapacket.i_bat = f1;
		  memcpy(&f2, can_rx_pkg.data + sizeof(float), sizeof(float));
		  //lorapacket.volt = f2;
		  lorapacket->v_bat = f2;
	  }
	  if(can_rx_pkg.identifier == 2){					// Charger
		  float f1, f2;
		  memcpy(&f1, can_rx_pkg.data, sizeof(float));
		  //lorapacket.curr = f1;
		  //lorapacket.i_charger = f1;
		  memcpy(&f2, can_rx_pkg.data + sizeof(float), sizeof(float));
		  //lorapacket.volt = f2;
		  lorapacket->v_charger = f2;
	  }
	  if(can_rx_pkg.identifier == 8){					// Painel 1
		  float f1, f2;
		  memcpy(&f1, can_rx_pkg.data, sizeof(float));
		  //lorapacket.curr = f1;
		  //lorapacket.i_p1 = f1;
		  memcpy(&f2, can_rx_pkg.data + sizeof(float), sizeof(float));
		  //lorapacket.volt = f2;
		  if(f2 < 3.3)
			  f2 = f2*10;
		  lorapacket->v_p1 = f2;
	  }
	  if(can_rx_pkg.identifier == 7){					// Painel 2
		  float f1, f2;
		  memcpy(&f1, can_rx_pkg.data, sizeof(float));
		  //lorapacket.curr = f1;
		  //lorapacket.i_p2 = f1;
		  memcpy(&f2, can_rx_pkg.data + sizeof(float), sizeof(float));
		  //lorapacket.volt = f2;
		  if(f2 < 3.3)
		  		f2 = f2*10;
		  lorapacket->v_p2 = f2;
	  }
}

float tempADC(ADC_HandleTypeDef* phadc, float R0){
	//Internal Variables
	uint32_t adc_val;
	float V;
	float R;
	float T;

	//Converting Constants
	const float B = 3900.0;
	const float T0 = 298.15;
	const float R_div = 100.0;
	const float V_ref = 3.3;

	//Read adc voltage
	HAL_ADC_Start(phadc);
	if(HAL_ADC_PollForConversion(phadc, 100) == HAL_OK){
		adc_val = HAL_ADC_GetValue(phadc);
		V = (float)((float)adc_val*3.3)/4095.0;
	}
	HAL_ADC_Stop(phadc);

	//Convert Voltage into Resistance
	R = (V * R_div)/(V_ref - V);

	//Convert Resistance into Temperature
	T = (T0 * B)/(T0*log(R/R0) + B);

	//Kelvin to Celsius
	T =  T - 273.15;

	return T;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
