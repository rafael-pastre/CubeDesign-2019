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
#include "Adafruit_BNO055.h"
#include "zenith_can_lib.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
Z_CAN_Package can_rx_pkg;
Z_CAN_Package can_tx_pkg;

typedef union _converter4{
	float convert_float;
	uint32_t convert_int;
} Converter4;

Converter4 c;

//VARIAVEIS DE CONTROLE
int estado = 0;

float q0, q1, q2, qx, qy;

float q0_vel = 1.265; //q0_vel = 1.493; // Primeira lenta, segunda rápida. Lembrar de atualizar q1 e q2 se for mudar
float q1_vel = 1.265*(-1.908), q2_vel = 1.265*0.9105;
float qx_vel = -1.872, qy_vel = 0.872;

float q0_pos = 1.741, q1_pos = -1.713, q2_pos = 0;
float qx_pos = -0.949, qy_pos = 0;

float e, e0, e00;
float u, u0, u00;

float Vcc;
float dutyCicle;
float saidaDesejada, saidaAtual;

float T0 = 0.04;
float dir;
float T0_PWM = 1000;

float pos_IMU, pos_IMU_Anterior;
float vel_IMU;

int init_flag = 1;
//CONFIGURACOES DO TIMER PWM
TIM_OC_InitTypeDef configPWM_17 = {0};
TIM_OC_InitTypeDef configPWM_16 = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
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
  MX_CAN1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  // Inicialização padrão dos PWMs; para funcionar, falta os starts.
  HAL_TIM_PWM_Init(&htim16);
  HAL_TIM_PWM_Init(&htim17);

  // Inicialização dos timers
  HAL_TIM_Base_Start_IT(&htim2);

  // Configuracao inicial dos timer
  configPWM_16.OCMode = TIM_OCMODE_PWM1;
  configPWM_16.Pulse = 0; // Largura do pulso
  configPWM_16.OCPolarity = TIM_OCPOLARITY_HIGH;
  configPWM_16.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  configPWM_16.OCFastMode = TIM_OCFAST_DISABLE;
  configPWM_16.OCIdleState = TIM_OCIDLESTATE_RESET;
  configPWM_16.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  configPWM_17.OCMode = TIM_OCMODE_PWM1;
  configPWM_17.Pulse = 0; // Largura do pulso
  configPWM_17.OCPolarity = TIM_OCPOLARITY_HIGH;
  configPWM_17.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  configPWM_17.OCFastMode = TIM_OCFAST_DISABLE;
  configPWM_17.OCIdleState = TIM_OCIDLESTATE_RESET;
  configPWM_17.OCNIdleState = TIM_OCNIDLESTATE_RESET;

/*
  double xPos = 0, yPos = 0, headingVel = 0;
  uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
  uint16_t PRINT_DELAY_MS = 500; // how often to print the data
  uint16_t printCount = 0; //counter to avoid printing every 10MS sample
  double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
  double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
  double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

  Adafruit_BNO055 bno = Adafruit_BNO055(hi2c2, 55, BNO055_ADDRESS_A);

  bno.begin();
*/
  // Initializing CAN Filter
  filterConfigCAN(&hcan1);
  HAL_CAN_Start(&hcan1);

  //Delay para configurar IMU
  HAL_Delay(5000);

  //Forca dados iniciais para pos_IMU e Vcc
  can_rx_pkg.identifier = 0;
  while(can_rx_pkg.identifier != 2){
	  can_rx_pkg = readCanMessages(&hcan1);
  }

  memcpy(&pos_IMU, can_rx_pkg.data, sizeof(float));
  memcpy(&Vcc, can_rx_pkg.data + sizeof(float), sizeof(float));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//Leitura CAN
	can_rx_pkg = readCanMessages(&hcan1);

	if(isEqual(can_rx_pkg, NULL_MSG)){				//Invalid Message
	}
	else if(can_rx_pkg.identifier == 1){

	}
	else if(can_rx_pkg.identifier == 2){
		memcpy(&pos_IMU, can_rx_pkg.data, sizeof(float));
		memcpy(&Vcc, can_rx_pkg.data + sizeof(float), sizeof(float));
	}

	//Para motores
	if(estado == 0) {
		// Stop nos dois PWMs; motor em repouso
		HAL_TIM_PWM_Stop(&htim16,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);

		init_flag = 1;
	}


	/*
	  sensors_event_t orientationData , linearAccelData;
	  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
	  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
	  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

	  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
	  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

	  // velocity of sensor in the direction it's facing
	  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);
	  HAL_Delay(10);
*/
	  // send CAN message
	  /*
	  c.convert_float = orientationData.orientation.x;
	  can_tx_pkg.data[0] = c.convert_int;
	  c.convert_float = orientationData.orientation.y;
	  can_tx_pkg.data[1] = c.convert_int;
	  c.convert_float = orientationData.orientation.z;
	  can_tx_pkg.data[2] = c.convert_int;
	  c.convert_float = linearAccelData.acceleration.x;
	  can_tx_pkg.data[3] = c.convert_int;
	  c.convert_float = linearAccelData.acceleration.y;
	  can_tx_pkg.data[4] = c.convert_int;
	  c.convert_float = linearAccelData.acceleration.z;
	  can_tx_pkg.data[5] = c.convert_int;
	  can_tx_pkg.data[6] = 0;
	  can_tx_pkg.data[7] = 0;
	  can_tx_pkg.identifier = 0;

	  sendCanMessage(&hcan1, can_tx_pkg);
	  */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 40000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 80;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 40;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 40;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1000;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin ENABLE_Pin */
  GPIO_InitStruct.Pin = LED_Pin|ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	TIM_OC_InitTypeDef configPWM_16 = {0};
	TIM_OC_InitTypeDef configPWM_17 = {0};

	configPWM_16.OCMode = TIM_OCMODE_PWM1;
	configPWM_16.Pulse = 0; // Largura do pulso
	configPWM_16.OCPolarity = TIM_OCPOLARITY_HIGH;
	configPWM_16.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	configPWM_16.OCFastMode = TIM_OCFAST_DISABLE;
	configPWM_16.OCIdleState = TIM_OCIDLESTATE_RESET;
	configPWM_16.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	configPWM_17.OCMode = TIM_OCMODE_PWM1;
	configPWM_17.Pulse = 0; // Largura do pulso
	configPWM_17.OCPolarity = TIM_OCPOLARITY_HIGH;
	configPWM_17.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	configPWM_17.OCFastMode = TIM_OCFAST_DISABLE;
	configPWM_17.OCIdleState = TIM_OCIDLESTATE_RESET;
	configPWM_17.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	if(htim->Instance == TIM2) { // Subrotina de interrupção do timer 2
		// Atualizar ação de controle: lembrar de girar para o sentido oposto


		//Maquina de estados
		if(estado == 0) {
			// Stop nos dois PWMs; motor em repouso
			HAL_TIM_PWM_Stop(&htim16,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);

			init_flag = 1;
		}
		if(estado == 1) {
			q0 = q0_vel;
			q1 = q1_vel;
			q2 = q2_vel;
			qx = qx_vel;
			qy = qy_vel;
			saidaDesejada = 0;

			// Atualizacao saida e tratamento inicializacao
			if(init_flag == 1){
				pos_IMU_Anterior = pos_IMU;

				u0 = 0; u00 = 0;
				e0 = 0; e00 = 0;

				init_flag = 0;
			}
			else{
				vel_IMU = (pos_IMU - pos_IMU_Anterior)/T0;
				if(abs(vel_IMU) > 800){
					vel_IMU = ((360-pos_IMU_Anterior) + pos_IMU)/T0;
					pos_IMU_Anterior = pos_IMU;
					return;
				}

				if(vel_IMU > 0){
					dir = 0;
				}
				else{
					dir = 0;
				}
				saidaAtual = vel_IMU;
				pos_IMU_Anterior = pos_IMU;
			}

		}

		if(estado == 2) {
			if(init_flag == 1){
				u0 = 0; u00 = 0;
				e0 = 0; e00 = 0;

				init_flag = 0;
			}

			q0 = q0_pos;
			q1 = q1_pos;
			q2 = 0;
			qx = qx_pos;
			qy = 0;

			saidaAtual = pos_IMU;
			//TODO
			saidaDesejada = 0;//posIni_IMU +;
		}

		if(estado != 0) {
			e = saidaDesejada - saidaAtual;
			u = q0*e+q1*e0+q2*e00 -qx*u0 -qy*u00;

			if(u > Vcc) {
				u = Vcc;
			}

			if(u < -Vcc) {
				u = -Vcc;
			}


			// Atualizar dutycicle
			dutyCicle += 0.01;
			if()
			//dutyCicle = u/Vcc;

			// Atualizar variáveis de memória
			e00 = e0;
			e0 = e;
			u00 = u0;
			u0 = u;

		}

		if(dutyCicle != 0) {
			//dir = 0;
			if(dir == 0) {

				configPWM_16.Pulse = dutyCicle*T0_PWM;
				configPWM_17.Pulse = 0;
				HAL_TIM_PWM_ConfigChannel(&htim16, &configPWM_16, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);

				HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);

			}

			if(dir == 1) {

				configPWM_17.Pulse = dutyCicle*T0_PWM;
				configPWM_16.Pulse = 0;
				HAL_TIM_PWM_ConfigChannel(&htim17, &configPWM_17, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);

				HAL_TIM_PWM_Stop(&htim16,TIM_CHANNEL_1);

			}

		}
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
