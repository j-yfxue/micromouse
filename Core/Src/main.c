/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "Motor_Encoder.h"
#include "lpf.h"
#include "pid.h"
#include "lsm6dsox.h"
#include "IR_driver.h"
#include "solver.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

encoder left_enc;
encoder right_enc;
PID     pid_left;
PID     pid_right;



//Gyroscope Define
uint8_t lsm6dsox_buf[12] = { 0 };
#define M_PI 3.14159265358979323846f
lsm6dsox_t lsm6dsox;
HAL_StatusTypeDef status;
int i = 0;
int deg_z = 0;
float ax, ay, az;
float ax_g, ay_g, az_g;
float gx_dps,gy_dps, gz_dps;


//////////////////////////


volatile int  left_wall;
volatile int  right_wall;
//uint32_t la_wall;
//uint32_t ra_wall;
volatile int front;



volatile uint8_t pid_update_flag = 0; // set in TIM6 ISR

#define COUNTS_PER_REV  1400.0f

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
    	pid_update_flag = 1;
    }
//    } else if (htim->Instance == TIM7) {
//    	sampling_flg = 1;
//    }
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef lsm6dsox_enable_accel(lsm6dsox_t *dev) {
    // CTRL1_XL: ODR=104Hz (0x4), FS=±2g (0x0), LPF1=ODR/4 (0x1)
    uint8_t data = 0x41; // 01000001
    HAL_StatusTypeDef status = lsm6dsox_write(dev, REG_CTRL1_XL, data);
    if (status != HAL_OK) return status;

    // CTRL8_XL: Enable LPF2 with bandwidth=ODR/4 (26Hz)
    data = 0x02; // 00000010
    return lsm6dsox_write(dev, REG_CTRL8_XL, data);
}

// Gyroscope: 104Hz, ±250dps, LPF1 enabled
HAL_StatusTypeDef lsm6dsox_enable_gyro(lsm6dsox_t *dev) {
    // CTRL2_G: ODR=104Hz (0x4), FS=250dps (0x0)
    uint8_t data = 0x44; // 01000100 (ODR=104Hz, FS=250dps)
    HAL_StatusTypeDef status = lsm6dsox_write(dev, REG_CTRL2_G, data);
    if (status != HAL_OK) return status;

    // CTRL6_C: Gyro LPF1 bandwidth=ODR/4 (26Hz)
    data = 0x00; // FTYPE[1:0]=00 (LPF1 bandwidth=ODR/4)
    return lsm6dsox_write(dev, REG_CTRL6_C, data);
}


void moveForward() {
	HAL_Delay(2000);
	float displacement = 0;
	LPF_Coef coeffs_left = calculate_lpf_coefficients(10.0, 1000.0, 0.707);
	LPF_Coef coeffs_right= calculate_lpf_coefficients(10.0, 1000.0, 0.707);
	LPF_State state_left = {0, 0, 0, 0};
	LPF_State state_right = {0, 0, 0, 0};
	Encoder_Reset(&left_enc);
	Encoder_Reset(&right_enc);

	float angle_deg = 0;
	int angle = 0;
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while (displacement < 43){
		if (pid_update_flag){

			pid_update_flag = 0;

			Encoder_Update(&right_enc, &htim4, 0.001, COUNTS_PER_REV, 1);
			Encoder_Update(&left_enc,  &htim1, 0.001, COUNTS_PER_REV, 1);
			float left_rpm = left_enc.rpm;
			float right_rpm = right_enc.rpm;
			float left = lpf_process(left_rpm, &state_left, coeffs_left.b0, coeffs_left.b1, coeffs_left.b2, coeffs_left.a1, coeffs_left.a2);
			float right = lpf_process(right_rpm, &state_right, coeffs_right.b0, coeffs_right.b1, coeffs_right.b2, coeffs_right.a1, coeffs_right.a2);
			float setval = 200;
			displacement = displacement + (0.5* (left + right) * 13.195 / 60) * 0.001;

			PID_Update(&pid_left, setval , left,  0.001);
			PID_Update(&pid_right, setval + 1.7 , right, 0.001);


			Motor_SetMotorPWM(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, pid_left.output);
			Motor_SetMotorPWM(&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, pid_right.output);
		}
	}


    Motor_SetMotorPWM(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, 0);
	Motor_SetMotorPWM(&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, 0);
}

void turnRight() {
	HAL_Delay(2000);
	float angle_deg = 0;
	int angle = 0;
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while (angle < 91) {
		if (pid_update_flag){

			status = lsm6dsox_read(&lsm6dsox, REG_OUTX_L_G, 6);
			if (status == HAL_OK) {
				gz_dps = getGZ_dps(lsm6dsox_buf);

				angle_deg += (gz_dps *0.002);
				angle = (int) wrap_angle(angle_deg);

			}

			Motor_SetMotorPWM(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, -70);
			Motor_SetMotorPWM(&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, 70);
		}

	}
    Motor_SetMotorPWM(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, 0);
	Motor_SetMotorPWM(&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, 0);

}
void turnLeft() {
	HAL_Delay(2000);
	float angle_deg = 0;
	int angle = 0;
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while (angle > -92) {
		if (pid_update_flag){

			status = lsm6dsox_read(&lsm6dsox, REG_OUTX_L_G, 6);
			if (status == HAL_OK) {
				gz_dps = getGZ_dps(lsm6dsox_buf);

				angle_deg += (gz_dps *0.002);
				angle = (int) wrap_angle(angle_deg);

			}
			Motor_SetMotorPWM(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, 70);
			Motor_SetMotorPWM(&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, -70);
		}

	}
    Motor_SetMotorPWM(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, 0);
	Motor_SetMotorPWM(&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, 0);

}


void decisions(Action a) {
    switch (a) {
        case LEFT:
            turnLeft();
            break;
        case RIGHT:
            turnRight();
            break;
        case FORWARD:
            moveForward();
            break;
        case TURNAROUND:
            turnLeft();
            turnLeft();
            break;
        default:
            // Handle unexpected cases
            break;
    }
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  // Zero PWM initially


  // Start encoders (Quadrature mode)
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // right motor?
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // left motor?

  // Start periodic base timer for the PID loop
  HAL_TIM_Base_Start_IT(&htim6);

	PID_Init(&pid_left,  1.0f, 0.00001f, 0.0001f, 0.0f);
	PID_Init(&pid_right, 1.0f, 0.00001f, 0.0001f, 0.0f);
//	HAL_TIM_Base_Start_IT(&htim2)

	//kp = 1.17, kd = 0, ki = 0.001



//Gyro init
	  status = lsm6dsox_init(&lsm6dsox, &hi2c1, lsm6dsox_buf, 12);
	  if (status != HAL_OK) {
	    Error_Handler();
	  }
	  status = lsm6dsox_enable_accel(&lsm6dsox);
	  if (status != HAL_OK) Error_Handler();
	  status = lsm6dsox_enable_gyro(&lsm6dsox);
	  if (status != HAL_OK) Error_Handler();

/////////

	while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET){

		//using the push button as a start

	  Motor_SetPWM(&htim3, TIM_CHANNEL_1, 0);
	  Motor_SetPWM(&htim3, TIM_CHANNEL_2, 0);
	  Motor_SetPWM(&htim3, TIM_CHANNEL_3, 0);
	  Motor_SetPWM(&htim3, TIM_CHANNEL_4, 0);

	}

	generateInitialWalls();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Action a = solver();
      if (a == IDLE) {
          printf("Solver is idle; done.\n");
          break;
      }
	  decisions(a);



//				uint32_t time_ms = HAL_GetTick();
//				char msg[64];
//				int len = snprintf(msg, sizeof(msg), "%1u,%.2f\r\n", time_ms, left);
//				HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);



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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */


  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */

/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 FRONT_IR_Pin RIGHT_IR_Pin LEFT_IR_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|FRONT_IR_Pin|RIGHT_IR_Pin|LEFT_IR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



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
