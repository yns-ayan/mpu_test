/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "math.h"
#include "mpu6050.h"
//#include "mpu_alt.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define G_MPS2				9.8100000000f
#define RAD_TO_DEG			57.2957795131f

#define COMP_FILTER_GAIN	0.95f

#define SAMPLE_TIME_MS_USB	10

#define PI					3.141592f
#define pi					3.141592f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
float LPF(float uk,float fCutoff,float tSample_time);
float HPF(float uk,float fCutoff,float tSample_time);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float fPhi_Hat_rad = 0.0f;
float fTheta_Hat_rad = 0.0f;
float fData_to_Be_Proccessed_Phi = 0.0f;
float fData_to_Be_Proccessed_Theta = 0.0f;
SensorData raw;
SensorData offsetRaw;
SensorData processedVal;
float dt = SAMPLE_TIME_MS_USB;
float fPhi_hat_acc_rad;
float fTheta_hat_acc_rad;
float fPhi_hat_rad;
float fTheta_hat_rad;
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
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(10);

  HAL_StatusTypeDef mpuState=1;

  while(mpuState != HAL_OK)
  {
	  mpuState = mpuInit(hi2c3);
	  HAL_Delay(100);
  }

  mpuCalibrate(hi2c3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		mpuRead(hi2c3);
		mpuProcessed();
		raw = sensorRaw;
		offsetRaw.aX = offset.aX;
		offsetRaw.aY = offset.aY;
		offsetRaw.aZ = offset.aZ;
		offsetRaw.gX = offset.gX;
		offsetRaw.gY = offset.gY;
		offsetRaw.gZ = offset.gZ;
		processedVal = sensorProcessed;
		__NOP();
		__NOP();

		// dataları çekeceğin registerları linkle
		float fGyroData_p = sensorProcessed.gX;
		float fGyroData_q = sensorProcessed.gY;

		float fAcceloData_x = sensorProcessed.aX;
		float fAcceloData_y = sensorProcessed.aY;
		float fAcceloData_z = sensorProcessed.aZ;


		// converting accelometer angles
		fPhi_hat_acc_rad = (atan2(fAcceloData_y,fAcceloData_z)+PI)*RAD_TO_DEG;
		fTheta_hat_acc_rad = (atan2(fAcceloData_x,fAcceloData_z)+PI)*RAD_TO_DEG;

		// complementary filter denklemleri
		// phi_hat(k+1) = (1-alpha)*(phi_hat(k)+dt*phi_dot(k))+alpha*(phi_hat_accelometer)
		fPhi_hat_rad = (1.0f-COMP_FILTER_GAIN)*(fPhi_hat_rad+(dt)*fGyroData_p)+
		COMP_FILTER_GAIN*fPhi_hat_acc_rad;
	
		// theta_hat(k+1) = (1-alpha)*(theta_hat(k)+dt*theta_dot(k))+alpha*(theta_hat_accelometer)
		fTheta_hat_rad = (1.0f-COMP_FILTER_GAIN)*(fTheta_hat_rad+(dt)*fGyroData_q)+
		COMP_FILTER_GAIN*fTheta_hat_acc_rad; // */
	  
	// complimentary filter equations
//	fPhi_Hat_rad = (COMP_FILTER_GAIN*fPhi_hat_acc_rad)+
//	(1-COMP_FILTER_GAIN)*(fPhi_Hat_rad+fGyroData_p*SAMPLE_TIME_MS_USB);
//	fTheta_Hat_rad = (COMP_FILTER_GAIN*fTheta_hat_acc_rad)+(
//	1-COMP_FILTER_GAIN)*(fTheta_Hat_rad+fGyroData_q*SAMPLE_TIME_MS_USB);

//	fData_to_Be_Proccessed_Phi = (COMP_FILTER_GAIN*fGyroData_p+ fPhi_hat_acc_rad);

//	fPhi_Hat_rad = LPF(fData_to_Be_Proccessed_Phi,COMP_FILTER_GAIN,SAMPLE_TIME_MS_USB);

//	fData_to_Be_Proccessed_Theta = (COMP_FILTER_GAIN*fGyroData_q+ fTheta_hat_acc_rad);

//	fTheta_Hat_rad = LPF(fData_to_Be_Proccessed_Theta,COMP_FILTER_GAIN,SAMPLE_TIME_MS_USB);

	 HAL_Delay(10);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
	__HAL_RCC_I2C1_CLK_ENABLE();
	HAL_Delay(10);
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */
	__HAL_RCC_I2C3_CLK_ENABLE();
	HAL_Delay(10);
  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : mpu_interrupt_Pin */
  GPIO_InitStruct.Pin = mpu_interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(mpu_interrupt_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

float LPF(float uk,float fCutoff,float tSample_time)
{
	static volatile float yk = 0; // Previous value of the output
	static volatile float yk1 = 0; // Previous value of the output
	static volatile float uk_1 = 0; // Previous value of the input

	float tau;
	float a;
	float b;

	// G(s) = 1/(tau*s+1) // tau = 1/(2*pi/cutoff)

	tau = 1/(2*pi*fCutoff);

	a = exp(-tSample_time/tau);
	b = 1-a;

	yk1 = a*yk+b*uk_1;

	yk = yk1;
	uk_1 = uk;

	return yk1;

}

float HPF(float uk,float fCutoff,float tSample_time)
{
	static volatile float yk = 0; // Previous value of the output
	static volatile float yk1 = 0; // Previous value of the output
	static volatile float uk_1 = 0; // Previous value of the input

	float tau;
	float a;

	// G(s) = tau/(tau*s+1) // tau = 1/(2*pi/cutoff)

	tau = 1/(2*pi*fCutoff);

	a = exp(-tSample_time/tau);

	yk1 = a*yk+uk_1-uk;

	yk = yk1;
	uk_1 = uk;

	return yk1;

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
