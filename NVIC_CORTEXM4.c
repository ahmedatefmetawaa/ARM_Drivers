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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define NVIC_ISER0 (*((volatile uint32_t *)(0xE000E100)))
#define NVIC_ISER1 (*((volatile uint32_t *)(0xE000E104)))
#define NVIC_ISER2 (*((volatile uint32_t *)(0xE000E108)))
#define NVIC_ISER3 (*((volatile uint32_t *)(0xE000E10c)))
#define NVIC_ISER4 (*((volatile uint32_t *)(0xE000E110)))
#define NVIC_ISER5 (*((volatile uint32_t *)(0xE000E114)))
#define NVIC_ISER6 (*((volatile uint32_t *)(0xE000E118)))
#define NVIC_ISER7 (*((volatile uint32_t *)(0xE000E11c)))

#define NVIC_ICER0 (*((volatile uint32_t *)(0xE000E180)))
#define NVIC_ICER1 (*((volatile uint32_t *)(0xE000E184)))
#define NVIC_ICER2 (*((volatile uint32_t *)(0xE000E188)))
#define NVIC_ICER3 (*((volatile uint32_t *)(0xE000E18C)))
#define NVIC_ICER4 (*((volatile uint32_t *)(0xE000E190)))
#define NVIC_ICER5 (*((volatile uint32_t *)(0xE000E194)))
#define NVIC_ICER6 (*((volatile uint32_t *)(0xE000E198)))
#define NVIC_ICER7 (*((volatile uint32_t *)(0xE000E19C)))

#define NVIC_ISPR0 (*((volatile uint32_t *)(0xE000E200)))
#define NVIC_ISPR1 (*((volatile uint32_t *)(0xE000E204)))
#define NVIC_ISPR2 (*((volatile uint32_t *)(0xE000E208)))
#define NVIC_ISPR3 (*((volatile uint32_t *)(0xE000E20C)))
#define NVIC_ISPR4 (*((volatile uint32_t *)(0xE000E210)))
#define NVIC_ISPR5 (*((volatile uint32_t *)(0xE000E214)))
#define NVIC_ISPR6 (*((volatile uint32_t *)(0xE000E218)))
#define NVIC_ISPR7 (*((volatile uint32_t *)(0xE000E20C)))
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	NVIC_ISER1 |= (1 << 7);       // enable the interrupt request line of NVIC
	NVIC_ISPR1 |= (1 << 7);      // set the pending state (39 % 32)
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/*execute your ISR */
void USART3_IRQHandler(void)
{
	while(1)
	{

	}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
