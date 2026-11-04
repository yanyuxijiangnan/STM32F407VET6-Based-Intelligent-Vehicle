/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_system.h"
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
static int uart3_putc(int ch)
{
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);
  return ch;
}

int fputc(int ch, FILE *f)
{
  (void)f;
  return uart3_putc(ch);
}

int __io_putchar(int ch)
{
  return uart3_putc(ch);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == &huart2)
	{
		HAL_UART_Transmit_DMA(&huart2,receiveData,Size);
		
		if(receiveData[0] == 0xAA)
		{
			if(receiveData[1] == Size)
			{
				uint8_t sum = 0;
				for(int i = 0;i < Size - 1;i++)
				{ 
					sum += receiveData[i];
				}
				
				if(sum == receiveData[Size - 1])
				{
					for(int i = 2;i < Size - 1;i += 1)
					{
						if(receiveData[i] == 0x01)
						{
							Motor_forward(50);
						}
						else if(receiveData[i] == 0x02)
						{
							Motor_backward(50);
						}
						else if(receiveData[i] == 0x03)
						{
							Motor_Coast();
						}
						else if(receiveData[i] == 0x04)
						{
							Motor_brake();
						}						
					}
				}
			}
		}
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2,receiveData,sizeof(receiveData));
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	OLED_NewFrame();
	OLED_DrawImage(0,0,&user_Img,OLED_COLOR_NORMAL);
	OLED_ShowFrame();
	HAL_Delay(1000);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,receiveData,sizeof(receiveData));
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);

	Encoder_Init();
	PT1Filter_InitWithFreq(&motor_pt1,5,100);
	PT1Filter_InitWithFreq(&motor_pt2,5,100);
	PT1Filter_InitWithFreq(&motor_pt3,5,100);
	PT1Filter_InitWithFreq(&motor_pt4,5,100);

	PID_init(&motor_pid1, 1.2f, 0.0f, 0.0f, 0.01f, 50.0f);
	PID_init(&motor_pid4, 1.2f, 0.0f, 0.0f, 0.01f, 50.0f);

	Motor_Init();
	CCD_Init();
	LineFollow_Init();
	Motor_SetLR(0, 0);

	/* start TIM7 after motor/PID ready */
	HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		loop();

		/* CCD in main loop so TIM7 speed loop stays short */
		if (CCD_ReadFrame() == HAL_OK)
		{
			CCD_ProcessFrame();
			LineFollow_Update();
		}

		printf("%u,%u,%u,%.1f,%u,%u\r\n",
			(unsigned)CCD_LineValid,
			(unsigned)CCD_Center,
			(unsigned)line_vL,
			(double)line_wz,
			(unsigned)line_vR,
			(unsigned)motor_cmd_L);
		HAL_Delay(20);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
#ifdef USE_FULL_ASSERT
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
