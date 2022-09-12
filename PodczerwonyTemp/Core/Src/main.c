/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EMISIVITY 0.985
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ambient_tmp;
uint16_t obj_tmp;
float ambient_tmp_cel;
float obj_tmp_cel;
uint8_t tmp[2];
char buf[80];
uint8_t len;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t crc8(uint8_t InCrc, uint8_t InData);
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if(EMISIVITY>0.1 && EMISIVITY<1)//Setting emisivity
  {
	    uint16_t new_emissivity = (uint16_t)round(65535.0 * EMISIVITY);
	    uint8_t Crc;
	  	uint8_t tmp_emis[3];
	  	uint8_t Address =0x24;

	  	Crc = crc8(0, (0x5A)<<1);
	  	Crc = crc8(Crc, Address);
	  	Crc = crc8(Crc, 0);
	  	Crc = crc8(Crc, 0);

	  	tmp_emis[0] = new_emissivity & 0xFF;
	  	tmp_emis[1] = new_emissivity>>8;
	  	tmp_emis[2] = Crc;
  	HAL_I2C_Mem_Write(&hi2c1, (0x5A)<<1, Address, 1, tmp_emis, 3, 10);
  	HAL_Delay(10);

	  	Crc = crc8(0, (0x5A)<<1);
	  	Crc = crc8(Crc, Address);
	  	Crc = crc8(Crc, new_emissivity & 0xFF);
	  	Crc = crc8(Crc, new_emissivity>>8);

	  	tmp_emis[0] = new_emissivity & 0xFF;
	  	tmp_emis[1] = new_emissivity>>8;
	  	tmp_emis[2] = Crc;
  	HAL_I2C_Mem_Write(&hi2c1, (0x5A)<<1, Address, 1, tmp_emis, 3, 10);
  	HAL_Delay(10); // Writing time ~5ms;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(HAL_I2C_Mem_Read(&hi2c1,(0x5A)<<1,0x06,1,tmp,2,10)==HAL_OK)
		{
		ambient_tmp = (uint16_t)tmp[1]<<8 | tmp[0];
		ambient_tmp_cel = (float)ambient_tmp * 0.02;
		ambient_tmp_cel = ambient_tmp_cel - 273.15;
		}
	  if(HAL_I2C_Mem_Read(&hi2c1,(0x5A)<<1,0x07,1,tmp,2,10)==HAL_OK)
		{
		  obj_tmp = (uint16_t)tmp[1]<<8 | tmp[0];
		  obj_tmp_cel = (float)obj_tmp * 0.02;
		  obj_tmp_cel = obj_tmp_cel - 273.15;
		}
	  len = sprintf(buf, "Ambient: %.2f\n,Object:%.2f \n\r", ambient_tmp_cel,obj_tmp_cel);
	  HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, 10);
	  HAL_Delay(500);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
uint8_t crc8(uint8_t InCrc, uint8_t InData)
{
	uint8_t i;
	uint8_t Data;
	Data = InCrc ^= InData;
	for ( i = 0; i < 8; i++ )
	{
		if (( Data & 0x80 ) != 0 )
		{
			Data <<= 1;
			Data ^= 0x07;
		}
		else
		{
			Data <<= 1;
		}
	}
	return Data;
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
