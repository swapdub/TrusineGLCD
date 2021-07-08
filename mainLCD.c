/*
 * main.c
 *
 *  Created on: Jun 15, 2019
 *      Author: Swapnil
 */


#include "stm32f2xx_gpio.h"
#include "main.h"
/*
#define PE11
void LCDinit()
{

}

void mainLCD()
{
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	GPIOC -> MODER |= GPIO_MODER_MODER6_0;
	GPIOC -> OTYPER &= ~(GPIO_OTYPER_OT_6);
	GPIOC -> OSPEEDR != GPIO_OSPEEDER_OSPEEDR6;
	GPIOC -> PUPDR &= ~(GPIO_PUPDR_PUPDR6);

	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	GPIOE -> MODER |= 0x55555555;
	GPIOE -> OTYPER &= ~(0x0000FFFF);
	GPIOE -> OSPEEDR != 0x00000000;  //GPIO_OSPEEDER_OSPEEDR6;
	GPIOE -> PUPDR &= ~0x55555555; //(GPIO_PUPDR_PUPDR6);
}
*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);
	  HAL_Delay(100);
  }

}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_CLK_Pin|LED_LATCH_Pin|LED_DS_Pin|LCD_D0_Pin
                          |LCD_D1_Pin|LCD_CLK_Pin|LCD_EN_Pin|LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_CLK_Pin LED_LATCH_Pin LED_DS_Pin LCD_D0_Pin
                           LCD_D1_Pin LCD_CLK_Pin LCD_EN_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = LED_CLK_Pin|LED_LATCH_Pin|LED_DS_Pin|LCD_D0_Pin
                          |LCD_D1_Pin|LCD_CLK_Pin|LCD_EN_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}
