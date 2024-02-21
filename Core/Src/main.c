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
#include "stdio.h"
#include "stm32g4xx_hal_tim.h"
#include <stdbool.h>
/*include "delay_us.h"*/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PORT GPIOA
#define LED_PIN_B GPIO_PIN_5
#define LED_PIN_G GPIO_PIN_6
#define LED_PIN_R GPIO_PIN_7
#define DELAY 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */
volatile uint32_t millis = 0; // Variable to count milliseconds
volatile uint8_t buttonFlag = 0; // Flag to indicate button press
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t counter1;
int sek, milli_in_sek;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// check if TIMER x is the one responsible the interrupts
	if (htim->Instance == TIM17) {
		millis++; // Increment milliseconds counter
		counter1++;
		milli_in_sek++;
		if (milli_in_sek == 500) {
			sek = sek + 0.5;
			milli_in_sek = 0;
		}
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
			printf("millisec: %lu\n ", millis);
		} else {
			printf("LED OFF \r\n");
		}
		// Read button state using HAL function
		buttonFlag = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET);

	}

}

int i = 1;
void PRINT_THROW() {
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
		printf("LED OFF\n\r");
		i = 1;
	} else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
		printf("LED ON\n\r");
	}
}

void LEDS_BLINKING() {
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
		HAL_GPIO_WritePin(GPIOA, LED_PIN_R | LED_PIN_G | LED_PIN_B,
				GPIO_PIN_RESET);
	} else {

		while (i <= 1) {
			PRINT_THROW();
			++i;
		}
	}
}

void LEDS_OFF() {
	HAL_GPIO_WritePin(GPIOA, LED_PIN_R, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LED_PIN_G, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LED_PIN_B, GPIO_PIN_RESET);
}

void Modus_1() {
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
		millis = 0;
	}
}

void Modus_2() {
}

uint32_t elapsed_ms(void) {
	static uint32_t previous;
	uint32_t ms = counter1;
	uint32_t diff = ms - previous;
	previous = ms;
	return diff;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	char uart_buf[50];
	int uart_buf_len;
	/*uint16_t timer_val;*/
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
	MX_UART4_Init();
	MX_TIM17_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	// Say something
	uart_buf_len = sprintf(uart_buf, "Timer test\r\n");
	HAL_UART_Transmit(&huart4, (uint8_t*) uart_buf, uart_buf_len, 100);
	// Start timer
	HAL_TIM_Base_Init(&htim17);
	HAL_TIM_PWM_MspInit(&htim2);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	printf(uart_buf);
	PRINT_THROW();
	HAL_TIM_Base_Start_IT(&htim17);
	HAL_TIM_Base_Init();
	HAL_TIM_PWM_Init();
	HAL_TIM_PWM_ConfigChannel();
	HAL_TIM_MspPostInit(&htim2);

	uint32_t pwm_register = 255;
	bool direction = true;
	while (1) {
		elapsed_ms();
		if (buttonFlag) {
			if (millis >= 0 && millis <= 300) {
				HAL_GPIO_WritePin(GPIOA, LED_PIN_R, GPIO_PIN_SET);
			} else if (millis >= 500 && millis <= 800) {
				HAL_GPIO_WritePin(GPIOA, LED_PIN_G, GPIO_PIN_SET);
			} else if (millis >= 1000 && millis <= 1300) {
				HAL_GPIO_WritePin(GPIOA, LED_PIN_B, GPIO_PIN_SET);
			} else if (millis >= 1500 && millis <= 1800) {
				HAL_GPIO_WritePin(GPIOA, LED_PIN_R | LED_PIN_G, GPIO_PIN_SET);
			} else if (millis >= 2000 && millis <= 2300) {
				HAL_GPIO_WritePin(GPIOA, LED_PIN_R | LED_PIN_B, GPIO_PIN_SET);
			} else if (millis >= 2500 && millis <= 2800) {
				HAL_GPIO_WritePin(GPIOA, LED_PIN_G | LED_PIN_B, GPIO_PIN_SET);
			} else if (millis >= 3000 && millis <= 3300) {
				HAL_GPIO_WritePin(GPIOA, LED_PIN_R | LED_PIN_G | LED_PIN_B,
						GPIO_PIN_SET);
			} else if (millis >= 3300 && millis <= 3500) {
				HAL_GPIO_WritePin(GPIOA, LED_PIN_R | LED_PIN_G | LED_PIN_B,
						GPIO_PIN_RESET);
			} else {
				// If the button is pressed but millis is not within the desired pattern
				LEDS_OFF();
			}
		} else {
			// If the button is not pressed, turn off all LEDs
			LEDS_OFF();
		}

		// Reset millis after every 1300ms cycle
		if (millis > 3500) {
			millis = 0;
		}

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		//pwm_register = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
			if (!(TIM2->CR1 & TIM_CR1_CEN)) {
				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
			}
			if (counter1 >= DELAY) {
				if ((direction == true) && pwm_register < 255) {
					pwm_register++;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_register);
					TIM2->CCR2 = pwm_register;
					counter1 = 0;
				}
				else if ((direction == false) && pwm_register > 0) {
					pwm_register--;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_register);
					TIM2->CCR2 = pwm_register;
					counter1 = 0;
				}
				else if (pwm_register == 0) {
					direction = true;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_register);
					TIM2->CCR2 = pwm_register;
					counter1 = 0;
				}
				else if (pwm_register == 255) {
					direction = false;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_register);
					TIM2->CCR2 = pwm_register;
					counter1 = 0;
				}
				else {
					pwm_register = 0;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_register);
					TIM2->CCR2 = pwm_register;
					counter1 = 0;
				}
			}
			uint32_t pwm_duty_cycle = TIM2->CCR2;
		}
		else {
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		}
	}

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 255;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 127;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */


}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void) {

	/* USER CODE BEGIN TIM17_Init 0 */

	/* USER CODE END TIM17_Init 0 */

	/* USER CODE BEGIN TIM17_Init 1 */

	/* USER CODE END TIM17_Init 1 */
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 40000 - 1;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 1;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
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
static void MX_UART4_Init(void) {

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
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA5 PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*PUTCHAR_PROTOTYPE
 {
 HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);
 return ch;
 }*/
int __io_putchar(int ch) {
	ITM_SendChar(ch);
	return ch;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
