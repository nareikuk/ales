/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//SPI_HandleTypeDef SpiHandle;
typedef struct _bmp280calibration {
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
	uint16_t dig_p1;
	int16_t dig_p2;
	int16_t dig_p3;
	int16_t dig_p4;
	int16_t dig_p5;
	int16_t dig_p6;
	int16_t dig_p7;
	int16_t dig_p8;
	int16_t dig_p9;
} Bmp280Calibration;

Bmp280Calibration bmpCalibration;

//#define BUFFERSIZE 64

// Pressure differences * 25600 between start alt and height above start alt
#define ALT_100M -305995     // -305995.1386
#define ALT_150M -457893     // -457892.7898
#define ALT_200M -609061     // -609060.7514

#define ALT_CUTOFF ALT_150M    // Change this to set the altimeter cutoff height
#define VERSION "1.1"

#define RCSAMPLES 8

#define VERBOSE 1

enum
    {
      BMP280_REGISTER_DIG_T1              = 0x88,
      BMP280_REGISTER_DIG_T2              = 0x8A,
      BMP280_REGISTER_DIG_T3              = 0x8C,

      BMP280_REGISTER_DIG_P1              = 0x8E,
      BMP280_REGISTER_DIG_P2              = 0x90,
      BMP280_REGISTER_DIG_P3              = 0x92,
      BMP280_REGISTER_DIG_P4              = 0x94,
      BMP280_REGISTER_DIG_P5              = 0x96,
      BMP280_REGISTER_DIG_P6              = 0x98,
      BMP280_REGISTER_DIG_P7              = 0x9A,
      BMP280_REGISTER_DIG_P8              = 0x9C,
      BMP280_REGISTER_DIG_P9              = 0x9E,

      BMP280_REGISTER_CHIPID             = 0xD0,
      BMP280_REGISTER_VERSION            = 0xD1,
      BMP280_REGISTER_SOFTRESET          = 0xE0,

      BMP280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

      BMP280_REGISTER_CONTROL            = 0xF4,
      BMP280_REGISTER_CONFIG             = 0xF5,
      BMP280_REGISTER_PRESSUREDATA       = 0xF7,
      BMP280_REGISTER_TEMPDATA           = 0xFA,
    };

typedef int32_t BMP280_S32_t;
typedef uint32_t BMP280_U32_t;
typedef int64_t BMP280_S64_t;

BMP280_S32_t t_fine;
volatile int rxPwm = 0;
int lastCounter = 0;
int counter = 0;
int idleThrottlePeriod = 1000;
//int motorRunSeconds = 0;
uint8_t throttleActive = 0; // The throttle is allowed to be moved from idle if throttleActive is 1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t mySPI_GetChipId(void);
uint8_t mySPI_GetCalibration(void);
int32_t mySPI_GetInt20(uint8_t address);
void BMP280Init(void);
BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T);
BMP280_S64_t bmp280_pressure_diff(BMP280_S32_t adc_P, BMP280_S64_t startPa);
void Timeout_Error_Handler(void);
void itoa(int n, char s[]);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int tFilter[RCSAMPLES] = {0};
uint8_t filterCount = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	int diff = 0;

	if (htim->Instance == TIM1) {
		// Ideally we'd like to know if the interrupt was due to the rising or falling edge
		// so we can capture the pulse time on the falling edge and start the counter on
		// the rising edge. Until that is figured out, go for a simpler version where we capture
		// two edges and see if the time is short enough to have been a pulse rather than the
		// gap between pulses. A pulse will be shorter than 2500uS (probably only around 2100uS
		// at most). The gap between pulses will be at least 17500uS.
		counter = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        if (lastCounter > counter) {
        	diff = 20000 - lastCounter + counter;
        } else {
        	diff = counter - lastCounter;
        }
        lastCounter = counter;

        if (diff < 2500 && diff > 900) {   // We must have the pulse time rather than the gap between pulses
			tFilter[filterCount++] = diff;
			filterCount %= RCSAMPLES;

			int avg = 0;
			for (int i = 0; i < RCSAMPLES; i++) {
				avg += tFilter[i];
			}

			rxPwm = avg / RCSAMPLES;
        }


        // TODO: if no pulses received for some time, then set rxPwm to 0 rather than leaving
        //       it stuck at last measured value
	}
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	  int32_t temperature;
#ifdef VERBOSE
	  char strbuf[100];
#endif
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
   BMP280Init();

   mySPI_GetCalibration();

   // Start PWM_IN capture
   HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);

   // Capture idle throttle. Must be below 1200uS and above 900uS for 2 seconds
   uint8_t idleCaptured = 0;
   while (!idleCaptured) {
	   int throttle = rxPwm;
	   while (throttle > 1200 || throttle < 900) {
		   HAL_Delay(100);
		   throttle = rxPwm;
#ifdef VERBOSE
		   strcpy(strbuf, "Waiting for thr1:");
		   itoa(throttle, &strbuf[strlen(strbuf)]);
		   strcat(strbuf, "\n");
		   HAL_UART_Transmit(&huart1, (uint8_t *)strbuf, strlen(strbuf), 5000);
#endif
	   }

	   int throttlePeriod = throttle;
	   uint8_t captureTime = 0;
	   while ((throttle >= throttlePeriod && (throttle - throttlePeriod) < 25) ||
			   (throttle < throttlePeriod && (throttlePeriod - throttle) < 25)) {
		   captureTime++;
		   if (captureTime > 20) {   // Wait 2 seconds for stable throttle position
			   idleCaptured = 1;
			   idleThrottlePeriod = throttle;
			   break;
		   }
#ifdef VERBOSE
		   strcpy(strbuf, "Waiting for thr2:");
		   itoa(throttle, &strbuf[strlen(strbuf)]);
		   strcat(strbuf, "\n");
		   HAL_UART_Transmit(&huart1, (uint8_t *)strbuf, strlen(strbuf), 5000);
#endif
		   HAL_Delay(100);
		   throttle = rxPwm;
	   }
   }

   // Start PWM_OUT timer: TIM1, Ch2
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, idleThrottlePeriod); // Set throttle to idle
   throttleActive = 1;

   BMP280_S64_t startPressure = 0;
   int pressureCaptureCount = 0;
   int throttleIdleCount = 0;
   int throttleOnTime = 0;
   int loopCount = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	   HAL_GPIO_TogglePin(LOOP_TICK_GPIO_Port, LOOP_TICK_Pin);

	   // Record loop time and make sure it runs at 1mS per loop
	   uint32_t tickstart = 0;
	   tickstart = HAL_GetTick();
	   int throttle = rxPwm;

	   int32_t rawPressure = mySPI_GetInt20(BMP280_REGISTER_PRESSUREDATA);
	   int32_t rawTemperature = mySPI_GetInt20(BMP280_REGISTER_TEMPDATA);

	   temperature = bmp280_compensate_T_int32(rawTemperature);
	   temperature += 0;  // make sure temperature is used to ensure above function is called and not optimized out
	   BMP280_S64_t pressureDiff = bmp280_pressure_diff(rawPressure, startPressure);

	   if (startPressure == 0 && pressureDiff > 0) {
		   pressureCaptureCount++;
		   if (pressureCaptureCount > 1000) {
			   pressureCaptureCount = 1000;
			   startPressure = pressureDiff;  // TODO: Check if pressure has been stable for 1 second
#ifdef VERBOSE
			   strcpy(strbuf, "Start press:");
			   itoa(startPressure, &strbuf[strlen(strbuf)]);
			   strcat(strbuf, "\n");
               HAL_UART_Transmit(&huart1, (uint8_t *)strbuf, strlen(strbuf), 5000);
#endif
		   }
	   }

	   // Check pressureDiff to see if the max altitude has been reached.
	   // PressureDiff is lower (more negative) with increasing altitude
	   // TODO: Verify altitude is stable or increasing rather than a 'glitch'
	   if ((pressureDiff <= ALT_CUTOFF) && throttleActive) {
		   throttleActive = 0;    // Force ESC to idle
		   throttleOnTime = 0;    // Reset on time
#ifdef VERBOSE
		   strcpy(strbuf, "Max alt reached\n");
		   HAL_UART_Transmit(&huart1, (uint8_t *)strbuf, strlen(strbuf), 5000);
#endif
	   }

#ifdef VERBOSE
	   if (loopCount % 100 == 0) {
		   int32_t temp1 = temperature / 100;
		   int32_t temp2 = temperature - temp1 * 100;

		   strcpy(strbuf, VERSION);
		   strcat(strbuf, ",T:");
		   itoa(temp1, &strbuf[strlen(strbuf)]);
		   strcat(strbuf, ".");
		   itoa(temp2, &strbuf[strlen(strbuf)]);
		   strcat(strbuf, "C,P:");
		   itoa(pressureDiff, &strbuf[strlen(strbuf)]);
		   strcat(strbuf, ",PWMIn:");
		   itoa(throttle, &strbuf[strlen(strbuf)]);
		   strcat(strbuf, ",OnTime:");
		   itoa(throttleOnTime, &strbuf[strlen(strbuf)]);
		   strcat(strbuf, ",Active:");
		   itoa(throttleActive, &strbuf[strlen(strbuf)]);
		   strcat(strbuf, ",IdleCount:");
		   itoa(throttleIdleCount, &strbuf[strlen(strbuf)]);
		   strcat(strbuf, ",IdlePeriod:");
		   itoa(idleThrottlePeriod, &strbuf[strlen(strbuf)]);
		   strcat(strbuf, "\n");

		   HAL_UART_Transmit(&huart1, (uint8_t *)strbuf, strlen(strbuf), 5000);
	   }
#endif

	   // Check if throttle is above idle
	   if (throttleActive && throttle >= idleThrottlePeriod + 40) {
		   //startTime = getCurrentTime();
		   throttleIdleCount = 0;
		   throttleOnTime++;
		   if (throttleOnTime >= 30000) {
			   throttleActive = 0;    // Force throttle to idle if it's been on for more than 30 seconds
#ifdef VERBOSE
		       strcpy(strbuf, "Max time reached\n");
		       HAL_UART_Transmit(&huart1, (uint8_t *)strbuf, strlen(strbuf), 5000);
#endif
		   }
	   }

	   // Check for throttle going to idle during active time
	   if (throttleActive && throttle <= idleThrottlePeriod + 25) {
		   throttleActive = 0;    // Force throttle to idle
		   throttleOnTime = 0;    // Reset throttle on time
#ifdef VERBOSE
		   strcpy(strbuf, "Thr idle\n");
		   HAL_UART_Transmit(&huart1, (uint8_t *)strbuf, strlen(strbuf), 5000);
#endif
	   }

	   // Check if throttle has been disarmed and has been set to idle for more than 3 seconds
	   if (throttleActive == 0 && throttle <= idleThrottlePeriod + 25) {
		   throttleIdleCount++;
		   if (throttleIdleCount > 3000) {
			   throttleIdleCount = 3000;
			   throttleActive = 1;        // Throttle is re-armed if it's been idle for more than 3 seconds
#ifdef VERBOSE
		   strcpy(strbuf, "Thr re-armed\n");
		   HAL_UART_Transmit(&huart1, (uint8_t *)strbuf, strlen(strbuf), 5000);
#endif
		   }
	   }

	   // Pass throttle to ESC if it is armed, otherwise force ESC to idle
	   if (throttleActive) {
		   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, throttle);
	   } else {
		   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, idleThrottlePeriod);
	   }

	   loopCount++;
	   loopCount %= 1000;
	   //HAL_Delay(1);         // TODO use HAL_GetTick() to make total loop time 1 second

	   while((HAL_GetTick() - tickstart) < 1)  // 1mS delay for entire loop - ensure rest of look takes less than 1mS though!
	   {
	   }

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_EN_GPIO_Port, SPI_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LOOP_TICK_GPIO_Port, LOOP_TICK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI_EN_Pin */
  GPIO_InitStruct.Pin = SPI_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LOOP_TICK_Pin */
  GPIO_InitStruct.Pin = LOOP_TICK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LOOP_TICK_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* reverse:  reverse string s in place */
void reverse(char s[])
{
    int c, i, j;

    for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}

/* itoa:  convert n to characters in s */
void itoa(int n, char s[])
{
    int i;
    uint8_t negative = 0;

    if (n < 0) { /* record sign */
        n = -n;          /* make n positive */
        negative = 1;
    }

    i = 0;
    do {       /* generate digits in reverse order */
        s[i++] = n % 10 + '0';   /* get next digit */
    } while ((n /= 10) > 0);     /* delete it */

    if (negative) {
        s[i++] = '-';
    }

    s[i] = '\0';

    reverse(s);
}

uint8_t mySPI_GetChipId(void) {
	uint8_t txbuf[2];
	uint8_t rxbuf[2];
	HAL_SPI_StateTypeDef status;

	txbuf[0] = 0x80 | BMP280_REGISTER_CHIPID;
	txbuf[1] = 0xFF;

	rxbuf[0] = 0x00;
	rxbuf[1] = 0x00;

	HAL_GPIO_WritePin(SPI_EN_GPIO_Port, SPI_EN_Pin, GPIO_PIN_RESET);

	status = HAL_SPI_TransmitReceive(&hspi1, txbuf, rxbuf, 2, 5000);
	if (status == HAL_SPI_STATE_ERROR) {
		Error_Handler();
	}

	HAL_GPIO_WritePin(SPI_EN_GPIO_Port, SPI_EN_Pin, GPIO_PIN_SET);

	return  rxbuf[1];
}

int32_t mySPI_GetInt20(uint8_t address) {
	uint8_t txbuf[4];
	uint8_t rxbuf[4];
	HAL_SPI_StateTypeDef status;

	txbuf[0] = 0x80 | address;
	txbuf[1] = 0xFF;
	txbuf[2] = 0xFF;
	txbuf[3] = 0xFF;

	rxbuf[0] = 0x00;
	rxbuf[1] = 0x00;
	rxbuf[2] = 0x00;
	rxbuf[3] = 0x00;

	HAL_GPIO_WritePin(SPI_EN_GPIO_Port, SPI_EN_Pin, GPIO_PIN_RESET);

	status = HAL_SPI_TransmitReceive(&hspi1, txbuf, rxbuf, 4, 5000);
	if (status == HAL_SPI_STATE_ERROR) {
		Error_Handler();
	}

	HAL_GPIO_WritePin(SPI_EN_GPIO_Port, SPI_EN_Pin, GPIO_PIN_SET);

	int32_t val = rxbuf[1] << 16 | rxbuf[2] << 8 | (rxbuf[3] & 0xF0);
	return  val;
}

uint8_t mySPI_GetCalibration(void) {
	uint8_t txbuf[25];
	uint8_t rxbuf[25];
	HAL_SPI_StateTypeDef status;



	txbuf[0] = 0x80 | BMP280_REGISTER_DIG_T1;
	for (int i = 1; i < 25; i++) {
		txbuf[i] = 0xFF;
	}

	for (int i = 0; i < 25; i++) {
			rxbuf[i] = 0x00;
	}

	HAL_GPIO_WritePin(SPI_EN_GPIO_Port, SPI_EN_Pin, GPIO_PIN_RESET);

	status = HAL_SPI_TransmitReceive(&hspi1, txbuf, rxbuf, 25, 5000);
	if (status == HAL_SPI_STATE_ERROR) {
		Error_Handler();
	}

	HAL_GPIO_WritePin(SPI_EN_GPIO_Port, SPI_EN_Pin, GPIO_PIN_SET);

	bmpCalibration.dig_t1 = rxbuf[2] << 8 | rxbuf[1];
	bmpCalibration.dig_t2 = rxbuf[4] << 8 | rxbuf[3];
	bmpCalibration.dig_t3 = rxbuf[6] << 8 | rxbuf[5];
	bmpCalibration.dig_p1 = rxbuf[8] << 8 | rxbuf[7];
	bmpCalibration.dig_p2 = rxbuf[10] << 8 | rxbuf[9];
	bmpCalibration.dig_p3 = rxbuf[12] << 8 | rxbuf[11];
	bmpCalibration.dig_p4 = rxbuf[14] << 8 | rxbuf[13];
	bmpCalibration.dig_p5 = rxbuf[16] << 8 | rxbuf[15];
	bmpCalibration.dig_p6 = rxbuf[18] << 8 | rxbuf[17];
	bmpCalibration.dig_p7 = rxbuf[20] << 8 | rxbuf[19];
	bmpCalibration.dig_p8 = rxbuf[22] << 8 | rxbuf[21];
	bmpCalibration.dig_p9 = rxbuf[24] << 8 | rxbuf[23];

	return 0;
}

void BMP280Init(void) {
	uint8_t config[4];
	uint8_t rxbuf[4];
	HAL_SPI_StateTypeDef status;

	config[0] = BMP280_REGISTER_CONTROL & 0x7F; // clear write bit
	config[1] = 0b01010111;  // osrs_t[3]: 2x, osrs_p[3]: 16x, mode[2]: normal mode
	config[2] = BMP280_REGISTER_CONFIG & 0x7F;
	config[3] = 0b00011100; // t_sb[3]:0.5mS , filter[3], res[1]:0, spi3w_en[1]:disabled

	HAL_GPIO_WritePin(SPI_EN_GPIO_Port, SPI_EN_Pin, GPIO_PIN_RESET);

	status = HAL_SPI_TransmitReceive(&hspi1, config, rxbuf, 4, 5000);
	if (status == HAL_SPI_STATE_ERROR) {
		Error_Handler();
	}

	HAL_GPIO_WritePin(SPI_EN_GPIO_Port, SPI_EN_Pin, GPIO_PIN_SET);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. // t_fine carries fine temperature as global value
BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T)
{
	BMP280_S32_t var1, var2, T;
	adc_T >>= 4;
	var1 = ((((adc_T>>3) - ((BMP280_S32_t)bmpCalibration.dig_t1<<1))) * ((BMP280_S32_t)bmpCalibration.dig_t2)) >> 11;
	var2 = (((((adc_T>>4) - ((BMP280_S32_t)bmpCalibration.dig_t1)) * ((adc_T>>4) - ((BMP280_S32_t)bmpCalibration.dig_t1))) >> 12) *
			((BMP280_S32_t)bmpCalibration.dig_t3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine*5+128)>>8;
	return T;
}

#if 0
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits). // Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
double bmp280_altitude(BMP280_S32_t adc_P, double seaLevelhPa)
{
	BMP280_S64_t var1, var2, p;
	adc_P >>= 4;
	var1 = ((BMP280_S64_t)t_fine) - 128000;
	var2 = var1 * var1 * (BMP280_S64_t)bmpCalibration.dig_p6;
	var2 = var2 + ((var1*(BMP280_S64_t)bmpCalibration.dig_p5)<<17);
	var2 = var2 + (((BMP280_S64_t)bmpCalibration.dig_p4)<<35);
	var1 = ((var1 * var1 * (BMP280_S64_t)bmpCalibration.dig_p3)>>8) + ((var1 * (BMP280_S64_t)bmpCalibration.dig_p2)<<12);
	var1 = (((((BMP280_S64_t)1)<<47)+var1))*((BMP280_S64_t)bmpCalibration.dig_p1)>>33;
	if (var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((BMP280_S64_t)bmpCalibration.dig_p9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((BMP280_S64_t)bmpCalibration.dig_p8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)bmpCalibration.dig_p7)<<4);

	double pressure = (double)p / 25600;

	double altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
	return altitude;
}
#endif

BMP280_S64_t bmp280_pressure_diff(BMP280_S32_t adc_P, BMP280_S64_t startPa)
{
	BMP280_S64_t var1, var2, p;
	adc_P >>= 4;
	var1 = ((BMP280_S64_t)t_fine) - 128000;
	var2 = var1 * var1 * (BMP280_S64_t)bmpCalibration.dig_p6;
	var2 = var2 + ((var1*(BMP280_S64_t)bmpCalibration.dig_p5)<<17);
	var2 = var2 + (((BMP280_S64_t)bmpCalibration.dig_p4)<<35);
	var1 = ((var1 * var1 * (BMP280_S64_t)bmpCalibration.dig_p3)>>8) + ((var1 * (BMP280_S64_t)bmpCalibration.dig_p2)<<12);
	var1 = (((((BMP280_S64_t)1)<<47)+var1))*((BMP280_S64_t)bmpCalibration.dig_p1)>>33;
	if (var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((BMP280_S64_t)bmpCalibration.dig_p9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((BMP280_S64_t)bmpCalibration.dig_p8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)bmpCalibration.dig_p7)<<4);

	return p - startPa;
}

/**
  * @brief  This function is executed in case of timeout error occurrence.
  * @param  None
  * @retval None
  */
void Timeout_Error_Handler(void)
{
	 //HAL_GPIO_WritePin(GPIOC, LD4_Pin, GPIO_PIN_SET);
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
	  //HAL_GPIO_WritePin(GPIOC, LD4_Pin, GPIO_PIN_SET);
	  //HAL_Delay(500);
	  //HAL_GPIO_WritePin(GPIOC, LD4_Pin, GPIO_PIN_RESET);
	  //HAL_Delay(500);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
