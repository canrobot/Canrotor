/**
  ******************************************************************************
  * @file    Quard Copter
  * @author  Wang Hak Seung
  * @version V1.0
  * @date    9-1-2017
  * @brief   STM32F407VGT(Discovery) 현재 PID 게인 조절 준비 전.
  ******************************************************************************
*/

#include "Board.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* AHRS/IMU structure */
TM_AHRSIMU_t AHRSIMU;
int Flight_Status;
uint32_t System_Time = 0, System_Count = 0;

int main(void)
{
  SystemInit();
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();  // Radio Signal
  MX_TIM3_Init();  // Motor Signal
  MX_TIM4_Init();  // Radio Signal
  MX_TIM7_Init();  // System Timer
  //MX_USART1_UART_Init(); // 블루 투스
  MX_USART3_UART_Init(); // computer RS-232
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  MPU9250_Init();
  /* Init structure with 100hZ sample rate, 0.1 beta and 3.5 inclination (3.5 degrees is inclination in Ljubljana, Slovenia) on July, 2016 */
  TM_AHRSIMU_Init(&AHRSIMU, 250, 0.5f, 0.0f);
  Calibration();
  RC_Init();
  PID_Init();
  mixerInit();

  while (1)
  {
	if(TIM7->CNT >= 4000){                          // 250Hz(4ms) Cycle
	  System_Time = TIM7->CNT;
	  TIM7->CNT = 0;

	  System_Count ++;
	  computeIMU();
	  Control();
	  mixTable();
	  PwmWriteMotor();

	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);  // GREEN
	}
	if(System_Count >= 2){
		//computeIMU();// 2 =  125Hz(8ms) Cycle , 50 = 5Hz(200ms) cycle
        computeRC();
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13); // ORANGE
	    //PrintData(1); // MPU-9250 RAW 데이터 출력
	    //PrintData(4);   // motor output
	    //PrintData(3);  //radio output
	    //PrintData(6);   //PID output
	    //PrintData(5);   //Angle output
	    PrintData(9);   //IMU
	    System_Count = 0;
	}
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
		                       |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}

void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();

  /**I2C2 GPIO Configuration  //MPU-9250
  PB10     ------> I2C2_SCL
  PB11     ------> I2C2_SDA
  */
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**USART1 GPIO Configuration  // 블루투스
  PB6     ------> USART1_TX
  PB7     ------> USART1_RX
  */
//  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

  /*UART3 Configure GPIO pin : PC10 PC11 */  // 컴퓨터 통신
 GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
 GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
 GPIO_InitStruct.Pull = GPIO_PULLUP;
 GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
 GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
 HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
 HAL_NVIC_EnableIRQ(USART3_IRQn);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15
   PD12 ------> Green
   PD13 ------> Orange
   PD14 ------> Red
   PD15 ------> Blue
  */
  __GPIOD_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
