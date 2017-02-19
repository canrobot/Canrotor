#include <Board.h>

extern int16_t motor[4];

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;


/* TIM2 init function */
void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;
  GPIO_InitTypeDef GPIO_InitStruct;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  /* Peripheral clock enable */
  __TIM2_CLK_ENABLE();

  /**TIM2 GPIO Configuration
     PA0     ------> TIM2_CH1
     PA1     ------> TIM2_CH2
     PA2     ------> TIM2_CH3
     PA3     ------> TIM2_CH4
     **/
     GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
     GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

  HAL_TIM_IC_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;

  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2);
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3);
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
}

/* TIM3 init function */
void MX_TIM3_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3999;  //4ms(250hz period), 19999(50hz=20ms period)
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  /* Peripheral clock enable */
  __TIM3_CLK_ENABLE();

  /**TIM3 GPIO Configuration
  PC6     ------> TIM3_CH1
  PC7     ------> TIM3_CH2
  PC8     ------> TIM3_CH3
  PC9     ------> TIM3_CH4
  */
//  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
  TIM3->CCER |=0x1111;   //TIM_OutputState_Enable

  TIM3->CCMR1 |= 0x808; //channel 1,2 TIM_OCPreload_Enable;
  TIM3->CCMR2 |= 0x808; //channel 3,4 TIM_OCPreload_Enable;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

/* TIM4 init function */
void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;
  GPIO_InitTypeDef GPIO_InitStruct;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFFFF;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  /* Peripheral clock enable */
  __TIM4_CLK_ENABLE();

  /**TIM4 GPIO Configuration
     PB6     ------> TIM4_CH1
     PB7     ------> TIM4_CH2
     PB8     ------> TIM4_CH3
     PB9     ------> TIM4_CH4
     **/
     GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
     GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

  HAL_TIM_IC_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;

  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1);
  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2);
  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3);
  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);
}

/* TIM7 init function */
void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  /* Peripheral clock enable */
  __TIM7_CLK_ENABLE();

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  HAL_TIM_Base_Init(&htim7);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

  HAL_TIM_Base_Start(&htim7);
}

void PwmWriteMotor(void)
{
	TIM3->CCR1 = motor[0];  // Program : FRONT_R   / Actual :
	TIM3->CCR2 = motor[1];  // Program : FRONT_L  / Actual :
	TIM3->CCR3 = motor[2];  // Program : REAR_R   / Actual :
	TIM3->CCR4 = motor[3];  // Program : REAR_L  / Actual :
}
