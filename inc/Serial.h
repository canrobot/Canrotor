#pragma once

void MX_USART1_UART_Init(void);  // 블루투스용 시리얼 통신
void MX_USART3_UART_Init(void);  // 시리얼 통신
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void clearbuffer1(void);
void clearbuffer2(void);
void PrintData(uint8_t command);
