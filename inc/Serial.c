#include <Board.h>

uint16_t Buf[50];

/* AHRS/IMU structure */
extern TM_AHRSIMU_t AHRSIMU;
extern imu_t imu;
extern rc RC;
extern int16_t motor[4];
extern float kp[3];
extern float ki[3];
extern float kd[3];
volatile unsigned char m = 0, command=0;
char Rx_data_BLE[20], Rx_buf_BLE[20];
char Rx_data_COM[20], Rx_buf_COM[20];
int i_BLE = 0, i_COM = 0;

//extern float angle_pitch, angle_roll, angle_yaw;
extern float output[3];
extern float input_r[2];
extern float input_p[2];
extern float input_y[2];
extern float gyro_cal[3];
extern float mag_cal[3];
extern uint32_t System_Time;
extern int Flight_Status;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;


/* USART1 init function */
void MX_USART1_UART_Init(void)
{
	__USART1_CLK_ENABLE();
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);
  HAL_UART_Receive_IT(&huart1, Rx_buf_BLE, 1);
}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{
	__USART3_CLK_ENABLE();
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);
  HAL_UART_Receive_IT(&huart3, Rx_buf_COM, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance == USART1) //current USART
		{
			Rx_data_BLE[m++] = Rx_buf_BLE[0];
			//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
		}
		HAL_UART_Receive_IT(&huart1, Rx_buf_BLE, 1);
		//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
		if(huart->Instance == USART3) //current USART
		{
			Rx_data_COM[m++] = Rx_buf_COM[0];
			//serialCom();
			  sprintf((uint16_t*)Buf, " KPr: (%c), %d , %d\n ", Rx_data_COM[i_COM],i_COM, command);
			  HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
			  i_COM++;
			if((Rx_data_COM[0] == 'K') & (Rx_data_COM[1] == 'P'))
			{command = 1; clearbuffer1(); sprintf((uint16_t*)Buf, " KP: ");
			  HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
			}
			if((Rx_data_COM[0] == 'K') & (Rx_data_COM[1] == 'I'))
			{command = 2; clearbuffer1(); sprintf((uint16_t*)Buf, " KI: ");
			  HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
			}
			if((Rx_data_COM[0] == 'K') & (Rx_data_COM[1] == 'D'))
			{command = 3; clearbuffer1(); sprintf((uint16_t*)Buf, " KD: ");
			  HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
			}
			if((Rx_data_COM[0] == 'K') & (Rx_data_COM[1] == 'P') & (Rx_data_COM[2] == 'Y'))
			{command = 4; clearbuffer1(); sprintf((uint16_t*)Buf, " KPY: ");
			  HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
			}
			if((Rx_data_COM[0] == 'K') & (Rx_data_COM[1] == 'I') & (Rx_data_COM[2] == 'Y'))
			{command = 5; clearbuffer1(); sprintf((uint16_t*)Buf, " KPY: ");
			  HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
			}
			if((Rx_data_COM[0] == 'K') & (Rx_data_COM[1] == 'D') & (Rx_data_COM[2] == 'Y'))
			{command = 6; clearbuffer1(); sprintf((uint16_t*)Buf, " KPY: ");
			  HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
			}
			if ((command == 0) & (m > 2) ) clearbuffer1();
			if((command == 1) & (Rx_data_COM[m-1] == '\n') & (atof(Rx_data_COM) != 0))
			{kp[0] = atof(Rx_data_COM); kp[1] = kp[0]; clearbuffer2();sprintf((uint16_t*)Buf, " KP: %.2f \n", kp[0]);
			 HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
			}
			if((command == 2) & (Rx_data_COM[m-1] == '\n') & (atof(Rx_data_COM) != 0))
			{ki[0] = atof(Rx_data_COM); ki[1] = ki[0]; clearbuffer2();sprintf((uint16_t*)Buf, " KI: %.2f \n", ki[0]);
			 HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
			}
			if((command == 3) & (Rx_data_COM[m-1] == '\n') & (atof(Rx_data_COM) != 0))
			{kd[0] = atof(Rx_data_COM); kd[1] = kd[0]; clearbuffer2();sprintf((uint16_t*)Buf, " KD: %.2f \n", kd[0]);
			 HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
			}
			if((command == 4) & (Rx_data_COM[m-1] == '\n') & (atof(Rx_data_COM) != 0))
			{kp[2] = atof(Rx_data_COM); clearbuffer2();sprintf((uint16_t*)Buf, " KPy: %.2f \n", kp[2]);
			 HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
			}
			if((command == 5) & (Rx_data_COM[m-1] == '\n') & (atof(Rx_data_COM) != 0))
			{ki[2] = atof(Rx_data_COM); clearbuffer2();sprintf((uint16_t*)Buf, " KPy: %.2f \n", ki[2]);
			 HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
			}
			if((command == 6) & (Rx_data_COM[m-1] == '\n') & (atof(Rx_data_COM) != 0))
			{kd[2] = atof(Rx_data_COM); clearbuffer2();sprintf((uint16_t*)Buf, " KPy: %.2f \n", kd[2]);
			 HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
			}
			HAL_UART_Receive_IT(&huart3, Rx_buf_COM, 1);
		}
	}

void clearbuffer1(void){			// Reset index vector Get char USART
	m = 0;
	for (int i = 0; i < 20; i++) Rx_data_COM[i] = 0;
	i_COM = 0;
}

void clearbuffer2(void){			// Reset index vector Get char USART and Reset command
	m = 0;
	for (int i = 0; i < 20; i++) Rx_data_COM[i] = 0;
	command = 0;
	i_COM = 0;
}

void PrintData(uint8_t command)
{
	switch(command)
	{
	case 1:
	     sprintf((uint16_t*)Buf, " acc (%6.f), (%6.f), (%6.f)",imu.accRaw[ROLL], imu.accRaw[PITCH], imu.accRaw[YAW]);
	     HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
	     sprintf((uint16_t*)Buf, " gyro (%6.f), (%6.f), (%6.f)",imu.gyroRaw[ROLL], imu.gyroRaw[PITCH], imu.gyroRaw[YAW]);
	     HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
	     sprintf((uint16_t*)Buf, " mag (%6.f), (%6.f), (%6.f)\r\n",imu.magRaw[ROLL], imu.magRaw[PITCH], imu.magRaw[YAW]);
	     HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
	     break;

	case 2:
		sprintf((uint16_t*)Buf, " gyro_x_cal: (%3.2f), gyro_x_cal: (%3.2f), gyro_x_cal: (%3.2f)\r\n", gyro_cal[ROLL], gyro_cal[PITCH], gyro_cal[YAW]);
		HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
		sprintf((uint16_t*)Buf, " mag_x_cal: (%3.2f), mag_x_cal: (%3.2f), mag_x_cal: (%3.2f)\r\n", mag_cal[ROLL], mag_cal[PITCH], mag_cal[YAW]);
		HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
	    break;

	case 3:
		sprintf((uint16_t*)Buf, " RC (%6.d) (%6.d) (%6.d) (%6.d)\r\n", RC.rcCommand[ROLL], RC.rcCommand[PITCH], RC.rcCommand[YAW], RC.rcCommand[THROTTLE]);
		HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
//		sprintf((uint16_t*)Buf, " GEAR (%6.d) AUX1 (%6.d) AUX2 (%6.d)", RC.rcCommand[GEAR], RC.rcCommand[AUX1], RC.rcCommand[AUX2]);
//		HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
		break;

	case 4:
		sprintf((uint16_t*)Buf, " motor(%6.d)(%6.d)(%6.d)(%6.d) ", motor[0], motor[1], motor[2], motor[3]);
		HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
		break;
	case 5:
	     sprintf((uint16_t*)Buf, " (%.2f), (%.2f), (%.2f)\r\n",input_r[0], input_p[0], input_y[0]);
	     HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
		break;
	case 6:
	     sprintf((uint16_t*)Buf, " (%.2f), (%.2f), (%.2f), (%d)(%d) \r\n",output[ROLL], output[PITCH], output[YAW], Flight_Status, System_Time);
	     HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
		break;
	case 7:
		  sprintf((uint16_t*)Buf, " KPr: (%d) \n ", Rx_data_COM[0]);
		  HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
		break;
	case 8:

		break;
	case 9:
	     sprintf((uint16_t*)Buf, "(%.2f)(%.2f)(%.2f)(%d)(%d)\r\n",AHRSIMU.Roll, AHRSIMU.Pitch, AHRSIMU.Yaw, Flight_Status, System_Time);
	     HAL_UART_Transmit(&huart3, (uint8_t*)Buf, strlen(Buf), 1000);
		break;
	}
}
