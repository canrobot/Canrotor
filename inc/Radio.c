#include "Board.h"

rc RC;
extern int Flight_Status;
extern float axisPID[3];
extern float iterm[3];


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 && htim->Instance == TIM2){
	        if(Ch1_PIN){  // Timer2 Ch1 pin(PA0) is High
	            TIM2->CCR1 = 0;
	            RC.capture_rise[0] = TIM2->CCR1; // read capture data
	            Ch1_POL_FALLING;  // to falling edge
	        }
	        else{   // Timer2 Ch1 pin(PA0) is Low
	            RC.capture_fall[0] = TIM2->CCR1; // read capture data
	            RC.rcADC[0] = RC.capture_fall[0] - RC.capture_rise[0];
	            Ch1_POL_RISING;   // to rising edge
	        }
	    }

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 && htim->Instance == TIM2){
	        if(Ch2_PIN){  // Timer2 Ch2 pin(PA1) is High
	            TIM2->CCR2 = 0;
	        	RC.capture_rise[1] = TIM2->CCR2; // read capture data
	            Ch2_POL_FALLING;  // to falling edge
	        }
	        else{   // Timer2 Ch2 pin(PA1) is Low
	        	RC.capture_fall[1] = TIM2->CCR2; // read capture data
	        	RC.rcADC[1] = RC.capture_fall[1] - RC.capture_rise[1];
	            Ch2_POL_RISING;   // to rising edge
	        }
	    }

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3 && htim->Instance == TIM2){
	        if(Ch3_PIN){  // Timer2 Ch3 pin(PB10) is High
	            TIM2->CCR3 = 0;
	        	RC.capture_rise[2] = TIM2->CCR3; // read capture data
	            Ch3_POL_FALLING;  // to falling edge
	        }
	        else{   // Timer2 Ch3 pin(PB11) is Low
	        	RC.capture_fall[2] = TIM2->CCR3; // read capture data
	        	RC.rcADC[2] = RC.capture_fall[2] - RC.capture_rise[2];
	            Ch3_POL_RISING;   // to rising edge
	        }
	    }

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 && htim->Instance == TIM2){
	        if(Ch4_PIN){  // Timer2 Ch4 pin(PB11) is High
	            TIM2->CCR4 = 0;
	        	RC.capture_rise[3] = TIM2->CCR4; // read capture data
	            Ch4_POL_FALLING;  // to falling edge
	        }
	        else{   // Timer2 Ch4 pin(PB11) is Low
	        	RC.capture_fall[3] = TIM2->CCR4; // read capture data
	            RC.rcADC[3] = RC.capture_fall[3] - RC.capture_rise[3];
	            Ch4_POL_RISING;   // to rising edge
	        }
	    }

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 && htim->Instance == TIM4){
	        if(Ch5_PIN){  // Timer4 Ch1 pin(PB6) is High
	            TIM4->CCR1 = 0;
	        	RC.capture_rise[4] = TIM4->CCR1; // read capture data
	            Ch5_POL_FALLING;  // to falling edge
	        }
	        else{   // Timer4 Ch1 pin(PB6) is Low
	        	RC.capture_fall[4] = TIM4->CCR1; // read capture data
	            RC.rcADC[4] = RC.capture_fall[4] - RC.capture_rise[4];
	            Ch5_POL_RISING;   // to rising edge
	        }
	    }

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 && htim->Instance == TIM4){
	        if(Ch6_PIN){  // Timer4 Ch2 pin(PB7) is High
	            TIM4->CCR2 = 0;
	        	RC.capture_rise[5] = TIM4->CCR2; // read capture data
	            Ch6_POL_FALLING;  // to falling edge
	        }
	        else{   // Timer4 Ch2 pin(PB7) is Low
	        	RC.capture_fall[5] = TIM4->CCR2; // read capture data
	            RC.rcADC[5] = RC.capture_fall[5] - RC.capture_rise[5];
	            Ch6_POL_RISING;   // to rising edge
	        }
	    }

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3 && htim->Instance == TIM4){
	        if(Ch7_PIN){  // Timer4 Ch3 pin(PB8) is High
	            TIM4->CCR3 = 0;
	        	RC.capture_rise[6] = TIM4->CCR3; // read capture data
	            Ch7_POL_FALLING;  // to falling edge
	        }
	        else{   // Timer4 Ch3 pin(PB8) is Low
	        	RC.capture_fall[6] = TIM4->CCR3; // read capture data
	            RC.rcADC[6] = RC.capture_fall[6] - RC.capture_rise[6];
	            Ch7_POL_RISING;   // to rising edge
	        }
	    }

//	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 && htim->Instance == TIM4){
//	        if(Ch8PIN){  // Timer4 Ch4 pin(PB9) is High
//	        	RC.capture_rise[7] = TIM4->CCR4; // read capture data
//	            Ch8_POL_FALLING;  // to falling edge
//	        }
//	        else{   // Timer4 Ch4 pin(PB9) is Low
//	        	RC.capture_fall[7] = TIM4->CCR4; // read capture data
//	            RC.rcADC[7] = RC.capture_fall[7] - RC.capture_rise[7];
//	            Ch8_POL_RISING;   // to rising edge
//	        }
//	    }
}
void RC_Init(void)
{
	Flight_Status = 0;  // 초기상태
	while((RC.rcCommand[THROTTLE] < 990) || (RC.rcCommand[THROTTLE] > 1020)){
		computeRC();
		Flight_Status ++;
		HAL_Delay(4);
		if(Flight_Status == 125){
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14); // RED RC_Init 반짝이면 준비가 안된 상테 (초록색)
			Flight_Status = 0;
		}
	}
	Flight_Status = 1;  // 조정기 초기상태 확인
}
void computeRC(void)
{
	RC.rcCommand[ROLL]     =  map(RC.rcADC[1], 1100, 1900, -42, 42);//0.125 * (RC.rcADC[1]-1100)-50;
	RC.rcCommand[PITCH]    =  map(RC.rcADC[0], 1100, 1900, -42, 42);//0.125 * (RC.rcADC[0]-1100)-50;
	RC.rcCommand[YAW]      =  map(RC.rcADC[3], 1100, 1900, -50, 50);//0.125 * (RC.rcADC[3]-1100)-50;
	RC.rcCommand[THROTTLE] =  map(RC.rcADC[2], 1100, 1900, 1000, 2000);//RC.rcADC[2];//1.25 * (RC.rcADC[2]-300);
	RC.rcCommand[GEAR]	   =  map(RC.rcADC[4], 1100, 1900, 1000, 2000);
	RC.rcCommand[AUX1] 	   =  map(RC.rcADC[5], 1100, 1900, 1000, 2000);
	RC.rcCommand[AUX2] 	   =  map(RC.rcADC[6], 1100, 1900, 1000, 2000);

	if(Flight_Status >= 1)
	{
	//Stopping the motors: throttle low and yaw right.
		if(Flight_Status == 3 && RC.rcCommand[THROTTLE] < 1050 && RC.rcCommand[YAW] > 40)Flight_Status = 1;
	//For starting the motors: throttle low and yaw left (step 1).
		if(RC.rcCommand[THROTTLE] < 1050 && RC.rcCommand[YAW] > 40)Flight_Status = 2;
	//When yaw stick is back in the center position start the motors (step 2).
		if(Flight_Status == 2 && RC.rcCommand[THROTTLE] < 1050 && RC.rcCommand[YAW] < -40){
			Flight_Status = 3;
	//Reset the PID controllers for a bumpless start.
			for(int i = 0; i < 3; i++){
			axisPID[i] = 0;
			iterm[i] = 0;
			}
	}}}

