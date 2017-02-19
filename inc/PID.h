#pragma once
void PID_Init(void);
float PID(int16_t setpoint, float *input, float *iterm, float kp, float ki, float kd, int out_max, int i_max);
void Control(void);
int constrain(int amt, int low, int high);

