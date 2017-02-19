#include <Board.h>

extern rc RC;

float kp[3] = {3.5f, 3.5f, 3.0f};
float ki[3] = {0.2f, 0.2f, 0.1f};
float kd[3] = {0.035f, 0.035f, 0.035f};

float error[3] = {0.0f, 0.0f, 0.0f};
float ITerm[3] = {0.0f, 0.0f, 0.0f};
float dInput[3] = {0.0f, 0.0f, 0.0f};
float lastInput[3] = {0.0f, 0.0f, 0.0f};
float output[3] = {0.0f, 0.0f, 0.0f};

float axisPID[3] = {0, 0, 0};
float input_r[2] = {0, 0};
float input_p[2] = {0, 0};
float input_y[2] = {0, 0};
float iterm[3] = {0, 0, 0};
extern int Flight_Status;
//extern uint32_t cycle_t;
// Configure PID
#define OUT_MAX	    	 200			// Out PID maximum150
#define OUT_MAX_Y		 200			// Out PID maximum150
#define I_MAX	         80			// Out I_term maximum
#define I_MAX_Y			 80			// Out I_term maximum
#define DIR				 1				// Direct PID Direction
/* AHRS/IMU structure */
extern TM_AHRSIMU_t AHRSIMU;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PID_Init(void)
{
	ki[ROLL] = ki[ROLL] * 0.004;
	kd[ROLL] = kd[ROLL] / 0.004;

	ki[PITCH] = ki[PITCH] * 0.004;
	kd[PITCH] = kd[PITCH] / 0.004;

	ki[YAW] = ki[YAW] * 0.004;
	kd[YAW] = kd[YAW] / 0.004;
	}
void Control(void)
{
//	if(Flight_Status <=2){
//		for(int i = 0; i < 3; i++){
//		axisPID[i] = 0;
//		iterm[i] = 0;
//		}
//	}
	  // PID process Roll process
//	  input_r[0] = AHRSIMU.Roll;
//	  axisPID[ROLL] = PID(RC.rcCommand[ROLL], input_r, &iterm[ROLL], kp[0], ki[0], kd[0], OUT_MAX_RP, I_MAX_RP);
//
//	  input_p[0] = AHRSIMU.Pitch;
//	  axisPID[PITCH] = PID(RC.rcCommand[PITCH], input_p, &iterm[PITCH], kp[1], ki[1], kd[1], OUT_MAX_RP, I_MAX_RP);
//
//	  input_y[0] = AHRSIMU.Yaw;
//	  axisPID[YAW] = PID(RC.rcCommand[YAW], input_y, &iterm[YAW], kp[2], ki[2], kd[2], OUT_MAX_Y, I_MAX_Y);

////////////////////////////////////////////////////////////////////////////////////////////////
	  /*Compute all the working error variables*/
	  error[ROLL] = RC.rcCommand[ROLL] - AHRSIMU.Roll;
	  ITerm[ROLL] +=(ki[ROLL] * error[ROLL]);
	  if(ITerm[ROLL] > I_MAX) ITerm[ROLL] = I_MAX;
	  else if(ITerm[ROLL] < -I_MAX) ITerm[ROLL] = -I_MAX;
	  dInput[ROLL] = (AHRSIMU.Roll - lastInput[ROLL]);

	  /*Compute PID Output*/
	  output[ROLL] = kp[ROLL] * error[ROLL] + ITerm[ROLL] - kd[ROLL] * dInput[ROLL];

	  if(output[ROLL] > OUT_MAX) output[ROLL] = OUT_MAX;
	  else if(output[ROLL] < -OUT_MAX) output[ROLL] = -OUT_MAX;

	  /*Remember some variables for next time*/
	  lastInput[ROLL] = AHRSIMU.Roll;

/////////////////////////////////////////////////////////////////////////////////////////////////

	  /*Compute all the working error variables*/
	  error[PITCH] = RC.rcCommand[PITCH] - AHRSIMU.Pitch;
	  ITerm[PITCH] +=(ki[PITCH] * error[PITCH]);
	  if(ITerm[PITCH] > I_MAX) ITerm[PITCH] = I_MAX;
	  else if(ITerm[PITCH] < -I_MAX) ITerm[PITCH] = -I_MAX;
	  dInput[PITCH] = (AHRSIMU.Pitch - lastInput[PITCH]);

	  /*Compute PID Output*/
	  output[PITCH] = kp[PITCH] * error[PITCH] + ITerm[PITCH] - kd[PITCH] * dInput[PITCH];

	  if(output[PITCH] > OUT_MAX) output[PITCH] = OUT_MAX;
	  else if(output[PITCH] < -OUT_MAX) output[PITCH] = -OUT_MAX;

	  /*Remember some variables for next time*/
	  lastInput[PITCH] = AHRSIMU.Pitch;

//////////////////////////////////////////////////////////////////////////////////////////////////

	  /*Compute all the working error variables*/
	  error[YAW] = RC.rcCommand[YAW] - AHRSIMU.Yaw;
	  ITerm[YAW] +=(ki[YAW] * error[YAW]);
	  if(ITerm[YAW] > I_MAX) ITerm[YAW] = I_MAX;
	  else if(ITerm[YAW] < -I_MAX) ITerm[YAW] = -I_MAX;
	  dInput[YAW] = (AHRSIMU.Yaw - lastInput[YAW]);

	  /*Compute PID Output*/
	  output[YAW] = kp[YAW] * error[YAW] + ITerm[YAW] - kd[YAW] * dInput[YAW];

	  if(output[YAW] > OUT_MAX) output[YAW] = OUT_MAX;
	  else if(output[YAW] < -OUT_MAX) output[YAW] = -OUT_MAX;

	  /*Remember some variables for next time*/
	  lastInput[YAW] = AHRSIMU.Yaw;

		if(Flight_Status <=2){
			for(int i = 0; i < 3; i++){
			output[i] = 0;
			ITerm[i] = 0;
			}
		}
}

float PID(int16_t setpoint, float *input, float *iterm, float kp, float ki, float kd, int out_max, int i_max)
{
	//Compute all the working error variables
	float error, Dinput;
	float output;

	ki = ki * 0.004;
	kd = kd / 0.004;

	error = (float)setpoint - input[0];
	*iterm += ki * error;
	if (*iterm > i_max) *iterm = i_max;
	else if (*iterm < -i_max) *iterm = -i_max;
	Dinput = (input[0] - input[1]);

	//Compute PID Output (Input Process)
	output = kp * error + *iterm - kd * Dinput;
	
	if (output > out_max) output = out_max;
	else if (output < -out_max) output = -out_max;
	// Save state actual
	input[1] = input[0];
	return (float)output;
}

int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}
