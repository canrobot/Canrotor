#include <Board.h>

extern rc RC;
extern float output[3];
int16_t motor[4];
extern int Flight_Status;

// Custom mixer data per motor
typedef struct motorMixer_t {
    float THROTTLE;
    float ROLL;
    float PITCH;
    float YAW;
} motorMixer_t;

// Custom mixer configuration
typedef struct mixer_t {
    uint8_t numberMotor;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

static motorMixer_t currentMixer[4];

//static const motorMixer_t mixerQuadP[] = {
//    { 1.0f,  0.0f,  1.0f,  1.0f },          // FRONT  (CCW)    1
//    { 1.0f,  1.0f,  0.0f, -1.0f },          // LEFT   (CW)     2
//    { 1.0f, -1.0f,  0.0f,  1.0f },          // RIGHT  (CCW)    3
//    { 1.0f,  0.0f, -1.0f, -1.0f },          // REAR   (CW)     4
//};

static const motorMixer_t mixerQuadP[] = {
    { 1.0f,  0.0f,  1.0f,  0.0f },          // FRONT  (CCW)    1
    { 0.0f,  0.0f,  0.0f,  0.0f },          // LEFT   (CW)     2
    { 0.0f,  0.0f,  0.0f,  0.0f },          // RIGHT  (CCW)    3
    { 1.0f,  0.0f, -1.0f,  0.0f },          // REAR   (CW)     4
};

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,   1.0f,  -1.0f },          // (CCW) 1  //FRONT_R
    { 1.0f,  1.0f,   1.0f,   1.0f },          // (CW)  2  //FRONT_L
    { 1.0f, -1.0f,  -1.0f,   1.0f },          // (CCW) 3  //REAR_R
    { 1.0f,  1.0f,  -1.0f,  -1.0f },          // (CW)  4  //REAR_L
}; // THR,  ROLL,   PITCH,   YAW

const mixer_t mixers[] = {
    { 4, 0, mixerQuadP },          // MULTITYPE_QUADP
    { 4, 0, mixerQuadX },          // MULTITYPE_QUADX
};

void mixerInit(void)
{
	int i;
    for (i = 0; i < 4; i++)
        currentMixer[i] = mixers[1].motor[i];   //0 = mixerQuadP, 1 = mixerQuadX
}

void mixTable(void)
{
	uint8_t i;
	if (Flight_Status == 3){                                                          //The motors are started.3
			if (RC.rcCommand[THROTTLE] > 1800) RC.rcCommand[THROTTLE] = 1800;                                   //We need some room to keep full control at full throttle.

			for (i = 0; i < 4; i++){
				motor[i] = (RC.rcCommand[THROTTLE] * currentMixer[i].THROTTLE) + (output[ROLL] * currentMixer[i].ROLL) + (output[PITCH] * currentMixer[i].PITCH) + ((-1 * output[YAW]) * currentMixer[i].YAW);
				if(RC.rcCommand[THROTTLE] < 1200) motor[i] = 1050;
				if(RC.rcCommand[THROTTLE] > 2000) motor[i] = 2000;
				}
			}else{
				for (i = 0; i < 4; i++)
				motor[i] = 1000;
			}
}
