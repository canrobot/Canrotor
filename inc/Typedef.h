#pragma once

enum rc {
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  GEAR,
  AUX1,
  AUX2
};

#define CALIBRATING_GYRO_CYCLES             1000
#define CALIBRATING_ACC_CYCLES              400
#define CALIBRATING_BARO_CYCLES             200

////////////////////PID//////////////////////////////
//float kp[3] = {3.5f, 3.5f, 3.0f};
//float ki[3] = {0.2f, 0.2f, 0.1f};
//float kd[3] = {0.035f, 0.035f, 0.035f};
//
//float error[3] = {0.0f, 0.0f, 0.0f};
//float ITerm[3] = {0.0f, 0.0f, 0.0f};
//float dInput[3] = {0.0f, 0.0f, 0.0f};
//float lastInput[3] = {0.0f, 0.0f, 0.0f};
//float output[3] = {0.0f, 0.0f, 0.0f};
//
//float axisPID[3] = {0, 0, 0};
//float input_r[2] = {0, 0};
//float input_p[2] = {0, 0};
//float input_y[2] = {0, 0};
//float iterm[3] = {0, 0, 0};
