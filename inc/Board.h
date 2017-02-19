#pragma once

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>


#define D2R (M_PI / 180.0f)
#define R2D (180.0f / M_PI)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define map(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


void resetConf(void);


//#ifdef
#include <I2C.h>
#include <System.h>
#include <Typedef.h>
#include <Sensor.h>
#include <IMU.h>
#include <Radio.h>
#include <PID.h>
#include <Mixer.h>
#include <PWM.h>
#include <Serial.h>
