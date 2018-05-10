#pragma once


#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include "rl_net.h"                     // Keil.MDK-Pro::Network:CORE
#include "stm32h7xx_hal.h"
#include "string.h"
#include "stm32h7xx.h"  
#include "arm_math.h"



#define Npoint (8)
#define Npoint2 (Npoint / 2)
#define Npoint4 (Npoint / 4)
#define NpspMin (11)
#define NpspMax (511)
#define NsinMin (1)
#define NsinMax (20)
#define FsmplMin (30000)
#define FsmplMax (50000)


extern uint16_t sinx[Npoint * ( 1 + NsinMax)];

extern TIM_HandleTypeDef htim1;

void setPSP(uint8_t *mas);
int32_t StartOuter();
int32_t SetFreqOuter(uint32_t& freq, int32_t& Nsin, int32_t& Npsp);
void EmitSignalBPSK();










