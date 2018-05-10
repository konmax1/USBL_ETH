#include "pwmOuter.h"
#include "arm_math.h"

uint16_t sinx[Npoint * ( 1 + NsinMax)];
uint8_t pspmas[NpspMax];
volatile int32_t N_sinG = 1;
volatile int32_t lenPSP = 1;
int32_t currpos = 0;
uint32_t* pmasOut;

uint16_t outBPSK[Npoint * NsinMax * NpspMax];


void setPSP(uint8_t *mas){
	uint16_t* p = &outBPSK[0];
	arm_copy_q7( (q7_t*)&mas[0], (q7_t*)pspmas, lenPSP);
	
	for(volatile int i=0 ; i < lenPSP;i++){
		pspmas[i] = pspmas[i]  * Npoint2;
	}
	//arm_copy_q15()
	for(volatile int i=0 ; i < lenPSP;i++){
		arm_copy_q15( (q15_t*)&sinx[pspmas[i]], (q15_t*)p,  /*2 * */N_sinG );
		p += N_sinG;
	}
}

void initSinTable(uint16_t arr){
  float	step = (float) (2*PI) / Npoint;
	for(volatile int i=0 ; i <= (Npoint * ( 1 + NsinMax) ) ; i++)
	{
		sinx[i] = (arm_sin_f32 ( i * step) * arr / 2) + (arr / 2); 
	}
}


int32_t SetFreqOuter(uint32_t& freq, int32_t& Nsin, int32_t& Npsp){	
	if(freq < FsmplMin || freq > FsmplMax)
		return -1;
	if( Nsin < NsinMin || Nsin > NsinMax)
		return -2;
	if( Npsp < NpspMin || Npsp > NpspMax)
		return -3;
	lenPSP = Npsp;
	N_sinG = Nsin * Npoint;
	uint32_t ARR = (float)(2 * HAL_RCC_GetPCLK2Freq())/(freq * Npoint);
	ARR = ARR - 1;
	initSinTable(ARR);
	TIM1->PSC = 0;
	TIM1->ARR = ARR;
	//HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_3,(uint32_t*)&sinx[0],(Npoint * 5));
	return 0;
}

int32_t StartOuter(){	

	TIM1->CCER |=  TIM_CCER_CC3NE;
	HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_3,(uint32_t*)&outBPSK[0],N_sinG * lenPSP);
	return 0;
}

void StopOuter(){
	TIM1->CCER &=  ~TIM_CCER_CC3NE;
	HAL_TIM_PWM_Stop_DMA(&htim1,TIM_CHANNEL_3);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	StopOuter();
}






