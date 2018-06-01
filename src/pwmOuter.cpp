#include "pwmOuter.h"

uint16_t sinx[Npoint * ( 1 + NsinMax)];
uint8_t pspmas[NpspMax];
volatile int32_t N_sinG = 1;
volatile int32_t lenPSP = 1;
int32_t currpos = 0;
uint32_t* pmasOut;
volatile uint32_t needOuterSignal = 0;
//uint16_t outBPSK[Npoint * NsinMax * NpspMax] __attribute__((section(".ARM.__at_0x30020000")));
uint16_t outBPSK[Npoint * NsinMax * NpspMax] __attribute__((section(".ARM.__at_0x24040000")));;

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
	arr = arr - 4;
  float	step = (float) (2*PI) / Npoint;
	for(volatile int i=0 ; i <= (Npoint * ( 1 + NsinMax) ) ; i++)
	{
		sinx[i] = (arm_sin_f32 ( i * step) * arr / 2) + (arr / 2); 
	}
}


int32_t SetFreqOuter(uint32_t& freq, int32_t& Nsin, int32_t& Npsp){	
	if(freq < FsmplMin || freq > FsmplMax)
		return -1;
	/*if( Nsin < NsinMin || Nsin > NsinMax)
		return -2;
	if( Npsp < NpspMin || Npsp > NpspMax)
		return -3;*/
	if( (Nsin*Npsp) > 8192)
		return -2;
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

volatile int32_t cntOuter = 0;

int32_t StartOuter(){	
	SET_BIT(GPIOG->ODR,GPIO_PIN_5);	
	SET_BIT(GPIOB->ODR,GPIO_PIN_7);
	TIM1->CCER |=  TIM_CCER_CC3NE;
	SCB_InvalidateDCache_by_Addr((uint32_t*)&outBPSK[0],N_sinG * lenPSP * 2);
	HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_3,(uint32_t*)&outBPSK[0],N_sinG * lenPSP);
	//cntOuter = 0;
	//TIM1->CCR3=outBPSK[cntOuter++];
	//HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_3);
	return 0;
}

int32_t preStartOuter(){
	SET_BIT(GPIOG->ODR,GPIO_PIN_5);	
	TIM1->CCER |=  TIM_CCER_CC3NE;
	SCB_InvalidateDCache_by_Addr((uint32_t*)&outBPSK[0],N_sinG * lenPSP * 2);
	TIM1->CNT = 0;
	
	htim1.hdma[TIM_DMA_ID_CC3]->XferCpltCallback = HAL_TIM_DMADelayPulseCplt;	
	htim1.hdma[TIM_DMA_ID_CC3]->XferErrorCallback = HAL_TIM_DMAError ;
  HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_CC3], (uint32_t)&outBPSK[0], (uint32_t)&htim1.Instance->CCR3,N_sinG * lenPSP);
	__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC3);
	TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);

	//HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_3,(uint32_t*)&outBPSK[0],N_sinG * lenPSP);
		return 0;
}

__inline__ void StopOuter(){
	TIM1->CCER &=  (~TIM_CCER_CC3NE) & (~TIM_CCER_CC3E);
	TIM1->DIER &= ~(TIM_DMA_CC3 | TIM_DIER_CC3IE);
	TIM1->BDTR &= ~(TIM_BDTR_MOE);
	TIM1->CR1 &= ~(TIM_CR1_CEN);
	htim1.State = HAL_TIM_STATE_READY;
	//HAL_TIM_PWM_Stop_DMA(&htim1,TIM_CHANNEL_3);	
	CLEAR_BIT(GPIOG->ODR,GPIO_PIN_5);	
	CLEAR_BIT(GPIOB->ODR,GPIO_PIN_7);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	StopOuter();
}


volatile int iCNT = 0;
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim){
	iCNT++;
}

/**
* @brief This function handles EXTI line0 interrupt.
*/
extern "C" void EXTI0_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
	if(needOuterSignal){
		needOuterSignal = 0;	
		__HAL_TIM_MOE_ENABLE(&htim1);
		SET_BIT(GPIOB->ODR,GPIO_PIN_7);
		__HAL_TIM_ENABLE(&htim1); 
		//StartOuter();
	}
  //HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);

}
