#include "thread.h"
#include "rl_net.h"
#include "pwmOuter.h"
 
int32_t udp_sock;                       // UDP socket handle
int32_t connStat = -1;

NET_ADDR addr_pc; 
volatile uint16_t cnt_adc = 0;
extern volatile uint32_t ipos;

// Notify the user application about UDP socket events.
uint32_t udp_cb_func (int32_t socket, const  NET_ADDR *addr, const uint8_t *buf, uint32_t len) {
	
	const netHeader *p = (const netHeader*) buf;
	uint32_t freq = 0;
	OuterData *p_outerdata;
	volatile float freqP = 2 * HAL_RCC_GetPCLK1Freq();
	switch(p->type){
		case tInitConn:
			addr_pc = *addr;
			break;
		case tStartADC:
			cnt_adc = 0;
			TIM2->CR1  |= TIM_CR1_CEN;
			break;
		case tStopADC:
			TIM2->CR1  &= ~TIM_CR1_CEN;
			break;
		case tSetFreqADC:
			memcpy(&freq,(buf + HEADER_SIZE),4);
			TIM2->ARR = (float)(freqP / freq) - 1;
			TIM2->CNT = 0;
			break;
		case tLogADC:
			SET_BIT(GPIOC->ODR,GPIO_PIN_2);
			CLEAR_BIT(GPIOC->ODR,GPIO_PIN_3);
			break;
		case tLinADC:
			CLEAR_BIT(GPIOC->ODR,GPIO_PIN_2);
			CLEAR_BIT(GPIOC->ODR,GPIO_PIN_3);
			break;
		case tOffADC:
			SET_BIT(GPIOC->ODR,GPIO_PIN_3);
			break;
		case tSetOuter:
			p_outerdata = (OuterData*)(buf + HEADER_SIZE);
			SetFreqOuter(p_outerdata->freqOuter,p_outerdata->Nperiod,p_outerdata->lenPSP);
			setPSP(&p_outerdata->pspMas[0]);
//			p_addrADCsmpl->type = tADCsmplOuter;
//			p_addrADCsmpl->data0 = ipos;		
			StartOuter();
			break;
	
		default:
			break;
	}
  // Data received
  /* Example
  if ((buf[0] == 0x01) && (len == 2)) {
    // Switch LEDs on and off
    // LED_out (buf[1]);
  }
  */
  return (0);
}
 
void fillQueue(){
	uint8_t *mas;
	uint32_t addr;
	netBuf* p_buf;
//	while( osMessageQueueGetCount(adc_buf) < NUMBER_BUF){
//		mas = netUDP_GetBuffer(BUF_SIZE);	
//		addr = (uint32_t)mas;		
//		p_buf = (netBuf*)mas;
//		p_buf->type=tADCsmpl;
//		if(mas)
//			osMessageQueuePut(adc_buf,&addr,NULL,0);
//		else
//			break;
//	}
}

void udp_Task(void *argument) {	
	netInitialize ();
	osStatus_t stat;
	volatile netStatus nstat;
	netBuf* p_buf;
	uint32_t addr_send;	
	uint8_t *mas;
		
	udp_sock = netUDP_GetSocket (udp_cb_func);
  if (udp_sock > 0)
    netUDP_Open (udp_sock, 8000);
//	tcp_sock = netTCP_GetSocket (tcp_cb_server);
//  if (tcp_sock > 0) {
//    netTCP_Listen (tcp_sock, 9000);
//	}
	
//	adc_buf = osMessageQueueNew(NUMBER_BUF, sizeof(uint8_t*), NULL);
//	send_buf = osMessageQueueNew(NUMBER_BUF, sizeof(uint8_t*), NULL);
	fillQueue();
	while(1){
		addr_send = fifoeth.getBuf();
		p_buf = (netBuf*)addr_send;
		p_buf->counter=cnt_adc;
		cnt_adc++;	
		nstat = netUDP_Send (udp_sock, &addr_pc, (uint8_t*)addr_send, BUF_SIZE);
	}
}
