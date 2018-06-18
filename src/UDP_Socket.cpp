#include "thread.h"
#include "rl_net.h"

#include "invensense.h"






int32_t udp_sock;                       // UDP socket handle
int32_t connStat = -1;

NET_ADDR addr_pc; 
volatile uint16_t cnt_adc = 0;
extern int32_t enSendQuat; 
extern volatile uint16_t cntQuat;
extern osSemaphoreId_t qspiSem_id;	
int32_t isInit = 0;
// Notify the user application about UDP socket events.
uint32_t masU32[200];
uint32_t udp_cb_func (int32_t socket, const  NET_ADDR *addr, const uint8_t *buf, uint32_t len) {
	
	const netHeader *p = (const netHeader*) buf;
	uint32_t freq = 0;
	OuterData *outdata;
    uint32_t sizes;
    FLASH_EraseInitTypeDef pEraseInit;
	volatile float freqP = 2 * HAL_RCC_GetPCLK1Freq();
	switch(p->type){
		case tInitConn:
            isInit = 1;
			addr_pc = *addr;
			break;
		case tStartADC:
            if(isInit ==1){
                SET_BIT(GPIOB->ODR,GPIO_PIN_14);
                cnt_adc = 0;
                sendUartCommand((uint8_t*)buf, (uint8_t*)buf+HEADER_SIZE, (len - HEADER_SIZE) );
            }
			break;
		case tStopADC:
			CLEAR_BIT(GPIOB->ODR,GPIO_PIN_14);
			sendUartCommand((uint8_t*)buf, (uint8_t*)buf+HEADER_SIZE, (len - HEADER_SIZE) );
			break;
		case tSetFreqADC:
			sendUartCommand((uint8_t*)buf, (uint8_t*)buf+HEADER_SIZE, (len - HEADER_SIZE) );
			break;
		case tLogADC:
			sendUartCommand((uint8_t*)buf, (uint8_t*)buf+HEADER_SIZE, (len - HEADER_SIZE) );
			break;
		case tLinADC:
			sendUartCommand((uint8_t*)buf, (uint8_t*)buf+HEADER_SIZE, (len - HEADER_SIZE) );
			break;
		case tOffADC:
			sendUartCommand((uint8_t*)buf, (uint8_t*)buf+HEADER_SIZE, (len - HEADER_SIZE) );
			break;
		case tSetOuter:
			outdata = (OuterData*)(buf + HEADER_SIZE);			
			SetFreqOuter(outdata->freqOuter,outdata->Nperiod,outdata->lenPSP);
			setPSP(&outdata->pspMas[0]);
			preStartOuter();
			needOuterSignal = 1;
			sendUartCommand((uint8_t*)buf, (uint8_t*)buf+HEADER_SIZE, 0 );
			//SetFreqOuter(p_outerdata->freqOuter,p_outerdata->Nperiod,p_outerdata->lenPSP);
			//setPSP(&p_outerdata->pspMas[0]);
//			p_addrADCsmpl->type = tADCsmplOuter;
//			p_addrADCsmpl->data0 = ipos;		
			//StartOuter();
			break;
        case tStartQuat:
            if(isInit ==1){
                cntQuat = 0;
                enSendQuat = 1;
            }
			break;
        case tStopQuat:
            enSendQuat = 0;
			break;
        case tQuatSave:
            uint32_t errCode;
            inv_get_mpl_state_size(&sizes);
            inv_save_mpl_states((uint8_t*)&masU32[0],sizes);
            HAL_FLASHEx_Unlock_Bank1();
            pEraseInit.Banks = FLASH_BANK_1;
            pEraseInit.NbSectors = 1;
            pEraseInit.Sector = FLASH_SECTOR_1;
            pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
            pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_1;
            HAL_FLASHEx_Erase(&pEraseInit,&errCode);
        for(volatile int i = 0 ; i < sizes ; i+=32){
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,0x08020000 + i,(uint64_t)&masU32[i/4]);
        }
            HAL_FLASHEx_Lock_Bank1();
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
	int32_t startval = fifoqspi.getCurrentSize();
	volatile int i;
	for(i = startval; i < fifoqspi.getBufNumber(); i++){
		mas = netUDP_GetBuffer(BUF_SIZE);	
		if(mas){
			fifoqspi.putBuf((uint32_t)mas);
		}
		
	}
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
	volatile int ff = 0;
	fillQueue();
	osSemaphoreRelease(qspiSem_id);
	while(1){
		addr_send = fifoeth.getBuf();
		p_buf = (netBuf*)addr_send;
		//p_buf->type=tADCsmpl;
		//p_buf->counter=p_buf->counter;
		//cnt_adc++;	
		//SCB_InvalidateDCache_by_Addr((uint32_t*)addr_send,1452);
		nstat = netUDP_Send (udp_sock, &addr_pc, (uint8_t*)&p_buf->type, BUF_SIZE);
		if(nstat != netOK){
			ff++;
		}
		fillQueue();		
	}
}
