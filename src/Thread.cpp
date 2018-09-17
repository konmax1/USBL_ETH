#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include "Thread.h"
#include "string.h"
#include "main.h"
#include "arm_math.h"
#include "MultiFifo.h"




osThreadId_t tid_Thread;                                      // thread id
osThreadId_t tid_udp_Task; 
osThreadId_t tid_qspi_Task; 
osSemaphoreId_t uartTx_id;	
osSemaphoreId_t qspiSem_id;	

void udp_Task(void *argument) ;
void qspi_Task(void *argument) ;
MultiFifo fifoqspi(BUF_SIZE,8, "QueueQSPI");
MultiFifo fifoeth(BUF_SIZE,8, "QueueEth");

uint32_t dataMPL[128*1024/4] __attribute__((section(".ARM.__at_0x08020000")));

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
}

int16_t ch0[20];
int16_t cnt = 0;
uint16_t bufADC[1600];
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
	osSemaphoreRelease(qspiSem_id);
	fifoeth.putBuf(DMA1_Stream2->M0AR);    
	ch0[cnt] = ((netBuf*)(DMA1_Stream2->M0AR))->mas[89][4];
    cnt = (cnt + 1)%20;
    
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	osSemaphoreRelease(qspiSem_id);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
	osSemaphoreRelease(qspiSem_id);
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi){
	osSemaphoreRelease(qspiSem_id);
}

//void initQSPIcomm(){
//	 QSPI_CommandTypeDef s_command;
//	
//  s_command.Instruction       = 4;
//  s_command.Address           = 0;
//	s_command.AlternateBytes		= 2;	
//  s_command.AddressSize       = QSPI_ADDRESS_8_BITS;
//	s_command.AddressSize				= QSPI_ALTERNATE_BYTES_8_BITS;	
//  s_command.DummyCycles       = 0;
//	s_command.InstructionMode   = QSPI_INSTRUCTION_NONE;
//	s_command.AddressMode       = QSPI_ADDRESS_NONE;
//  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
//  s_command.DataMode          = QSPI_DATA_4_LINES;
//  s_command.NbData            = 1440;
//	s_command.DdrMode						= QSPI_DDR_MODE_DISABLE;
//	s_command.DdrHoldHalfCycle	= QSPI_DDR_HHC_ANALOG_DELAY;
//  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
//  
//  /* Configure the command */
//  HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
//}
void qspi_Task (void *argument) {		
	uint32_t addr_rec;
	//initQSPIcomm();
	osSemaphoreAttr_t attr;
	memset(&attr,0,sizeof(attr));
	attr.name = "QSPI_RX";
	qspiSem_id = osSemaphoreNew(1,0,&attr);   
  while (1) {	
		osSemaphoreAcquire(qspiSem_id,osWaitForever);
		addr_rec = fifoqspi.getBuf();      
		//addr_rec = GetBUF();
        //addr_rec = (uint32_t)&bufADC[0];
		if(addr_rec){
			//SCB_InvalidateDCache_by_Addr((uint32_t*)addr_rec,PacketSizeBytes);
			//HAL_QSPI_Receive_DMA(&hqspi,((uint8_t*)addr_rec) + 12);
            //SCB_CleanDCache_by_Addr((uint32_t*)addr_rec,PacketSizeShort+32);
            //SCB_CleanInvalidateDCache_by_Addr((uint32_t*)((uint32_t)addr_rec & 0xFFFFFFE0),PacketSizeBytes+32);
			HAL_SPI_Receive_DMA(&hspi1,(uint8_t*)addr_rec,PacketSizeShort);
		}else{
            printf("addr not valid\n");
        }
  }
}

extern "C" void Thread (void *argument) {		
	osThreadAttr_t worker_attr;
	memset(&worker_attr, 0, sizeof(worker_attr));
    worker_attr.stack_size = 2000;
    uint8_t uart_init[12];
    netHeader *pn = (netHeader*)&uart_init[0]; 
    pn->type = tStopADC;
	fifoqspi.init(0);
	fifoeth.init(0);    
    
        
    *(__IO uint32_t*)(0x5C001004) = 0x00700000;
    *(__IO uint32_t*)(0x5C004FB0) = 0xC5ACCE55; //UNLOCK FUNNEL
    *(__IO uint32_t*)(0x5C003FB0) = 0xC5ACCE55; //UNLOCK FUNNEL
    *(__IO uint32_t*)(0x5C003010) = (*(__IO uint32_t*)(0x5C003010) & 0xfffff000) | 0x000000C2; //SWO current output divisor register
    *(__IO uint32_t*)(0x5C0030F0) = 0x00000002; //SWO selected pin protocol register 
    *(__IO uint32_t*)(0x5C004000) = *(__IO uint32_t*)(0x5C004000) | 0x00000001;  //Enable ITM input of SWO trace funne
    *(__IO uint32_t*)(0x580244E0) = *(__IO uint32_t*)(0x580244E0) | 0x00000002;    //RCC_AHB4ENR enable GPIOB clock
    *(__IO uint32_t*)(0x58020400) = (*(__IO uint32_t*)(0x58020400) & 0xffffff3f) | 0x00000080;  // Configure GPIOB pin 3 as AF
    *(__IO uint32_t*)(0x58020408) = *(__IO uint32_t*)(0x58020408) | 0x00000080;    // Configure GPIOB pin 3 Speed
    *(__IO uint32_t*)(0x58020420) = *(__IO uint32_t*)(0x58020420) & 0xFFFF0FFF; // Force AF0 for GPIOB pin 3

    printf("Started\n");
	uartTx_id = osSemaphoreNew(1,1,NULL);
    
	sendUartCommand((uint8_t*)pn, (uint8_t*)pn+HEADER_SIZE, (12 - HEADER_SIZE) );   
    
    worker_attr.priority = osPriorityNormal;
    worker_attr.stack_size = 2000;
	tid_udp_Task = osThreadNew (udp_Task, NULL, &worker_attr);
    
    worker_attr.priority = osPriorityBelowNormal;
    worker_attr.stack_size = 4000;
	tid_InvensenseTask = osThreadNew (InvensenseTask, NULL, &worker_attr);	
    
	worker_attr.priority = osPriorityHigh;
    worker_attr.stack_size = 2000;
	tid_qspi_Task = osThreadNew (qspi_Task, NULL, &worker_attr);	
  while (1) {
    osDelay(1000);    
  }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	osSemaphoreRelease(uartTx_id);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	osSemaphoreRelease(uartTx_id);
}

void sendUartCommand(uint8_t* header,uint8_t* data,uint32_t len){
	osStatus_t stat;
	
	stat = osSemaphoreAcquire(uartTx_id,300);
	if(stat != osOK)
		return;
	HAL_UART_Transmit_DMA(&huart4,header,HEADER_SIZE);
	
	if(len >0){
		osDelay(1);
		stat = osSemaphoreAcquire(uartTx_id,300);
		if(stat != osOK)
			return;
		HAL_UART_Transmit_DMA(&huart4,data,len);
	}
}




