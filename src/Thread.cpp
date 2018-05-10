#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include "Thread.h"
#include "string.h"
#include "main.h"
#include "arm_math.h"
#include "MultiFifo.h"




osThreadId_t tid_Thread;                                      // thread id
	
MultiFifo fifoqspi(BUF_SIZE,5);
MultiFifo fifoeth(BUF_SIZE,5);

void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi){

}



void sendUartCommand(uint8_t* header, uint8_t* data,uint32_t len){
	HAL_UART_Transmit_DMA(&huart4,header,HEADER_SIZE);
}

void initQSPIcomm(){
	 QSPI_CommandTypeDef s_command;
	
  s_command.Instruction       = 4;
  s_command.Address           = 0;
	s_command.AlternateBytes		= 2;	
  s_command.AddressSize       = QSPI_ADDRESS_8_BITS;
	s_command.AddressSize				= QSPI_ALTERNATE_BYTES_8_BITS;	
  s_command.DummyCycles       = 0;
	s_command.InstructionMode   = QSPI_INSTRUCTION_NONE;
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.NbData            = 1440;
	s_command.DdrMode						= QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle	= QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  
  /* Configure the command */
  HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
}

extern "C" void Thread (void *argument) {		
	fifoqspi.init(0);
	fifoeth.init(0);
  while (1) {
    osDelay(1000);    
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
}






