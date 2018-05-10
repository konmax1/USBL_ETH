#pragma once
#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include "stm32h7xx_hal.h"
#include "string.h"
#include "stm32h7xx.h"                  // Device header
#include "main.h"
#include "Multififo.h"



#define HEADER_SIZE (12)
#define SMPL_CNT (90)
#define BUF_SIZE (SMPL_CNT * 2 * 8 + HEADER_SIZE)


struct adcBuffer{
	int16_t mas[SMPL_CNT][8];
};

enum MessageStat : int32_t{
	mOK = 1,
	mERR = -1,
	mTimeout = -2,
};

enum typeCMD : uint16_t{
	tInitConn			=	0x0001,
	tStartADC			=	0x0002,
	tStopADC 			=	0x0003,
	tSetFreqADC		= 0x0004,
	tADCsmpl 			=	0x0005,	
	tSetOuter			=	0x0006,
	tADCsmplOuter	=	0x0007,
	tLogADC				=	0x0010,
	tLinADC				=	0x0011,
	tOffADC				=	0x0012,
	tErr					=	0x00FF,
	tUnknown			=	0xFFFF,
};

struct netHeader{
	typeCMD type;
	uint16_t counter;
	uint16_t data0;
	uint16_t data1;
	uint16_t data2;
	uint16_t data3;
};

struct OuterData{
	uint32_t freqOuter;
	int32_t Nperiod;
	int32_t lenPSP;
	uint8_t pspMas[511];
};

struct netBuf{
	typeCMD type;
	uint16_t counter;	
	uint16_t data0;
	uint16_t data1;
	uint16_t data2;
	uint16_t data3;
	int16_t mas[SMPL_CNT][8];
};

void sendUartCommand(uint8_t* header, uint8_t* data,uint32_t len);

extern osThreadId_t tid_Thread;                                      // thread id

extern TIM_HandleTypeDef htim1;

extern DMA_HandleTypeDef hdma_memtomem_dma2_stream2;

extern QSPI_HandleTypeDef hqspi;

extern MDMA_HandleTypeDef hmdma_quadspi_fifo_th;

extern UART_HandleTypeDef huart4;

extern MultiFifo fifoqspi;
extern MultiFifo fifoeth;



