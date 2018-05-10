#include "MultiFifo.h"




void MultiFifo::setBufSize(int32_t val, int32_t internal){
	bufsize = val;
	reloadParam(internal);
}

int32_t MultiFifo::getBufSize(){
	return bufsize;
}

void MultiFifo::setBufNumber(int32_t val, int32_t internal){
	bufnumber = val;
	reloadParam(internal);
}

int32_t MultiFifo::getBufNumber(){	
	return bufnumber;
}


void MultiFifo::init(int32_t internal){	
	reloadParam(internal);
}

MultiFifo::MultiFifo(){
	bufnumber = 10;
	bufsize = 1000;
}

MultiFifo::~MultiFifo(){
	
}

MultiFifo::MultiFifo(int32_t _bufsize, int32_t _bufnumber){
	bufnumber = _bufnumber;
	bufsize = _bufsize;
}


int32_t MultiFifo::reloadParam(int32_t internal){
	//----queue
	if(adc_buf)
		osMessageQueueDelete(adc_buf);
	adc_buf = osMessageQueueNew(bufnumber, sizeof(uint8_t*), NULL);
	if (adc_buf == NULL) {
    return 1;
  }
	//-----memorypool
	if(internal == 1){
		if(memory)
			osMemoryPoolDelete(memory);
		memory = osMemoryPoolNew(bufnumber,bufsize,NULL);
		if (memory == NULL) {
			return 2;
		}
	}
	
	
	return 0;
}

void MultiFifo::fillFifo(){
		for(volatile int i = 0 ; i < bufnumber ; i++){
			putBuf();
		}
}

uint32_t MultiFifo::getBuf(uint32_t timeout){	
	osStatus_t stat;
	uint32_t addr_send;
	stat = osMessageQueueGet(adc_buf,&addr_send,NULL,timeout);
	if(stat == osOK)	{	
		return addr_send;
	}
	return 0;
}

void MultiFifo::putBuf(uint32_t addr){
	osMessageQueuePut(adc_buf,&addr,NULL,NULL);	
}

void MultiFifo::putBuf(){
	uint8_t* mas;
	mas = (uint8_t*)osMemoryPoolAlloc(memory,NULL);
	uint32_t addr = (uint32_t)mas;
	if(addr == 0) 
		return;
	osMessageQueuePut(adc_buf,&addr,NULL,NULL);
}

void MultiFifo::freeBlock(uint32_t addr){
	osMemoryPoolFree(adc_buf,(void*)addr);
}













