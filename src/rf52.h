
#ifndef RF52_H_
#define RF52_H_

#include <stdint.h>

typedef void(*rf52RxCallback)(uint8_t *pRxData, uint16_t len);

int8_t rf52Init(rf52RxCallback rxCallback);
int8_t rf52Transmit(uint8_t* pTxBuffer, uint16_t len);
void   rf52SetChannel(uint8_t ch);

#endif