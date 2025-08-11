#ifndef LED_H_
#define LED_H_

#include <stdint.h>

void ledInit(void);
void ledWrite(uint8_t led, uint8_t val);
void ledToggle(uint8_t led);

#endif