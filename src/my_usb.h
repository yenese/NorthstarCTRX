#ifndef MY_USB_H_
#define MY_USB_H_

#include <stdint.h>
#include <stdarg.h>
#include <zephyr/device.h>

#define USB_RX_TIMEOUT_MS   100
#define FIFO_READ_CHUNK     64

int8_t   usbInit(void);
uint16_t usbAvailable(void);
int16_t  usbRead(uint8_t* pRxData, uint16_t maxLen);
int16_t  usbReadByte(void);
int16_t  usbReceive(uint8_t* pRxData, uint16_t len);
int8_t   usbTransmit(const uint8_t* pTxData, uint16_t len);
void     usbCallback(const struct device *dev, void *user_data);
void     usbPrint(char* format, ...);

#endif 
