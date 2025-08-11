#include "my_usb.h"
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#define RING_BUF_SIZE 1024  
#define FIFO_READ_CHUNK 128
#define USB_RX_TIMEOUT_MS 1000

static uint8_t ring_buffer[RING_BUF_SIZE];
static struct ring_buf usb_rx_ringbuf;
static volatile uint32_t usb_rx_overflow_count = 0;

static struct k_sem usb_rx_sem; /* SEMAPHORE */
static struct k_sem usb_tx_sem; 

const struct device *cdc_uart = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
 
int8_t usbInit(void){
    /* USB Start Listening */
    if (!cdc_uart || !device_is_ready(cdc_uart)) return -1;

    int ret = usb_enable(NULL);
    
    if (ret != 0) return -2;

    ring_buf_init(&usb_rx_ringbuf, sizeof(ring_buffer), ring_buffer);
    k_sem_init(&usb_rx_sem, 0, K_SEM_MAX_LIMIT);
    k_sem_init(&usb_tx_sem, 1, 1);  // TX available on start

    uart_irq_callback_set(cdc_uart, usbCallback);
    uart_irq_rx_enable(cdc_uart);

    return 0;
}

// Arduino Serial.available() + Serial.read() mantığı
int16_t usbRead(uint8_t* pRxData, uint16_t maxLen) {
    // Ring buffer'da veri var mı kontrol et (non-blocking)
    uint32_t available_bytes = ring_buf_size_get(&usb_rx_ringbuf);
    
    if (available_bytes == 0) {
        return 0; // Veri yok
    }
    
    // Maksimum okuma miktarını belirle
    uint16_t read_len = (available_bytes < maxLen) ? available_bytes : maxLen;
    
    // Veriyi oku
    uint32_t actual_read = ring_buf_get(&usb_rx_ringbuf, pRxData, read_len);
    
    return (int16_t)actual_read;
}

// Sadece mevcut veri miktarını döndür (Arduino Serial.available() gibi)
uint16_t usbAvailable(void) {
    return (uint16_t)ring_buf_size_get(&usb_rx_ringbuf);
}

// Tek byte oku (Arduino Serial.read() gibi)
int16_t usbReadByte(void) {
    uint8_t byte;
    if (ring_buf_get(&usb_rx_ringbuf, &byte, 1) == 1) {
        return byte;
    }
    return -1; // Veri yok
}

// Orijinal blocking receive fonksiyonu (eski davranış için)
int16_t usbReceive(uint8_t* pRxData, uint16_t len) {
    if (k_sem_take(&usb_rx_sem, K_MSEC(USB_RX_TIMEOUT_MS)) != 0) return -1; // Timeout

    uint32_t read_len = ring_buf_get(&usb_rx_ringbuf, pRxData, len); /* Read Data */
    
    if (!ring_buf_is_empty(&usb_rx_ringbuf)) { k_sem_give(&usb_rx_sem); /* Give semaphore again */ }

    return (int16_t)read_len;
}

int8_t usbTransmit(const uint8_t* pTxData, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) { uart_poll_out(cdc_uart, pTxData[i]); }
    return 0;
}

void usbCallback(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);
    
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        
        // RX interrupt
        if (uart_irq_rx_ready(dev)) {
            uint8_t buf[FIFO_READ_CHUNK];
            int recv_len = uart_fifo_read(dev, buf, sizeof(buf));
            
            if (recv_len > 0) {
                uint32_t stored_len = ring_buf_put(&usb_rx_ringbuf, buf, recv_len);
                
                if (stored_len < recv_len) {
                    usb_rx_overflow_count += (recv_len - stored_len);
                }
                
                k_sem_give(&usb_rx_sem);
            }
        }

        // TX interrupt
        if (uart_irq_tx_ready(dev)) {
            uart_irq_tx_disable(dev);  // TX interrupt'ı kapat
            k_sem_give(&usb_tx_sem);   // TX tamamlandı sinyali
        }
    }
}