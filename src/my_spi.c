#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <stdint.h>
#include "my_spi.h"

struct spi_config spi_cfg = {
    .frequency = 8000000U,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB, // Mode 0 (CPOL=0, CPHA=0)
    .slave = 0,
    .cs = NULL // Manuel CS kontrolü
};

// 16-bit transfer: 1. byte komut/adres, 2. byte data/dummy
int nrf21540_spi_read_register(const struct device *spi_dev,
                               const struct device *gpio_dev,
                               uint8_t csn_pin,
                               uint8_t reg,
                               uint8_t *rx_value)
{
    uint8_t tx_buf[2] = { 0x80 | (reg & 0x3F), 0x00 }; // READ: b10xxxxxx
    uint8_t rx_buf[2] = { 0x00, 0x00 };

    struct spi_buf tx_bufs[] = { { .buf = tx_buf, .len = 2 } };
    struct spi_buf rx_bufs[] = { { .buf = rx_buf, .len = 2 } };

    struct spi_buf_set tx = { .buffers = tx_bufs, .count = 1 };
    struct spi_buf_set rx = { .buffers = rx_bufs, .count = 1 };

    gpio_pin_set(gpio_dev, csn_pin, 0);
    int err = spi_transceive(spi_dev, &spi_cfg , &tx, &rx);
    gpio_pin_set(gpio_dev, csn_pin, 1);

    if (err == 0 && rx_value) {
        *rx_value = rx_buf[1];
    }
    return err;
}

// nRF21540 register WRITE: b11xxxxxx + data
int nrf21540_spi_write_register(const struct device *spi_dev,
                                const struct device *gpio_dev,
                                uint8_t csn_pin,
                                uint8_t reg,
                                uint8_t data,
                                uint8_t *old_value)
{
    uint8_t tx_buf[2] = { 0xC0 | (reg & 0x3F), data }; // WRITE: b11xxxxxx
    uint8_t rx_buf[2] = { 0x00, 0x00 };

    struct spi_buf tx_bufs[] = { { .buf = tx_buf, .len = 2 } };
    struct spi_buf rx_bufs[] = { { .buf = rx_buf, .len = 2 } };

    struct spi_buf_set tx = { .buffers = tx_bufs, .count = 1 };
    struct spi_buf_set rx = { .buffers = rx_bufs, .count = 1 };

    gpio_pin_set(gpio_dev, csn_pin, 0);
    int err = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
    gpio_pin_set(gpio_dev, csn_pin, 1);

    // Önceki register değeri rx_buf[1]'de gelir (datasheet)
    if (err == 0 && old_value) {
        *old_value = rx_buf[1];
    }
    return err;
}
