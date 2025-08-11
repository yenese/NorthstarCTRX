#ifndef MY_SPI_H_
#define MY_SPI_H_

#include <stdint.h>
#include <zephyr/device.h>

int nrf21540_spi_read_register(const struct device *spi_dev,
                               const struct device *gpio_dev,
                               uint8_t csn_pin,
                               uint8_t reg,
                               uint8_t *rx_value);

int nrf21540_spi_write_register(const struct device *spi_dev,
                                const struct device *gpio_dev,
                                uint8_t csn_pin,
                                uint8_t reg,
                                uint8_t data,
                                 uint8_t *old_value); // Eski değeri döndürür (isteğe bağlı)



#endif 
