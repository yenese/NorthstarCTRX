#ifndef NRF21540_H
#define NRF21540_H

#include <stdint.h>
#include <zephyr/device.h>

// Status
typedef enum {
    NRF21540_OK = 0,
    NRF21540_ERR = -1
} nrf21540_status_t;

typedef struct {
    const struct device *spi_dev;
    const struct device *gpio_dev;
    uint8_t csn_pin;
    uint8_t pdn_pin;
    uint8_t tx_gain;
} nrf21540_hw_t;

typedef struct {
    nrf21540_hw_t hw;
} nrf21540_dev_t;


nrf21540_status_t nrf21540_read_register(nrf21540_dev_t *dev, uint8_t reg, uint8_t *val);
nrf21540_status_t nrf21540_write_register(nrf21540_dev_t *dev, uint8_t reg, uint8_t val, uint8_t *old_val);
nrf21540_status_t nrf21540_init(nrf21540_dev_t *dev, nrf21540_hw_t *hw);
nrf21540_status_t nrf21540_setup(nrf21540_dev_t *nrf21540_dev, nrf21540_hw_t *hw_config);
nrf21540_status_t nrf21540_set_tx_gain(nrf21540_dev_t *dev, uint8_t gain);
nrf21540_status_t nrf21540_enable_tx(nrf21540_dev_t *dev);
nrf21540_status_t nrf21540_enable_rx(nrf21540_dev_t *dev);
nrf21540_status_t nrf21540_standby(nrf21540_dev_t *dev);

// Timing (us)
#define NRF21540_T_TRX_TO_PD_US   5
#define NRF21540_T_PD_TO_PG_US    20
#define NRF21540_T_PG_TO_TRX_US   13
#define NRF21540_T_TRX_TO_PG_US   5

#endif


