#include "nrf21540.h"
#include "my_spi.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdint.h>


LOG_MODULE_REGISTER(nrf21540, LOG_LEVEL_DBG);

#define REG_CONFREG0    0x00
#define REG_CONFREG1    0x01

nrf21540_status_t nrf21540_read_register(nrf21540_dev_t *dev, uint8_t reg, uint8_t *val) {
    int8_t status = nrf21540_spi_read_register(
        dev->hw.spi_dev, dev->hw.gpio_dev, dev->hw.csn_pin, reg, val
    );
    return (status == 0) ? NRF21540_OK : NRF21540_ERR;
}

nrf21540_status_t nrf21540_write_register(nrf21540_dev_t *dev, uint8_t reg, uint8_t val, uint8_t *old_val) {
    int8_t status = nrf21540_spi_write_register(
        dev->hw.spi_dev, dev->hw.gpio_dev, dev->hw.csn_pin, reg, val, old_val
    );
    return (status == 0) ? NRF21540_OK : NRF21540_ERR;
}

// INIT (sadece PDN pini GPIO, diğerleri SPI)
nrf21540_status_t nrf21540_init(nrf21540_dev_t *dev, nrf21540_hw_t *hw)
{
    memcpy(&dev->hw, hw, sizeof(nrf21540_hw_t));

    gpio_pin_configure(dev->hw.gpio_dev, dev->hw.pdn_pin, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(dev->hw.gpio_dev, dev->hw.csn_pin, GPIO_OUTPUT_ACTIVE);

    gpio_pin_set(dev->hw.gpio_dev, dev->hw.pdn_pin, 0); // Power down
    k_busy_wait(NRF21540_T_TRX_TO_PD_US);
    gpio_pin_set(dev->hw.gpio_dev, dev->hw.pdn_pin, 1); // Power up
    k_busy_wait(NRF21540_T_PD_TO_PG_US);

    // Standby (TX/RX disable, gain=initial_gain)
    nrf21540_standby(dev);
    nrf21540_set_tx_gain(dev, hw->tx_gain);

    return NRF21540_OK;
}

nrf21540_status_t nrf21540_setup(nrf21540_dev_t *nrf21540_dev, nrf21540_hw_t *hw_config)
{

    if (!device_is_ready(hw_config->spi_dev) || !device_is_ready(hw_config->gpio_dev)) {
        LOG_ERR("SPI or GPIO not ready");
        return -ENODEV;
    }


    if (nrf21540_init(nrf21540_dev, hw_config) != NRF21540_OK) {
        LOG_ERR("nRF21540 init error!");
        return -EIO;
    }

    uint8_t part_number;
    if (nrf21540_read_register(nrf21540_dev, 0x14, &part_number) == NRF21540_OK)
        LOG_INF("nRF21540 Part Number: 0x%02X", part_number);
    else
        LOG_ERR("nRF21540 chip read fail");

    nrf21540_standby(nrf21540_dev);
    LOG_INF("nRF21540 setup complete, gain=%d, standby mode", hw_config->tx_gain);
    return NRF21540_OK;
}



nrf21540_status_t nrf21540_set_tx_gain(nrf21540_dev_t *dev, uint8_t gain)
{
    uint8_t confreg0 = ((gain & 0x1F) << 2);  // Sadece gain, TX ve MODE disable
    return nrf21540_write_register(dev, REG_CONFREG0, confreg0, NULL);
}

// TX mode: MODE=0, TX_EN=1, RX_EN=0, gain ayarla
nrf21540_status_t nrf21540_enable_tx(nrf21540_dev_t *dev)
{
    uint8_t confreg0 = (1 << 0)              // TX_EN = 1
                     | (0 << 1)              // MODE = 0 (POUTA)
                     | ((dev->hw.tx_gain & 0x1F) << 2); // GAIN
    nrf21540_write_register(dev, REG_CONFREG0, confreg0, NULL);
    nrf21540_write_register(dev, REG_CONFREG1, 0x00, NULL); // RX_EN=0
    k_busy_wait(NRF21540_T_PG_TO_TRX_US);
    return NRF21540_OK;
}

// RX mode: MODE=0, TX_EN=0, RX_EN=1
nrf21540_status_t nrf21540_enable_rx(nrf21540_dev_t *dev)
{
    uint8_t confreg0 = 0x00; // TX_EN=0, MODE=0, gain=0
    nrf21540_write_register(dev, REG_CONFREG0, confreg0, NULL);
    nrf21540_write_register(dev, REG_CONFREG1, 0x01, NULL); // RX_EN=1
    k_busy_wait(NRF21540_T_PG_TO_TRX_US);
    return NRF21540_OK;
}

// Standby (TX_EN=0, RX_EN=0, gain=0)
nrf21540_status_t nrf21540_standby(nrf21540_dev_t *dev)
{
    nrf21540_write_register(dev, REG_CONFREG0, 0x00, NULL);
    nrf21540_write_register(dev, REG_CONFREG1, 0x00, NULL);
    k_busy_wait(NRF21540_T_TRX_TO_PG_US);
    return NRF21540_OK;
}

