#include "rf52.h"
#include "nrf21540.h"
#include "my_esb.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <esb.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

static struct esb_payload rx_payload;
static nrf21540_dev_t nrf21540_dev;
static volatile bool tx_done_flag = false;
static rf52RxCallback user_rx_callback = NULL;

void esbCallback(const struct esb_evt *event){
    switch (event->evt_id) {
        case ESB_EVENT_RX_RECEIVED:
            while (esb_read_rx_payload(&rx_payload) == 0) {
                if (user_rx_callback != NULL) {
                    user_rx_callback(rx_payload.data, rx_payload.length);
                }
            }
            break;
        case ESB_EVENT_TX_SUCCESS:
        case ESB_EVENT_TX_FAILED:
            tx_done_flag = true;
            break;
        default:
            break;
    }
}

int8_t rf52Init(rf52RxCallback rxCallback)
{
    user_rx_callback = rxCallback;
        
    /* NRF21540 Config */
    #define GPIO_NODE DT_NODELABEL(gpio0)
    const struct device *gpio_dev = DEVICE_DT_GET(GPIO_NODE);
    #define SPI_NODE DT_NODELABEL(spi1)
    const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);
    int err = 0;
    #define TX_GAIN_VALUE       31
    #define NRF21540_CSN_PIN    2
    #define NRF21540_PDN_PIN    8
    nrf21540_hw_t hw_config = {
        .spi_dev = spi_dev,
        .gpio_dev = gpio_dev,
        .csn_pin = NRF21540_CSN_PIN,
        .pdn_pin = NRF21540_PDN_PIN,
        .tx_gain = TX_GAIN_VALUE
    };

    /* ESB Config */
    my_esb_config_t esb_config;
    esb_config.mode = ESB_RX;
    esb_config.event_handler = esbCallback;
    esb_config.bitrate = MY_ESB_BITRATE_1MBPS;
    esb_config.tx_power = MY_ESB_TX_POWER_4DBM;
    esb_config.crc_mode = MY_ESB_CRC_16BIT;
    esb_config.selective_auto_ack = true;
    esb_config.rf_channel = 60;
    esb_config.retransmit_delay = 600;
    esb_config.retransmit_count = 3;

    if (nrf21540_setup(&nrf21540_dev, &hw_config) != 0) {
        return -1;
    }
    if (my_esb_initialize(&esb_config) != 0) {
        return -1;
    }
    
    my_esb_print_config();
    
    /* ESB Start Listening */
    nrf21540_enable_rx(&nrf21540_dev);
    esb_start_rx();
    return 0;
}

int8_t rf52Transmit(uint8_t* pTxBuffer, uint16_t len)
{
    esb_stop_rx();
    nrf21540_enable_tx(&nrf21540_dev);
    k_busy_wait(20);

    esb_flush_tx();
    my_esb_transmit(pTxBuffer, len);

    while (!tx_done_flag) {
        k_msleep(1);
    }
    tx_done_flag = false;

    nrf21540_enable_rx(&nrf21540_dev);
    esb_start_rx();
    
    return 0;
}

void rf52SetChannel(uint8_t ch){
    my_esb_set_rf_channel(ch);
}
