#include "my_usb.h"
#include "nrf21540.h"
#include "my_esb.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <esb.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/uart.h>

#include "led.h"

LOG_MODULE_REGISTER(esb_bridge, LOG_LEVEL_DBG);
 
static struct esb_payload rx_payload;
static nrf21540_dev_t nrf21540_dev;
 
static volatile bool tx_done_flag = false;
 
void esbCallback(const struct esb_evt *event)
{
    switch (event->evt_id) {
        case ESB_EVENT_RX_RECEIVED:
			ledToggle(0);
            while (esb_read_rx_payload(&rx_payload) == 0) {
                usbTransmit(rx_payload.data, rx_payload.length);
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
 
#if defined(CONFIG_CLOCK_CONTROL_NRF)
int clocks_start(void)
{
    int err, res;
    struct onoff_manager *clk_mgr;
    struct onoff_client clk_cli;
    clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (!clk_mgr) return -ENXIO;
    sys_notify_init_spinwait(&clk_cli.notify);
    err = onoff_request(clk_mgr, &clk_cli);
    if (err < 0) return err;
    do {
        err = sys_notify_fetch_result(&clk_cli.notify, &res);
        if (!err && res) return res;
    } while (err);
    LOG_DBG("HF clock started");
    return 0;
}
#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif

void esbTransmit(uint8_t* pTxBuffer, uint16_t len){

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
}

int main()
{
	if (clocks_start() != 0) return 0;
    
	ledInit();
	if (usbInit() != 0){ LOG_ERR("USB init failed."); return 0; }
	else LOG_ERR("USB init complete!");
	
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
 
    if (nrf21540_setup(&nrf21540_dev, &hw_config) != 0) {LOG_ERR("nRF21540 setup failed, err %d", err); return 0;}
    if (my_esb_initialize(&esb_config) != 0) {LOG_ERR("ESB initialization failed, err %d", err); return 0;}
	
	my_esb_print_config();
	
	/* ESB Start Listening */
	nrf21540_enable_rx(&nrf21540_dev);
	esb_start_rx();

    LOG_INF("Initialization complete"); 
 	while (1){
		uint8_t esbTxBuffer[32];
		uint16_t len = usbAvailable();
		if(len > 0){
			usbRead(esbTxBuffer, len);
			esbTransmit(esbTxBuffer, len);
			ledToggle(1);
		}
		k_msleep(10);
	}
                       
    return 0;
}