#include <stdio.h>
#include <string.h>  
#include <stdint.h>
#include "my_esb.h"
#include "nrf21540.h"
#include "zephyr/drivers/gpio.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <esb.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

LOG_MODULE_REGISTER(my_esb, LOG_LEVEL_DBG);

// Global değişkenler
static bool esb_initialized = false;
static my_esb_config_t current_config;  // Burada tanımlı - static global değişken
static uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
static uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
static uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};


int my_esb_initialize(const my_esb_config_t *config)
{
    int err;
    
    // Eğer zaten başlatılmışsa önce deinit yap
    if (esb_initialized) {
        LOG_WRN("ESB already initialized, deinitializing first");
        my_esb_deinit();
    }
    
    // Konfigürasyonu sakla
    memcpy(&current_config, config, sizeof(my_esb_config_t));
    
    // ESB konfigürasyonunu hazırla
    struct esb_config esb_cfg;
    esb_cfg.protocol = ESB_PROTOCOL_ESB_DPL;
    esb_cfg.retransmit_delay = config->retransmit_delay;
    esb_cfg.retransmit_count = config->retransmit_count;
    esb_cfg.bitrate = (enum esb_bitrate)config->bitrate;  // Cast gerekli
    esb_cfg.crc = (enum esb_crc)config->crc_mode;         // Cast gerekli
    esb_cfg.event_handler = config->event_handler;
    esb_cfg.selective_auto_ack = config->selective_auto_ack;
    esb_cfg.tx_output_power = (enum esb_tx_power)config->tx_power;  // Cast gerekli
    
    if(config->mode == ESB_RX) {
        esb_cfg.mode = ESB_MODE_PRX;
    } else if(config->mode == ESB_TX) {
        esb_cfg.mode = ESB_MODE_PTX;
    }
    
    // ESB'yi başlat
    err = esb_init(&esb_cfg);
    if (err) {
        LOG_ERR("ESB init failed: %d", err);
        return err;
    }
    
    // Adres ayarları
    err = esb_set_base_address_0(base_addr_0);
    if (err) {
        LOG_ERR("Failed to set base address 0: %d", err);
        return err;
    }
    
    err = esb_set_base_address_1(base_addr_1);
    if (err) {
        LOG_ERR("Failed to set base address 1: %d", err);
        return err;
    }
    
    err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
    if (err) {
        LOG_ERR("Failed to set prefixes: %d", err);
        return err;
    }
    
    // RF kanalını ayarla
    err = esb_set_rf_channel(config->rf_channel);
    if (err) {
        LOG_ERR("Failed to set RF channel: %d", err);
        return err;
    }
    
    esb_initialized = true;
    LOG_INF("ESB initialized - Mode: %s, Bitrate: %d, Channel: %d, TX Power: %d",
            config->mode == ESB_RX ? "RX" : "TX",
            config->bitrate,
            config->rf_channel,
            config->tx_power);
    
    return 0;
}

int my_esb_deinit(void)
{
    if (!esb_initialized) {
        LOG_WRN("ESB not initialized");
        return -EALREADY;
    }
    
    // ESB'yi durdur
    esb_stop_rx();
    
    // ESB'yi devre dışı bırak - esb_disable() void dönüyor
    esb_disable();
    
    esb_initialized = false;
    LOG_INF("ESB deinitialized");
    
    return 0;
}

int my_esb_transmit(const uint8_t *data, uint8_t len)
{
    if (!esb_initialized) {
        LOG_ERR("ESB not initialized");
        return -EINVAL;
    }
    
    while (!esb_is_idle()) {
        k_sleep(K_MSEC(1));
    }

    struct esb_payload payload = {0};
    payload.length = len;
    payload.pipe   = 0;
    payload.noack  = 1;
    memcpy(payload.data, data, len);

    int err = esb_write_payload(&payload);
    if (err) return err;
    
    err = esb_start_tx();
    return err;
}

int my_esb_receive(struct esb_payload *payload)
{
    if (!esb_initialized) {
        LOG_ERR("ESB not initialized");
        return -EINVAL;
    }
    
    while (!esb_is_idle()) {
        k_sleep(K_MSEC(1));
    }

    int err = esb_start_rx();
    if (err) return err;

    return esb_read_rx_payload(payload);
}

// Ayar fonksiyonları
int my_esb_set_bitrate(my_esb_bitrate_e bitrate)
{
    if (!esb_initialized) {
        LOG_ERR("ESB not initialized");
        return -EINVAL;
    }
    
    // Mevcut konfigürasyonu güncelle ve yeniden başlat
    current_config.bitrate = bitrate;
    return my_esb_initialize(&current_config);
}

int my_esb_set_tx_power(my_esb_tx_power_e tx_power)
{
    if (!esb_initialized) {
        LOG_ERR("ESB not initialized");
        return -EINVAL;
    }
    
    current_config.tx_power = tx_power;
    return my_esb_initialize(&current_config);
}

int my_esb_set_rf_channel(uint8_t channel)
{

    esb_stop_rx();

    if (!esb_initialized) {
        LOG_ERR("ESB not initialized");
        return -EINVAL;
    }
    
    if (channel > 100) {
        LOG_ERR("Invalid RF channel: %d", channel);
        return -EINVAL;
    }
    
    int err = esb_set_rf_channel(channel);
    if (err == 0) {
        current_config.rf_channel = channel;
        LOG_INF("RF channel set to: %d", channel);
    }

    esb_start_rx();
    
    return err;
}

int my_esb_set_retransmit_params(uint16_t delay, uint8_t count)
{
    if (!esb_initialized) {
        LOG_ERR("ESB not initialized");
        return -EINVAL;
    }
    
    current_config.retransmit_delay = delay;
    current_config.retransmit_count = count;
    return my_esb_initialize(&current_config);
}

int my_esb_set_crc_mode(my_esb_crc_e crc_mode)
{
    if (!esb_initialized) {
        LOG_ERR("ESB not initialized");
        return -EINVAL;
    }
    
    current_config.crc_mode = crc_mode;
    return my_esb_initialize(&current_config);
}

// Durum sorgulama fonksiyonları
int my_esb_get_rf_channel(uint8_t *channel)
{
    if (!esb_initialized) {
        LOG_ERR("ESB not initialized");
        return -EINVAL;
    }
    
    if (channel == NULL) {
        return -EINVAL;
    }
    
    *channel = current_config.rf_channel;
    return 0;
}

bool my_esb_is_initialized(void)
{
    return esb_initialized;
}

// Mevcut konfigürasyonu döndür
my_esb_config_t* my_esb_get_config(void)
{
    if (!esb_initialized) {
        return NULL;
    }
    
    return &current_config;
}

void my_esb_print_config(void)
{
    if (!esb_initialized) {
        LOG_ERR("ESB not initialized");
        return;
    }
    
    LOG_INF("=== ESB Configuration ===");
    LOG_INF("Mode: %s", current_config.mode == ESB_RX ? "RX" : "TX");
    
    const char *bitrate_str;
    switch(current_config.bitrate) {
        case MY_ESB_BITRATE_1MBPS:   bitrate_str = "1 Mbps"; break;
        case MY_ESB_BITRATE_2MBPS:   bitrate_str = "2 Mbps"; break;
        default: bitrate_str = "Unknown"; break;
    }
    LOG_INF("Bitrate: %s", bitrate_str);
    
    LOG_INF("TX Power: %d dBm", current_config.tx_power);
    LOG_INF("RF Channel: %d", current_config.rf_channel);
    LOG_INF("Retransmit Delay: %d us", current_config.retransmit_delay);
    LOG_INF("Retransmit Count: %d", current_config.retransmit_count);
    
    const char *crc_str;
    switch(current_config.crc_mode) {
        case MY_ESB_CRC_OFF:   crc_str = "OFF"; break;
        case MY_ESB_CRC_8BIT:  crc_str = "8-bit"; break;
        case MY_ESB_CRC_16BIT: crc_str = "16-bit"; break;
        default: crc_str = "Unknown"; break;
    }
    LOG_INF("CRC Mode: %s", crc_str);
    LOG_INF("Selective Auto ACK: %s", current_config.selective_auto_ack ? "Enabled" : "Disabled");
    LOG_INF("========================");
}