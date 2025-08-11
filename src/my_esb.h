#ifndef MY_ESB_H
#define MY_ESB_H

#include <stdint.h>
#include <esb.h>

typedef enum {
    ESB_TX,
    ESB_RX
} my_esb_mode_e;

typedef enum {
    MY_ESB_BITRATE_1MBPS   = ESB_BITRATE_1MBPS,
    MY_ESB_BITRATE_2MBPS   = ESB_BITRATE_2MBPS
} my_esb_bitrate_e;

typedef enum {
    MY_ESB_TX_POWER_NEG40DBM = ESB_TX_POWER_NEG40DBM,
    MY_ESB_TX_POWER_NEG20DBM = ESB_TX_POWER_NEG20DBM,
    MY_ESB_TX_POWER_NEG16DBM = ESB_TX_POWER_NEG16DBM,
    MY_ESB_TX_POWER_NEG12DBM = ESB_TX_POWER_NEG12DBM,
    MY_ESB_TX_POWER_NEG8DBM  = ESB_TX_POWER_NEG8DBM,
    MY_ESB_TX_POWER_NEG4DBM  = ESB_TX_POWER_NEG4DBM,
    MY_ESB_TX_POWER_0DBM     = ESB_TX_POWER_0DBM,
    MY_ESB_TX_POWER_4DBM     = ESB_TX_POWER_4DBM,
    MY_ESB_TX_POWER_8DBM     = ESB_TX_POWER_8DBM
} my_esb_tx_power_e;

typedef enum {
    MY_ESB_CRC_OFF   = ESB_CRC_OFF,
    MY_ESB_CRC_8BIT  = ESB_CRC_8BIT,
    MY_ESB_CRC_16BIT = ESB_CRC_16BIT
} my_esb_crc_e;

// ESB yapılandırma yapısı
typedef struct {
    my_esb_mode_e mode;
    my_esb_bitrate_e bitrate;
    my_esb_tx_power_e tx_power;
    uint16_t retransmit_delay;      // uint8_t yerine uint16_t
    uint8_t retransmit_count;
    uint8_t rf_channel;
    my_esb_crc_e crc_mode;
    bool selective_auto_ack;
    void (*event_handler)(const struct esb_evt *);
} my_esb_config_t;


// Fonksiyon prototipleri
int my_esb_initialize(const my_esb_config_t *config);
int my_esb_deinit(void);
int my_esb_transmit(const uint8_t *data, uint8_t len);
int my_esb_receive(struct esb_payload *payload);

// Ayar fonksiyonları
int my_esb_set_bitrate(my_esb_bitrate_e bitrate);
int my_esb_set_tx_power(my_esb_tx_power_e tx_power);
int my_esb_set_rf_channel(uint8_t channel);
int my_esb_set_retransmit_params(uint16_t delay, uint8_t count);  // uint16_t
int my_esb_set_crc_mode(my_esb_crc_e crc_mode);

// Durum sorgulama fonksiyonları
int my_esb_get_rf_channel(uint8_t *channel);
bool my_esb_is_initialized(void);
my_esb_config_t* my_esb_get_config(void);
void my_esb_print_config(void);

#endif