
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <dk_buttons_and_leds.h>

#include "led.h"

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

// static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
// static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

static const struct gpio_dt_spec leds[2] = {
    GPIO_DT_SPEC_GET(LED0_NODE, gpios),
    GPIO_DT_SPEC_GET(LED1_NODE, gpios)
};

void ledInit(){
    if (!device_is_ready(leds[0].port)) return 0;

    gpio_pin_configure_dt(&leds[0], GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&leds[1], GPIO_OUTPUT_ACTIVE);
}

void ledWrite(uint8_t led, uint8_t val){
    gpio_pin_set_dt(&leds[led], (int)val);
}

void ledToggle(uint8_t led){
    gpio_pin_toggle_dt(&leds[led]);
}
