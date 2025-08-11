#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

#include "my_usb.h"
#include "led.h"
#include "systime.h"
#include "Northlib/ntrprouter.h"

LOG_MODULE_REGISTER(esb_bridge, LOG_LEVEL_DBG);

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
    return 0;
}
#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif

int main(){
    ledInit();
    if (clocks_start() != 0) return -1;

    if (usbInit() != 0) {LOG_ERR("USB init failed.");  return 0;}
    LOG_INF("USB init complete!");
    
    NTRPR_Init();
    
    LOG_INF("EXIT!");
      
    return 0;
}