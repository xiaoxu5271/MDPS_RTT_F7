/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 * 2018-11-19     flybreak     add stm32f407-atk-explorer bsp
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "fal.h"
#include <dfs_fs.h>
#include "wlan_app.h"
#include "dw1000_usr.h"
// #include "drv_usr.h"
#include <rtdbg.h>

#define FS_PARTITION_NAME "filesystem"

/* defined the LED0 pin: PF9 */
#define LED0_PIN GET_PIN(F, 0)
#define LED1_PIN GET_PIN(F, 1)
#define LED2_PIN GET_PIN(F, 2)

#define APP_VERSION "V0.0.3"

int main(void)
{
    struct rt_device *mtd_dev = RT_NULL;
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);

    rt_pin_write(LED0_PIN, PIN_LOW);
    rt_pin_write(LED1_PIN, PIN_LOW);
    rt_pin_write(LED2_PIN, PIN_LOW);

    fal_init();
    mtd_dev = fal_mtd_nor_device_create(FS_PARTITION_NAME);
    if (!mtd_dev)
    {
        LOG_E("Can't create a mtd device on '%s' partition.", FS_PARTITION_NAME);
    }
    else
    {
        /* 挂载 littlefs */
        if (dfs_mount(FS_PARTITION_NAME, "/", "lfs", 0, 0) == 0)
        {
            LOG_I("Filesystem initialized!");
        }
        else
        {
            /* 格式化文件系统 */
            dfs_mkfs("lfs", FS_PARTITION_NAME);
            /* 挂载 littlefs */
            if (dfs_mount(FS_PARTITION_NAME, "/", "lfs", 0, 0) == 0)
            {
                LOG_I("Filesystem initialized!");
            }
            else
            {
                LOG_E("Failed to initialize filesystem!");
            }
        }
    }
    // DW1000_init();
    // wifi_connect();
    run_dw1000_task();
    // rt_wlan_config_autoreconnect(RT_TRUE); //开启自动重连
    rt_kprintf("The current version of APP firmware is %s\n", APP_VERSION);

    while (1)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_pin_write(LED2_PIN, PIN_HIGH);
        rt_thread_mdelay(2);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_pin_write(LED1_PIN, PIN_LOW);
        rt_pin_write(LED2_PIN, PIN_LOW);
        rt_thread_mdelay(1000);
        // rt_pin_write(LED0_PIN, PIN_LOW);
        // rt_pin_write(LED1_PIN, PIN_LOW);
        // // rt_pin_write(LED2_PIN, PIN_HIGH);
        // rt_thread_mdelay(100);
    }

    return RT_EOK;
}

/**
 * Function    ota_app_vtor_reconfig
 * Description Set Vector Table base location to the start addr of app(RT_APP_PART_ADDR).
*/
// static int ota_app_vtor_reconfig(void)
// {
// #define NVIC_VTOR_MASK 0x3FFFFF80
//     /* Set the Vector Table base location by user application firmware definition */
//     SCB->VTOR = RT_APP_PART_ADDR & NVIC_VTOR_MASK;

//     return 0;
// }
// INIT_BOARD_EXPORT(ota_app_vtor_reconfig);
