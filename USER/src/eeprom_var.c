/*
 * Copyright (c) 2015-2019, Cloudforce Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-9-26       Tony       初始化存入EEPROM中的参数
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include <string.h>

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG
#define DBG_COLOR
#include <rtdbg.h>

#include "eeprom_var.h"
#include "at24cxx.h"
#include "dw1000_usr.h"

// at24cxx_device_t dev = RT_NULL;

char server_ip[16];
// char APP_VERSION[10]; //vxx.xx.xx

void AT24_READ_DEV_SET(void)
{
    char *read_temp;
    // char dev_panid_str[6];
    // char s_pwr_str[1];
    // char pwr_val_str[10];
    // char anchor_mode_str[1];
    // char sync_period_str[5];
    // char range_period_str[5];
    // char resp_slot_str[5];
    // char dwt_prq_dealy_16m_str[10];
    // char cm_led_str[1];

    at24_read(AT24C08_ADDR_P0, DEV_ADD_STR_ADDR, dev_address_str, 5);
    sscanf(dev_address_str, "%x", &dev_address);
    rt_kprintf("dev_address_str:%04x\n", dev_address);
    rt_memset(read_temp, 0, 5);

    at24_read(AT24C08_ADDR_P0, DEV_PANID_ADDR, read_temp, 6);
    sscanf(read_temp, "%x", &dev_panid);
    rt_kprintf("dev_panid:%04x\n", dev_panid);
    rt_memset(read_temp, 0, 6);

    at24_read(AT24C08_ADDR_P0, DEV_S_PER, read_temp, 1);
    sscanf(read_temp, "%d", &s_pwr);
    rt_kprintf("s_pwr:%d\n", s_pwr);
    rt_memset(read_temp, 0, 1);

    at24_read(AT24C08_ADDR_P0, DEV_PER_VAL, read_temp, 10);
    sscanf(read_temp, "%x", &pwr_val);
    rt_kprintf("pwr_val:%08x\n", pwr_val);
    rt_memset(read_temp, 0, 10);

    at24_read(AT24C08_ADDR_P0, DEV_ANCHOR_MODE, read_temp, 1);
    sscanf(read_temp, "%d", &anchor_mode);
    rt_kprintf("anchor_mode:%d\n", anchor_mode);
    rt_memset(read_temp, 0, 1);

    at24_read(AT24C08_ADDR_P0, DEV_SYNC_PERIOD, read_temp, 5);
    sscanf(read_temp, "%d", &sync_period);
    rt_kprintf("sync_period:%d\n", sync_period);
    rt_memset(read_temp, 0, 5);

    at24_read(AT24C08_ADDR_P0, DEV_RANG_PERIOD, read_temp, 5);
    sscanf(read_temp, "%d", &range_period);
    rt_kprintf("range_period:%d\n", range_period);
    rt_memset(read_temp, 0, 5);

    at24_read(AT24C08_ADDR_P0, DEV_RESP_SLOT, read_temp, 1);
    sscanf(read_temp, "%d", &resp_slot);
    rt_kprintf("resp_slot:%d\n", resp_slot);
    rt_memset(read_temp, 0, 1);

    at24_read(AT24C08_ADDR_P0, DEV_PRQ_DELAY, read_temp, 10);
    dwt_prq_dealy_16m = strtod(read_temp, RT_NULL);
    rt_kprintf("dwt_prq_dealy_16m:%s\n", read_temp);
    rt_memset(read_temp, 0, 10);

    // sscanf(read_temp, "%d", &dwt_prq_dealy_16m);

    at24_read(AT24C08_ADDR_P0, DEV_CM_LED, read_temp, 1);
    sscanf(read_temp, "%d", &cm_led);
    rt_kprintf("cm_led:%d\n", cm_led);
    rt_memset(read_temp, 0, 1);

    at24_read(AT24C08_ADDR_P0, SERVER_IP_ADDR, server_ip, 16);
    uint16_t a, b, c, d;
    if (sscanf(server_ip, "%d.%d.%d.%d", &a, &b, &c, &d) == 4 &&
        a >= 0 && a <= 255 &&
        b >= 0 && b <= 255 &&
        c >= 0 && c <= 255 &&
        d >= 0 && d <= 255)
    {
        rt_sprintf(server_ip, "%d.%d.%d.%d", a, b, c, d); //把格式化的数据写入字符串
        rt_kprintf("server_ip:%s\n", server_ip);
    }
    else
    {
        rt_memset(server_ip, 0, 16);
        // server_ip = RT_NULL;
        rt_kprintf("server_ip err! LEN:%d\n", strlen(server_ip));
    }

    rt_free(read_temp);
}

/******************************************************************************
  Empty eeprom 
******************************************************************************/
void Empty_eeprom(uint8_t slaAddr)
{
    for (uint8_t i = 0; i < 255; i++)
    {
        at24_write_one_byte(slaAddr, i, 0);
        rt_thread_mdelay(10);
    }
}

// INIT_APP_EXPORT(AT24_READ_DEV_SET);
