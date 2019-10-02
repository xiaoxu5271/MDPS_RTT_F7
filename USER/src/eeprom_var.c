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

void AT24_READ_DEV_SET(void)
{
    char dev_panid_str[6];
    char s_pwr_str[1];
    char pwr_val_str[10];
    char anchor_mode_str[1];
    char sync_period_str[5];
    char range_period_str[5];
    char resp_slot_str[5];
    char dwt_prq_dealy_16m_str[10];
    char cm_led_str[1];

    at24_read(AT24C08_ADDR_P0, DEV_ADD_STR_ADDR, dev_address_str, 5);
    // rt_kprintf("dev_address_str:%s\n", dev_address_str);
    sscanf(dev_address_str, "%x", &dev_address);

    at24_read(AT24C08_ADDR_P0, DEV_PANID_ADDR, dev_panid_str, 6);
    // rt_kprintf("dev_panid:%s\n", dev_panid_str);
    sscanf(dev_panid_str, "%x", &dev_panid);

    at24_read(AT24C08_ADDR_P0, DEV_S_PER, s_pwr_str, 1);
    // rt_kprintf("s_pwr:%s\n", s_pwr_str);
    sscanf(s_pwr_str, "%d", &s_pwr);

    at24_read(AT24C08_ADDR_P0, DEV_PER_VAL, pwr_val_str, 10);
    // rt_kprintf("pwr_val:%s\n", pwr_val_str);
    sscanf(pwr_val_str, "%x", &pwr_val);

    at24_read(AT24C08_ADDR_P0, DEV_ANCHOR_MODE, anchor_mode_str, 1);
    // rt_kprintf("anchor_mode:%s\n", anchor_mode_str);
    sscanf(anchor_mode_str, "%d", &anchor_mode);

    at24_read(AT24C08_ADDR_P0, DEV_SYNC_PERIOD, sync_period_str, 5);
    // rt_kprintf("sync_period:%s\n", sync_period_str);
    sscanf(sync_period_str, "%d", &sync_period);

    at24_read(AT24C08_ADDR_P0, DEV_RANG_PERIOD, range_period_str, 5);
    // rt_kprintf("range_period:%s\n", range_period_str);
    sscanf(range_period_str, "%d", &range_period);

    at24_read(AT24C08_ADDR_P0, DEV_RESP_SLOT, resp_slot_str, 1);
    // rt_kprintf("resp_slot:%s\n", resp_slot_str);
    sscanf(resp_slot_str, "%d", &resp_slot);

    at24_read(AT24C08_ADDR_P0, DEV_PRQ_DELAY, dwt_prq_dealy_16m_str, 10);
    // rt_kprintf("dwt_prq_dealy_16m:%s\n", dwt_prq_dealy_16m_str);
    dwt_prq_dealy_16m = strtod(dwt_prq_dealy_16m_str, RT_NULL);

    // sscanf(read_temp, "%d", &dwt_prq_dealy_16m);

    at24_read(AT24C08_ADDR_P0, DEV_CM_LED, cm_led_str, 1);
    // rt_kprintf("cm_led:%s\n", cm_led_str);
    sscanf(cm_led_str, "%d", &cm_led);

    rt_kprintf(" dev_address:%04x \n\
                 dev_panid=%04x \n \
                 s_pwr:%d \n\
                 pwr_val:%08x \n \
                 anchor_mode:%d \n \
                 sync_period:%d\n\
                 range_period:%d\n\
                 resp_slot:%d\n\
                 cm_led:%d\n",
               dev_address,
               dev_panid,
               s_pwr,
               pwr_val,
               anchor_mode,
               sync_period,
               range_period,
               resp_slot,
               cm_led);
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
