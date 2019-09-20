/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-5      SummerGift   first version
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

#include <rtthread.h>
#include <board.h>

#define NOR_FLASH_DEV_NAME "norflash0"
#define RT_APP_PART_ADDR 0x08020000

// #define FLASH_SIZE_GRANULARITY_16K 0              //(4 * 16 * 1024)
// #define FLASH_SIZE_GRANULARITY_64K 0              //(64 * 1024)
#define FLASH_SIZE_GRANULARITY_128K (1024 * 1024) //(7 * 128 * 1024) // 内部 flash 大小

// #define STM32_FLASH_START_ADRESS_16K STM32_FLASH_START_ADRESS
// #define STM32_FLASH_START_ADRESS_64K (STM32_FLASH_START_ADRESS_16K + FLASH_SIZE_GRANULARITY_16K)
#define STM32_FLASH_START_ADRESS_128K STM32_FLASH_START_ADRESS //(STM32_FLASH_START_ADRESS_64K + FLASH_SIZE_GRANULARITY_64K)

// extern const struct fal_flash_dev stm32_onchip_flash_16k;
// extern const struct fal_flash_dev stm32_onchip_flash_64k;
extern const struct fal_flash_dev stm32_onchip_flash_128k;
extern struct fal_flash_dev nor_flash0;

/* flash device table */
#define FAL_FLASH_DEV_TABLE       \
    {                             \
        &stm32_onchip_flash_128k, \
            &nor_flash0,          \
    }
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG

/* partition table */
#define FAL_PART_TABLE                                                                                                 \
    {                                                                                                                  \
        {FAL_PART_MAGIC_WROD, "bootloader", "onchip_flash_128k", 0, (128 * 1024), 0},                                  \
            {FAL_PART_MAGIC_WROD, "app", "onchip_flash_128k", (128 * 1024), (896 * 1024), 0},                          \
            {FAL_PART_MAGIC_WORD, "download", NOR_FLASH_DEV_NAME, 0, 1024 * 1024, 0},                                  \
            {FAL_PART_MAGIC_WORD, "factory", NOR_FLASH_DEV_NAME, 1024 * 1024, 1024 * 1024, 0},                         \
            {FAL_PART_MAGIC_WORD, "filesystem", NOR_FLASH_DEV_NAME, (1024 * 1024 + 1024 * 1024), 10 * 1024 * 1024, 0}, \
    }

#endif /* FAL_PART_HAS_TABLE_CFG */
#endif /* _FAL_CFG_H_ */
