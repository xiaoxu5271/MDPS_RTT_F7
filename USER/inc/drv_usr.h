#ifndef SPI_USR_H
#define SPI_USR_H
#include "rtthread.h"
#include <board.h>
#include <rtdevice.h>

void DW1000_init(void);
void spi_write(uint8_t *pData, uint16_t Size);
void spi_read(uint8_t *pData, uint16_t Size);

void spi_speed_set(int scalingfactor);
void DW1000_enableirq(void);
void DW1000_disableirq(void);

//BOARD FLASH CS PIN

#define DW1000_CS GET_PIN(F, 7)
#define DW1000_RST GET_PIN(F, 8)
#define DW1000_IRQ GET_PIN(F, 9)
#define DW1000_5V_EN GET_PIN(F, 10)

#define DW1000_DW_RST_on rt_pin_write(DW1000_RST, PIN_HIGH) //(GPIOE->BSRR = GPIO_PIN_4)
#define DW1000_DW_RST_off rt_pin_write(DW1000_RST, PIN_LOW) //(GPIOE->BSRR = (uint32_t)GPIO_PIN_4 << 16U)

#define DW1000_CS_off() rt_pin_write(DW1000_CS, PIN_LOW) //(GPIOE->BSRR = (uint32_t)GPIO_PIN_2 << 16U)/*0*///(GPIOF->BSRR = (uint32_t)GPIO_PIN_7 << 16U) /*0*/ //rt_pin_write(DW1000_CS, PIN_HIGH) //(GPIOE->BSRR = (uint32_t)GPIO_PIN_2 << 16U)/*0*/
#define DW1000_CS_on() rt_pin_write(DW1000_CS, PIN_HIGH) //(GPIOE->BSRR = GPIO_PIN_2)/*1*/

#define DW1000_IRQ_PIN GPIO_PIN_9
#define DW1000_IRQ_PORT GPIOF
#define DW1000_IRQ_DATA ((DW1000_IRQ_PORT->IDR & DW1000_IRQ_PIN) != (uint32_t)GPIO_PIN_RESET)

#endif
