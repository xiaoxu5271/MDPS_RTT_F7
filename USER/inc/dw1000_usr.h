#ifndef DW1000_USR_H
#define DW1000_USR_H
#include "rtthread.h"
#include "stdio.h"
#include <stdbool.h>

#include "stm32f7xx_hal.h"

#define NO_RESP_TAG_NUM 332

extern uint16_t no_resp_tag;
extern uint16_t no_resp_addr[NO_RESP_TAG_NUM];

extern unsigned long dw1000_irq_tick;

extern uint32_t Datanum; //UWB总数据量

//eeprom var
extern char dev_address_str[5];  //基站唯一地址 （字符格式）
extern uint16_t dev_address;     //基站唯一地址 （数值格式）
extern uint16_t dev_panid;       //分组名称
extern uint8_t s_pwr;            //0手动设置功率，1为自动功率设置
extern uint64_t pwr_val;         //手动设置功率值
extern uint8_t anchor_mode;      //0为从基站，1为主基站
extern uint16_t sync_period;     //TDOA主基站同步时间（ms）
extern uint16_t range_period;    //基站自标定时间周期（ms）
extern uint8_t resp_slot;        //TWR定位基站返回序号(1-4) 隧道号
extern uint8_t dev_speed;        //SPI速率
extern uint8_t dev_channel;      //信道2为3.993G，信道5为6.489G；
extern double dwt_prq_dealy_16m; //天线延时
extern uint8_t cm_led;
//----------

typedef struct dw1000_usr_test
{
  uint8_t code;
  uint32_t s_addr;
  uint32_t d_addr;
  uint32_t sys_Tick;
  uint8_t message[64];
  /* data */
} dw1000_debug_message_tag;
typedef struct message_mdps
{
  char buf[256];
  uint32_t len;
} message_mdps_t;
extern struct rt_messagequeue message_send; //udp_send

/*
#define NRF_DRV_SPI_FREQ_1M 1
#define NRF_DRV_SPI_FREQ_2M 2
#define NRF_DRV_SPI_FREQ_4M 4
#define NRF_DRV_SPI_FREQ_8M 20
*/
#define NRF_DRV_SPI_FREQ_1M SPI_BAUDRATEPRESCALER_16
#define NRF_DRV_SPI_FREQ_2M SPI_BAUDRATEPRESCALER_8
#define NRF_DRV_SPI_FREQ_4M SPI_BAUDRATEPRESCALER_4
#define NRF_DRV_SPI_FREQ_8M SPI_BAUDRATEPRESCALER_2

void delay_ms(uint32_t n_ms);
void DECA_SPI_Config_Rate(int scalingfactor);
void dw1000_irq_isr_handler(void *p);

void reset_DW1000(void);

int readfromspi(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t readlength, uint8_t *readBuffer);
int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodyLength, const uint8_t *bodyBuffer);

void DW_IRQn_enable(void);
void DW_IRQn_disenable(void);
bool check_noresp_addrs(uint16_t tag_addr);
void run_dw1000_task(void);

void PA_LNA_OFF();
void PA_LNA_ON();

#endif
