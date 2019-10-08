#ifndef __EEPROM_H__
#define __EEPROM_H__

#include <rthw.h>
#include <rtthread.h>

#define DEV_ADD_STR_ADDR 0 //STR-"AXXX" 4
#define DEV_PANID_ADDR 5   //(DEV_ADD_STR_ADDR + 5) //UINT 6
#define DEV_S_PER 12       //(DEV_PANID_ADDR + 6)        //u8 1
#define DEV_PER_VAL 14     // (DEV_S_PER + 1)           //u64 10
#define DEV_ANCHOR_MODE 25 //(DEV_PER_VAL + 10)    //U8 1
#define DEV_SYNC_PERIOD 27 //(DEV_ANCHOR_MODE + 1) //U16 5
#define DEV_RANG_PERIOD 33 // (DEV_SYNC_PERIOD + 5) //U16  5
#define DEV_RESP_SLOT 40   //(DEV_RANG_PERIOD + 5)   //U8 1
#define DEV_SPEED 42       //(DEV_RESP_SLOT + 1)         //U8 1
#define DEV_CHANNEL 44     //(DEV_SPEED + 1)     //U8 1
#define DEV_PRQ_DELAY 46   //(DEV_CHANNEL + 1) //double 10
#define DEV_CM_LED 57      // (DEV_PRQ_DELAY + 10)       // 1
#define SERVER_IP_ADDR 74  // 16

void AT24_READ_DEV_SET(void);
void Empty_eeprom(uint8_t slaAddr);

extern char server_ip[16];
#define APP_VERSION "V0.0.4"

#endif