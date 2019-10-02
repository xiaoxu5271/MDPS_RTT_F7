/*******************************************************************************
  * @file     
  * @author 
  * @version
  * @date 
  * @brief
  ******************************************************************************
  * @attention
  *
  *
*******************************************************************************/

/*-------------------------------- Includes ----------------------------------*/
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "cJSON.h"

#include "cJSON_Parse.h"
#include "at24cxx.h"
#include "eeprom_var.h"
#include "dw1000_usr.h"

#define SUCCESS 0
#define FAILURE -1

/******************************************************************************
//parse uart command
******************************************************************************/
int Parse_UartCmd(char *Cmd_buf)
{
  rt_kprintf(" Begin Parse_UartCmd!!!\n");

  if (NULL == Cmd_buf) //null
  {
    return FAILURE;
  }

  cJSON *pJson = cJSON_Parse(Cmd_buf); //parse json data
  if (NULL == pJson)
  {
    cJSON_Delete(pJson); //delete pJson

    return FAILURE;
  }

  cJSON *pSub = cJSON_GetObjectItem(pJson, "Command"); //"Command"
  if (NULL != pSub)
  {
    if (!strcmp((char const *)pSub->valuestring, "SetupProduct")) //Command:SetupProduct
    {
      Empty_eeprom(AT24C08_ADDR_P0);
      rt_kprintf(" Begin SetupProduct, now empty eeprom!!!\n");

      pSub = cJSON_GetObjectItem(pJson, "PanId"); //"PanId"
      if (NULL != pSub)
      {
        rt_kprintf("json parse PanId:%s\n", pSub->valuestring);
        at24_write(AT24C08_ADDR_P0, DEV_PANID_ADDR, pSub->valuestring, 6);
      }

      pSub = cJSON_GetObjectItem(pJson, "Address"); //"Address"
      if (NULL != pSub)
      {
        rt_kprintf("json parse Address:%s\n", pSub->valuestring);
        at24_write(AT24C08_ADDR_P0, DEV_ADD_STR_ADDR, pSub->valuestring, 4);
        // osi_at24c08_write(DEVICE_ADDR_ADDR, (uint16_t)strtoul(pSub->valuestring, 0, 16));
      }

      pSub = cJSON_GetObjectItem(pJson, "Channel"); //"Channel"
      if (NULL != pSub)
      {
        rt_kprintf("json parse Channel:%s\n", pSub->valuestring);
        // osi_at24c08_write_byte(DEVICE_CHANNEL_ADDR, (uint8_t)pSub->valueint);
      }

      pSub = cJSON_GetObjectItem(pJson, "Speed"); //"Speed"
      if (NULL != pSub)
      {
        rt_kprintf("json parse Speed:%s\n", pSub->valuestring);
        // osi_at24c08_write_byte(DEVICE_SPEED_ADDR, (uint8_t)pSub->valueint);
      }

      pSub = cJSON_GetObjectItem(pJson, "cm_led"); //"cm_led"
      if (NULL != pSub)
      {
        rt_kprintf("json parse cm_led:%s\n", pSub->valuestring);
        at24_write(AT24C08_ADDR_P0, DEV_CM_LED, pSub->valuestring, 1);
      }

      pSub = cJSON_GetObjectItem(pJson, "s_pwr"); //"s_pwr"
      if (NULL != pSub)
      {
        rt_kprintf("json parse s_pwr:%s\n", pSub->valuestring);
        at24_write(AT24C08_ADDR_P0, DEV_S_PER, pSub->valuestring, 1);
      }

      pSub = cJSON_GetObjectItem(pJson, "pwr_val"); //"pwr_val"
      if (NULL != pSub)
      {
        rt_kprintf("json parse Address:%s\n", pSub->valuestring);
        at24_write(AT24C08_ADDR_P0, DEV_PER_VAL, pSub->valuestring, 10);
      }

      pSub = cJSON_GetObjectItem(pJson, "Host_Anchor"); //"Host_Anchor"
      if (NULL != pSub)
      {
        rt_kprintf("json parse Host_Anchor:%s\n", pSub->valuestring);
        at24_write(AT24C08_ADDR_P0, DEV_ANCHOR_MODE, pSub->valuestring, 1);
      }

      pSub = cJSON_GetObjectItem(pJson, "RespSlot"); //"RespSlot"
      if (NULL != pSub)
      {
        rt_kprintf("json parse RespSlot:%s\n", pSub->valuestring);
        at24_write(AT24C08_ADDR_P0, DEV_RESP_SLOT, pSub->valuestring, 1);
      }

      pSub = cJSON_GetObjectItem(pJson, "SyncPeriod"); //"SyncPeriod"
      if (NULL != pSub)
      {
        rt_kprintf("json parse SyncPeriod:%s\n", pSub->valuestring);
        at24_write(AT24C08_ADDR_P0, DEV_SYNC_PERIOD, pSub->valuestring, 5);
      }

      pSub = cJSON_GetObjectItem(pJson, "range_period"); //"range_period"
      if (NULL != pSub)
      {
        rt_kprintf("json parse range_period:%s\n", pSub->valuestring);
        at24_write(AT24C08_ADDR_P0, DEV_RANG_PERIOD, pSub->valuestring, 5);
      }

      pSub = cJSON_GetObjectItem(pJson, "prq_delay"); //"prq_delay"
      if (NULL != pSub)
      {
        rt_kprintf("json parse prq_delay:%s\n", pSub->valuestring);
        at24_write(AT24C08_ADDR_P0, DEV_PRQ_DELAY, pSub->valuestring, 10);
      }

      cJSON_Delete(pJson); //delete pJson

      return SUCCESS;
    }
    // else if (!strcmp((char const *)pSub->valuestring, "ReadProduct")) //Command:ReadProduct
    // {
    //   cJSON_Delete(pJson); //delete pJson

    //   return 2;
    // }
    // else if (!strcmp((char const *)pSub->valuestring, "ResetSystem")) //ResetSystem
    // {
    //   cJSON_Delete(pJson); //delete pJson

    //   return 3;
    // }

    // else if (!strcmp((char const *)pSub->valuestring, "NoResp")) //NoResp
    // {
    //   pSub = cJSON_GetObjectItem(pJson, "DestAddr"); //"DestAddr"
    //   if (NULL != pSub)
    //   {
    //     if ((uint16_t)strtoul(pSub->valuestring, 0, 16) == dev_address)
    //     {
    //       pSub = cJSON_GetObjectItem(pJson, "Num"); //"Num"
    //       if (NULL != pSub)
    //       {
    //         uint16_t non_resp_tags = (uint16_t)pSub->valueint;

    //         pSub = cJSON_GetObjectItem(pJson, "Addrs"); //"Addrs"

    //         parse_noresp_addrs(non_resp_tags, pSub->valuestring);
    //       }
    //     }
    //   }

    //   cJSON_Delete(pJson); //delete pJson

    //   return SUCCESS;
    // }

    // else if (!strcmp((char const *)pSub->valuestring, "ReadNoResp")) //ReadNoResp
    // {
    //   cJSON_Delete(pJson); //delete pJson

    //   return 4;
    // }
  }
  cJSON_Delete(pJson); //delete pJson

  return FAILURE;
}

/*******************************************************************************
//make product set data json
*******************************************************************************/
// void Read_Product(char *read_buf, uint16_t data_len)
// {
//   snprintf(read_buf, data_len, "{\"p\":%d,\"Host_Anchor\":%d,\"PanId\":\"%04x\",\"Address\":\"%04x\",\"Channel\":%d,\"Speed\":%d,\"s_pwr\":%d,\"pwr_val\":\"%08x\",\"RespSlot\":%d,\"SyncPeriod\":%d,\"range_period\":%d,\"prq_delay\":\"%.4f\",\"cm_led\":%d,\"firmware\":\"%s\"}\n\r", ANCHOR_SETINT_MSG, anchor_mode, (uint16_t)dev_panid, (uint16_t)dev_address, dev_channel, dev_speed, s_pwr, pwr_val, resp_slot, sync_period, range_period, prq_delay_set_val, cm_led, FIRMWARE);
// }

/*******************************************************************************
//range set data init
*******************************************************************************/
// void Range_SetData_read(void)
// {
//   dev_panid = (uint16_t)at24c08_read(DEVICE_PANID_ADDR);

//   dev_address = (uint16_t)at24c08_read(DEVICE_ADDR_ADDR);

//   dev_channel = at24c08_read_byte(DEVICE_CHANNEL_ADDR);

//   dev_speed = at24c08_read_byte(DEVICE_SPEED_ADDR);

//   cm_led = (bool)at24c08_read_byte(CM_LED_ADDR);

//   s_pwr = (bool)at24c08_read_byte(SMART_PWR_ADDR);

//   if (s_pwr)
//   {
//     pwr_val = 0x15355575;
//   }
//   else
//   {
//     //pwr_val = 0x1f1f1f1f;
//     pwr_val = (uint32_t)at24c08_read(PWR_VAL_ADDR);
//   }

//   short resp_val = -1;
//   uint8_t string_buf[32];

//   resp_val = at24c08_ReadString(PRQ_DELAY_ADDR, string_buf, sizeof(string_buf), 1);
//   if (resp_val == SUCCESS)
//   {
//     prq_delay_set_val = atof((char *)string_buf);
//   }
//   else
//   {
//     prq_delay_set_val = DWT_PRF_16M_RFDLY;
//   }

//   dwt_prq_dealy_16m = (prq_delay_set_val / 2.0) * 1e-9 / DWT_TIME_UNITS;

//   anchor_mode = (bool)at24c08_read_byte(HOST_ANCHOR_ADDR);

//   resp_slot = (uint8_t)at24c08_read_byte(DEVICE_RESPSLOT_ADDR);

//   sync_period = (uint16_t)at24c08_read(DEVICE_SYNCPERIOD_ADDR);

//   range_period = (uint16_t)at24c08_read(DEVICE_RANGEPERIOD_ADDR);

// }

void uart_cmd(int argc, char *argv[])
{
  rt_kprintf("argc:%d \n", argc, argv[1]);
  if (argc == 2)
  {
    Parse_UartCmd(argv[1]);
  }
  else
  {
    rt_kprintf("\nDo not use spaces in command !!!\n");
  }
}

MSH_CMD_EXPORT(uart_cmd, Parse uart cmd);
/*******************************************************************************
                                      END         
*******************************************************************************/
