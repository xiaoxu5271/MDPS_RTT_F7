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
#include "stdint.h"
#include "dw1000_driver.h"

#define UART_WAIT_TIMEOUT       120000
#define HOST_ANCHOR_SYNC        0
#define SLAVE_ANCHOR_SYNC       1
#define TAG_BORADCAST           2
#define TOF_RANGING             3
#define ANCHOR_RANGING          4

#ifdef TIME_STAMP_DEBUG
  #define ANCHOR_TIMESTAMP_DEBUG        8
#endif
#define NO_RESP_TAG_MSG         7
#define ANCHOR_SETINT_MSG       9
#define ANCHOR_RESULT_MSG       10

typedef struct
{
  uint8_t msg_type;  //message type
  uint16_t sec_num;  //security number
  uint16_t s_addr;  //source address
  uint16_t d_addr;  //destination address
  uint32_t rx_time[2];  //recive time stamp
#ifdef TIME_STAMP_DEBUG
  uint32_t poll_rx_time[2];  
  uint32_t anchor_tx_time[2];  
  uint32_t tag_poll_time[2];      
  uint32_t tag_rx_time[2]; 
  uint32_t tag_final_time[2];
#endif
  uint32_t distance_val;
  int rssi_val;
  uint8_t msg_buf[64];
}UartMsgStruct;

#ifdef ANCHOR_MODE
  extern void Uart_print_task(void *pvParameters);
#endif
/*******************************************************************************
                                      END         
*******************************************************************************/




