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
#ifndef DW1000_DRVIER_H
#define DW1000_DRVIER_H

#include "deca_types.h"

//#define TIME_STAMP_DEBUG

// #define TAG_MODE
// #define CARD_TAG
//#define GOODS_TAG
//#define GOODS_TAG_NEW
// #define ANCHOR_TAG
//#define TAG_BRACELET_L

#ifndef TAG_MODE
#define ANCHOR_MODE
#endif

#define TODA_RANGING 0
#define TWR_RANGING 1

#define HOST_ANCHOR 0
#define SLAVE_ANCHOR 1

#define DWT_PRF_64M_RFDLY (514.462f) //(514.462f)

#ifdef TAG_MODE
#define DWT_PRF_16M_RFDLY (516.25f) //(513.9067f)
#else
#define DWT_PRF_16M_RFDLY (516.25f) //(513.9067f)
#endif

#define TOF_TAG_POLL_MSG_LEN 1
#define ANCHOR_TEST_MSG_LEN 1
#define TAG_FINAL_MSG_LEN 40
#define TOF_TAG_MSG_LEN 39
#define TOF_TAG_MSG_CODE 40

#define TDOA_TAG_POLL_MSG_LEN 2
#define TDOA_TAG_MSG_LEN 1
#define TDOA_TAG_MSG_CODE 2

#define TDOA_ANCH_MSG_CLE_LEN 1
#define TDOA_ANCH_MSG_SYNC_LEN 6

#define ANCH_RESPONSE_MSG_LEN 2
#define RESP_MSG_ANCHOR_SLOT 1

#define STANDARD_FRAME_SIZE 127
#define ADDR_BYTE_SIZE_L 8
#define ADDR_BYTE_SIZE_S 2
#define FRAME_CONTROL_BYTES 2
#define FRAME_SEQ_NUM_BYTES 1
#define FRAME_PANID 2
#define FRAME_CRC 2
#define FRAME_SOURCE_ADDRESS_S (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP (FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID)                                        //5
#define FRAME_CRTL_AND_ADDRESS_L (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP)                       //21 bytes for 64-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_S (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP)                       //9 bytes for 16-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_LS (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP)                      //15 bytes for 1 16-bit address and 1 64-bit address)
#define MAX_USER_PAYLOAD_STRING_LL (STANDARD_FRAME_SIZE - FRAME_CRTL_AND_ADDRESS_L - TAG_FINAL_MSG_LEN - FRAME_CRC)  //127 - 21 - 16 - 2 = 88
#define MAX_USER_PAYLOAD_STRING_SS (STANDARD_FRAME_SIZE - FRAME_CRTL_AND_ADDRESS_S - TAG_FINAL_MSG_LEN - FRAME_CRC)  //127 - 9 - 16 - 2 = 100
#define MAX_USER_PAYLOAD_STRING_LS (STANDARD_FRAME_SIZE - FRAME_CRTL_AND_ADDRESS_LS - TAG_FINAL_MSG_LEN - FRAME_CRC) //127 - 15 - 16 - 2 = 94
#define MAX_USER_PAYLOAD_STRING MAX_USER_PAYLOAD_STRING_LL

typedef struct
{
  uint8_t msg_len; //message type
  uint8_t msg_buf[40];
} PollMsgStruct;

typedef enum instanceModes
{
  LISTENER,
  TAG,
  ANCHOR,
  ANCHOR_SYNC,
  ANCHOR_RNG,
  NUM_MODES
} INST_MODE; //instance mode (tag or anchor)

typedef struct
{
  uint8 channelNumber; // valid range is 1 to 11
  uint8 preambleCode;  // 00 = use NS code, 1 to 24 selects code
  uint8 pulseRepFreq;  // NOMINAL_4M, NOMINAL_16M, or NOMINAL_64M
  uint8 dataRate;      // DATA_RATE_1 (110K), DATA_RATE_2 (850K), DATA_RATE_3 (6M81)
  uint8 preambleLen;   // values expected are 64, (128), (256), (512), 1024, (2048), and 4096
  uint8 pacSize;
  uint8 nsSFD;
  uint16 sfdTO; // SFD timeout value (in symbols) e.g. preamble length (128) + SFD(8) - PAC + some margin ~ 135us... DWT_SFDTOC_DEF; //default value
} instanceConfig_t;

typedef struct
{
  uint8 channel;
  uint8 prf;
  uint8 datarate;
  uint8 preambleCode;
  uint8 preambleLength;
  uint8 pacSize;
  uint8 nsSFD;
  uint16 sfdTO;
} chConfig_t;

typedef struct
{
  uint16 slotPeriod;   //slot period (time for 1 tag to range to 4 anchors)
  uint16 numSlots;     // number of slots in one superframe (number of tags supported)
  uint16 sfPeriod;     // superframe period in ms
  uint16 pollSleepDly; // the minimum SLEEP time should be FRAME PERIOD so that tags don't interfere
  uint16 replyDly;     //response delay time (Tag or Anchor when sending Final/Response messages respectively)
} sfConfig_t;

// TX power and PG delay configuration structure
// TX POWER
// 31:24     BOOST_0.125ms_PWR
// 23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
// 15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
// 7:0       DEFAULT_PWR-TX_DATA_PWR
typedef struct
{
  uint8 PGdelay;
  uint32 txPwr[2];
} tx_struct;

/*******************************************************************************
 FUNCTION PROTOTYPES
*******************************************************************************/
extern void instance_clearevents(void); //Clear instance events

extern uint64 convertmicrosectodevicetimeu(double microsecu); //convert microseconds to device time

extern int dw1000_init(uint8 dev_channel_speed, uint16 dev_addr); // Configures DW1000 Device

#endif

/*******************************************************************************
                                      END         
*******************************************************************************/
