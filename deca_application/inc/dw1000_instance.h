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
#include "deca_device_api.h"
#include "deca_types.h"

#define MAX_TAG_LIST_SIZE (8)
#define MAX_ANCHOR_LIST_SIZE (4) //this is limited to 4 in this application
#define MAX_ANCHOR_RESP_SIZE (4)
#define MAX_EVENT_NUMBER (10)
#define INVALID_TOF (0xABCDFFFF)
#define ANCTOANCTWR (0)        //if set to 1 then anchor to anchor TWR will be done in the last slot
#define CORRECT_RANGE_BIAS (1) // Compensate for small bias due to uneven accumulator growth at close up high power
#define DEEP_SLEEP (1)

#define TOF_TAG_POLL_MSG (0x81)      // Tag poll message
#define TOF_TAG_FINAL_MSG (0x82)     // Tag final massage back to Anchor
#define ANCHOR_RANG_FINAL_MSG (0x83) // Tag poll message
#define TDOA_TAG_POLL_MSG (0x85)     // Tag poll message

#define RTLS_DEMO_MSG_ANCH_RESP (0x70)  // Anchor response to poll
#define RTLS_DEMO_MSG_ANCH_POLL (0x71)  // Anchor to anchor poll message
#define RTLS_DEMO_MSG_ANCH_RESP2 (0x72) // Anchor response to poll from anchor
#define RTLS_DEMO_MSG_ANCH_FINAL (0x73) // Anchor final massage back to Anchor

#define TDOA_ANCH_MSG_CLE (0x75)  // Anchor poll message for sync through CLE
#define TDOA_ANCH_MSG_SYNC (0x76) // Anchor poll message for sync between Anchor

#define SIG_RX_UNKNOWN (99) // Received an unknown frame

#define NUM_EXPECTED_RESPONSES (3) //e.g. MAX_ANCHOR_LIST_SIZE-1
#define PTXT (1)                   // Poll TX time
#define FTXT (34)                  // Final TX time

#define FCODE 0        // Function code is 1st byte of messageData
#define TOFR 3         // ToF (n-1) 4 bytes
#define RES_TAG_SLP0 1 // Response tag sleep correction LSB
#define RES_TAG_SLP1 2 // Response tag sleep correction MSB
#define TOFRN 7        // range number 1 byte
#define RRXT0 6        // A0 Response RX time

#define TOF_REPORT_NUL 0
#define TOF_REPORT_T2A 1
#define TOF_REPORT_A2A 2

#define SPEED_OF_LIGHT (299702547.0) // in m/s in air
#define MASK_40BIT (0x00FFFFFFFFFF)  // DW1000 counter is 40 bits
#define MASK_TXDTS (0x00FFFFFFFE00)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.

//#define RX_RESPONSE1_TURNAROUND         (200)                   //takes about 200 us for the 1st response to come back (from A0)
//#define NUM_EXPECTED_RESPONSES_ANC0     (2)                     //anchor A0 expects response from A1 and A2

#define RX_RESPONSE1_TURNAROUND (200)   //takes about 200 us for the 1st response to come back (from A0)
#define NUM_EXPECTED_RESPONSES_ANC0 (2) //anchor A0 expects response from A1 and A2

//#define GATEWAY_ANCHOR_ADDR             (0x8000)
//#define A1_ANCHOR_ADDR                  (0x8001)
//#define A2_ANCHOR_ADDR                  (0x8002)

//#define RX_RESPONSE1_TURNAROUND_6M81    (700)  //takes about 100 us for response to come back 3,4
//#define RX_RESPONSE1_TURNAROUND_6M81    (750)  //takes about 100 us for response to come back 3,4
//#define RX_RESPONSE1_TURNAROUND_6M81    (800)  //takes about 100 us for response to come back 3
//#define RX_RESPONSE1_TURNAROUND_6M81    (1100)  //takes about 100 us for response to come back 2 3
//#define RX_RESPONSE1_TURNAROUND_6M81    (2300)  //takes about 100 us for response to come back 1
#define RX_RESPONSE1_TURNAROUND_6M81 (850) //takes about 100 us for response to come back 1

#define RX_RESPONSE1_TURNAROUND_110K (750) //takes about 100 us for response to come back

#define INST_NOT_DONE_YET 0                //this signifies that the instance is still processing the current event
#define INST_DONE_WAIT_FOR_NEXT_EVENT 1    //this signifies that the current event has been processed and instance is ready for next one
#define INST_DONE_WAIT_FOR_NEXT_EVENT_TO 2 //this signifies that the current event has been processed and that instance is waiting for next one with a timeout

typedef enum inst_states
{
  TA_INIT,                   //0
  TA_TXE_WAIT,               //1 - state in which the instance will enter sleep (if ranging finished) or proceed to transmit a message
  TA_TXPOLL_WAIT_SEND,       //2 - configuration and sending of Poll message
  TA_TXFINAL_WAIT_SEND,      //3 - configuration and sending of Final message
  TA_TXRESPONSE_WAIT_SEND,   //4 - a place holder - response is sent from call back
  TA_TX_WAIT_CONF,           //6 - confirmation of TX done message
  TA_RXE_WAIT,               //7
  TA_RX_WAIT_DATA,           //8
  TA_SLEEP_DONE,             //9
  TA_TXRESPONSE_SENT_POLLRX, //10
  TA_TXRESPONSE_SENT_RESPRX, //11
  TA_TXRESPONSE_SENT_TORX    //12
} INST_STATES;

typedef struct
{
  uint8 frameCtrl[2];                            //  frame control bytes 00-01
  uint8 seqNum;                                  //  sequence_number 02
  uint8 panID[2];                                //  PAN ID 03-04
  uint8 destAddr[ADDR_BYTE_SIZE_L];              //  05-12 using 64 bit addresses
  uint8 sourceAddr[ADDR_BYTE_SIZE_L];            //  13-20 using 64 bit addresses
  uint8 messageData[MAX_USER_PAYLOAD_STRING_LL]; //  22-124 (application data and any user payload)
  uint8 fcs[2];                                  //  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlsl;

typedef struct
{
  uint8 frameCtrl[2];                            //  frame control bytes 00-01
  uint8 seqNum;                                  //  sequence_number 02
  uint8 panID[2];                                //  PAN ID 03-04
  uint8 destAddr[ADDR_BYTE_SIZE_S];              //  05-06
  uint8 sourceAddr[ADDR_BYTE_SIZE_S];            //  07-08
  uint8 messageData[MAX_USER_PAYLOAD_STRING_SS]; //  09-124 (application data and any user payload)
  uint8 fcs[2];                                  //  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dsss;

typedef struct
{
  uint8 frameCtrl[2];                            //  frame control bytes 00-01
  uint8 seqNum;                                  //  sequence_number 02
  uint8 panID[2];                                //  PAN ID 03-04
  uint8 destAddr[ADDR_BYTE_SIZE_L];              //  05-12 using 64 bit addresses
  uint8 sourceAddr[ADDR_BYTE_SIZE_S];            //  13-14
  uint8 messageData[MAX_USER_PAYLOAD_STRING_LS]; //  15-124 (application data and any user payload)
  uint8 fcs[2];                                  //  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlss;

typedef struct
{
  uint8 frameCtrl[2];                            //  frame control bytes 00-01
  uint8 seqNum;                                  //  sequence_number 02
  uint8 panID[2];                                //  PAN ID 03-04
  uint8 destAddr[ADDR_BYTE_SIZE_S];              //  05-06
  uint8 sourceAddr[ADDR_BYTE_SIZE_L];            //  07-14 using 64 bit addresses
  uint8 messageData[MAX_USER_PAYLOAD_STRING_LS]; //  15-124 (application data and any user payload)
  uint8 fcs[2];                                  //  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dssl;

typedef struct
{
  uint8 type;      // event type - if 0 there is no event in the queue
  uint8 type_save; // holds the event type - does not clear (used to show what event has been processed)
  uint8 type_pend; // set if there is a pending event
  uint16 rxLength; // length of RX data (does not apply to TX events)

  uint32 uTimeStamp;   //32 bit system counter (ms) - STM32 tick time (at time of IRQ)
  uint64 timeStamp;    // last timestamp (Tx or Rx) - 40 bit DW1000 time
  uint32 timeStamp32l; // last tx/rx timestamp - low 32 bits of the 40 bit DW1000 time
  uint32 timeStamp32h; // last tx/rx timestamp - high 32 bits of the 40 bit DW1000 time

  union {
    uint8 frame[STANDARD_FRAME_SIZE]; //holds received frame (after a good RX frame event)
    //srd_msg_dlsl rxmsg_ll ; //64 bit addresses
    //srd_msg_dssl rxmsg_sl ;
    //srd_msg_dlss rxmsg_ls ;
    srd_msg_dsss rxmsg_ss; //16 bit addresses
  } msgu;

} event_data_t;

typedef struct
{
  INST_MODE mode;            //instance mode (tag or anchor)
  INST_STATES testAppState;  //state machine - current state
  INST_STATES nextState;     //state machine - next state
  INST_STATES previousState; //state machine - previous state
  int done;                  //done with the current event/wait for next event to arrive

  //configuration structures
  dwt_config_t configData;   //DW1000 channel configuration
  dwt_txconfig_t configTX;   //DW1000 TX power configuration
  uint16 txAntennaDelay;     //DW1000 TX antenna delay
  uint16 rxAntennaDelay;     //DW1000 RX antenna delay
  uint32 txPower;            //DW1000 TX power
  uint8 txPowerChanged;      //power has been changed - update the register on next TWR exchange
  uint8 antennaDelayChanged; //antenna delay has been changed - update the register on next TWR exchange
  uint16 instanceAddress16;  //contains tag/anchor address

  //this is the delay used for the delayed transmit
  uint64 pollTx2FinalTxDelay;    //this is delay from Poll Tx time to Final Tx time in DW1000 units (40-bit)
  uint64 pollTx2FinalTxDelayAnc; //this is delay from Poll Tx time to Final Tx time in DW1000 units (40-bit) for Anchor to Anchor ranging
  uint64 fixedReplyDelayAnc;
  uint32 fixedReplyDelayAncP;
  int ancRespRxDelay;
  int fwtoTime_sy; //this is final message duration (longest out of ranging messages)
  int fwtoTimeAnc_sy;
  uint32 delayedReplyTime; // delayed reply time of ranging-init/response/final message

  //message structure used for holding the data of the frame to transmit before it is written to the DW1000
  srd_msg_dsss msg_f; // ranging message frame with 16-bit addresses

  //Tag function address/message configuration
  uint8 shortAdd_idx; // device's 16-bit address low byte (used as index into arrays [0 - 3])
  uint8 eui64[8];     // device's EUI 64-bit address
  uint16 psduLength;  // used for storing the TX frame length
  uint8 frameSN;      // modulo 256 frame sequence number - it is incremented for each new frame transmission
  uint8 frameSN_ar;
  uint16 panID; // panid used in the frames

  //union of TX timestamps,64 bit timestamps
  union {
    uint64 txTimeStamp;      // last tx timestamp
    uint64 tagPollTxTime;    // tag's poll tx timestamp
    uint64 anchorRespTxTime; // anchor's reponse tx timestamp
  } txu;

  uint64 tagPollRxTime; // receive time of poll message

  //application control parameters
  uint8 wait4ack; // if this is set to DWT_RESPONSE_EXPECTED, then the receiver will turn on automatically after TX completion

  int8 responseTO;
  uint8 instToSleep;     // if set the instance will go to sleep before sending the blink/poll message
  uint8 stopTimer;       // stop/disable an active timer
  uint8 instanceTimerEn; // enable/start a timer
  uint8 gotTO;           // got timeout event
  uint8 rxResponseMask;  // bit mask - bit 0 = received response from anchor ID = 0, bit 1 from anchor ID = 1 etc...
  uint16 sframePeriod;
  uint16 slotPeriod;
  uint32 a0SlotTime;
  uint32 a1SlotTime;

  int txMsgCount; //number of transmitted messages
  int rxMsgCount; //number of received messages
  int lateTX;     //number of "LATE" TX events
  int lateRX;     //number of "LATE" RX events

  int newRangeAncAddress; //last 4 bytes of anchor address - used for printing/range output display
  int newRangeTagAddress; //last 4 bytes of tag address - used for printing/range output display

  //event queue - used to store DW1000 events as they are processed by the dw_isr/callback functions
  event_data_t dwevent[MAX_EVENT_NUMBER]; //this holds any TX/RX events and associated message data
  uint8 dweventIdxOut;
  uint8 dweventIdxIn;
  uint8 dweventPeek;
  uint8 monitor;
  uint32 timeofTx;
  int dwIDLE; //set to 1 when the RST goes high after wake up (it is set in process_dwRSTn_irq)

} instance_data_t;

/*******************************************************************************
 FUNCTION PROTOTYPES
*******************************************************************************/
extern void instance_txcallback(const dwt_callback_data_t *txd); // This function is called from the (TX) interrupt handler

extern void instance_rxcallback(const dwt_callback_data_t *rxd); // this is the receive event callback handler

extern void instance_run(void);

extern void inst_processrxtimeout(instance_data_t *inst);

extern void instance_backtoanchor(instance_data_t *inst);

/************************************* END ************************************/
