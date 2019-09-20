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
#include "string.h"
#include "math.h"
//#include "Peri_driver.h"
#include "deca_device_api.h"
#include "deca_types.h"
#include "dw1000_driver.h"
#include "dw1000_instance.h"
#include "deca_param_types.h"
//#include "osi.h"
#include "dw1000_usr.h"
#include "drv_usr.h"

extern uint16_t dev_panid;
extern uint8_t s_pwr;
extern unsigned long pwr_val;
instance_data_t instance_data;

#ifdef ANCHOR_MODE
extern double dwt_prq_dealy_16m;
#endif

/*******************************************************************************
 mode 1 - channel 2,110k speed
 mode 2 - channel 5,110k speed
 mode 3 - channel 2,6.8M speed
 mode 4 - channel 5,6.8M speed
*******************************************************************************/
chConfig_t chConfig[4] = {
    {
        2,               // channel
        DWT_PRF_16M,     // prf
        DWT_BR_110K,     // datarate
        4,               // preambleCode
        DWT_PLEN_1024,   // preambleLength
        DWT_PAC32,       // pacSize
        1,               // non-standard SFD
        (1025 + 64 - 32) // SFD timeout
    },
    {
        2,            // channel
        DWT_PRF_16M,  // prf
        DWT_BR_6M8,   // datarate
        4,            // preambleCode
        DWT_PLEN_128, // preambleLength
        DWT_PAC8,     // pacSize
        0,            // non-standard SFD
        (129 + 8 - 8) // SFD timeout
    },
    {
        5,               // channel
        DWT_PRF_16M,     // prf
        DWT_BR_110K,     // datarate
        3,               // preambleCode
        DWT_PLEN_1024,   // preambleLength
        DWT_PAC32,       // pacSize
        1,               // non-standard SFD
        (1025 + 64 - 32) // SFD timeout
    },
    {
        5,            // channel
        DWT_PRF_16M,  // prf
        DWT_BR_6M8,   // datarate
        3,            // preambleCode
        DWT_PLEN_128, // preambleLength
        DWT_PAC8,     // pacSize
        0,            // non-standard SFD
        (129 + 8 - 8) // SFD timeout
    }};

/*******************************************************************************
 mode 1 - channel 2,110k speed
 mode 2 - channel 5,110k speed
 mode 3 - channel 2,6.8M speed
 mode 4 - channel 5,6.8M speed
*******************************************************************************/
sfConfig_t sfConfig[4] =
    {
        {(28),      //slot period ms
         (50),      //thus 10 slots - thus 280ms superframe means 3.57 Hz location rate (10 slots are needed as AtoA ranging takes 30+ ms)
         (50 * 28), //superframe period
         (50 * 28), //poll sleep delay
         (25000)},
        /*
  {
    (10),       // slot period ms
    (50),       // number of slots (only 10 are used) - thus 100 ms superframe means 10 Hz location rate
    (50*10),    // superframe period (100 ms - gives 10 Hz)
    (50*10),    // poll sleep delay (tag sleep time, usually = superframe period)
    (6000)
  },

  {
    (28),       // slot period ms
    (50),       // thus 10 slots - thus 280ms superframe means 3.57 Hz location rate
    (50*28),    // superframe period
    (50*28),    // poll sleep delay
    (25000)
  },
  */
        {
            (28),      // slot period ms
            (50),      // thus 10 slots - thus 280ms superframe means 3.57 Hz location rate
            (50 * 28), // superframe period
            (50 * 28), // poll sleep delay
            (40000)},
        {(28),      // slot period ms
         (50),      // thus 10 slots - thus 280ms superframe means 3.57 Hz location rate
         (50 * 28), // superframe period
         (50 * 28), // poll sleep delay
         (40000)},
        {
            (10),      // slot period ms
            (50),      // number of slots (only 10 are used) - thus 100 ms superframe means 10 Hz location rate
            (50 * 10), // superframe period (100 ms - gives 10 Hz)
            (50 * 10), // poll sleep (tag sleep time, usually = superframe period)
            (6000)     // this is the Poll to Final delay - 2.4ms (NOTE: if using 6.81 so only 1 frame per ms allowed LDC)
        }};

/*******************************************************************************
 The table below specifies the default TX spectrum configuration parameters... 
 this has been tuned for DW EVK hardware units,the table is set for smart power 
 see below in the instance_config function how this is used when not using smart power
*******************************************************************************/
const tx_struct txSpectrumConfig[8] =
    {
        //Channel 0 -- this is just a place holder so the next array element is channel 1
        {
            0x0,
            {0x0,
             0x0}},
        //Channel 1
        {
            0xc9, //PG_DELAY
            {
                0x15355575, //16M prf power
                0x07274767  //64M prf power
            }

        },
        //Channel 2
        {
            0xc2, //PG_DELAY
            {
                0x15355575, //16M prf power //0x1f1f1f1f, //
                0x07274767  //64M prf power  //0x1f1f1f1f //
            }},
        //Channel 3
        {
            0xc5, //PG_DELAY
            {
                0x0f2f4f6f, //16M prf power
                0x2b4b6b8b  //64M prf power
            }},
        //Channel 4
        {
            0x95, //PG_DELAY
            {
                0x1f1f3f5f, //16M prf power
                0x3a5a7a9a  //64M prf power
            }},
        //Channel 5
        {
            0xc0, //PG_DELAY
            {
                0x0E082848, //16M prf power
                0x25456585  //64M prf power  //0x1f1f1f1f //
            }},
        //Channel 6 -- this is just a place holder so the next array element is channel 7
        {
            0x0,
            {0x0,
             0x0}},
        //Channel 7
        {
            0x93, //PG_DELAY
            {
                0x32527292, //16M prf power
                0x5171B1d1  //64M prf power
            }}};

/*******************************************************************************
 these are default antenna delays for EVB1000
 these can be used if there is no calibration data in the DW1000,
 or instead of the calibration data
*******************************************************************************/
const uint16 rfDelays[2] =
    {
        (uint16)((DWT_PRF_16M_RFDLY / 2.0) * 1e-9 / DWT_TIME_UNITS), //PRF 16
        (uint16)((DWT_PRF_64M_RFDLY / 2.0) * 1e-9 / DWT_TIME_UNITS)  //PRF 64
};

/*******************************************************************************
 these are default TREK Tag/Anchor antenna delays
*******************************************************************************/
const uint16 rfDelaysTREK[2] =
    {
        (uint16)((514.83f / 2.0) * 1e-9 / DWT_TIME_UNITS), //channel 2
        (uint16)((514.65f / 2.0) * 1e-9 / DWT_TIME_UNITS)  //channel 5
};

/******************************************************************************
 clear counts/averages/range values
 return: None
*******************************************************************************/
static void instanceclearcounts(void)
{
  dwt_configeventcounters(1); //enable and clear

  instance_data.frameSN = 0;

  instance_data.frameSN_ar = 0;
  /*
  instance_data.rxTimeouts = 0 ;
  
  instance_data.longTermRangeCount  = 0;

  int i= 0 ;
  
  for(i=0; i<MAX_ANCHOR_LIST_SIZE; i++)
  {
    instance_data.tofArray[i] = INVALID_TOF;
  }

  for(i=0; i<MAX_TAG_LIST_SIZE; i++)
  {
    instance_data.tof[i] = INVALID_TOF;
  }
*/
}

/*******************************************************************************
 Clear instance events
 Returns: None
*******************************************************************************/
void instance_clearevents(void)
{
  int i = 0;

  for (i = 0; i < MAX_EVENT_NUMBER; i++)
  {
    memset(&instance_data.dwevent[i], 0, sizeof(event_data_t));
  }

  instance_data.dweventIdxIn = 0;
  instance_data.dweventIdxOut = 0;
  instance_data.dweventPeek = 0;
}

/*******************************************************************************
 initialise instance structures
 Returns: success: 0, on error: -1
*******************************************************************************/
static int instance_init(void)
{
  /*
  int i;
*/
  int result;

  instance_data.instToSleep = FALSE;
  instance_data.testAppState = TA_INIT;

  //this initialises DW1000 and uses specified configurations from OTP/ROM
  result = dwt_initialise(DWT_LOADUCODE | DWT_LOADLDOTUNE | DWT_LOADTXCONFIG | DWT_LOADANTDLY | DWT_LOADXTALTRIM);
  if (result != DWT_SUCCESS)
  {
    return -1; // device initialise has failed
  }

  instanceclearcounts(); //clear counts/averages/range values

  instance_data.panID = dev_panid;
  instance_data.wait4ack = 0;
  instance_data.stopTimer = 0;

  instance_clearevents();

  memset(instance_data.eui64, 0, ADDR_BYTE_SIZE_L);

  dwt_setautorxreenable(0); //disable auto RX re-enable

  dwt_setdblrxbuffmode(0); //disable double RX buffer

  // if using auto CRC check (DWT_INT_RFCG and DWT_INT_RFCE) are used instead of DWT_INT_RDFR flag
  dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO), 1);

  dwt_setcallbacks(instance_txcallback, instance_rxcallback);

  instance_data.monitor = 0;

  instance_data.lateTX = 0;

  instance_data.lateRX = 0;

  instance_data.responseTO = -1; //initialise

  instance_data.delayedReplyTime = 0;
  return 0;
}

/*******************************************************************************
 convert microseconds to device time
*******************************************************************************/
uint64 convertmicrosectodevicetimeu(double microsecu)
{
  uint64 dt;
  long double dtime;

  dtime = (microsecu / (double)DWT_TIME_UNITS) / 1e6;

  dt = (uint64)(dtime);

  return dt;
}

/*******************************************************************************
 function to set the fixed reply delay time (in us)
 This sets delay for RX to TX - Delayed Send, and for TX to RX delayed receive (wait for response) functionality,
 and the frame wait timeout value to use.  This is a function of data rate, preamble length, and PRF
*******************************************************************************/
static void instancesetreplydelay(int delayus) //delay in us
{
  int margin = 3000; //2000 symbols
  int respframe = 0;
  int respframe_sy = 0;

  //configure the rx delay receive delay time, it is dependent on the message length
  float msgdatalen = 0;
  float preamblelen = 0;
  int sfdlen = 0;
  int x = 0;

  //Set the RX timeouts based on the longest expected message - the Final message
  //Poll = 13, Response = 20, Final = 44 bytes
  //msgdatalen = TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
  msgdatalen = ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;

  x = (int)ceil(msgdatalen * 8 / 330.0f);

  msgdatalen = msgdatalen * 8 + x * 48;

  //add some margin so we don't timeout too soon
  margin = 0; //(TAG_FINAL_MSG_LEN - TAG_POLL_MSG_LEN);

  x = (int)ceil(margin * 8 / 330.0f);

  margin = margin * 8 + x * 48;

  //assume PHR length is 172308ns for 110k and 21539ns for 850k/6.81M
  if (instance_data.configData.dataRate == DWT_BR_110K)
  {
    msgdatalen *= 8205.13f;
    msgdatalen += 172308; // PHR length in nanoseconds
    margin *= 8205.13f;
  }
  else if (instance_data.configData.dataRate == DWT_BR_850K)
  {
    msgdatalen *= 1025.64f;
    msgdatalen += 21539; // PHR length in nanoseconds
    margin *= 1025.64f;
  }
  else
  {
    msgdatalen *= 128.21f;
    msgdatalen += 21539; // PHR length in nanoseconds
    margin *= 128.21f;
  }

  //SFD length is 64 for 110k (always)
  //SFD length is 8 for 6.81M, and 16 for 850k, but can vary between 8 and 16 bytes
  sfdlen = dwnsSFDlen[instance_data.configData.dataRate];

  switch (instance_data.configData.txPreambLength)
  {
  case DWT_PLEN_4096:
    preamblelen = 4096.0f;
    break;
  case DWT_PLEN_2048:
    preamblelen = 2048.0f;
    break;
  case DWT_PLEN_1536:
    preamblelen = 1536.0f;
    break;
  case DWT_PLEN_1024:
    preamblelen = 1024.0f;
    break;
  case DWT_PLEN_512:
    preamblelen = 512.0f;
    break;
  case DWT_PLEN_256:
    preamblelen = 256.0f;
    break;
  case DWT_PLEN_128:
    preamblelen = 128.0f;
    break;
  case DWT_PLEN_64:
    preamblelen = 64.0f;
    break;
  }

  //preamble  = plen * (994 or 1018) depending on 16 or 64 PRF
  if (instance_data.configData.prf == DWT_PRF_16M)
  {
    preamblelen = (sfdlen + preamblelen) * 0.99359f;
  }
  else
  {
    preamblelen = (sfdlen + preamblelen) * 1.01763f;
  }

  respframe_sy = (16 + (int)((preamblelen + ((msgdatalen + margin) / 1000.0)) / 1.0256));

  //this is the delay used for the delayed transmit (when sending the response, and final messages)
  instance_data.pollTx2FinalTxDelay = convertmicrosectodevicetimeu(delayus);
  /*
  //the anchor to anchor ranging consist of A0 ranging to A1 and A2 and A1 ranging to A2
  //as there are less messages the ranging time is shorter (thus divide by 2)
  instance_data.pollTx2FinalTxDelayAnc = convertmicrosectodevicetimeu (delayus/2 + 100);
*/
  //this is the delay the anchors 1, 2, etc.. will send the response back at...
  //anchor 2 will have the delay set to 2 * fixedReplyDelayAnc
  //andhor 3 will have the delay set to 3 * fixedReplyDelayAnc and so on...
  //this delay depends on how quickly the tag can receive and process the message from previous anchor
  //(and also the frame length of course)
  respframe = (int)(preamblelen + (msgdatalen / 1000.0)); //length of response frame (micro seconds)
  if (instance_data.configData.dataRate == DWT_BR_110K)
  {
    //set the frame wait timeout time - total time the frame takes in symbols
    instance_data.fwtoTime_sy = respframe_sy + RX_RESPONSE1_TURNAROUND_110K + 400; //add some margin because of the resp to resp RX turn on time

    instance_data.fwtoTimeAnc_sy = respframe_sy; //add some margin so we don't timeout too soon
    instance_data.fixedReplyDelayAnc = convertmicrosectodevicetimeu(respframe + RX_RESPONSE1_TURNAROUND_110K);
    instance_data.fixedReplyDelayAncP = (uint32)(((uint64)convertmicrosectodevicetimeu(preamblelen)) >> 8) + 16;

    instance_data.ancRespRxDelay = RX_RESPONSE1_TURNAROUND_110K;
  }
  else
  {
    //set the frame wait timeout time - total time the frame takes in symbols
    instance_data.fwtoTime_sy = respframe_sy + RX_RESPONSE1_TURNAROUND_6M81; //add some margin because of the resp to resp RX turn on time

    instance_data.fwtoTimeAnc_sy = respframe_sy;
    instance_data.fixedReplyDelayAnc = convertmicrosectodevicetimeu(respframe + RX_RESPONSE1_TURNAROUND_6M81);
    instance_data.fixedReplyDelayAncP = (uint32)(((uint64)convertmicrosectodevicetimeu(preamblelen)) >> 8) + 16;

    instance_data.ancRespRxDelay = RX_RESPONSE1_TURNAROUND_6M81;
  }
}

/******************************************************************************
 allow application configuration be passed into instance and affect underlying device operation
 Return: None
*******************************************************************************/
void instance_config(instanceConfig_t *config, sfConfig_t *sfConfig)
{
  uint8 otprev;
  //  uint32 power = 0;

  instance_data.configData.chan = config->channelNumber;
  instance_data.configData.rxCode = config->preambleCode;
  instance_data.configData.txCode = config->preambleCode;
  instance_data.configData.prf = config->pulseRepFreq;
  instance_data.configData.dataRate = config->dataRate;
  instance_data.configData.txPreambLength = config->preambleLen;
  instance_data.configData.rxPAC = config->pacSize;
  instance_data.configData.nsSFD = config->nsSFD;
  instance_data.configData.phrMode = DWT_PHRMODE_STD;
  instance_data.configData.sfdTO = config->sfdTO;

  //the DW1000 will automatically use gating gain for frames < 1ms duration (i.e. 6.81Mbps data rate)
  //smartPowerEn should be set based on the frame length, but we can also use dtaa rate.

  /*
  if(instance_data.configData.dataRate == DWT_BR_6M8)
  {
    instance_data.configData.smartPowerEn = 1;
  }
  else
  {
    instance_data.configData.smartPowerEn = 0;
  }
*/
  instance_data.configData.smartPowerEn = s_pwr;

  dwt_setsmarttxpower(instance_data.configData.smartPowerEn);

  //configure the channel parameters
  dwt_configure(&instance_data.configData, DWT_LOADXTALTRIM);

  instance_data.configTX.PGdly = txSpectrumConfig[config->channelNumber].PGdelay;
  /*
  //firstly check if there are calibrated TX power value in the DW1000 OTP
  power = dwt_getotptxpower(config->pulseRepFreq, instance_data.configData.chan);

  if((power == 0x0) || (power == 0xFFFFFFFF)) //if there are no calibrated values... need to use defaults
  {
    power = txSpectrumConfig[config->channelNumber].txPwr[config->pulseRepFreq- DWT_PRF_16M];
  }

  //Configure TX power
  instance_data.configTX.power = power;
*/
  instance_data.configTX.power = pwr_val;

  //configure the tx spectrum parameters (power and PG delay)
  dwt_configuretxrf(&instance_data.configTX);

  otprev = dwt_otprevision(); // this revision tells us how OTP is programmed.

  if ((2 == otprev) || (3 == otprev)) // board is calibrated with TREK1000 with antenna delays set for each use case
  {
    uint8 mode = (instance_data.mode == ANCHOR ? 1 : 0);
    uint8 chanindex = 0;

    instance_data.txAntennaDelay = dwt_getTREKOTPantennadelay(
        mode,
        instance_data.configData.chan,
        instance_data.configData.dataRate);

    // if nothing was actually programmed then set a reasonable value anyway
    if ((instance_data.txAntennaDelay == 0) || (instance_data.txAntennaDelay == 0xffff))
    {
      if (instance_data.configData.chan == 5)
      {
        chanindex = 1;
      }

      instance_data.txAntennaDelay = rfDelaysTREK[chanindex];
    }
  }
  else // assume it is older EVK1000 programming.
  {
    //get the antenna delay that was read from the OTP calibration area
    instance_data.txAntennaDelay = dwt_readantennadelay(config->pulseRepFreq) >> 1;

    // if nothing was actually programmed then set a reasonable value anyway
    if ((instance_data.txAntennaDelay == 0) || (instance_data.txAntennaDelay == 0xffff))
    {
#ifdef ANCHOR_MODE
      instance_data.txAntennaDelay = (uint16_t)dwt_prq_dealy_16m; //rfDelays[config->pulseRepFreq - DWT_PRF_16M];
#else
      instance_data.txAntennaDelay = rfDelays[config->pulseRepFreq - DWT_PRF_16M];
#endif
    }
  }

  // set the antenna delay, we assume that the RX is the same as TX.
  dwt_setrxantennadelay(instance_data.txAntennaDelay);

  dwt_settxantennadelay(instance_data.txAntennaDelay);

  instance_data.rxAntennaDelay = instance_data.txAntennaDelay;

  if (config->preambleLen == DWT_PLEN_64) //if preamble length is 64
  {
    DECA_SPI_Config_Rate(NRF_DRV_SPI_FREQ_2M); //reduce SPI to < 3MHz

    dwt_loadopsettabfromotp(0);

    DECA_SPI_Config_Rate(NRF_DRV_SPI_FREQ_8M);
    ; //increase SPI to max
  }
  instance_data.sframePeriod = sfConfig->sfPeriod;
  instance_data.slotPeriod = sfConfig->slotPeriod;

  //last two slots are used for anchor to anchor ranging
  instance_data.a0SlotTime = (sfConfig->numSlots - 2) * instance_data.slotPeriod;

  instancesetreplydelay(sfConfig->replyDly); //set the default response delays
}

/*******************************************************************************
 Configures DW1000 Device
 return: SUCCESS: 0, FAILURE: -1
*******************************************************************************/
int dw1000_init(uint8 dev_channel_speed, uint16 dev_addr)
{
  int result;
  uint32_t dw1000_id;
  instanceConfig_t instConfig;
  DECA_SPI_Config_Rate(NRF_DRV_SPI_FREQ_1M);
  dw1000_id = dwt_readdevid(); //DW1000 id is 0xDECA0130
  rt_kprintf("dw1000_id1:%x\r\n", dw1000_id);
  if (dw1000_id != DWT_DEVICE_ID) //if the read of device ID fails, the DW1000 could be asleep
  {
    DW1000_CS_off(); //SPI CS low

    delay_ms(2); //200 us to wake up

    DW1000_CS_on();
    ; //SPI CS high

    delay_ms(7); //waits 5ms for DW1000 XTAL to stabilise

    dw1000_id = dwt_readdevid(); //DW1000 id is 0xDECA0130
    rt_kprintf("dw1000_id2:%x\r\n", dw1000_id);
    if (dw1000_id != DWT_DEVICE_ID) // SPI not working or Unsupported Device ID
    {
      reset_DW1000(); //reset the DW1000 by driving the RSTn line low

      return -1;
    }
    dwt_softreset(); //clear the sleep bit - so that after the hard reset below the DW does not go into sleep
  }

  reset_DW1000(); //reset the DW1000 by driving the RSTn line low
  rt_kprintf("reset dw1000\r\n");
  result = instance_init(); //initialise instance structures
  if (result < 0)
  {
    rt_kprintf("init dw1000 err\r\n");
    return -1; // Some failure has occurred
  }
  //DECA_SPI_Config_Rate(NRF_DRV_SPI_FREQ_8M); //increase SPI to max

  dw1000_id = dwt_readdevid(); //DW1000 id is 0xDECA0130
  rt_kprintf("dw1000_id31:%x\r\n", dw1000_id);
  if (dw1000_id != DWT_DEVICE_ID)
  {
    return -1; // Means it is NOT DW1000 device
  }

#ifdef TAG_MODE
  instance_data.mode = TAG;
#endif
#ifdef ANCHOR_MODE
  instance_data.mode = ANCHOR;
#endif

  instance_data.instanceAddress16 = dev_addr;

  instConfig.channelNumber = chConfig[dev_channel_speed].channel;
  instConfig.preambleCode = chConfig[dev_channel_speed].preambleCode;
  instConfig.pulseRepFreq = chConfig[dev_channel_speed].prf;
  instConfig.pacSize = chConfig[dev_channel_speed].pacSize;
  instConfig.nsSFD = chConfig[dev_channel_speed].nsSFD;
  instConfig.sfdTO = chConfig[dev_channel_speed].sfdTO;
  instConfig.dataRate = chConfig[dev_channel_speed].datarate;
  instConfig.preambleLen = chConfig[dev_channel_speed].preambleLength;

  instance_config(&instConfig, &sfConfig[dev_channel_speed]); // Set operating channel etc
  return 0;
}

/*******************************************************************************
                                      END         
*******************************************************************************/

/*******************************************************************************
                                      END         
*******************************************************************************/