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
#include "stdbool.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "time.h"
#include "deca_device_api.h"
#include "deca_types.h"
#include "deca_regs.h"
#include "dw1000_driver.h"
#include "dw1000_instance.h"
//#include "Peri_driver.h"
//#include "at24c08.h"
//#include "osi.h"
#include "Uart_task.h"
#include "dw1000_usr.h"
#include "rtthread.h"

#define PA_LNA_MODE

extern struct rt_messagequeue message_print;

//extern struct rt_messagequeue print_data;

extern dw1000_debug_message_tag dw1000_debug_message;

extern volatile unsigned long sys_Tick;
event_data_t dw_event_g;
extern instance_data_t instance_data;

#ifdef TAG_MODE
  extern OsiMsgQ_t Msg_Queue;  //Used for msg poll
  extern unsigned long slp_time;
  extern uint8_t range_type;
  unsigned long wake_up_time = 0;
#endif

#ifdef ANCHOR_MODE
//  extern OsiSyncObj_t dw_rx_Semaphore;
  //extern OsiMsgQ_t UartMsg_Queue;  //Used for Uart Print
  extern uint8_t anchor_mode;
  extern uint8_t resp_slot;
  extern uint16_t sync_period;
  extern uint16_t range_period;
  unsigned long sync_tick = 0;
  unsigned long range_tick = 0;
  uint16_t tag_poll_addr;
#endif

/*******************************************************************************
 put and save the event
 return: None
*******************************************************************************/
void instance_putevent(event_data_t newevent, uint8 etype)
{
  instance_data.dwevent[instance_data.dweventIdxIn] = newevent;  //copy event

  //set type - this makes it a new event (making sure the event data is copied before event is set as new)
  //to make sure that the get event function does not get an incomplete event
  instance_data.dwevent[instance_data.dweventIdxIn].type = etype;

  instance_data.dweventIdxIn++;

  if(instance_data.dweventIdxIn == MAX_EVENT_NUMBER)
  {
    instance_data.dweventIdxIn = 0;
  }
}

/*******************************************************************************
 This function is called from the (TX) interrupt handler
 return: None
*******************************************************************************/
void instance_txcallback(const dwt_callback_data_t *txd)
{
  event_data_t dw_event;
  uint8 txevent = txd->event;
  uint8 txTimeStamp[5] = {0, 0, 0, 0, 0};

  if(txevent == DWT_SIG_TX_DONE)  //get TX good done
  {
    dwt_readtxtimestamp(txTimeStamp) ;
    
    dw_event.timeStamp32l = (uint32)txTimeStamp[0] + ((uint32)txTimeStamp[1] << 8) + ((uint32)txTimeStamp[2] << 16) + ((uint32)txTimeStamp[3] << 24);
    dw_event.timeStamp = txTimeStamp[4];
    dw_event.timeStamp <<= 32;
    dw_event.timeStamp += dw_event.timeStamp32l;
    dw_event.timeStamp32h = ((uint32)txTimeStamp[4] << 24) + (dw_event.timeStamp32l >> 8);
    
    instance_data.stopTimer = 0;

    dw_event.rxLength = instance_data.psduLength;
    dw_event.type =  0;
    dw_event.type_pend =  0;
    dw_event.type_save = DWT_SIG_TX_DONE;

    memcpy((uint8 *)&dw_event.msgu.frame[0], (uint8 *)&instance_data.msg_f, instance_data.psduLength);
      
    instance_putevent(dw_event, DWT_SIG_TX_DONE);

    instance_data.txMsgCount++;
  }
  else if(txevent == DWT_SIG_TX_AA_DONE)  
  {
    //auto ACK confirmation
    dw_event.rxLength = 0;
    dw_event.type =  0;
    dw_event.type_save = DWT_SIG_TX_AA_DONE;
    
    instance_putevent(dw_event, DWT_SIG_TX_AA_DONE);
  }
  instance_data.monitor = 0;
}

/******************************************************************************
 function to re-enable the receiver and also adjust the timeout before sending the final message
 if it is time so send the final message, the callback will notify the application, else the receiver is automatically re-enabled
 this function is only used for tag when ranging to other anchors
******************************************************************************/
uint8 tagrxreenable(void)
{
  uint8 type_pend = DWT_SIG_DW_IDLE;

  if(instance_data.responseTO > 0)  //can get here as result of error frame so need to check
  {
    dwt_setrxtimeout((uint16)instance_data.fwtoTime_sy * instance_data.responseTO); //reconfigure the timeout
    
    dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
    
    type_pend = DWT_SIG_RX_PENDING ;
  }
  else //last response was not received (got error/frame was corrupt)
  {
    type_pend = DWT_SIG_DW_IDLE;  //report timeout - send the final
  }

  return type_pend;
}

/*******************************************************************************
//this function either enables the receiver (delayed)
*******************************************************************************/
void ancenablerx(void)
{
  //subtract preamble length
  dwt_setdelayedtrxtime(instance_data.delayedReplyTime - instance_data.fixedReplyDelayAncP) ;
  
  if(dwt_rxenable(DWT_START_RX_DELAYED)) //delayed rx
  {
    //if the delayed RX failed - time has passed - do immediate enable
    dwt_setrxtimeout((uint16)instance_data.fwtoTimeAnc_sy*2); //reconfigure the timeout before enable
    
    //longer timeout as we cannot do delayed receive... so receiver needs to stay on for longer
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    
    dwt_setrxtimeout((uint16)instance_data.fwtoTimeAnc_sy); //restore the timeout for next RX enable
    
    instance_data.lateRX++;
  }
}

#ifdef ANCHOR_MODE
/*******************************************************************************
// this function either re-enables the receiver (delayed or immediate) or transmits the response frame
//the sourceAddress is the address of the sender of the current received frame
//ancToAncTWR == 1 means that the anchor is ranging to another anchor, if == 0 then ranging to a tag
*******************************************************************************/
uint8 anctxorrxreenable(uint16 sourceAddress, int ancToAncTWR)
{
  uint8 type_pend = DWT_SIG_DW_IDLE;
  
  if(instance_data.responseTO == 0)  //go back to RX without TO - ranging has finished. (wait for Final but no TO)
  {
    dwt_setrxtimeout(0);  //reconfigure the timeout
    
    dwt_setpreambledetecttimeout(0);
  }
  
//  srand(sys_Tick);
  
  //configure delayed reply time (this is incremented for each received frame) it is timed from Poll rx time

  instance_data.delayedReplyTime += resp_slot*(instance_data.fixedReplyDelayAnc >> 8)/**(1-(rand()%10000)/200000)*/;
  
  //this checks if to send a frame
  if(((ancToAncTWR & 1) == 0) && (instance_data.responseTO == NUM_EXPECTED_RESPONSES)) //it's our turn to tx
  {
    //response is expected
    instance_data.wait4ack = DWT_RESPONSE_EXPECTED; //re has/will be re-enabled

    dwt_setdelayedtrxtime(instance_data.delayedReplyTime) ;
       
    if(dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED))
    {
      //if TX has failed - we need to re-enable RX for the next response or final reception...
      dwt_setrxaftertxdelay(0);
      instance_data.wait4ack = 0; //clear the flag as the TX has failed the TRX is off
      instance_data.lateTX++;
      instance_data.delayedReplyTime += 2*(instance_data.fixedReplyDelayAnc >> 8); //to take into account W4R
      ancenablerx();
      type_pend = DWT_SIG_RX_PENDING ;
      printf("send err\r\n");
    }
    else
    {
      instance_data.delayedReplyTime += (instance_data.fixedReplyDelayAnc >> 8); //to take into account W4R
      type_pend = DWT_SIG_TX_PENDING ; // exit this interrupt and notify the application/instance that TX is in progress.
      instance_data.timeofTx = sys_Tick;
      instance_data.monitor = 1;
      // printf("send ok\r\n");
    }
  }
  else //stay in receive
  {
    dwt_setrxtimeout(0);
    
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    
    type_pend = DWT_SIG_RX_PENDING ;
  }

  return type_pend;
}
#endif

/*******************************************************************************
 this function handles frame error event
 it will either signal TO or re-enable the receiver
*******************************************************************************/
void handle_error_unknownframe(event_data_t dw_event)
{
  //re-enable the receiver (after error frames as we are not using auto re-enable
  //for ranging application rx error frame is same as TO - as we are not going to get the expected frame
  
#ifdef ANCHOR_MODE
    //if we are participating in the ranging (i.e. Poll was received)
    //and we get an rx error (in one of the responses)
    //need to consider this as a timeout as we could be sending our response next and
    //the applications needs to know to change the state
    if(instance_data.responseTO > 0)
    {
      instance_data.responseTO--;

      //send a response or re-enable rx
      dw_event.type_pend = anctxorrxreenable(0, 0);
      dw_event.type = 0;
      dw_event.type_save = 0x40 | DWT_SIG_RX_TIMEOUT;
      dw_event.rxLength = 0;

      instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
    }
    else
    {
      dwt_setrxtimeout(0); //reconfigure the timeout
      dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
    }
#endif
#ifdef TAG_MODE
    instance_data.responseTO--;  //got something (need to reduce timeout (for remaining responses))

    dw_event.type_pend = tagrxreenable();  //check if receiver will be re-enabled or it's time to send the final
    dw_event.type = 0;
    dw_event.type_save = 0x40 | DWT_SIG_RX_TIMEOUT;
    dw_event.rxLength = 0;

    instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
#endif
}

#ifdef ANCHOR_MODE
/*******************************************************************************
 this function prepares and writes the anchor to tag response frame into the TX buffer
 it is called after anchor receives a Poll from a tag
*******************************************************************************/
void ancprepareresponse(uint16 sourceAddress, uint8 srcAddr_index, uint8 fcode_index, uint8 *frame)
{
  uint16 frameLength = 0;

  instance_data.psduLength = frameLength = ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
  memcpy(&instance_data.msg_f.destAddr[0], &frame[srcAddr_index], ADDR_BYTE_SIZE_S); //remember who to send the reply to (set destination address)
  
  instance_data.msg_f.sourceAddr[0] = instance_data.eui64[0];
  instance_data.msg_f.sourceAddr[1] = instance_data.eui64[1];

//  srand(sys_Tick);
  
  //set the delayed rx on time (the final message will be sent after this delay)
//  if((sys_Tick%2)==0)
//  {
    dwt_setrxaftertxdelay(instance_data.ancRespRxDelay/*+(rand()%25)*/);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)
//  }
//  else
//  {
//    dwt_setrxaftertxdelay(instance_data.ancRespRxDelay-(rand()%25));  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)
//  }

  instance_data.msg_f.messageData[FCODE] = RTLS_DEMO_MSG_ANCH_RESP;  //message function code (specifies if message is a poll, response or other...)
  
  instance_data.msg_f.messageData[RESP_MSG_ANCHOR_SLOT] = resp_slot;  
      
  dwt_writetxfctrl(frameLength, 0);  //write the TX data
  
  dwt_writetxdata(frameLength, (uint8 *)  &instance_data.msg_f, 0) ;  // write the frame data
}
#endif

/*******************************************************************************
//get the First path index.
//return: the quality of the received message
*******************************************************************************/
uint16_t get_fp_index(void)
{
  uint8 dev_reg[2];
  uint16 fp_index;
  
  dwt_readfromdevice(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET, 2, dev_reg) ; //read the HW FP index
  //LDE result 0 is in bytes 40->55
  // convert (and save) hardware estimate to show on graph
  fp_index = dev_reg[0];
  fp_index = fp_index + (dev_reg[1] << 8);
  return fp_index/64;
}

/*******************************************************************************
//get the receive power
//return: receive power value
*******************************************************************************/
float Get_Receive_Power(void)
{
  uint8_t power_reg[RX_FQUAL_LEN];
  uint8_t rx_frame_reg[RX_FINFO_LEN];
  uint32_t two17 = 131072;
  uint16_t c_val,n_val;
  float a_val = 113.77;
  float corrfac = 2.3334;
  float rx_pwr;
  
  dwt_readfromdevice(RX_FQUAL_ID,0,RX_FQUAL_LEN,power_reg);
  
  c_val = (uint16_t)power_reg[2] | ((uint16_t)power_reg[3] << 8);
    
  dwt_readfromdevice(RX_FINFO_ID,0,RX_FINFO_LEN,rx_frame_reg);
    
  n_val = (((uint16_t)rx_frame_reg[2] >> 4) & 0xFF) | ((uint16_t)rx_frame_reg[3] << 4);
    
  rx_pwr = 10*log10((c_val*two17)/(n_val*n_val)) - a_val;
  if(rx_pwr > -88)
  {
    rx_pwr += (rx_pwr+88)*corrfac;
  }

  return rx_pwr;
}

/*******************************************************************************
 this is the receive event callback handler
return: None
*******************************************************************************/
void instance_rxcallback(const dwt_callback_data_t *rxd)
{
  uint8 rxd_event = 0;
  uint8 fcode_index  = 0;
  uint8 srcAddr_index = 0;
  event_data_t dw_event;
  uint8 rxTimeStamp[5]  = {0, 0, 0, 0, 0};
//  uint16_t fp_value;    
  dw_event.uTimeStamp = sys_Tick;
  
  if(rxd->event == DWT_SIG_RX_OKAY)  //if we got a frame with a good CRC - RX OK
  {
    dw_event.rxLength = rxd->datalength;
    if(((rxd->fctrl[0] == 0x41) || (rxd->fctrl[0] == 0x61))&&((rxd->fctrl[1] & 0xCC) == 0x88)) //short address
    {
      fcode_index = FRAME_CRTL_AND_ADDRESS_S; //function code is in first byte after source address
      
      srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_S;
      
      rxd_event = DWT_SIG_RX_OKAY;
    }
    else
    {
      rxd_event = SIG_RX_UNKNOWN;  //not supported - used short addressed
    }

    dwt_readrxtimestamp(rxTimeStamp) ;  //read RX timestamp
    
    dwt_readrxdata((uint8 *)&dw_event.msgu.frame[0], rxd->datalength, 0);  // Read Data Frame
    
    dw_event.timeStamp32l =  (uint32)rxTimeStamp[0] + ((uint32)rxTimeStamp[1] << 8) + ((uint32)rxTimeStamp[2] << 16) + ((uint32)rxTimeStamp[3] << 24);
    dw_event.timeStamp = rxTimeStamp[4];
    dw_event.timeStamp <<= 32;
    dw_event.timeStamp += dw_event.timeStamp32l;
    dw_event.timeStamp32h = ((uint32)rxTimeStamp[4] << 24) + (dw_event.timeStamp32l >> 8);
    
    dw_event.type = 0;  //type will be added as part of adding to event queue
    dw_event.type_save = rxd_event;
    dw_event.type_pend = DWT_SIG_DW_IDLE;

//    uint16 sourceAddress = (((uint16)dw_event.msgu.frame[srcAddr_index+1]) << 8) + dw_event.msgu.frame[srcAddr_index];
//    
//    fp_value = get_fp_index();
//    
//    if(sourceAddress == 0x0007)
//       
//    printf("fp_index:%d\n\r",fp_value);
    
//    if((rxd_event == DWT_SIG_RX_OKAY)&&(fp_value > 650)) //Process good/known frame types
    if(rxd_event == DWT_SIG_RX_OKAY) //Process good/known frame types
    {
      uint16 rev_panid = dw_event.msgu.rxmsg_ss.panID[0] + 256*dw_event.msgu.rxmsg_ss.panID[1];
      
      if(rev_panid == instance_data.panID)
      { 
        uint16_t dest_addr = dw_event.msgu.rxmsg_ss.destAddr[0] + 256*dw_event.msgu.rxmsg_ss.destAddr[1];
          
#ifdef ANCHOR_MODE
        uint16 sourceAddress = (((uint16)dw_event.msgu.frame[srcAddr_index+1]) << 8) + dw_event.msgu.frame[srcAddr_index];
#endif
      
        if((instance_data.mode == TAG)||(instance_data.mode == ANCHOR_RNG)) //if tag got a good frame 
        {
          instance_data.responseTO--; //got 1 more response or other RX frame - need to reduce timeout (for next response)
        }

        switch(dw_event.msgu.frame[fcode_index])  //check if this is a TWR message
        {
          case TOF_TAG_POLL_MSG:
          {
            if(instance_data.mode == TAG)  //tag should ignore any other Polls from tags
            {
              instance_data.responseTO++;  //as will be decremented in the function and was also decremented above
              handle_error_unknownframe(dw_event);
              instance_data.stopTimer = 1;
              instance_data.rxMsgCount++;
              return;
            }
#ifdef ANCHOR_MODE  
            if(instance_data.mode != ANCHOR_RNG)
            {
              tag_poll_addr = sourceAddress;
              
              if(!check_noresp_addrs(sourceAddress))
              {              
                //prepare the response and write it to the tx buffer
                ancprepareresponse(sourceAddress, srcAddr_index, fcode_index, &dw_event.msgu.frame[0]);

                dwt_setrxtimeout((uint16)instance_data.fwtoTimeAnc_sy); //reconfigure the timeout for response

                instance_data.delayedReplyTime = dw_event.timeStamp32h ;
                instance_data.responseTO = NUM_EXPECTED_RESPONSES; //set number of expected responses to 3 (from other anchors)

                dw_event.type_pend = anctxorrxreenable(instance_data.instanceAddress16, 2+0);
              }
              else
              {
                dw_event.msgu.frame[fcode_index] = TDOA_TAG_POLL_MSG;
                
                dw_event.type_pend = DWT_SIG_DW_IDLE;
              }
            }
#endif
          }
          break;
          
          //we got a response from a "responder" (anchor)
          case RTLS_DEMO_MSG_ANCH_RESP:
          {
            //we are a tag
            if(((instance_data.mode == TAG)||(instance_data.mode == ANCHOR_RNG))&&(dest_addr == instance_data.instanceAddress16)) //if tag got a good frame 
            {
              uint8 index ;

              if(dw_event.msgu.rxmsg_ss.messageData[RESP_MSG_ANCHOR_SLOT] <= MAX_ANCHOR_RESP_SIZE)
              {
                instance_data.responseTO = MAX_ANCHOR_RESP_SIZE - dw_event.msgu.rxmsg_ss.messageData[RESP_MSG_ANCHOR_SLOT];
              }
                
              dw_event.type_pend = tagrxreenable(); //responseTO decremented above...

              index = RRXT0 + 7*instance_data.rxResponseMask;
              
              instance_data.rxResponseMask += 1; //add anchor ID to the mask

              if(instance_data.rxResponseMask <= MAX_ANCHOR_RESP_SIZE )
              {
                // Write Response RX time field of Final message
                memcpy(&(instance_data.msg_f.messageData[index]), &(dw_event.msgu.frame[srcAddr_index]), ADDR_BYTE_SIZE_S);
                
                // Write Response RX time field of Final message
                memcpy(&(instance_data.msg_f.messageData[index+2]), rxTimeStamp, 5);
              }
            }
          }
          break;

          case TOF_TAG_FINAL_MSG:
          {
            if((instance_data.mode == TAG)||(instance_data.mode == ANCHOR_RNG)) //tag should ignore any other Final from anchors
            {
              instance_data.responseTO++; //as will be decremented in the function and was also decremented above
              handle_error_unknownframe(dw_event);
              instance_data.stopTimer = 1;
              instance_data.rxMsgCount++;
              return;
            }
          }
          //if anchor fall into case below and process the frame
          default:  //process rx frame
          {
            dw_event.type_pend = DWT_SIG_DW_IDLE;
          }
          break;
        }
        instance_data.stopTimer = 1;

        instance_putevent(dw_event, rxd_event);

        instance_data.rxMsgCount++;
      }
      else
      {
        handle_error_unknownframe(dw_event);
      }

    }
    else //if (rxd_event == SIG_RX_UNKNOWN) //need to re-enable the rx (got unknown frame type)
    {
//      printf("fp_index:%d\n\r",fp_value);
      
      handle_error_unknownframe(dw_event);
    }
  }
  else if (rxd->event == DWT_SIG_RX_TIMEOUT) //if tag and got TO, then did not get any or some responses - check if need to send final.
  {
    dw_event.type_pend = DWT_SIG_DW_IDLE;
    
#ifdef ANCHOR_MODE
    if(instance_data.mode == ANCHOR)
    {
      //check if anchor has received all of the responses from other anchors (it could have received only 1 or 2)
      //it's timed out (re-enable rx or tx response)
      if(instance_data.responseTO > 0)
      {
        instance_data.responseTO--;
        
        //send a response or re-enable rx
        dw_event.type_pend = anctxorrxreenable(instance_data.instanceAddress16, 6+0);
      }
    }
#endif
    
    dw_event.type = 0;
    dw_event.type_save = DWT_SIG_RX_TIMEOUT;
    dw_event.rxLength = 0;
    dw_event.timeStamp = 0;
    dw_event.timeStamp32l = 0;
    dw_event.timeStamp32h = 0;

    instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
  }
  else //assume other events are errors
  {
    handle_error_unknownframe(dw_event);
  }
//#ifdef ANCHOR_MODE    
//  osi_SyncObjSignalFromISR(&dw_rx_Semaphore); 
//endif
}

/*******************************************************************************
//function to construct the message/frame header bytes
*******************************************************************************/
void instanceconfigframeheader16(instance_data_t *inst)
{
  //set frame type (0-2), SEC (3), Pending (4), ACK (5), PanIDcomp(6)
  inst->msg_f.frameCtrl[0] = 0x1 | 0x40;  /*frame type 0x1 == data*/ /*PID comp*/

  //source/dest addressing modes and frame version
  inst->msg_f.frameCtrl[1] = 0x8 | 0x80;  /*dest extended address (16bits)*//*src extended address (16bits)*/

  inst->msg_f.panID[0] = (inst->panID) & 0xff;
  
  inst->msg_f.panID[1] = inst->panID >> 8;

  inst->msg_f.seqNum = 0;
}

/*******************************************************************************
// get dw1000 event
*******************************************************************************/
event_data_t* instance_getevent(void)
{
  int indexOut = instance_data.dweventIdxOut;

  if(instance_data.dwevent[indexOut].type == 0)  //exit with "no event"
  {
    dw_event_g.type = 0;
    dw_event_g.type_save = 0;
    
    return &dw_event_g;
  }

  //copy the event
  dw_event_g.type_save = instance_data.dwevent[indexOut].type_save ;
  dw_event_g.type_pend = instance_data.dwevent[indexOut].type_pend ;
  dw_event_g.rxLength = instance_data.dwevent[indexOut].rxLength ;
  dw_event_g.timeStamp = instance_data.dwevent[indexOut].timeStamp ;
  dw_event_g.timeStamp32l = instance_data.dwevent[indexOut].timeStamp32l ;
  dw_event_g.timeStamp32h = instance_data.dwevent[indexOut].timeStamp32h ;
  dw_event_g.uTimeStamp = instance_data.dwevent[indexOut].uTimeStamp ;
  
  memcpy(&dw_event_g.msgu, &instance_data.dwevent[indexOut].msgu, sizeof(instance_data.dwevent[indexOut].msgu));

  dw_event_g.type = instance_data.dwevent[indexOut].type ;

  instance_data.dwevent[indexOut].type = 0; //clear the event

  instance_data.dweventIdxOut++;
  
  if(MAX_EVENT_NUMBER == instance_data.dweventIdxOut) //wrap the counter
  {
    instance_data.dweventIdxOut = 0;
  }
  instance_data.dweventPeek = instance_data.dweventIdxOut; //set the new peek value

  return &dw_event_g;
}

/*******************************************************************************
 update the antenna delay if it has changed
*******************************************************************************/
void instancesetantennadelays(void)
{
  if(instance_data.antennaDelayChanged == 1)
  {
    dwt_setrxantennadelay(instance_data.rxAntennaDelay);
    
    dwt_settxantennadelay(instance_data.txAntennaDelay);
    
    instance_data.antennaDelayChanged = 0;
  }
}

/*******************************************************************************
 configure TX power if it has changed
*******************************************************************************/
void instancesettxpower(void)
{
  if(instance_data.txPowerChanged == 1)
  {
    dwt_write32bitreg(0x1E, instance_data.txPower);  //Configure TX power

    instance_data.txPowerChanged = 0;
  }
}

double convertdevicetimetosec(int32 dt)
{
  double f = 0;

  f =  dt * DWT_TIME_UNITS ;  // seconds #define TIME_UNITS          (1.0/499.2e6/128.0) = 15.65e-12

  return f ;
}

double range_distance(uint32 tof_val)
{
  int32 tofi ;
  double tof ;
  double distance ;
  double distance_to_correct;
  
  tofi = (int32) tof_val ;  // make it signed
  
  tof = convertdevicetimetosec(tofi) ;  //this is divided by 4 to get single time of flight
  
  distance = tof * SPEED_OF_LIGHT;
  
  if(instance_data.configData.smartPowerEn)
  { 
    if(instance_data.configData.chan == 5)  //1.51 for channel 5
    {
      distance_to_correct = distance/1.51;
    }
    else  
    {
      distance_to_correct = distance/1.31;  //1.31 for channel 2
    }
  }
  else
  {
    distance_to_correct = distance;
  }

  distance = distance - dwt_getrangebias(instance_data.configData.chan, (float) distance_to_correct, instance_data.configData.prf);
  
  return 1000*distance;
}

/*******************************************************************************
// Set delay to send
*******************************************************************************/
int instancesenddlypacket(instance_data_t *inst, int delayedTx)
{
  int result = 0;

  dwt_writetxfctrl(inst->psduLength, 0);
  
  if(delayedTx == DWT_START_TX_DELAYED)
  {
    dwt_setdelayedtrxtime(inst->delayedReplyTime) ; //should be high 32-bits of delayed TX TS
  }

  //begin delayed TX of frame
  if (dwt_starttx(delayedTx | inst->wait4ack))  // delayed start was too late
  {
    result = 1; //late/error
    inst->lateTX++;
  }
  else
  {
    inst->timeofTx = sys_Tick;
    inst->monitor = 1;
  }
  
  return result;  // state changes
}

/*******************************************************************************
// Go back to anchor mode
*******************************************************************************/
void instance_backtoanchor(instance_data_t *inst)
{
  inst->testAppState = TA_RXE_WAIT ;  //stay in RX and behave as anchor
  inst->mode = ANCHOR ;
  dwt_setrxtimeout(0);
  dwt_setpreambledetecttimeout(0);
  dwt_setrxaftertxdelay(0);
}

/*******************************************************************************
//RX time out event
*******************************************************************************/
void inst_processrxtimeout(instance_data_t *inst)
{
  inst->done = INST_NOT_DONE_YET;

  if(inst->mode == ANCHOR) //we did not receive the final - wait for next poll
  {
    //only enable receiver when not using double buffering
    
    inst->testAppState = TA_RXE_WAIT ;  // wait for next frame
    
    dwt_setrxtimeout(0);
  }
  else if(inst->mode == ANCHOR_SYNC)
  {
    instance_backtoanchor(inst);
  }
  else //if((inst->mode == TAG)||(inst->mode == ANCHOR_RNG))
  {
    //if tag times out - no response (check if we are to send a final)
    //send the final only if it has received response from anchor
    if((inst->previousState == TA_TXPOLL_WAIT_SEND) && (inst->rxResponseMask == 0))
    {
      if((inst->mode == TAG))
      {
        inst->instToSleep = TRUE ;  //set sleep to TRUE so that tag will go to DEEP SLEEP before next ranging attempt
        inst->testAppState = TA_TXE_WAIT ;
        inst->nextState = TA_TXPOLL_WAIT_SEND ;
      }
      else
      {
        instance_backtoanchor(inst);
      }
    }
    else if(inst->previousState == TA_TXFINAL_WAIT_SEND)  //got here from main (error sending final - handle as timeout)
    {
      if((inst->mode == TAG))
      {
        dwt_forcetrxoff();  //this will clear all events
        inst->instToSleep = TRUE ;
        
        // initiate the re-transmission of the poll that was not responded to
        inst->testAppState = TA_TXE_WAIT ;
        inst->nextState = TA_TXPOLL_WAIT_SEND ;
      }
      else
      {
        instance_backtoanchor(inst);
      }
    }
    else //send the final
    {
      // initiate the re-transmission of the poll that was not responded to
      inst->testAppState = TA_TXE_WAIT ;
      inst->nextState = TA_TXFINAL_WAIT_SEND ;
    }
  }
}

/*******************************************************************************
//the main instance state machine 
//all the instance modes Tag, Anchor use the same statemachine
*******************************************************************************/
int testapprun(instance_data_t *inst, int message)
{
#ifdef TAG_MODE 
  PollMsgStruct p_Msg = {0};
  OsiReturnVal_e mag_result= NULL;
#endif
  
#ifdef ANCHOR_MODE
  UartMsgStruct Msg_data = {0};
#endif
  
  switch (inst->testAppState)
  {
    case TA_INIT :
    {
#ifdef TAG_MODE
      dwt_setpanid(inst->panID);  //set the PAN ID
      
      memcpy(inst->eui64, &inst->instanceAddress16, ADDR_BYTE_SIZE_S);
      
      dwt_seteui(inst->eui64);  //set the EUI 64-bit (long) address

      dwt_setaddress16(inst->instanceAddress16);  //sets the 16 bit short address
      
      dwt_enableframefilter(DWT_FF_COORD_EN | DWT_FF_BEACON_EN |DWT_FF_DATA_EN | DWT_FF_ACK_EN); //allow data, ack frames

      inst->testAppState = TA_TXE_WAIT;
      inst->nextState = TA_TXPOLL_WAIT_SEND;  //Start off by Sleeping 1st
      inst->instToSleep = FALSE ;  //set instToSleep to FALSE

      uint16 sleep_mode = (DWT_LOADUCODE|DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV);

      if(dwt_getldotune() != 0) //if we need to use LDO tune value from OTP kick it after sleep
      {
        sleep_mode |= DWT_LOADLDO;
      }

      if(inst->configData.txPreambLength == DWT_PLEN_64)  //if using 64 length preamble then use the corresponding OPSet
      {
        sleep_mode |= DWT_LOADOPSET;
      }

#if (DEEP_SLEEP == 1)
      dwt_configuresleep(sleep_mode, DWT_WAKE_CS|DWT_SLP_EN);  //configure the on wake parameters (upload the IC config settings)
#endif
      instanceconfigframeheader16(inst);  //construct the message/frame header bytes   
#endif  //#ifdef TAG_MODE
        
#ifdef ANCHOR_MODE
      dwt_setpanid(inst->panID);  //set the PAN ID
      
      memcpy(inst->eui64, &inst->instanceAddress16, ADDR_BYTE_SIZE_S);
      
      dwt_seteui(inst->eui64);  //set the EUI 64-bit (long) address

      dwt_setaddress16(inst->instanceAddress16);  //sets the 16 bit short address

      dwt_enableframefilter(DWT_FF_COORD_EN | DWT_FF_BEACON_EN | DWT_FF_DATA_EN | DWT_FF_ACK_EN);  //allow data, ack frames

      dwt_setrxaftertxdelay(0);  // First time anchor listens we don't do a delayed RX
      
      inst->testAppState = TA_RXE_WAIT ;  //change to next state - wait to receive a message

      dwt_setrxtimeout(0);  //enables RX timeout (SY_STAT_RFTO event)
      
      dwt_setpreambledetecttimeout(0);  //enables preamble timeout (SY_STAT_RXPTO event)
      
      instanceconfigframeheader16(inst);  //construct the message/frame header bytes
#endif  //#ifdef ANCHOR_MODE
      
#ifdef PA_LNA_MODE 
      PA_LNA_ON();
#endif
      
      dwt_setautorxreenable(1);  //enables the auto rx re-enable feature
    }
    inst->done = INST_NOT_DONE_YET;
    
    break; // end case TA_INIT

#ifdef TAG_MODE
    case TA_SLEEP_DONE :
    {
      event_data_t* dw_event = instance_getevent(); //clear the event from the queue
      // waiting for timout from application to wakup IC
      if((dw_event->type != DWT_SIG_RX_TIMEOUT)&&(dw_event->type != DWT_SIG_RX_NOERR))
      {
        // if no pause and no wake-up timeout continu waiting for the sleep to be done.
        inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //wait here for sleep timeout
        
        break;
      }

      inst->done = INST_NOT_DONE_YET;
      inst->instToSleep = FALSE ;
      inst->testAppState = inst->nextState;
      inst->nextState = TA_INIT; //clear
      
#if (DEEP_SLEEP == 1)

      uint8 x = 0;
      uint32 devID ;
      
      //wake up device from low power mode,NOTE - in the ARM  code just drop chip select for 200us
      DW1000_CS_off();  //SPI CS low
      
      instance_data.dwIDLE = 0; //reset DW1000 IDLE flag

      SET_DECA_RST_PIN_IRQ(1); //enable RSTn IRQ

      delay_ms(1);   //200 us to wake up - need 2 as Sleep(1) is ~ 175 us
      
      //then wait 5ms for DW1000 XTAL to stabilise - instead of wait we wait for RSTn to go high
      //Sleep(5);

      //need to poll to check when the DW1000 is in IDLE, the CPLL interrupt is not reliable
      //when RSTn goes high the DW1000 is in INIT, it will enter IDLE after PLL lock (in 5 us)
      while(instance_data.dwIDLE == 0) // this variable will be sent in the IRQ (process_dwRSTn_irq)
      {
        x++;  //wait for DW1000 to go to IDLE state RSTn pin to go high
        
        if(x > 10) break;
        
        delay_ms(1);
      }
      
      SET_DECA_RST_PIN_IRQ(0); //disable RSTn IRQ
      
      DW1000_CS_on();;  //SPI CS high

      for(x=0;x<5;x++)
      {
        //!!! NOTE it takes ~35us for the DW1000 to download AON and lock the PLL and be in IDLE state
        //do some dummy reads of the dev ID register to make sure DW1000 is in IDLE before setting LEDs
        devID = dwt_readdevid(); //dummy read... need to wait for 5 us to exit INIT state (5 SPI bytes @ ~18 MHz)
        
        if(devID == DWT_DEVICE_ID) 
        {
          break;
        }
      }

      //MP bug - TX antenna delay needs reprogramming as it is not preserved (only RX)
      dwt_settxantennadelay(inst->txAntennaDelay) ;

      //set EUI as it will not be preserved unless the EUI is programmed and loaded from NVM
      dwt_seteui(inst->eui64);
#else
      delay_ms(3); //to approximate match the time spent in the #if above
#endif

      instancesetantennadelays(); //this will update the antenna delay if it has changed
      
      instancesettxpower(); //configure TX power if it has changed
    }
    break;
#endif  //#ifdef TAG_MODE
    
    case TA_TXE_WAIT : //either go to sleep or proceed to TX a message
    {
      //if we are scheduled to go to sleep before next transmission then sleep first.
      if((inst->nextState == TA_TXPOLL_WAIT_SEND) && (inst->instToSleep) && (inst->mode == TAG))  //go to sleep before sending the next poll/ starting new ranging exchange 
      {
        //the app should put chip into low power state and wake up after tagSleepTime_ms time...
        //the app could go to *_IDLE state and wait for uP to wake it up...
        inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT_TO; //don't sleep here but kick off the Sleep timer countdown
        
        inst->testAppState = TA_SLEEP_DONE;

#if (DEEP_SLEEP == 1)
        dwt_entersleep(); //go to sleep,put device into low power mode
#endif	
        
#ifdef TAG_MODE 
        
  #ifdef PA_LNA_MODE 
        PA_LNA_OFF();
  #endif 
        
        Green_led_off();
        
        srand(sys_Tick);
        
        wake_up_time = 0;
				
        osi_Sleep(slp_time + rand()%50);		

        Green_led_on();
        
  #ifdef PA_LNA_MODE 
        PA_LNA_ON();
  #endif
        
        wake_up_time = sys_Tick;
#endif	
      }
      else //proceed to configuration and transmission of a frame
      {
        inst->testAppState = inst->nextState;
        inst->nextState = TA_INIT; //clear
      }
    }
    break ; // end case TA_TXE_WAIT

    case TA_TXPOLL_WAIT_SEND :
    {    
      memset(inst->msg_f.messageData,0,MAX_USER_PAYLOAD_STRING_SS);
      
#ifdef TAG_MODE
      if(range_type == TWR_RANGING)
      {
        inst->msg_f.messageData[FCODE] = TOF_TAG_POLL_MSG ;  //message function code
        
        inst->psduLength = (TOF_TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
      }
      if(range_type == TODA_RANGING)
      {
        inst->msg_f.messageData[FCODE] = TDOA_TAG_POLL_MSG ;  //message function code
        
        mag_result = osi_MsgQRead(&Msg_Queue,&p_Msg,OSI_NO_WAIT);  //Wait Tag Message
        if(mag_result == OSI_OK)
        {
          inst->msg_f.messageData[TDOA_TAG_MSG_LEN] = p_Msg.msg_len;  //message function code
          
          memcpy(&inst->msg_f.messageData[TDOA_TAG_MSG_CODE],&p_Msg.msg_buf,inst->msg_f.messageData[TDOA_TAG_MSG_LEN]);
        }
        else
        {
          inst->msg_f.messageData[TDOA_TAG_MSG_LEN] = 0;  //message function code
        }
        
        inst->psduLength = (inst->msg_f.messageData[TDOA_TAG_MSG_LEN] + TDOA_TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
      }
      
      inst->msg_f.seqNum = inst->frameSN++; //copy sequence number and then increment
#endif
        
#ifdef ANCHOR_MODE
      if(inst->mode == ANCHOR_SYNC)
      {
        inst->msg_f.messageData[FCODE] = TDOA_ANCH_MSG_CLE;  //message function code
      
        inst->psduLength = (TDOA_ANCH_MSG_CLE_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
        
        inst->msg_f.seqNum = inst->frameSN++; //copy sequence number and then increment
      } 
      else if(inst->mode == ANCHOR_RNG)
      {
        inst->msg_f.messageData[FCODE] = TOF_TAG_POLL_MSG ;  //message function code
        
        inst->psduLength = (TOF_TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
        
        inst->msg_f.seqNum = inst->frameSN_ar++; //copy sequence number and then increment
      }
#endif

      inst->msg_f.sourceAddr[0] = inst->eui64[0]; //copy the address
      inst->msg_f.sourceAddr[1] = inst->eui64[1]; //copy the address
      
      inst->msg_f.destAddr[0] = 0xff;  //set the destination address (broadcast == 0xffff)
      inst->msg_f.destAddr[1] = 0xff;  //set the destination address (broadcast == 0xffff)
      
#ifdef TAG_MODE
      if(range_type == TWR_RANGING)
      {
        dwt_writetxdata(inst->psduLength, (uint8 *)&inst->msg_f, 0) ;	// write the frame data
        
        //set the delayed rx on time (the response message will be sent after this delay)
        dwt_setrxaftertxdelay((uint32)RX_RESPONSE1_TURNAROUND);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)
        
        inst->responseTO = MAX_ANCHOR_RESP_SIZE;  //expecting 4 responses
        
        dwt_setrxtimeout((uint16)inst->fwtoTime_sy * inst->responseTO);  //configure the RX FWTO
        
        inst->rxResponseMask = 0;  //reset/clear the mask of received responses when tx poll
        
        inst->wait4ack = DWT_RESPONSE_EXPECTED;  //response is expected - automatically enable the receiver

        dwt_writetxfctrl(inst->psduLength, 0);  //write frame control

        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED); //transmit the frame

        inst->testAppState = TA_TX_WAIT_CONF ;  // wait confirmation
        
        inst->previousState = TA_TXPOLL_WAIT_SEND ;
      }
      if(range_type == TODA_RANGING)
      {
        dwt_writetxdata(inst->psduLength, (uint8 *)  &inst->msg_f, 0) ;	// write the frame data
        
        inst->wait4ack = 0; //clear the flag not using wait for response as this message ends the ranging exchange
        
        if(instancesenddlypacket(inst, DWT_START_TX_IMMEDIATE))
        {
          inst->testAppState = TA_TXE_WAIT ; //go to TA_TXE_WAIT first to check if it's sleep time
          inst->nextState = TA_TXPOLL_WAIT_SEND ;
        }
        else
        {
          inst->testAppState = TA_TX_WAIT_CONF;                                               // wait confirmation
          inst->previousState = TA_TXFINAL_WAIT_SEND;
        }
        
        inst->instToSleep = TRUE ;
      }
#endif
        
#ifdef ANCHOR_MODE
      if(inst->mode == ANCHOR_SYNC)
      {
        dwt_writetxdata(inst->psduLength, (uint8 *)  &inst->msg_f, 0) ;	// write the frame data
        
        inst->wait4ack = 0; //clear the flag not using wait for response as this message ends the ranging exchange
    
        if(instancesenddlypacket(inst, DWT_START_TX_IMMEDIATE))
        {
          instance_backtoanchor(inst);
        }
        else
        {
          inst->testAppState = TA_TX_WAIT_CONF;  // wait confirmation
        
          inst->previousState = TA_TXFINAL_WAIT_SEND;
        }
      }
      else if(inst->mode == ANCHOR_RNG)
      {
        dwt_writetxdata(inst->psduLength, (uint8 *)&inst->msg_f, 0) ;	// write the frame data
        
        //set the delayed rx on time (the response message will be sent after this delay)
        dwt_setrxaftertxdelay((uint32)RX_RESPONSE1_TURNAROUND);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)
        
        inst->responseTO = MAX_ANCHOR_RESP_SIZE;  //expecting 4 responses
        
        dwt_setrxtimeout((uint16)inst->fwtoTime_sy * inst->responseTO);  //configure the RX FWTO
        
        inst->rxResponseMask = 0;  //reset/clear the mask of received responses when tx poll
        
        inst->wait4ack = DWT_RESPONSE_EXPECTED;  //response is expected - automatically enable the receiver

        dwt_writetxfctrl(inst->psduLength, 0);  //write frame control

        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED); //transmit the frame

        inst->testAppState = TA_TX_WAIT_CONF ;  // wait confirmation
        
        inst->previousState = TA_TXPOLL_WAIT_SEND ;
      }
#endif

      inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set above)
    }
    break;

    case TA_TXFINAL_WAIT_SEND :
    {
      if(inst->mode == TAG)
      {
        //message function code (specifies if message is a poll, response or other...)
        inst->msg_f.messageData[FCODE] = TOF_TAG_FINAL_MSG;
      
        inst->psduLength = (inst->msg_f.messageData[TOF_TAG_MSG_LEN] + TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
        
        inst->msg_f.seqNum = inst->frameSN; //copy sequence number and then increment
      }
      else if(inst->mode == ANCHOR_RNG)
      {
        //message function code (specifies if message is a poll, response or other...)
        inst->msg_f.messageData[FCODE] = ANCHOR_RANG_FINAL_MSG;
        
        inst->psduLength = (TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
        
        inst->msg_f.seqNum = inst->frameSN_ar; //copy sequence number and then increment
      }
      
      dwt_writetxdata(inst->psduLength, (uint8 *)  &inst->msg_f, 0) ;	// write the frame data

      inst->wait4ack = 0; //clear the flag not using wait for response as this message ends the ranging exchange

      if(instancesenddlypacket(inst, DWT_START_TX_DELAYED))
      {
        // initiate the re-transmission
        if(inst->mode == TAG)
        {
          inst->testAppState = TA_TXE_WAIT ; //go to TA_TXE_WAIT first to check if it's sleep time
          inst->nextState = TA_TXPOLL_WAIT_SEND ;
        }
        else
        {
          //go back to RX and behave as anchor
          instance_backtoanchor(inst);
        }

        break; //exit this switch case...
      }
      else
      {
        inst->testAppState = TA_TX_WAIT_CONF;  // wait confirmation
        inst->previousState = TA_TXFINAL_WAIT_SEND;
      }
      
      if(inst->mode == TAG)
      {
        inst->instToSleep = TRUE ;
      }
      
      inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set above)
    }
    break;

    case TA_TX_WAIT_CONF :
    {
      event_data_t* dw_event = instance_getevent(); //get and clear this event

      //NOTE: Can get the ACK before the TX confirm event for the frame requesting the ACK
      //this happens because if polling the ISR the RX event will be processed 1st and then the TX event
      //thus the reception of the ACK will be processed before the TX confirmation of the frame that requested it.
      if(dw_event->type != DWT_SIG_TX_DONE) //wait for TX done confirmation
      {
        if(dw_event->type != DWT_SIG_RX_NOERR)
        {
          if(dw_event->type == DWT_SIG_RX_TIMEOUT) //got RX timeout - i.e. did not get the response (e.g. ACK)
          {
            inst->gotTO = 1;  //we need to wait for SIG_TX_DONE and then process the timeout and re-send the frame if needed
          }
        }
        else
        {
          //instance_backtoanchor(inst);
        }
        inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT;
        
        break;
      }
      else
      {
#ifdef ANCHOR_MODE
        if(dw_event->msgu.rxmsg_ss.messageData[FCODE] == TDOA_ANCH_MSG_CLE)
        {
          Msg_data.msg_type = HOST_ANCHOR_SYNC;
          
          Msg_data.sec_num = dw_event->msgu.rxmsg_ss.seqNum;
          
          Msg_data.s_addr = inst->instanceAddress16;
            
          memcpy(&Msg_data.rx_time, (uint8 *)&dw_event->timeStamp, 5);  // Write Response RX time field of Final message
          
          //xQueueSend( message_print, ( void* )&Msg_data, 0 );
					rt_mq_send( &message_print, ( void* )&Msg_data, sizeof(Msg_data) );
          //osi_MsgQWrite(&UartMsg_Queue,&Msg_data,OSI_NO_WAIT);  //send message data
        }
#endif
      }

      inst->done = INST_NOT_DONE_YET;

      if(inst->previousState == TA_TXFINAL_WAIT_SEND)
      {
        if(inst->mode == TAG)
        {
          inst->testAppState = TA_TXE_WAIT ;
          inst->nextState = TA_TXPOLL_WAIT_SEND ;
          break;
        }
        else
        {
          instance_backtoanchor(inst);
        }
      }
      else if (inst->gotTO == 1) //timeout
      {
        inst_processrxtimeout(inst);
        inst->gotTO = 0;
        inst->wait4ack = 0 ; //clear this
        break;
      }
      else
      {
        inst->txu.txTimeStamp = dw_event->timeStamp;

        if(inst->previousState == TA_TXPOLL_WAIT_SEND)
        {
          uint64 tagCalculatedFinalTxTime ;
          
          // Embed into Final message: 40-bit pollTXTime,  40-bit respRxTime,  40-bit finalTxTime
          tagCalculatedFinalTxTime =  (inst->txu.txTimeStamp + inst->pollTx2FinalTxDelay) & MASK_TXDTS;

          inst->delayedReplyTime = tagCalculatedFinalTxTime >> 8; //high 32-bits
          
          // Calculate Time Final message will be sent and write this field of Final message
          // Sending time will be delayedReplyTime, snapped to ~125MHz or ~250MHz boundary by
          // zeroing its low 9 bits, and then having the TX antenna delay added
          // getting antenna delay from the device and add it to the Calculated TX Time
          tagCalculatedFinalTxTime = tagCalculatedFinalTxTime + inst->txAntennaDelay;
          tagCalculatedFinalTxTime &= MASK_40BIT;

          // Write Poll TX time field of Final message
          memcpy(&(inst->msg_f.messageData[PTXT]), (uint8 *)&inst->txu.tagPollTxTime, 5);
          
          // Write Calculated TX time field of Final message
          memcpy(&(inst->msg_f.messageData[FTXT]), (uint8 *)&tagCalculatedFinalTxTime, 5);
          
#ifdef TAG_MODE 
          mag_result = osi_MsgQRead(&Msg_Queue,&p_Msg,OSI_NO_WAIT);  //Wait Tag Message
          if(mag_result == OSI_OK)
          {
            inst->msg_f.messageData[TOF_TAG_MSG_LEN] = p_Msg.msg_len;  //message function code
            
            memcpy(&inst->msg_f.messageData[TOF_TAG_MSG_CODE],&p_Msg.msg_buf,inst->msg_f.messageData[TOF_TAG_MSG_LEN]);
          }
          else
          {
            inst->msg_f.messageData[TOF_TAG_MSG_LEN] = 0;  //message function code
          }
#endif	
        }

        inst->testAppState = TA_RXE_WAIT ;  // After sending, tag expects response/report, anchor waits to receive a final/new poll

        message = 0;  //fall into the next case (turn on the RX)
      }
    }  // end case TA_TX_WAIT_CONF
    //break ; 

    case TA_RXE_WAIT :
    {
      if(inst->wait4ack == 0) //if this is set the RX will turn on automatically after TX
      {
        dwt_rxenable(DWT_START_RX_IMMEDIATE) ;  // turn RX on, without delay
      }
      else
      {
        inst->wait4ack = 0; //clear the flag, the next time we want to turn the RX on it might not be auto
      }
      
      inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //using RX FWTO

      inst->testAppState = TA_RX_WAIT_DATA;   // let this state handle it

      if(message == 0) 
      {
        break;
      } 
      // end case TA_RXE_WAIT, don't break, but fall through into the TA_RX_WAIT_DATA state to process it immediately.
    }

    case TA_RX_WAIT_DATA :
    {
//#ifdef ANCHOR_MODE
//      osi_SyncObjWait(&dw_rx_Semaphore,5);  //Wait Task Operation Message
//#endif   
      switch (message)
      {
        //if we have received a DWT_SIG_RX_OKAY event - this means that the message is IEEE data type - need to check frame control to know which addressing mode is used
        case DWT_SIG_RX_OKAY :
        {
          event_data_t* dw_event = instance_getevent(); //get and clear this event
          int fn_code = 0;
          uint8  srcAddr[2] = {0,0};
          uint8  dstAddr[2] = {0,0};

          inst->stopTimer = 0; //clear the flag, as we have received a message
          
          fn_code = dw_event->msgu.rxmsg_ss.messageData[FCODE];
          memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_ss.sourceAddr[0]), ADDR_BYTE_SIZE_S);
          memcpy(&dstAddr[0], &(dw_event->msgu.rxmsg_ss.destAddr[0]), ADDR_BYTE_SIZE_S);

#ifdef ANCHOR_MODE
          uint8 *messageData;
          
          messageData = &dw_event->msgu.rxmsg_ss.messageData[0];
#endif     
          switch(fn_code)
          {  
#ifdef ANCHOR_MODE
            case TDOA_TAG_POLL_MSG:
            {
              inst->newRangeTagAddress = srcAddr[0] + ((uint16) srcAddr[1] << 8);
 
              Msg_data.msg_type = TAG_BORADCAST;
          
              Msg_data.sec_num = dw_event->msgu.rxmsg_ss.seqNum;
              
              Msg_data.s_addr = inst->newRangeTagAddress;
              
              Msg_data.d_addr = inst->instanceAddress16;

              memcpy(&Msg_data.rx_time, (uint8 *)&dw_event->timeStamp, 5);  // Write Response RX time field of Final message
              
              Msg_data.rssi_val = (int)Get_Receive_Power();
              
              if((messageData[TDOA_TAG_MSG_LEN]>0)&&(messageData[TDOA_TAG_MSG_LEN]<sizeof(Msg_data.msg_buf)))
              {
                if((messageData[TDOA_TAG_MSG_CODE] == '{')&&(messageData[TDOA_TAG_MSG_CODE + messageData[TDOA_TAG_MSG_LEN] - 1] == '}'))
                {
                  memcpy(Msg_data.msg_buf,&messageData[TDOA_TAG_MSG_CODE],messageData[TDOA_TAG_MSG_LEN]);
                }
              }
              
              //osi_MsgQWrite(&UartMsg_Queue,&Msg_data,OSI_NO_WAIT);  //send message data
              rt_mq_send ( &message_print, ( void* )&Msg_data, sizeof(Msg_data) );

              instancesetantennadelays(); //this will update the antenna delay if it has changed
              
              instancesettxpower(); // configure TX power if it has changed
              
              inst->testAppState = TA_RXE_WAIT ;  // wait for next frame
            }
            break;
            
            case TDOA_ANCH_MSG_CLE:
            {
              inst->newRangeTagAddress = srcAddr[0] + ((uint16) srcAddr[1] << 8);

              Msg_data.msg_type = SLAVE_ANCHOR_SYNC;
          
              Msg_data.sec_num = dw_event->msgu.rxmsg_ss.seqNum;
              
              Msg_data.s_addr = inst->newRangeTagAddress;
              
              Msg_data.d_addr = inst->instanceAddress16; 
              
              memcpy(&Msg_data.rx_time, (uint8 *)&dw_event->timeStamp, 5);  // Write Response RX time field of Final message
              
              Msg_data.rssi_val = (int)Get_Receive_Power();
              
              //osi_MsgQWrite(&UartMsg_Queue,&Msg_data,OSI_NO_WAIT);  //send message data
              //xQueueSend( message_print, ( void* )&Msg_data, 0 );
              rt_mq_send( &message_print, ( void* )&Msg_data, sizeof(Msg_data) );
              instancesetantennadelays(); //this will update the antenna delay if it has changed
              
              instancesettxpower(); // configure TX power if it has changed
              
              inst->testAppState = TA_RXE_WAIT ;  // wait for next frame
            }
            break;
            
            case TOF_TAG_POLL_MSG:
            {
              inst->tagPollRxTime = dw_event->timeStamp ; //save Poll's Rx time

              inst->newRangeTagAddress = srcAddr[0] + ((uint16) srcAddr[1] << 8);
   
              Msg_data.msg_type = TAG_BORADCAST;
          
              Msg_data.sec_num = dw_event->msgu.rxmsg_ss.seqNum;
              
              Msg_data.s_addr = inst->newRangeTagAddress;
              
              Msg_data.d_addr = inst->instanceAddress16;

              memcpy(&Msg_data.rx_time, (uint8 *)&dw_event->timeStamp, 5);  // Write Response RX time field of Final message
              
              Msg_data.rssi_val = (int)Get_Receive_Power();
              
              if((messageData[TDOA_TAG_MSG_LEN]>0)&&(messageData[TDOA_TAG_MSG_LEN]<sizeof(Msg_data.msg_buf)))
              {
                if((messageData[TDOA_TAG_MSG_CODE] == '{')&&(messageData[TDOA_TAG_MSG_CODE + messageData[TDOA_TAG_MSG_LEN] - 1] == '}'))
                {
                  memcpy(Msg_data.msg_buf,&messageData[TDOA_TAG_MSG_CODE],messageData[TDOA_TAG_MSG_LEN]);
                }
              }
              
              //osi_MsgQWrite(&UartMsg_Queue,&Msg_data,OSI_NO_WAIT);  //send message data
              //xQueueSend( message_print, ( void* )&Msg_data, 0 );
               rt_mq_send( &message_print, ( void* )&Msg_data, sizeof(Msg_data) );
              //the response has been sent - await TX done event
              if(dw_event->type_pend == DWT_SIG_TX_PENDING)
              {
                inst->testAppState = TA_TX_WAIT_CONF;  // wait confirmation
                  
                inst->previousState = TA_TXRESPONSE_SENT_POLLRX ;  //wait for TX confirmation of sent response  
              }
              //already re-enabled the receiver
              else if (dw_event->type_pend == DWT_SIG_RX_PENDING)
              {
                //stay in RX wait for next frame...
                //RX is already enabled...
                inst->testAppState = TA_RX_WAIT_DATA ;  // wait for next frame
              }
              else //the DW1000 is idle (re-enable from the application level)
              {
                //stay in RX wait for next frame...
                inst->testAppState = TA_RXE_WAIT ;  // wait for next frame
              }
            }
            break; //RTLS_DEMO_MSG_TAG_POLL/RTLS_DEMO_MSG_ANCH_POLL
#endif 
            
            case RTLS_DEMO_MSG_ANCH_RESP:
            {
              //the response has been sent - await TX done event
              if(dw_event->type_pend == DWT_SIG_TX_PENDING) //anchor received response from anchor ID - 1 so is sending it's response now back to tag
              {
                inst->testAppState = TA_TX_WAIT_CONF;  // wait confirmation
                inst->previousState = TA_TXRESPONSE_SENT_RESPRX ;  //wait for TX confirmation of sent response
              }
              //already re-enabled the receiver
              else if(dw_event->type_pend == DWT_SIG_RX_PENDING)
              {
                // stay in TA_RX_WAIT_DATA - receiver is already enabled.
              }
              //DW1000 idle - send the final
              else //if(dw_event->type_pend == DWT_SIG_DW_IDLE)
              {
                if(((inst->mode == TAG)||(inst->mode == ANCHOR_RNG)) && (inst->rxResponseMask > 0))
                {
                  inst->testAppState = TA_TXFINAL_WAIT_SEND ; // send our response / the final
                }
                else //go to sleep
                {
                  if(TAG == inst->mode)
                  {
                    inst->testAppState = TA_TXE_WAIT ; //go to TA_TXE_WAIT first to check if it's sleep time
                    inst->nextState = TA_TXPOLL_WAIT_SEND ;
                    inst->instToSleep = TRUE;
                  }
                  else
                  {
                    instance_backtoanchor(inst);
                  }
                }
              }
            }
            break; //RTLS_DEMO_MSG_ANCH_RESP
            
#ifdef ANCHOR_MODE
             case TOF_TAG_FINAL_MSG:
             case ANCHOR_RANG_FINAL_MSG:
            {
              int64 Rb, Da, Ra, Db ;
              uint64 tagFinalTxTime  = 0;
              uint64 tagFinalRxTime  = 0;
              uint64 tagPollTxTime  = 0;
              uint64 anchorRespRxTime  = 0;
              uint64 tof = INVALID_TOF;

              double RaRbxDaDb = 0;
              double RbyDb = 0;
              double RayDa = 0;

              inst->newRangeTagAddress = srcAddr[0] + ((uint16) srcAddr[1] << 8);
              
              inst->newRangeAncAddress = inst->instanceAddress16;  //output data over USB...
              
              uint8_t r_data_n;
              uint8_t r_data_addr = RRXT0;
              uint16_t r_anchor_addr;
              
              for(r_data_n = 0; r_data_n < MAX_ANCHOR_RESP_SIZE; r_data_n++)
              {
                memcpy(&r_anchor_addr, &(messageData[r_data_addr]), 2);
                
                if(r_anchor_addr == inst->instanceAddress16)
                {
                  r_data_addr += 2;
                  
                  break;
                }
                else
                {
                  r_data_addr += 7;
                } 
              }
              
              if((r_anchor_addr == inst->instanceAddress16)&&(tag_poll_addr == inst->newRangeTagAddress))
              {
                // time of arrival of Final message
                tagFinalRxTime = dw_event->timeStamp ; //Final's Rx time

                inst->delayedReplyTime = 0 ;

                // times measured at Tag extracted from the message buffer
                // extract 40bit times
                memcpy(&tagPollTxTime, &(messageData[PTXT]), 5);
                memcpy(&anchorRespRxTime, &(messageData[r_data_addr]), 5);
                memcpy(&tagFinalTxTime, &(messageData[FTXT]), 5);

                // poll response round trip delay time is calculated as
                if(anchorRespRxTime >= tagPollTxTime)
                {
                  Ra = (int64)((anchorRespRxTime - tagPollTxTime) & MASK_40BIT);
                }
                else
                {
                  Ra = (int64)((anchorRespRxTime + (MASK_40BIT - tagPollTxTime)) & MASK_40BIT);
                }
                if(inst->txu.anchorRespTxTime >= inst->tagPollRxTime)
                {
                  Db = (int64)((inst->txu.anchorRespTxTime - inst->tagPollRxTime) & MASK_40BIT);
                }
                else
                {
                  Db = (int64)((inst->txu.anchorRespTxTime + (MASK_40BIT - inst->tagPollRxTime)) & MASK_40BIT);
                }

                // response final round trip delay time is calculated as
                if(tagFinalRxTime >= inst->txu.anchorRespTxTime)
                {
                  Rb = (int64)((tagFinalRxTime - inst->txu.anchorRespTxTime) & MASK_40BIT);
                }
                else
                {
                  Rb = (int64)((tagFinalRxTime + (MASK_40BIT - inst->txu.anchorRespTxTime)) & MASK_40BIT);
                }
                if(tagFinalTxTime >= anchorRespRxTime)
                {
                  Da = (int64)((tagFinalTxTime - anchorRespRxTime) & MASK_40BIT);
                }
                else
                {
                  Da = (int64)((tagFinalTxTime + (MASK_40BIT - anchorRespRxTime)) & MASK_40BIT);
                }

                RaRbxDaDb = (((double)Ra))*(((double)Rb)) - (((double)Da))*(((double)Db));

                RbyDb = ((double)Rb + (double)Db);

                RayDa = ((double)Ra + (double)Da);

                tof = (int32) ( RaRbxDaDb/(RbyDb + RayDa) );

                Msg_data.sec_num = dw_event->msgu.rxmsg_ss.seqNum;
                
                Msg_data.s_addr = inst->newRangeTagAddress;
                
                Msg_data.d_addr = inst->instanceAddress16;
                
#ifdef TIME_STAMP_DEBUG
                Msg_data.msg_type = ANCHOR_TIMESTAMP_DEBUG;
                memcpy(Msg_data.poll_rx_time,&inst->tagPollRxTime,8);
                memcpy(Msg_data.anchor_tx_time,&inst->txu.anchorRespTxTime,8);
                memcpy(Msg_data.tag_poll_time,&tagPollTxTime,8);
                memcpy(Msg_data.tag_rx_time,&anchorRespRxTime,8);
                memcpy(Msg_data.tag_final_time,&tagFinalTxTime,8);
                osi_MsgQWrite(&UartMsg_Queue,&Msg_data,OSI_NO_WAIT);  //send message data 
#endif            
                if(fn_code == TOF_TAG_FINAL_MSG)
                {
                  Msg_data.msg_type = TOF_RANGING;
                }
                else if(fn_code == ANCHOR_RANG_FINAL_MSG)
                {
                  Msg_data.msg_type = ANCHOR_RANGING;
                }
                
                Msg_data.distance_val = (uint32)range_distance((uint32)tof);
                
                Msg_data.rssi_val = (int)Get_Receive_Power();
                 
                if((messageData[TOF_TAG_MSG_LEN]>0)&&(messageData[TOF_TAG_MSG_LEN]<sizeof(Msg_data.msg_buf)))
                {
                  if((messageData[TOF_TAG_MSG_CODE] == '{')&&(messageData[TOF_TAG_MSG_CODE + messageData[TOF_TAG_MSG_LEN] - 1] == '}'))
                  {
                    memcpy(Msg_data.msg_buf,&messageData[TOF_TAG_MSG_CODE],messageData[TOF_TAG_MSG_LEN]);
                  }
                }
                  
                //osi_MsgQWrite(&UartMsg_Queue,&Msg_data,OSI_NO_WAIT);  //send message data 
                rt_mq_send( &message_print, ( void* )&Msg_data, sizeof(Msg_data) );

              }

              instancesetantennadelays(); //this will update the antenna delay if it has changed
              instancesettxpower(); // configure TX power if it has changed

              inst->testAppState = TA_RXE_WAIT ;  // wait for next frame
            }
            break; //RTLS_DEMO_MSG_TAG_FINAL
#endif
            default:
            {
              //only enable receiver when not using double buffering
              inst->testAppState = TA_RXE_WAIT ;  // wait for next frame
              dwt_setrxaftertxdelay(0);
            }
            break;
          } //end switch (fcode)

          if(dw_event->msgu.frame[0] & 0x20)
          {
            //as we only pass the received frame with the ACK request bit set after the ACK has been sent
            instance_getevent(); //get and clear the ACK sent event
          }
        }
        break ; //end of DWT_SIG_RX_OKAY

        case DWT_SIG_RX_TIMEOUT :
        {
          event_data_t* dw_event = instance_getevent(); //get and clear this event

          //Anchor can time out and then need to send response - so will be in TX pending
          if(dw_event->type_pend == DWT_SIG_TX_PENDING)
          {
            inst->testAppState = TA_TX_WAIT_CONF;  // wait confirmation
            inst->previousState = TA_TXRESPONSE_SENT_TORX ;    //wait for TX confirmation of sent response
          }
          else if(dw_event->type_pend == DWT_SIG_DW_IDLE) //if timed out and back in receive then don't process as timeout
          {
            inst_processrxtimeout(inst);
          }
          //else if RX_PENDING then wait for next RX event...
          message = 0; //clear the message as we have processed the event
        }
        break ;

        case DWT_SIG_TX_AA_DONE: //ignore this event - just process the rx frame that was received before the ACK response
        default :
        {
          if(message) // == DWT_SIG_TX_DONE)
          {
            inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT;
          }

          if(inst->done == INST_NOT_DONE_YET) 
          {
            inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT;
          }
        }
        break;
      }
    }
    break ; // end case TA_RX_WAIT_DATA
    default:
    {
      break;
    }
  } // end switch on testAppState
  
  return inst->done;
} 

/*******************************************************************************
// Range Run
*******************************************************************************/
void instance_run(void)
{
  int done = INST_NOT_DONE_YET;
  int message = instance_data.dwevent[instance_data.dweventPeek].type; //get any of the received events from ISR

  while(done == INST_NOT_DONE_YET)
  {
    done = testapprun(&instance_data, message) ;  // run the communications application

    message = 0;  //we've processed message
  }

  if(done == INST_DONE_WAIT_FOR_NEXT_EVENT_TO)  //we are in RX and need to timeout (Tag needs to send another poll if no Rx frame)
  {
    if(instance_data.mode == TAG)  //Tag (is either in RX or sleeping)
    {
      event_data_t dw_event;
  
      dw_event.rxLength = 0;
      dw_event.type = 0;
      dw_event.type_save = 0x80 | DWT_SIG_RX_TIMEOUT;
      instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT); 
    }
    instance_data.stopTimer = 0 ; //clear the flag - timer can run if instancetimer_en set (set above)
    instance_data.done = INST_NOT_DONE_YET;
  }

#ifdef ANCHOR_MODE
  if(instance_data.mode == ANCHOR)
  {
    if((anchor_mode == HOST_ANCHOR)&&((sys_Tick - sync_tick) >= sync_period)&&(sync_period))
    {
      sync_tick = sys_Tick;
      
      instance_data.mode = ANCHOR_SYNC;
      
      DW_IRQn_disenable();  //disable the external interrupt line
      
      dwt_forcetrxoff();  //disable DW1000
      
      instance_clearevents(); //clear any events
      
      instance_data.testAppState = TA_TXPOLL_WAIT_SEND ;  //change state to send a Poll
      
      DW_IRQn_enable();  //enable ScenSor IRQ before starting
    }
    else if(((sys_Tick - range_tick) >= range_period)&&(range_period))
    { 
      range_tick = sys_Tick;
      
      instance_data.mode = ANCHOR_RNG;
      
      DW_IRQn_disenable();  //disable the external interrupt line
      
      dwt_forcetrxoff();  //disable DW1000
      
      instance_clearevents(); //clear any events
      
      instance_data.testAppState = TA_TXPOLL_WAIT_SEND ;  //change state to send a Poll
      
      DW_IRQn_enable();  //enable ScenSor IRQ before starting
    }
  }
  else
  {
    if((instance_data.mode == ANCHOR_SYNC)&&((sys_Tick-sync_tick)>100))
    {
      instance_backtoanchor(&instance_data);
    }
    else if(((instance_data.mode == ANCHOR_RNG)&&(sys_Tick-range_tick)>100))
    {
      instance_backtoanchor(&instance_data);
    }
  }
#endif
}


/*******************************************************************************
                                      END         
*******************************************************************************/
