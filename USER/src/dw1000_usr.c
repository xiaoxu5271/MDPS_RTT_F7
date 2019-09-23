#include "dw1000_usr.h"
#include "drv_usr.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "rtthread.h"

#include "Uart_task.h"

#include "dw1000_usr.h"
#include "drv_usr.h"

#include "deca_device_api.h"
#include "dw1000_instance.h"
#include "dw1000_driver.h"
#include "cJSON.h"

#define printf UART_PRINT

#define SPI_BUF_SIZE 512
//#define WDT_TIMEOUT             5000  //ms
/*
{"p":9,"Host_Anchor":0,"PanId":"0411","Address":"a666",
"Channel":2,"Speed":1,"s_pwr":0,"pwr_val":"1f1f1f1f","RespSlot":1
,"SyncPeriod":500,"range_period":1000,"prq_delay":"515.9067",
"cm_led":1,"firmware":"a_v1.2.7"}
*/
#ifdef TAG_MODE
unsigned long slp_time;
uint8_t range_type = TODA_RANGING;
uint16_t static_fn;
#endif
//----------------------------------
//需要定义的地方,test
uint16_t dev_panid = 0x0411;

//uint16_t dev_address = 0xa67A;

uint16_t dev_address = 0xa666;
uint8_t s_pwr = 0;
unsigned long pwr_val = 0x1f1f1f1f;
//uint8_t s_pwr = 1;
//unsigned long pwr_val = 0x15355575;
//
uint8_t anchor_mode = 1;
uint16_t sync_period = 500;
uint16_t range_period = 1000 * 5;
//uint8_t resp_slot = 3;
uint8_t resp_slot = 4;
//uint8_t dev_speed = 1;
//uint8_t dev_channel = 5;
uint8_t dev_speed = 1;
uint8_t dev_channel = 2;
unsigned long dw1000_runtick = 0;

#ifdef ANCHOR_MODE
//extern unsigned long nrf_irq_tick;
unsigned long dw_irq_tick;
#endif
volatile unsigned long sys_Tick = 0;
unsigned long dw1000_irq_tick = 0;
//--------------------------------
extern instance_data_t instance_data;

uint16_t no_resp_tag = 7;
uint16_t no_resp_addr[NO_RESP_TAG_NUM] = {0};

// double dwt_prq_dealy_16m = 515.9067;

double dwt_prq_dealy_16m = 16498.0;
//double dwt_prq_dealy_16m = 0;

//extern OsiSyncObj_t dw1000_Semaphore;
//extern instance_data_t instance_data;

//extern volatile unsigned long sys_Tick;
//extern unsigned long dw1000_irq_tick;

#ifdef ANCHOR_MODE
extern unsigned long dw_irq_tick;
#endif

static rt_sem_t dw1000_irq_xSemaphore = NULL; //用于中断同步
static rt_mutex_t dw1000_spi_mutex = NULL;
//static rt_sem_t dw1000_irq_lock = NULL;

//static struct rt_messagequeue MsgQueue_instance_data;
//static struct rt_messagequeue MsgQueue_instance_data_tx;

struct rt_messagequeue message_print;
struct rt_messagequeue print_data;
struct rt_messagequeue message_send; //udp_send
dw1000_debug_message_tag dw1000_debug_message;

void process_deca_irq(void *pvParameters);

rt_base_t level_dw;

/*******************************************************************************
//delay ms
*******************************************************************************/
void delay_ms(uint32_t n_ms)
{
    //vTaskDelay(n_ms);
    rt_thread_mdelay(n_ms);
}
/*******************************************************************************
// Configures SPI Rate
// Return: None
*******************************************************************************/
void DECA_SPI_Config_Rate(int scalingfactor)
{
    //rt_kprintf("set spi before\r\n");
    //xSemaphoreTake( dw1000_spi_xSemaphore, portMAX_DELAY );
    rt_mutex_take(dw1000_spi_mutex, RT_WAITING_FOREVER);
    spi_speed_set(scalingfactor);
    //rt_kprintf("set spi\r\n");
    rt_mutex_release(dw1000_spi_mutex);
    //xSemaphoreGive( dw1000_spi_xSemaphore );
}
/*******************************************************************************
 reset dw1000 chip by driving the RSTn line low
 return: None
*******************************************************************************/
void reset_DW1000(void)
{
    DW1000_DW_RST_off;
    //delay_ms(3);  //delay about 3ms
    rt_thread_mdelay(3);
    DW1000_DW_RST_on;
}
void dw1000_irq_isr_handler(void *p)
{
    /* 关闭中断 */
    level_dw = rt_hw_interrupt_disable();

    if (DW1000_IRQ_DATA)
    {
        rt_sem_release(dw1000_irq_xSemaphore);
    }

    /* 恢复中断 */
    rt_hw_interrupt_enable(level_dw);
}
//--------------------------------------
/*******************************************************************************
// Function: writetospi() - "deca_device_api.h"
// Takes two separate byte buffers for write header and write data
// returns: 0 for success or -1 for error
*******************************************************************************/
int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodyLength, const uint8_t *bodyBuffer)
{
    uint8_t spi_tx_rx_size;
    //  decaIrqStatus_t  stat ;

    //stat = decamutexon() ;

    //osi_SyncObjWait(&spi_mutex,OSI_WAIT_FOREVER);  //SPI Mutex Take
    //xSemaphoreTake( dw1000_spi_xSemaphore, portMAX_DELAY );
    rt_mutex_take(dw1000_spi_mutex, RT_WAITING_FOREVER);
    DW1000_CS_off();

    while (headerLength)
    {
        spi_tx_rx_size = (uint8_t)(headerLength >= SPI_BUF_SIZE ? SPI_BUF_SIZE : headerLength);

        //spi2_WriteRead_byte((uint8_t const *)headerBuffer,spi_tx_rx_size,NULL,0);
        //spi_readwrite_bytes(spi_dw1000,(uint8_t *)headerBuffer,spi_tx_rx_size,NULL,0);
        //spi_write_bytes(spi_dw1000,(uint8_t *)headerBuffer,spi_tx_rx_size);
        spi_write((uint8_t *)headerBuffer, spi_tx_rx_size);
        headerLength -= spi_tx_rx_size;

        if (headerLength)
        {
            headerBuffer += spi_tx_rx_size;
        }
    }

    while (bodyLength)
    {
        spi_tx_rx_size = (uint8_t)(bodyLength >= SPI_BUF_SIZE ? SPI_BUF_SIZE : bodyLength);

        //spi2_WriteRead_byte((uint8_t const *)bodyBuffer,spi_tx_rx_size,NULL,0);
        //spi_write_buf(spi_dw1000,(uint8_t *)bodyBuffer,spi_tx_rx_size);
        //spi_readwrite_bytes(spi_dw1000,(uint8_t *)bodyBuffer,spi_tx_rx_size,NULL,0);
        //spi_write_bytes(spi_dw1000,(uint8_t *)bodyBuffer,spi_tx_rx_size);
        spi_write((uint8_t *)bodyBuffer, spi_tx_rx_size);
        bodyLength -= spi_tx_rx_size;
        if (bodyLength)
        {
            bodyBuffer += spi_tx_rx_size;
        }
    }

    DW1000_CS_on();

    //osi_SyncObjSignal(&spi_mutex);  //SPI Mutex Give
    //xSemaphoreGive( dw1000_spi_xSemaphore );
    rt_mutex_release(dw1000_spi_mutex);
    //decamutexoff(stat) ;

    return 0;
}

/*******************************************************************************
 Function: readfromspi() - "deca_device_api.h"
 Takes two separate byte buffers for write header and read data
*******************************************************************************/
int readfromspi(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t readlength, uint8_t *readBuffer)
{
    uint8_t spi_tx_rx_size;
    uint16_t rx_len = 0;
    //  decaIrqStatus_t  stat ;

    //stat = decamutexon();

    //osi_SyncObjWait(&spi_mutex,OSI_WAIT_FOREVER);  //SPI Mutex Take
    /* 互斥信号量句柄 */
    //xSemaphoreTake( dw1000_spi_xSemaphore, portMAX_DELAY );
    rt_mutex_take(dw1000_spi_mutex, RT_WAITING_FOREVER);
    DW1000_CS_off();

    while (headerLength)
    {
        spi_tx_rx_size = (uint8_t)(headerLength >= SPI_BUF_SIZE ? SPI_BUF_SIZE : headerLength);

        //spi2_WriteRead_byte((uint8_t const *)headerBuffer,spi_tx_rx_size,NULL,0);
        //spi_readwrite_bytes(spi_dw1000,(uint8_t *)headerBuffer,spi_tx_rx_size,NULL,0);
        //spi_write_bytes(spi_dw1000,(uint8_t *)headerBuffer,spi_tx_rx_size);
        //spi_read_buf(spi_dw1000, (uint8_t *)headerBuffer,spi_tx_rx_size);
        spi_write((uint8_t *)headerBuffer, spi_tx_rx_size);
        //printf("%x\r\n",*headerBuffer);
        headerLength -= spi_tx_rx_size;
        if (headerLength)
        {
            headerBuffer += spi_tx_rx_size;
        }
    }

    while (readlength)
    {
        spi_tx_rx_size = (uint8_t)(readlength >= SPI_BUF_SIZE ? SPI_BUF_SIZE : readlength);

        //spi2_WriteRead_byte(NULL,0,(uint8_t *)(readBuffer+rx_len),spi_tx_rx_size);
        //spi_readwrite_bytes(spi_dw1000,NULL,0,(uint8_t *)(readBuffer+rx_len),spi_tx_rx_size);
        //spi_read_bytes(spi_dw1000,(uint8_t *)(readBuffer+rx_len),spi_tx_rx_size);
        //spi_read_buf(spi_dw1000, (uint8_t *)readBuffer,spi_tx_rx_size);
        spi_read((uint8_t *)(readBuffer + rx_len), spi_tx_rx_size);
        readlength -= spi_tx_rx_size;
        if (readlength)
        {
            rx_len += spi_tx_rx_size;
        }
    }

    DW1000_CS_on();

    //osi_SyncObjSignal(&spi_mutex);  //SPI Mutex Give
    //xSemaphoreGive( dw1000_spi_xSemaphore );
    rt_mutex_release(dw1000_spi_mutex);
    //decamutexoff(stat) ;

    return 0;
}

static char dw1000_irq_flag = 0;
/*******************************************************************************
 This function should disable interrupts - "deca_device_api.h"
 return: the irq state before disable, this value is used to re-enable in decamutexoff call
*******************************************************************************/
decaIrqStatus_t decamutexon(void)
{

    //decaIrqStatus_t s = 0;
    decaIrqStatus_t s = dw1000_irq_flag;
    //decaIrqStatus_t s = 0;
#if 0
    decaIrqStatus_t s = EXTI_GetITEnStatus(GPIOTE_INTENSET_IN2_Msk);

    if(s)
    {
        NRF_GPIOTE->INTENCLR  = GPIOTE_INTENCLR_IN2_Clear << GPIOTE_INTENCLR_IN2_Pos;
    }
#endif
    if (s)
        //GPIO_disableInt(DW1000_IRQ);
        DW1000_disableirq();
    return s; // return state before disable, value is used to re-enable in decamutexoff call
}

/*******************************************************************************
 This function should re-enable interrupts - "deca_device_api.h"
 return: None
*******************************************************************************/
void decamutexoff(decaIrqStatus_t s)
{
#if 0
    if(s)
    {
        NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN2_Set << GPIOTE_INTENSET_IN2_Pos;  //enable ScenSor IRQ before starting
    }
#endif
    if (s)
        //GPIO_enableInt(DW1000_IRQ);
        DW1000_enableirq();
}

/*******************************************************************************
//enable dw irq
*******************************************************************************/
void DW_IRQn_enable(void)
{
    //NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN2_Set << GPIOTE_INTENSET_IN2_Pos;
    //GPIO_enableInt(DW1000_IRQ);
    DW1000_enableirq();
    //GPIO_disableInt(DW1000_IRQ);
    dw1000_irq_flag = 1;
}

/*******************************************************************************
//disenable dw irq
*******************************************************************************/
void DW_IRQn_disenable(void)
{
    //NRF_GPIOTE->INTENCLR  = GPIOTE_INTENCLR_IN2_Clear << GPIOTE_INTENCLR_IN2_Pos;
    //GPIO_disableInt(DW1000_IRQ);
    DW1000_enableirq();
    dw1000_irq_flag = 0;
}
/*******************************************************************************
//DW1000 IRQ Pin Interrupt Process
*******************************************************************************/
void process_deca_irq(void *pvParameters)
{
    //for(;;)
    {
        //osi_SyncObjWait(&dw1000_Semaphore,OSI_WAIT_FOREVER);  //Wait Task Operation Message

        do
        {
            //printf("dw isr:%d\r\n",(int)sys_Tick);
#ifdef ANCHOR_MODE
            dw_irq_tick = sys_Tick;
#endif
            dw1000_irq_tick = sys_Tick;

            dwt_isr(); //call device interrupt handler

            dw1000_irq_tick = 0;
            //}while(GPIO_read(DW1000_IRQ) == 1);  //while IRS line active
            //} while(HAL_GPIO_ReadPin(DW1000_IRQ_PORT,DW1000_IRQ)== GPIO_PIN_SET);
        } while (DW1000_IRQ_DATA);
        //}while(nrf_gpio_pin_read(DW_IRQn_PIN) == 1);  //while IRS line active
        //}while(gpio_get_level(PIN_NUM_DW_IRQ)==1);
        //}while(0);
    }
}

static void dw1000_irq_task(void *arg)
{
    //    uint32_t io_num;
    static rt_err_t result;
    for (;;)
    {

        //if(xQueueReceive(dw1000_irq_evt_queue, &io_num, portMAX_DELAY))

        //if(xQueueReceive(dw1000_irq_evt_queue, &io_num, portMAX_DELAY))
        //if( xSemaphoreTake( dw1000_irq_xSemaphore, portMAX_DELAY ) == pdTRUE )
        result = rt_sem_take(dw1000_irq_xSemaphore, RT_WAITING_FOREVER);
        if (result != RT_EOK)
        {
            rt_kprintf("take semaphore, failed.\n");
            rt_sem_delete(dw1000_irq_xSemaphore);
            return;
        }
        else
        {

            //printf("111");
            //vTaskEnterCritical();
            //vTaskSuspendAll();
            //xSemaphoreTake( dw1000_irq_lock, portMAX_DELAY );
            process_deca_irq(arg);
            //xSemaphoreTake( dw1000_spi_xSemaphore, portMAX_DELAY );
            //xSemaphoreGive( dw1000_irq_lock );
            //xTaskResumeAll();
            //vTaskExitCritical();
        }
        //printf("222");
        //vTaskDelay(1);
    }
}
extern void instance_run(void);
static void dw1000_app_task(void *arg)
{
    uint8_t channel_speed = 0;

    if (dev_channel == 5)
    {
        channel_speed |= 0x02;
    }
    if (dev_speed == 1)
    {
        channel_speed |= 0x01;
    }

    dw1000_runtick = sys_Tick;

    DW_IRQn_disenable();

    while (dw1000_init(channel_speed, (uint16_t)dev_address) != 0) // Configures DW1000 Device
    {
        rt_kprintf("config dw1000 err\r\n");
        rt_thread_mdelay(1000);
    }
    //while(dw1000_init(1,(uint16_t)dev_address) != 0);  // Configures DW1000 Device

    DW_IRQn_enable();

    dw1000_runtick = 0;

    for (;;)
    {
        dw1000_runtick = sys_Tick;

        //xSemaphoreTake( dw1000_irq_lock, portMAX_DELAY );
        instance_run();
        //xSemaphoreGive( dw1000_irq_lock );

        //if delayed TX scheduled but did not happen after expected time then it has failed... (has to be < slot period)
        //if anchor just go into RX and wait for next message from tags/anchors,if tag handle as a timeout
        if ((instance_data.monitor == 1) && ((sys_Tick - instance_data.timeofTx) > instance_data.slotPeriod))
        {
            instance_data.wait4ack = 0;

            if (instance_data.mode == TAG)
            {
                inst_processrxtimeout(&instance_data);
            }
            else
            {
                dwt_forcetrxoff(); //this will clear all events
                rt_kprintf("test\r\n");
                instance_data.testAppState = TA_RXE_WAIT;
            }

            instance_data.monitor = 0;
        }

        dw1000_runtick = 0;
        //printf("tick:%d\r\n",(int)sys_Tick);
        //vTaskDelay(1000 / portTICK_RATE_MS);
        //vTaskDelay(1);
        // rt_thread_mdelay(1);
    }
}
static rt_timer_t timer1;
static void dw1000_tick_callback(void *parameter)
{
    sys_Tick += 1;
    //process_deca_irq((void *)0);
    //rt_kprintf("++\r\n");
}

void dw1000_tick_task(void)
{

    timer1 = rt_timer_create("timer1", dw1000_tick_callback,
                             RT_NULL, 10,
                             RT_TIMER_FLAG_PERIODIC);

    if (timer1 != RT_NULL)
        rt_timer_start(timer1);
}

static void print_data_task(void *p)
{

    //  uint16_t data;
    dw1000_debug_message_tag message;
    while (1)
    {
        /* 从队列中获取内容 */
        if (rt_mq_recv(&print_data, &message, sizeof(message), RT_WAITING_FOREVER) == RT_EOK)
        {
            //printf("call_tx:%x\r\n",data&0xff);
            rt_kprintf("message:%x,s_addr:%x,d_addr:%x,%d,%s\r\n", message.code, message.s_addr, message.d_addr, message.sys_Tick, message.message);
        }
    }
}

static void print_task(void *p)
{
    //instance_data_t ins_data;
    //	char dev_addr[5];
    //char pan_id[5];
    rt_err_t result;
    UartMsgStruct p_Msg;
    uint32_t len;
    message_mdps_t msg_send;
    message_mdps_t uart_send;
    // char uart_send_buff[256];
    char s_addr_tem[5];
    char d_addr_tem[5];
    char rx_time_tem[11];
    // memset(uart_send_buff, 0, sizeof(uart_send_buff));

    while (1)
    {
        /* 从队列中获取内容 */
        if (rt_mq_recv(&message_print, &p_Msg, sizeof(p_Msg), RT_WAITING_FOREVER) == RT_EOK)
        {

            // memset(uart_send.buf, 0, sizeof(uart_send.buf));
            // memset(s_addr_tem, 0, sizeof(s_addr_tem));
            // memset(d_addr_tem, 0, sizeof(d_addr_tem));
            // memset(rx_time_tem, 0, sizeof(rx_time_tem));
            snprintf(s_addr_tem, sizeof(s_addr_tem), "%04x", p_Msg.s_addr);                                      //s
            snprintf(d_addr_tem, sizeof(d_addr_tem), "%04x", p_Msg.d_addr);                                      //d
            snprintf(rx_time_tem, sizeof(rx_time_tem), "%02x%08x", (uint8_t)p_Msg.rx_time[1], p_Msg.rx_time[0]); //t
            cJSON *root = cJSON_CreateObject();
            if (!root)
            {
                rt_kprintf("No memory for cJSON root!\n");
                cJSON_Delete(root);
                continue;
            }
            cJSON *data = cJSON_CreateObject();
            if (!data)
            {
                rt_kprintf("No memory for cJSON data!\n");
                cJSON_Delete(data);
                continue;
                // return;
            }

            switch (p_Msg.msg_type)
            {
            case HOST_ANCHOR_SYNC:
            {
                cJSON_AddNumberToObject(data, "p", p_Msg.msg_type);
                cJSON_AddNumberToObject(data, "n", p_Msg.sec_num);
                cJSON_AddStringToObject(data, "s", s_addr_tem);
                cJSON_AddStringToObject(data, "t", rx_time_tem);
                break;
            }
            case SLAVE_ANCHOR_SYNC:
            {
                cJSON_AddNumberToObject(data, "p", p_Msg.msg_type);
                cJSON_AddNumberToObject(data, "n", p_Msg.sec_num);
                cJSON_AddStringToObject(data, "s", s_addr_tem);
                cJSON_AddStringToObject(data, "d", d_addr_tem);
                cJSON_AddStringToObject(data, "t", rx_time_tem);
                cJSON_AddNumberToObject(data, "r", p_Msg.rssi_val);
                break;
            }
            case TAG_BORADCAST:
            {
                cJSON_AddNumberToObject(data, "p", p_Msg.msg_type);
                cJSON_AddNumberToObject(data, "n", p_Msg.sec_num);
                cJSON_AddStringToObject(data, "s", s_addr_tem);
                cJSON_AddStringToObject(data, "d", d_addr_tem);
                cJSON_AddStringToObject(data, "t", rx_time_tem);
                cJSON_AddNumberToObject(data, "r", p_Msg.rssi_val);
                cJSON_AddStringToObject(data, "msg", p_Msg.msg_buf);
                break;
            }
            case TOF_RANGING:
            {
                cJSON_AddNumberToObject(data, "p", p_Msg.msg_type);
                cJSON_AddNumberToObject(data, "n", p_Msg.sec_num);
                cJSON_AddStringToObject(data, "s", s_addr_tem);
                cJSON_AddStringToObject(data, "d", d_addr_tem);
                cJSON_AddNumberToObject(data, "l", p_Msg.distance_val);
                cJSON_AddNumberToObject(data, "r", p_Msg.rssi_val);
                cJSON_AddStringToObject(data, "msg", p_Msg.msg_buf);
                break;
            }
            case ANCHOR_RANGING:
            {
                cJSON_AddNumberToObject(data, "p", p_Msg.msg_type);
                cJSON_AddNumberToObject(data, "n", p_Msg.sec_num);
                cJSON_AddStringToObject(data, "s", s_addr_tem);
                cJSON_AddStringToObject(data, "d", d_addr_tem);
                cJSON_AddNumberToObject(data, "l", p_Msg.distance_val);
                cJSON_AddNumberToObject(data, "r", p_Msg.rssi_val);
                cJSON_AddStringToObject(data, "msg", "");
                break;
            }
#ifdef TIME_STAMP_DEBUG
            case ANCHOR_TIMESTAMP_DEBUG:
            {
                rt_kprintf("{\"p\":%d,\"n\":%d,\"s\":\"%04x\",\"d\":\"%04x\",\"tp\":\"%02x%08x\",\"pr\":\"%02x%08x\",\"at\":\"%02x%08x\",\"tr\":\"%02x%08x\",\"tf\":\"%02x%08x\"}\n\r", ANCHOR_TIMESTAMP_DEBUG, p_Msg.sec_num, p_Msg.s_addr, p_Msg.d_addr, (uint8_t)p_Msg.tag_poll_time[1], p_Msg.tag_poll_time[0], (uint8_t)p_Msg.poll_rx_time[1], p_Msg.poll_rx_time[0], (uint8_t)p_Msg.anchor_tx_time[1], p_Msg.anchor_tx_time[0], (uint8_t)p_Msg.tag_rx_time[1], p_Msg.tag_rx_time[0], (uint8_t)p_Msg.tag_final_time[1], p_Msg.tag_final_time[0]);
            }
#endif
            default:
            {
                cJSON_Delete(data);
                cJSON_Delete(root);
                continue; //返回主循环
            }
            }

            char *data_str = cJSON_PrintUnformatted(data);
            uart_send.len = rt_sprintf(uart_send.buf, "%s", data_str);
            // memcpy(uart_send.buf, data_str, strlen(data_str) + 1);
            rt_free(data_str);
            cJSON_Delete(data);

            cJSON_AddStringToObject(root, "data", uart_send.buf);
            char *root_str = cJSON_PrintUnformatted(root);
            cJSON_Delete(root);
            msg_send.len = rt_sprintf(msg_send.buf, "%s", root_str);
            // memcpy(msg_send.buf, root_str, strlen(root_str) + 1);
            rt_free(root_str);

            // if (strcmp(s_addr_tem, "0302") == 0)
            // if (p_Msg.msg_type == 4)
            // rt_kprintf("%s\n", uart_send.buf);
            // rt_kprintf("msg_send:%s\n", msg_send.buf);
            rt_mq_send(&message_send, (void *)&msg_send, sizeof(msg_send));
        }
    }
}
static rt_uint8_t message_print_pool[1024];
static rt_uint8_t print_data_pool[1024];
static rt_uint8_t message_send_pool[2048];

//------------------------------------------
void run_dw1000_task(void)
{
    /*
    dw1000_spi_xSemaphore = xSemaphoreCreateMutex();
    dw1000_irq_lock = xSemaphoreCreateMutex();
    dw1000_irq_xSemaphore = xSemaphoreCreateBinary();
    */
    DW1000_init();

    dw1000_spi_mutex = rt_mutex_create("spi_mute", RT_IPC_FLAG_FIFO);
    //		dw1000_irq_lock = rt_sem_create("irq_lok", 0, RT_IPC_FLAG_FIFO);
    dw1000_irq_xSemaphore = rt_sem_create("irq_xSem", 0, RT_IPC_FLAG_FIFO);
    /*
    dw1000_spi_init(1);
    dw1000_irq_init();
    */

    //message_print = xQueueCreate( 5 , sizeof( UartMsgStruct ) );
    //print_data = xQueueCreate( 5 , sizeof( dw1000_debug_message_tag ) );
    rt_err_t result;

    /* ??????? */
    result = rt_mq_init(&message_print,
                        "mqt1",
                        &message_print_pool[0],
                        sizeof(UartMsgStruct),
                        sizeof(message_print_pool),
                        RT_IPC_FLAG_FIFO);

    if (result != RT_EOK)
    {
        rt_kprintf("init message queue failed.\n");
        return;
    }
    result = rt_mq_init(&print_data,
                        "mqt2",
                        &print_data_pool[0],
                        sizeof(dw1000_debug_message_tag),
                        sizeof(print_data_pool),
                        RT_IPC_FLAG_FIFO);

    if (result != RT_EOK)
    {
        rt_kprintf("init message queue failed.\n");
        return;
    }
    result = rt_mq_init(&message_send,
                        "mq_send",
                        &message_send_pool[0],
                        sizeof(message_mdps_t),
                        sizeof(message_send_pool),
                        RT_IPC_FLAG_FIFO);

    if (result != RT_EOK)
    {
        rt_kprintf("init message queue failed.\n");
        return;
    }
    //dw1000_init(0,0);
    dw1000_tick_task(); //

    static rt_thread_t tid1 = RT_NULL;

#if 1
    //xTaskCreate(print_task, "print_task", 1024*2, NULL, 1, NULL);
    tid1 = rt_thread_create("thread1",
                            print_task, RT_NULL,
                            4096,
                            RT_THREAD_PRIORITY_MAX - 2, 20);

    if (tid1 != RT_NULL)
        rt_thread_startup(tid1);

        //xTaskCreate(print_data_task, "print_data_task", 1024*8, NULL, 1, NULL);
        // tid1 = rt_thread_create("print_data_task",
        //                         print_data_task, RT_NULL,
        //                         1024,
        //                         0, 20);

        // if (tid1 != RT_NULL)
        //     rt_thread_startup(tid1);

#endif
    //xTaskCreate(dw1000_irq_task, "dw1000_irq_task", 1024*8, NULL, 5, NULL);
    tid1 = rt_thread_create("dw1000_irq_task",
                            dw1000_irq_task, RT_NULL,
                            2048,
                            0, 20);

    if (tid1 != RT_NULL)
        rt_thread_startup(tid1);
#if 1

    //xTaskCreate(dw1000_app_task, "dw1000_app_task", 1024*8, NULL, 4, NULL);
    tid1 = rt_thread_create("dw1000_app_task",
                            dw1000_app_task, RT_NULL,
                            2048,
                            RT_THREAD_PRIORITY_MAX - 2, 20);

    if (tid1 != RT_NULL)
        rt_thread_startup(tid1);
#endif
}

/*******************************************************************************

//parse no resp tag address

*******************************************************************************/

void parse_noresp_addrs(uint16_t tag_num, char *addr_buf)

{

    uint16_t i = 0;

    char *InpString;

    no_resp_tag = tag_num > NO_RESP_TAG_NUM ? NO_RESP_TAG_NUM : tag_num;

    memset(no_resp_addr, 0, NO_RESP_TAG_NUM);

    if (no_resp_tag > 0)

    {

        InpString = strtok(addr_buf, ",");

        no_resp_addr[i] = (uint16_t)strtoul(InpString, 0, 16);

        for (i = 1; i < no_resp_tag; i++)

        {

            InpString = strtok(NULL, ",");

            no_resp_addr[i] = (uint16_t)strtoul(InpString, 0, 16);
        }
    }
}

/*******************************************************************************

//check no resp tag address

*******************************************************************************/

bool check_noresp_addrs(uint16_t tag_addr)

{

    uint16_t i;

    for (i = 0; i < no_resp_tag; i++)

    {

        if (tag_addr == no_resp_addr[i])

        {

            return 1;
        }
    }

    return 0;
}

/*******************************************************************************

//read no resp tag address

*******************************************************************************/

void read_noresp_addrs(char *addr_buf, uint16_t buf_size)

{

    uint16_t i;

    char addr[6] = {0};

    snprintf(addr_buf, buf_size, "{\"p\":%d,\"src_addr\":\"%04x\",\"Num\":%d,\"Addrs\":\"", NO_RESP_TAG_MSG, dev_address, no_resp_tag);

    for (i = 0; i < no_resp_tag; i++)

    {

        memset(addr, 0, 6);

        if (i == (no_resp_tag - 1))

        {

            snprintf(addr, 6, "%04x", no_resp_addr[i]);
        }

        else

        {

            snprintf(addr, 6, "%04x,", no_resp_addr[i]);
        }

        strcat(addr_buf, addr);
    }

    strcat(addr_buf, "\"}");
}

void PA_LNA_ON()
{
    rt_pin_write(DW1000_5V_EN, PIN_HIGH);
}

void PA_LNA_OFF()
{
    rt_pin_write(DW1000_5V_EN, PIN_LOW);
}