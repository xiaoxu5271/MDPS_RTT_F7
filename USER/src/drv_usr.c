#include <rtthread.h>
#include <rtdevice.h>

#include "drv_spi.h"
#include "dw1000_usr.h"
#include "drv_usr.h"
#include "board.h"

struct rt_spi_device *spi_dev_dw1000;

SPI_HandleTypeDef hspi2;
static void MX_SPI1_Init(unsigned int rate)
{

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = rate; //SPI_BAUDRATEPRESCALER_8;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
}

static int rt_hw_spi_DW1000_init(void)
{
    rt_err_t result;
    __HAL_RCC_GPIOF_CLK_ENABLE();
    // rt_hw_spi_device_attach("spi2", "spi20", GPIOF, GPIO_PIN_7);

    spi_dev_dw1000 = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));
    RT_ASSERT(spi_dev_dw1000 != RT_NULL);

    result = rt_spi_bus_attach_device(spi_dev_dw1000, "spi20", "spi2", RT_NULL);

    if (result != RT_EOK)
    {
        rt_kprintf("attach to faild, %d\n", result);
    }

    // spi_dev_dw1000 = (struct rt_spi_device *)rt_device_find("spi20");
    // if (spi_dev_dw1000 == RT_NULL)
    // {
    //     rt_kprintf("not find spi20 device!!!\n");
    //     return RT_ERROR;
    // }

    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_NO_CS;
    cfg.max_hz = 5 * 1000 * 1000; /* 20M */

    result = rt_spi_configure(spi_dev_dw1000, &cfg);
    if (result != RT_EOK)
    {
        rt_kprintf("rt_spi_configure faild, %d\n", result);
    }

    spi_dev_dw1000 = (struct rt_spi_device *)rt_device_find("spi20");
    if (spi_dev_dw1000 == RT_NULL)
    {
        rt_kprintf("not find spi20 device!!!\n");
        return RT_ERROR;
    }
    return RT_EOK;
}
/* ???????? */
// INIT_COMPONENT_EXPORT(rt_hw_spi_DW1000_init);

void process_deca_irq(void *pvParameters);

void DW1000_gpio_init(void)
{
    // rt_pin_mode(W25Q128_CS, PIN_MODE_OUTPUT); //FLASH CS
    // rt_pin_write(W25Q128_CS, PIN_HIGH);       //À­¸ßflash ½ûÖ¹flash²Ù×÷
    rt_pin_mode(DW1000_5V_EN, PIN_MODE_OUTPUT); //DW1000 RESET
    rt_pin_mode(DW1000_CS, PIN_MODE_OUTPUT);    //DW1000 SPI CS
    rt_pin_mode(DW1000_RST, PIN_MODE_OUTPUT);   //DW1000 RESET

    rt_pin_mode(DW1000_IRQ, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(DW1000_IRQ, PIN_IRQ_MODE_RISING_FALLING, dw1000_irq_isr_handler, (void *)"callbackargs");
    rt_pin_irq_enable(DW1000_IRQ, PIN_IRQ_ENABLE);

    rt_pin_irq_enable(DW1000_IRQ, PIN_IRQ_DISABLE);
}
void DW1000_init(void)
{
    DW1000_gpio_init();
    HAL_SPI_MspDeInit(&hspi2);
    MX_SPI1_Init(SPI_BAUDRATEPRESCALER_16);
    // rt_hw_spi_DW1000_init();
    // HAL_SPI_MspDeInit(&hspi1);
    // MX_SPI1_Init(SPI_BAUDRATEPRESCALER_32);
    // char send_buff[4] = {0, 1, 2, 3};

    // spi_write(send_buff, sizeof(send_buff));
}
//-----------------------------------
void spi_write(uint8_t *pData, uint16_t Size)
{
    // rt_spi_send(spi_dev_dw1000, pData, Size);
    HAL_SPI_Transmit(&hspi2, pData, Size, 0xffff);
}
void spi_read(uint8_t *pData, uint16_t Size)
{
    // rt_spi_recv(spi_dev_dw1000, pData, Size);
    HAL_SPI_Receive(&hspi2, pData, Size, 0xffff);
}
void DW1000_enableirq(void)
{
    rt_pin_irq_enable(DW1000_IRQ, PIN_IRQ_ENABLE);
}
void DW1000_disableirq(void)
{
    rt_pin_irq_enable(DW1000_IRQ, PIN_IRQ_DISABLE);
}
/*
#define SPI_BAUDRATEPRESCALER_2         0x00000000U
#define SPI_BAUDRATEPRESCALER_4         0x00000008U
#define SPI_BAUDRATEPRESCALER_8         0x00000010U
#define SPI_BAUDRATEPRESCALER_16        0x00000018U
#define SPI_BAUDRATEPRESCALER_32        0x00000020U
#define SPI_BAUDRATEPRESCALER_64        0x00000028U
#define SPI_BAUDRATEPRESCALER_128       0x00000030U
#define SPI_BAUDRATEPRESCALER_256       0x00000038U
*/
//speed:SPI_BAUDRATEPRESCALER_2,SPI_BAUDRATEPRESCALER_4....
void spi_speed_set(int speed)
{
    HAL_SPI_MspDeInit(&hspi2);
    MX_SPI1_Init(speed);
    HAL_SPI_MspInit(&hspi2);
    rt_kprintf("set spi speed : %d \r\n", speed);
}
