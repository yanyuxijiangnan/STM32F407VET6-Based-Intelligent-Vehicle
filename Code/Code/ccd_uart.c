#include "ccd_uart.h"
#include "ccd.h"
#include "usart.h"

#define CCD_UART_FRAME_LEN 270

static uint8_t ccd_uart_txbuf[CCD_UART_FRAME_LEN];

static volatile uint8_t ccd_uart_busy = 0;

static const char HEX_TABLE[] =
    "0123456789ABCDEF";


void CCD_UART_Init(void)
{
    ccd_uart_busy = 0;
}


HAL_StatusTypeDef CCD_UART_SendFrame(void)
{
    uint16_t i;
    uint8_t pixel;
    HAL_StatusTypeDef status;

    /* 上一帧还没有发送完成 */
    if (ccd_uart_busy)
    {
        return HAL_BUSY;
    }


    /* =========================
       1. 帧头
       ========================= */

    ccd_uart_txbuf[0] = '*';


    /* =========================
       2. 10字节占位
       LD00000000
       ========================= */

    ccd_uart_txbuf[1]  = 'L';
    ccd_uart_txbuf[2]  = 'D';
    ccd_uart_txbuf[3]  = '0';
    ccd_uart_txbuf[4]  = '0';
    ccd_uart_txbuf[5]  = '0';
    ccd_uart_txbuf[6]  = '0';
    ccd_uart_txbuf[7]  = '0';
    ccd_uart_txbuf[8]  = '0';
    ccd_uart_txbuf[9]  = '0';
    ccd_uart_txbuf[10] = '0';


    /* =========================
       3. 128个CCD像素

       每个12位ADC值：
       0~4095

       >>4 后：
       0~255

       然后转换成2位HEX ASCII
       ========================= */

    for (i = 0; i < 128; i++)
    {
        pixel = (uint8_t)(CCD_Raw[i] >> 4);

        ccd_uart_txbuf[11 + i * 2]
            = HEX_TABLE[(pixel >> 4) & 0x0F];

        ccd_uart_txbuf[12 + i * 2]
            = HEX_TABLE[pixel & 0x0F];
    }


    /* =========================
       4. 两个尾部占位字符
       ========================= */

    ccd_uart_txbuf[267] = '0';
    ccd_uart_txbuf[268] = '0';


    /* =========================
       5. 帧尾
       ========================= */

    ccd_uart_txbuf[269] = '#';


    /* =========================
       DMA发送
       ========================= */

    ccd_uart_busy = 1;

    status = HAL_UART_Transmit_DMA(
            &huart3,
            ccd_uart_txbuf,
            CCD_UART_FRAME_LEN);
    if (status != HAL_OK)
    {
        ccd_uart_busy = 0;
    }
    return status;
}


/*
 * UART DMA发送完成回调
 */
void HAL_UART_TxCpltCallback(
        UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        ccd_uart_busy = 0;
    }
}
