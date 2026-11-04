#ifndef __CCD_UART_H
#define __CCD_UART_H

#include "main.h"

void CCD_UART_Init(void);
HAL_StatusTypeDef CCD_UART_SendFrame(void);

#endif
