/*******************************************************************************
 *
 * Copyright (c) 2020
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description:
 *
 * Author: HoangNH
 *
 * Last Changed By:  $Author: HoangNH $
 * Revision:         $Revision: 1.1 $
 * Last Changed:     $Date: 10/07/20 $
 *
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdio.h>
#include "rs232.h"
#include "timer.h"
#include "buff.h"
#include "buff.h"
#include "utilities.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define SIZE_BUFF_DATA_RX                 256

#if (SIZE_BUFF_DATA_RX & (SIZE_BUFF_DATA_RX - 1)) != 0
#error "SIZE_BUFF_DATA_RX must be a power of two"
#endif

#define SIZE_BUFF_DATA_TX                 256

#if (SIZE_BUFF_DATA_TX & (SIZE_BUFF_DATA_TX - 1)) != 0
#error "SIZE_BUFF_DATA_TX must be a power of two"
#endif

/*! @brief Field lenght */
#define FlenBuf                             (*byRxBuffer)
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
uint8_t rxData;

static uint8_t byRxBufState;
static uint8_t byIndexRxBuffer;
static uint8_t byCheckXorRxBuf;
static uint32_t dwRxTimeStamp;

uint8_t byRxBuffer[RS232_RX_BUFFER_SIZE] = {0};

static buffqueue_t serialQueueRx;
static uint8_t pBuffDataRx[SIZE_BUFF_DATA_RX];

static void *g_pUartQueueRx[USART_COUNT] = { 0 };

static rs232_receiver_handle_event pRS232ReceiverEventCallback;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
extern UART_HandleTypeDef huart6;
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void UART_RegBufferRx(uint8_t byUartNumber, buffqueue_p pQueueRx);
static uint8_t rs232_calculate_crc(uint8_t *byRespFrame, uint8_t bySizeFrame);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/**
 * @func   RS232_Init
 * @brief  Initialize UART and receive buffer register
 * @param  None
 * @retval None
 */
void RS232_Init(rs232_receiver_handle_event pReceiverCallback) {
	// Initialize callback handler data receive UART3
	if (pReceiverCallback != NULL) {
		pRS232ReceiverEventCallback = pReceiverCallback;
	}

    /* Initializes receive register buffer  */
    bufInit(pBuffDataRx, &serialQueueRx, sizeof(pBuffDataRx[0]), SIZE_BUFF_DATA_RX);
	UART_RegBufferRx(USART3_IDX, &serialQueueRx);

    /* Initializes UART6 */
	byRxBufState = (uint8_t)RS232_RX_STATE_START_BYTE;

	HAL_UART_Receive_IT(&huart6, &rxData, 1);
}

/**
 * @func   RS232_PollRxBuffer
 * @brief  Process received data
 * @param  None
 * @retval byUartState: uart state received
 */
uint8_t RS232_PollRxBuffer(void) {
    uint8_t byRxData;
    uint8_t byUartState = (uint8_t)RS232_UART_STATE_IDLE;
    
    while ((bufNumItems(&serialQueueRx) != 0) && (byUartState == RS232_UART_STATE_IDLE))
    {
        bufDeDat(&serialQueueRx, &byRxData);

//        printf("deq: %x(%d)\n", byRxData, byRxBufState);

        switch (byRxBufState) {
            case RS232_RX_STATE_START_BYTE:
                if (byRxData == RS232_FRAME_SOF) {
                	byIndexRxBuffer = 0;
                	byCheckXorRxBuf = RS232_FRAME_SOF;
                    byRxBufState = RS232_RX_STATE_DATA_BYTES;
                    dwRxTimeStamp = GetMilSecTick();
                } else {
                    byUartState = RS232_UART_STATE_FRAME_ERROR;
                }
                break;
            
            case RS232_RX_STATE_DATA_BYTES:
            	if (byIndexRxBuffer < RS232_RX_BUFFER_SIZE) {
					byRxBuffer[byIndexRxBuffer] = byRxData;
					if (byIndexRxBuffer >= 0) {
						/* Calculate check XOR */
						byCheckXorRxBuf ^= byRxData;
					}
					if (++byIndexRxBuffer == (FlenBuf + 1)) {
						byRxBufState = RS232_RX_STATE_CXOR_BYTE;
					}
					dwRxTimeStamp = GetMilSecTick();
				}
				else {
					byRxBufState = RS232_RX_STATE_START_BYTE;
					byUartState = RS232_UART_STATE_FRAME_ERROR;
				}
                break;
                
            case RS232_RX_STATE_CXOR_BYTE:
            	if (byRxData == byCheckXorRxBuf) {
					byUartState = RS232_UART_STATE_FRAME_RECEIVED;
				}
				else {
					byUartState = RS232_UART_STATE_FRAME_ERROR;
				}
                
            default:
            	byRxBufState = RS232_RX_STATE_START_BYTE;
                break;
        }
    }
    
    /* Check timeout rx */
	if (byUartState == RS232_UART_STATE_IDLE) {
		if ((uint32_t)(GetMilSecTick() - dwRxTimeStamp) >= RS232_RX_TIMEOUT) {
			byRxBufState = RS232_RX_STATE_START_BYTE;
			byUartState  = RS232_UART_STATE_RX_TIMEOUT;
		}
	}

    return byUartState;
}

/**
 * @func   RS232_ReceiverProcedure
 * @brief  Process data received from uart
 * @param  None
 * @retval None
 */
void
RS232_ReceiverProcedure(void) {
    uint8_t stateRx = RS232_PollRxBuffer();

	switch (stateRx) {
		case RS232_UART_STATE_FRAME_RECEIVED:
			if (pRS232ReceiverEventCallback != NULL) {
				pRS232ReceiverEventCallback(byRxBuffer[0], &byRxBuffer[1]);
			}
			break;

		case RS232_UART_STATE_FRAME_ERROR:
		case RS232_UART_STATE_RX_TIMEOUT:
			break;

		default:
			break;
	}
}

/**
 * @func   Serial_SendPacket
 * @brief  Process transmit message uart
 * @param  byOption: Option
 * @param  byCmdId: Identify
 * @param  byType: Type
 * @param  pPayload: Payload
 * @param  byLengthPayload: Length payload
 * @retval None
 */
void RS232_SendPacket(
	uint8_t cmdId,
	uint8_t cmdType,
	uint8_t* pPayload,
	uint8_t lengthPayload
) {
	static uint8_t bySeq = 0;
    uint8_t byOffset = 0;
    uint8_t byBufferTx[32];
    uint8_t checkxor;
    
    byBufferTx[byOffset++] = RS232_FRAME_SOF;
    byBufferTx[byOffset++] = lengthPayload + 4; /* Include: id + type + seq + cxor */
    byBufferTx[byOffset++] = cmdId;
    byBufferTx[byOffset++] = cmdType;
    
    for (uint8_t i = 0; i < lengthPayload; i++) {
        byBufferTx[byOffset++] = pPayload[i];
    }
    
    byBufferTx[byOffset++] = bySeq++;
    checkxor = rs232_calculate_crc(&byBufferTx[2], byOffset - 2);
    byBufferTx[byOffset++] = checkxor;
    
    /* Send frame to Host via UART */
    for (uint8_t i = 0; i < byOffset; i++) {
        while(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TXE) == RESET);
        huart6.Instance->DR = byBufferTx[i];
        while(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) == RESET);
    }
}

/**
 * @func   rs232_calculate_crc
 * @brief  Calculate value XOR
 * @param  pbyTxBuffer:
           bySizeFrame: Size of frame
 * @retval check_XOR
 */
static uint8_t
rs232_calculate_crc(
    uint8_t *pbyTxBuffer,
    uint8_t bySizeFrame
) {
    uint8_t byCXOR = RS232_CXOR_INIT_VAL;
    for (uint8_t byCntSize = 0; byCntSize < bySizeFrame; byCntSize++)
    {
        byCXOR ^= *pbyTxBuffer;
        pbyTxBuffer++;
    }
    return byCXOR;
}

/**
 * @func   UART_RegBufferRx
 * @brief  Initializes register buffer receive
 * @param  byUartNumber: Select the UART peripheral
 *   This parameter can be one of the following values:
 *   UART0_IDX, UART1_IDX, UART2_IDX
 * @param  pQueueRx:
 * @retval None
 */
void
UART_RegBufferRx(
    uint8_t byUartNumber,
    buffqueue_p pQueueRx
) {
    g_pUartQueueRx[byUartNumber] = pQueueRx;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART6) {
		buffqueue_p pUartBuffQueueRx = (buffqueue_p) g_pUartQueueRx[0];
		if (bufEnDat(pUartBuffQueueRx, &rxData) == ERR_BUF_FULL) {}
		HAL_UART_Receive_IT(&huart6, &rxData, 1);
	}
}

//void USART2_IRQHandler(void) {
//    __disable_irq();
//    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
//        buffqueue_p pUartBuffQueueRx = (buffqueue_p) g_pUartQueueRx[0];
//        uint8_t byReceiverData = (uint8_t)(huart2.Instance->DR & 0xFF);
//        if (bufEnDat(pUartBuffQueueRx, &byReceiverData) == ERR_BUF_FULL) {}
//        __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);
//    }
//    __enable_irq();
//}


/* END FILE */
