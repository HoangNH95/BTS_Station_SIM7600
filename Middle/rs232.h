/*******************************************************************************
 *
 * Copyright (c) 2020
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description: Include file for application
 *
 * Author: HoangNH
 *
 * Last Changed By:  $Author: HoangNH $
 * Revision:         $Revision: 1.1 $
 * Last Changed:     $Date: 10/07/20 $
 *
 ******************************************************************************/
#ifndef _RS232_H_
#define _RS232_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include "stm32f4xx_hal.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
/* --------------------------------RECEIVER------------------------------------
 * Definition state receiver
 * ---------------------------------------------------------------------------*/
#define USART3_IDX						 0
#define USART_COUNT                      1

//UART communicate
#define RS232_USART_INSTANCE             USART2
#define RS232_USART_CLK                  RCC_APB1Periph_USART2
#define RS232_USART_CLK_INIT             RCC_APB1PeriphClockCmd

#define RS232_USART_TX_PIN               GPIO_Pin_2
#define RS232_USART_TX_GPIO_PORT         GPIOA
#define RS232_USART_TX_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define RS232_USART_TX_GPIO_INIT         RCC_AHB1PeriphClockCmd

#define RS232_USART_RX_PIN               GPIO_Pin_3
#define RS232_USART_RX_GPIO_PORT         GPIOA
#define RS232_USART_RX_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define RS232_USART_RX_GPIO_INIT         RCC_APB2PeriphClockCmd

/*! @brief Size of rx buffer */
#define RS232_RX_BUFFER_SIZE             128

/*! @brief Timeout rx */
#define RS232_RX_TIMEOUT                 20

/*! @brief Start of frame */
#define RS232_FRAME_SOF                  0x55

/*! @brief Frame ack and nack */
#define RS232_FRAME_ACK                  0x06
#define RS232_FRAME_NACK                 0x15

/*! @brief Check xor init */
#define RS232_CXOR_INIT_VAL              0xFF

/*! @brief State receiver */
typedef enum {
	RS232_RX_STATE_START_BYTE,
	RS232_RX_STATE_DATA_BYTES,
	RS232_RX_STATE_CXOR_BYTE
} RS232_RX_STATE;

/*! @brief State frame uart */
typedef enum {
	RS232_UART_STATE_IDLE,
	RS232_UART_STATE_FRAME_RECEIVED,
	RS232_UART_STATE_FRAME_ERROR,
	RS232_UART_STATE_RX_TIMEOUT,
} RS232_UART_STATE;

typedef struct {
    uint8_t cmdid;
    uint8_t type;
} cmd_common_t;

typedef struct {
    uint8_t cmdid;
    uint8_t type;
    uint8_t epoint;
    uint8_t state;
} cmd_button_state_t;

typedef struct {
    uint8_t cmdid;
    uint8_t type;
    uint8_t state;
} cmd_buzzer_state_t;

typedef struct {
    uint8_t cmdid;
    uint8_t type;
    uint8_t text[20];
} cmd_lcd_display_t;

typedef struct {
    uint8_t cmdid;
    uint8_t type;
    uint8_t mode;
    uint8_t data[2];
} cmd_setup_config_t;

typedef union {
    cmd_common_t          cmdCommon;
    cmd_button_state_t    buttonState;
    cmd_buzzer_state_t    buzzerState;
    cmd_lcd_display_t     lcdDisplay;
    cmd_setup_config_t    setConfig;
} cmd_receive_t, *cmd_receive_p;

typedef void (* rs232_receiver_handle_event)(uint8_t lenPayload, uint8_t *payload);
/* -------------------------------TRANSMITER-----------------------------------
 * Definition state transmitter and fields of frame
 * ---------------------------------------------------------------------------*/
/*! @brief Field type */
#define CMD_TYPE_GET                            0x00
#define CMD_TYPE_RES                            0x01
#define CMD_TYPE_SET                            0x02

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/**
 * @func   RS232_Init
 * @brief  Initialize UART and buffer
 * @param  None
 * @retval None
 */
void RS232_Init(rs232_receiver_handle_event pReceiverCallback);

/**
 * @func   RS232_ReceiverProcedure
 * @brief  Process data received from uart
 * @param  None
 * @retval None
 */
void RS232_ReceiverProcedure(void);

/**
 * @func   RS232_SendPacket
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
	uint8_t byLengthPayload
);

#endif

/* END FILE */
