/*******************************************************************************
 *
 * Copyright (c) 2020
 * Lumi, JSC.
 * All Rights Reserved
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
#ifndef _EVENT_BUTTON_H_
#define _EVENT_BUTTON_H_ 
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
/*! @brief Button state */
#define BUTTON_PRESSED_1_TIME       0x01u
#define BUTTON_PRESSED_2_TIMES      0x02u
#define BUTTON_PRESSED_3_TIMES      0x03u
#define BUTTON_PRESSED_4_TIMES      0x04u
#define BUTTON_PRESSED_5_TIMES      0x05u
#define BUTTON_IS_RELEASED          0x10u
#define BUTTON_IS_HELD_DOWN         0x11u
#define BUTTON_IS_HOLD_3s           0x12u
#define BUTTON_IS_HOLD_5s           0x13u
#define BUTTON_IS_HOLD_7s           0x14u
#define BUTTON_IS_HOLD_10s          0x15u
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
 * @func   EventButton_Init
 * @brief  Initializes events of button
 * @param  None
 * @retval None
 */
void
EventButton_Init(void);

#endif

/* END FILE */
