/*******************************************************************************
 *
 * Copyright (c) 2018
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
 * Last Changed:     $Date: 9/15/18 16:30 $
 *
 ******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "led.h"
#include "simcom.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define TIMEOUT_RETRY_TRANSMIT    20000 //ms
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
const char apn[] = "v-internet"; // My Provider details - Viettel
const char server[] = "171.253.77.48"; // My domain
const int  port = 3000;
//const char publisher_resource[] = "/apiControlAC/getData";
//const char subscriber_resource[] = "/apiControlAC";
const char publisher_resource[] = "/getData";
const char subscriber_resource[] = "/search/show";
char ATcommand[80];
uint8_t buffer[512] = {0};
uint8_t ATisOK = 0;
uint8_t CGREGisOK = 0;
uint8_t CIPOPENisOK = 0;
uint8_t NETOPENisOK = 0;
uint32_t previousTick;

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static subscriber_handler_callback pSubscriberCb;
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
static void SIMCOM_TransmitAndReceive(char *cmd)
{
  memset(buffer, 0, sizeof(buffer));

  HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), 1000);
  printf("%s", (char *)cmd);

  HAL_UART_Receive(&huart1, buffer, sizeof(buffer), 1000);
  printf("%s", (char *)buffer);
}

void SIMCOM_Init(subscriber_handler_callback cb) {
	ATisOK = 0;
	CGREGisOK = 0;
	NETOPENisOK = 0;
	CIPOPENisOK = 0;

	if (cb != NULL) {
		pSubscriberCb = cb;
	}

//	SIMCOM_TransmitAndReceive("AT+CRESET\r\n");

	// Check for OK response for AT
	previousTick = HAL_GetTick();
	while (!ATisOK && previousTick + TIMEOUT_RETRY_TRANSMIT > HAL_GetTick()) {
		SIMCOM_TransmitAndReceive("AT\r\n");
		if (strstr((char *)buffer, "OK")) {
		    ATisOK = 1;
		}
	}

	// Check for network registration.
	if (ATisOK) {
		previousTick = HAL_GetTick();
		while (!CGREGisOK  && previousTick + TIMEOUT_RETRY_TRANSMIT > HAL_GetTick()) {
			SIMCOM_TransmitAndReceive("AT+CGREG?\r\n");
			if (strstr((char *)buffer, "+CGREG: 0,1")) {
				CGREGisOK = 1;
			}
		}
	}

    // Check for Internet IP Connection
	if (CGREGisOK) {
		previousTick =  HAL_GetTick();
		while(!NETOPENisOK  && previousTick  + TIMEOUT_RETRY_TRANSMIT >  HAL_GetTick()) {
			SIMCOM_TransmitAndReceive("AT+NETCLOSE\r\n");
			if(strstr((char *)buffer,"+NETCLOSE: 0") || strstr((char *)buffer,"+NETCLOSE: 2")) {
				sprintf(ATcommand,"AT+CGDCONT=1,\"IP\",\"%s\",\"0.0.0.0\",0,0\r\n",apn);
				SIMCOM_TransmitAndReceive(ATcommand);
				SIMCOM_TransmitAndReceive("AT+CIPRXGET=0\r\n");
				SIMCOM_TransmitAndReceive("AT+CIPMODE=0\r\n");
				SIMCOM_TransmitAndReceive("AT+CIPSENDMODE=0\r\n");
				SIMCOM_TransmitAndReceive("AT+CIPCCFG=10,0,0,0,0,0,120000\r\n");
				SIMCOM_TransmitAndReceive("AT+CIPTIMEOUT=120000,120000,120000\r\n");

				SIMCOM_TransmitAndReceive("AT+NETOPEN\r\n");
				previousTick =  HAL_GetTick();
				while (!NETOPENisOK  && previousTick  + TIMEOUT_RETRY_TRANSMIT >  HAL_GetTick()) {
					SIMCOM_TransmitAndReceive("AT+NETOPEN?\r\n");
					if(strstr((char *)buffer,"+NETOPEN: 1")) {
						NETOPENisOK = 1;
						led_event_show(LEDEV_NETWORK_JOINED);
					}
				}
			}

		}
	}
}

void SIMCOM_httpGet(void) {
	// If all Connection success (Wiring, Registration and TCP/IP)
	if (CGREGisOK && NETOPENisOK && !CIPOPENisOK) {
		// Perform http GET request
		sprintf(ATcommand,"AT+CIPOPEN=0,\"TCP\",\"%s\",%d\r\n",server,port);
		previousTick =  HAL_GetTick();
		while(previousTick  + TIMEOUT_RETRY_TRANSMIT >  HAL_GetTick()) {
			SIMCOM_TransmitAndReceive(ATcommand);
			if(strstr((char *)buffer,"+CIPOPEN: 0,0")) {
				break;
			}
		}

		sprintf(ATcommand,"AT+CIPSEND=0,%d\r\n",strlen(subscriber_resource)+15);
		SIMCOM_TransmitAndReceive(ATcommand);
		if(strstr((char *)buffer,">")) {
			sprintf(ATcommand,"GET %s HTTP/1.1\r\n",subscriber_resource);
			SIMCOM_TransmitAndReceive(ATcommand);
		}

		sprintf(ATcommand,"AT+CIPSEND=0,%d\r\n",strlen(server)+8);
		SIMCOM_TransmitAndReceive(ATcommand);
		if(strstr((char *)buffer,">")) {
			sprintf(ATcommand,"Host: %s\r\n",server);
			SIMCOM_TransmitAndReceive(ATcommand);
		}

//		SIMCOM_TransmitAndReceive("AT+CIPSEND=0,19\r\n");
//		if(strstr((char *)buffer,">")) {
//			SIMCOM_TransmitAndReceive("Connection: close\r\n");
//		}

		SIMCOM_TransmitAndReceive("AT+CIPSEND=0,2\r\n");
		if(strstr((char *)buffer,">")) {
			SIMCOM_TransmitAndReceive("\r\n");
		}

		// Clear the buffer before receiving
//		memset(buffer, 0, sizeof(buffer));
//		HAL_UART_Receive(&huart1, buffer, sizeof(buffer), 5000);

		// Find the "+IPD" substring in the received data
		int32_t length;
		char* ipdPtr = strstr((char *)buffer, "+IPD");
		if (ipdPtr != NULL) {
			ipdPtr += 4; // Skip the "+IPD" part
			length = atoi(ipdPtr); // Convert the following string to an integer
			char* dataPtr = strstr(ipdPtr, "\r\n\r\n");
			if (dataPtr != NULL) {
				dataPtr += 4; // Move past the "\r\n" sequence
			}

			if (pSubscriberCb != NULL) {
				pSubscriberCb((uint8_t *)dataPtr, length);
			}
		}

		// Close connections
		SIMCOM_TransmitAndReceive("AT+CIPCLOSE=0\r\n");
	}
}

void SIMCOM_httpPost(char *data) {
	// If all Connection success (Wiring, Registration and TCP/IP)
	if (CGREGisOK && NETOPENisOK && !CIPOPENisOK) {
		// Perform http POST request
		sprintf(ATcommand,"AT+CIPOPEN=0,\"TCP\",\"%s\",%d\r\n",server,port);
		previousTick =  HAL_GetTick();
		while(previousTick  + TIMEOUT_RETRY_TRANSMIT >  HAL_GetTick()) {
			SIMCOM_TransmitAndReceive(ATcommand);
			if(strstr((char *)buffer,"+CIPOPEN: 0,0")) {
				break;
			}
		}

		sprintf(ATcommand,"AT+CIPSEND=0,%d\r\n", strlen(publisher_resource)+16);
		SIMCOM_TransmitAndReceive(ATcommand);
		if(strstr((char *)buffer,">")) {
			sprintf(ATcommand,"POST %s HTTP/1.1\r\n", publisher_resource);
			SIMCOM_TransmitAndReceive(ATcommand);
		}

		sprintf(ATcommand,"AT+CIPSEND=0,%d\r\n",strlen(server)+8);
		SIMCOM_TransmitAndReceive(ATcommand);
		if(strstr((char *)buffer,">")) {
			sprintf(ATcommand,"Host: %s\r\n",server);
			SIMCOM_TransmitAndReceive(ATcommand);
		}

//		SIMCOM_TransmitAndReceive("AT+CIPSEND=0,19\r\n");
//		if(strstr((char *)buffer,">")) {
//			SIMCOM_TransmitAndReceive("Connection: close\r\n");
//		}

		SIMCOM_TransmitAndReceive("AT+CIPSEND=0,49\r\n");
		if(strstr((char *)buffer,">")) {
			SIMCOM_TransmitAndReceive("Content-Type: application/x-www-form-urlencoded\r\n");
		}

		SIMCOM_TransmitAndReceive("AT+CIPSEND=0,16\r\n");
		if(strstr((char *)buffer,">")) {
			SIMCOM_TransmitAndReceive("Content-Length: ");
		}

		char sLength[11];
		snprintf(sLength, 11, "%d", strlen(data));
		sprintf(ATcommand,"AT+CIPSEND=0,%d\r\n",strlen(sLength));
		SIMCOM_TransmitAndReceive(ATcommand);
		if(strstr((char *)buffer,">")) {
			SIMCOM_TransmitAndReceive(sLength);
		}

		SIMCOM_TransmitAndReceive("AT+CIPSEND=0,2\r\n");
		if(strstr((char *)buffer,">")) {
			SIMCOM_TransmitAndReceive("\r\n");
		}

		SIMCOM_TransmitAndReceive("AT+CIPSEND=0,2\r\n");
		if(strstr((char *)buffer,">")) {
			SIMCOM_TransmitAndReceive("\r\n");
		}

		sprintf(ATcommand,"AT+CIPSEND=0,%d\r\n",strlen(data));
		SIMCOM_TransmitAndReceive(ATcommand);
		if(strstr((char *)buffer,">")) {
			SIMCOM_TransmitAndReceive(data);
		}

//		SIMCOM_TransmitAndReceive("AT+CIPSEND=0,2\r\n");
//		if(strstr((char *)buffer,">")) {
//			SIMCOM_TransmitAndReceive("\r\n");
//		}

		// Close connections
		SIMCOM_TransmitAndReceive("AT+CIPCLOSE=0\r\n");
	}
}

/* END FILE */
