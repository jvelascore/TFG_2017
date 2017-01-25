/* 
 * ======== PGN_protocol.h ========
 */
#ifndef _PGN_PROTOCOL_H_
#define _PGN_PROTOCOL_H_

/*----------------------------------------------------------------------------+
 | Global defines                                                         |
 +----------------------------------------------------------------------------*/
#define FALSE                         0
#define TRUE                          1
#define PGN_ID                        0xAA
#define EOF_1                         0x0D
#define EOF_2                         0x0A
#define EOF                           0x0D0A
#define TIMEOUT_UART                  1000        // For a timeout of 50 ms, SMCLK=8MHz -> 80 = 8MHz/(2*0.050)

/*----------------------------------------------------------------------------+
 | Code BYTE 1                                                         |
 +----------------------------------------------------------------------------*/
#define REQUEST_FRAME         0x10          // Request frame from smartphone to PGN
#define RESPONSE_FRAME        0x08          // Response frame from PGN to smartphone
#define NUF_FRAME             0x04          // NUF frame from PGN to smartphone
#define ACK_FRAME             0x02          // ACK frame from smartphone to PGN

/*----------------------------------------------------------------------------+
 | Code BYTE 2   Peticiones del móvil a la red (y sus respuestas)      |
 +----------------------------------------------------------------------------*/
//manejo de la red
#define GET_NODE_LIST         0x01
#define GET_NUM_NODES         0x02
//sensores
#define GET_TEMPERATURE       0x08
#define GET_HUMIDITY          0x09
//leds
#define CLEAR_LED             0x10
#define SET_LED               0x11
#define TOGGLE_LED            0x12          
#define SET_ALL_LEDS          0x13
#define CLEAR_ALL_LEDS        0x14 
#define TOGGLE_ALL_LEDS       0x15

/*----------------------------------------------------------------------------+
 | Code BYTE 2   Mensajes NUF de la red al móvil                       |
 +----------------------------------------------------------------------------*/
//manejo de la red
#define NUF_NEW_NODE          0x01
//sensores
#define NUF_UPDATE_TEMP       0x08
#define NUF_UPDATE_HUM        0x09

/*----------------------------------------------------------------------------+
 | Default responses                                                   |
 +----------------------------------------------------------------------------*/
#define DEFAULT_RSP_GET_NODE_LIST_LEN 8
#define RSP_GET_NODE_LIST_LEN         0x05
#define DEFAULT_NUM_NODES             0x01
#define DEFAULT_RSP_GET_NODE_LIST     {RSP_GET_NODE_LIST_LEN,PGN_ID,RESPONSE_FRAME,GET_NODE_LIST,DEFAULT_NUM_NODES,0xBB,EOF_1,EOF_2}

#define DEFAULT_RSP_TOGGLE_LED_LEN    6
#define DEFAULT_RSP_TOGGLE_LED        {0x03,PGN_ID,RESPONSE_FRAME,TOGGLE_LED,EOF_1,EOF_2}


#endif  
/*
 * _PGN_PROTOCOL_H_
 *------------------------ Nothing Below This Line --------------------------
 */