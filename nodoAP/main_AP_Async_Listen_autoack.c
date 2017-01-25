/*----------------------------------------------------------------------------
 *  Demo Application for SimpliciTI
 *
 *  L. Friedman
 *  Texas Instruments, Inc.
 *----------------------------------------------------------------------------
 */

/**********************************************************************************************
  Copyright 2007-2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS”
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/
#include <string.h>
#include "bsp.h"
#include "mrfi.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "nwk_pll.h"

#include "bsp_sensors.h"
#include "bsp_digio.h"

#include "PGN_protocol.h"

#ifndef APP_AUTO_ACK
#error ERROR: Must define the macro APP_AUTO_ACK for this application.
#endif

void toggleLED(uint8_t);

/**************************** COMMENTS ON ASYNC LISTEN APPLICATION ***********************
Summary:
  This AP build includes implementation of an unknown number of end device peers in
  addition to AP functionality. In this scenario all End Devices establish a link to
  the AP and only to the AP. The AP acts as a data hub. All End Device peers are on
  the AP and not on other distinct ED platforms.

  There is still a limit to the number of peers supported on the AP that is defined
  by the macro NUM_CONNECTIONS. The AP will support NUM_CONNECTIONS or fewer peers
  but the exact number does not need to be known at build time.

  In this special but common scenario SimpliciTI restricts each End Device object to a
  single connection to the AP. If multiple logical connections are required these must
  be accommodated by supporting contexts in the application payload itself.

Solution overview:
  When a new peer connection is required the AP main loop must be notified. In essence
  the main loop polls a semaphore to know whether to begin listening for a peer Link
  request from a new End Device. There are two solutions: automatic notification and
  external notification. The only difference between the automatic notification
  solution and the external notification solution is how the listen semaphore is
  set. In the external notification solution the sempahore is set by the user when the
  AP is stimulated for example by a button press or a commend over a serial link. In the
  automatic scheme the notification is accomplished as a side effect of a new End Device
  joining.

  The Rx callback must be implemented. When the callback is invoked with a non-zero
  Link ID the handler could set a semaphore that alerts the main work loop that a
  SMPL_Receive() can be executed successfully on that Link ID.

  If the callback conveys an argument (LinkID) of 0 then a new device has joined the
  network. A SMPL_LinkListen() should be executed.

  Whether the joining device supports ED objects is indirectly inferred on the joining
  device from the setting of the NUM_CONNECTIONS macro. The value of this macro should
  be non-zero only if ED objects exist on the device. This macro is always non-zero
  for ED-only devices. But Range Extenders may or may not support ED objects. The macro
  should be be set to 0 for REs that do not also support ED objects. This prevents the
  Access Point from reserving resources for a joinng device that does not support any
  End Device Objects and it prevents the AP from executing a SMPL_LinkListen(). The
  Access Point will not ever see a Link frame if the joining device does not support
  any connections.

  Each joining device must execute a SMPL_Link() after receiving the join reply from the
  Access Point. The Access Point will be listening.

*************************** END COMMENTS ON ASYNC LISTEN APPLICATION ********************/

/************  THIS SOURCE FILE REPRESENTS THE AUTOMATIC NOTIFICATION SOLUTION ************/

/* reserve space for the maximum possible peer Link IDs */
static linkID_t sLID[NUM_CONNECTIONS] = {0};
static uint8_t  sNumCurrentPeers = 0;
static uint8_t  numNode = 0;

static linkID_t sLinkPGN;  /*  PGN Link ID*/
static linkID_t sLinkAP  = 0x00;  /*  AP Link ID*/
static uint8_t  nameAP = 0xC1; /*  Name AP device*/

//Guardamos la ID que el AP asigna al ED que se acaba de conectar a la red
static linkID_t endDevicePeerID[NUM_CONNECTIONS] = {0};
static uint8_t endDsIDs[NUM_CONNECTIONS];

//Global variables rf 
//static volatile uint8_t byteCountRf = 0;
static uint8_t byteCountRf = 0;
static uint8_t rx_buf_rf[NUM_CONNECTIONS];
//static uint8_t tx_buf_rf[NUM_CONNECTIONS];
static uint8_t tx_buf_rf[10];

/* callback handler */
static uint8_t sCB(linkID_t);

/* received message handler */
static void processMessage(linkID_t, uint8_t *, uint8_t);

/* Frequency Agility helper functions */
static void    checkChangeChannel(void);
static void    changeChannel(void);

/* work loop semaphores */
static volatile uint8_t sPeerFrameSem = 0;
static volatile uint8_t sJoinSem = 0;

#ifdef FREQUENCY_AGILITY
/*     ************** BEGIN interference detection support */

#define INTERFERNCE_THRESHOLD_DBM (-70)
#define SSIZE    25
#define IN_A_ROW  3
static int8_t  sSample[SSIZE];
static uint8_t sChannel = 0;
#endif  /* FREQUENCY_AGILITY */

/* blink LEDs when channel changes... */
static volatile uint8_t sBlinky = 0;

/*     ************** END interference detection support                       */

#define SPIN_ABOUT_A_QUARTER_SECOND   NWK_DELAY(250)

void main (void)
{
  bspIState_t intState;

#ifdef FREQUENCY_AGILITY
  memset(sSample, 0x0, sizeof(sSample));
#endif
  
  BSP_Init();

  /* If an on-the-fly device address is generated it must be done before the
   * call to SMPL_Init(). If the address is set here the ROM value will not
   * be used. If SMPL_Init() runs before this IOCTL is used the IOCTL call
   * will not take effect. One shot only. The IOCTL call below is conformal.
   */
#ifdef I_WANT_TO_CHANGE_DEFAULT_ROM_DEVICE_ADDRESS_PSEUDO_CODE
  {
    addr_t lAddr;

    createRandomAddress(&lAddr);
    SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &lAddr);
  }
#endif /* I_WANT_TO_CHANGE_DEFAULT_ROM_DEVICE_ADDRESS_PSEUDO_CODE */

  SMPL_Init(sCB);

  /* green and red LEDs on solid to indicate waiting for a Join. */
  if (!BSP_LED2_IS_ON())
  {
    toggleLED(2);
  }
  if (!BSP_LED1_IS_ON())
  {
    toggleLED(1);
  }
  
  /* main work loop */
  while (1)
  {
    /* manage FHSS schedule if FHSS is active */
    FHSS_ACTIVE( nwk_pllBackgrounder( false ) );
    
    /* Wait for the Join semaphore to be set by the receipt of a Join frame from a
     * device that supports an End Device.
     *
     * An external method could be used as well. A button press could be connected
     * to an ISR and the ISR could set a semaphore that is checked by a function
     * call here, or a command shell running in support of a serial connection
     * could set a semaphore that is checked by a function call.
     */ 
           
    if (sJoinSem && (sNumCurrentPeers < NUM_CONNECTIONS))   
    {
      /* listen for a new connection */
      while (1)
      {
        if (SMPL_SUCCESS == SMPL_LinkListen(&sLID[sNumCurrentPeers]))
        {
          break;
        }
        /* Implement fail-to-link policy here. otherwise, listen again. */
      }    
      toggleLED(1);
      toggleLED(2);
      //Guardamos el identificador ID asignado los dispositivos conectados hasta este momento   
      endDsIDs[sNumCurrentPeers+1] = sLID[sNumCurrentPeers];
      sNumCurrentPeers++;
      numNode = sNumCurrentPeers;
      //Guardamos el número de dispositivos conectados en la red hasta este momento
      endDsIDs[0] = numNode;
      
      BSP_ENTER_CRITICAL_SECTION(intState);
      sJoinSem--;
      BSP_EXIT_CRITICAL_SECTION(intState);
    }

    /* Have we received a frame on one of the ED connections?
     * No critical section -- it doesn't really matter much if we miss a poll
     */
    if (sPeerFrameSem)
    {
      uint8_t     msg[MAX_APP_PAYLOAD], len, i;
      /* process all frames waiting */
      for (i=0; i<sNumCurrentPeers; ++i)
      {
        if (SMPL_SUCCESS == SMPL_Receive(sLID[i], msg, &len))
        {
          if((msg[2] == REQUEST_FRAME) && (msg[1] == SLINK_AP)){
          processMessage(sLID[i], msg, len);
          }
          if(msg[2] == RESPONSE_FRAME){
          SMPL_Send(sLinkPGN, msg, len);  
          }  
          else{
          //SMPL_Send(0x01, msg, len);
          //SMPL_Send(sLID[i], msg, len);
          SMPL_Send(msg[1], msg, len);
          //toggleLED(1);
          }
          BSP_ENTER_CRITICAL_SECTION(intState);
          sPeerFrameSem--;
          BSP_EXIT_CRITICAL_SECTION(intState);
        }
      }
    }
        
    if (BSP_BUTTON1())
    {
      SPIN_ABOUT_A_QUARTER_SECOND;  /* debounce */
      changeChannel();
    }
    else
    {
      checkChangeChannel();
    }
    BSP_ENTER_CRITICAL_SECTION(intState);
    if (sBlinky)
    {
      if (++sBlinky >= 0xF)
      {
        sBlinky = 1;
        toggleLED(1);
        toggleLED(2);
      }
    }
    BSP_EXIT_CRITICAL_SECTION(intState);
  }

}


static void processMessage(linkID_t lid, uint8_t *msg, uint8_t len)
{
  uint8_t i;
  int temperature, humidity;
  uint8_t temp_data[2];
  uint8_t hum_data[2];

    switch(msg[3])
    {
      
    case  GET_NUM_NODES:
                 byteCountRf  = len+1;
                 tx_buf_rf[0] = byteCountRf - 3;
                 tx_buf_rf[1] = lid;
                 tx_buf_rf[2] = RESPONSE_FRAME;
                 tx_buf_rf[3] = GET_NUM_NODES;
                 tx_buf_rf[4] = numNode;
                 tx_buf_rf[5] = 0x0D;
                 tx_buf_rf[6] = 0x0A;
                 SMPL_Send(lid, tx_buf_rf, byteCountRf);
                 byteCountRf = 0;
    break;
       
    case  GET_PGN_ID:
                 sLinkPGN = lid;
                 byteCountRf  = len+1;
                 tx_buf_rf[0] = byteCountRf - 3;
                 tx_buf_rf[1] = lid;
                 tx_buf_rf[2] = RESPONSE_FRAME;
                 tx_buf_rf[3] = GET_PGN_ID;
                 tx_buf_rf[4] = sLinkPGN;
                 tx_buf_rf[5] = 0x0D;
                 tx_buf_rf[6] = 0x0A;
                 SMPL_Send(lid, tx_buf_rf, byteCountRf);
                 byteCountRf = 0;
    break;
       
    case  GET_AP_ID:
                 byteCountRf  = len+1;
                 tx_buf_rf[0] = byteCountRf - 3;
                 tx_buf_rf[1] = lid;
                 tx_buf_rf[2] = RESPONSE_FRAME;
                 tx_buf_rf[3] = GET_AP_ID;
                 tx_buf_rf[4] = nameAP;
                 tx_buf_rf[5] = 0x0D;
                 tx_buf_rf[6] = 0x0A;
                 SMPL_Send(lid, tx_buf_rf, byteCountRf);
                 byteCountRf = 0;
       break;
    
    case  GET_TEMPERATURE:
                temperature = SHT75_medirTemperatura();
                temp_data[0] = (temperature>>8) & 0xff; // most significant part first
                temp_data[1] = temperature & 0xff;
                byteCountRf  = len+2;
                tx_buf_rf[0] = byteCountRf - 3;
                tx_buf_rf[1] = lid;
                tx_buf_rf[2] = RESPONSE_FRAME;
                tx_buf_rf[3] = GET_TEMPERATURE;
                tx_buf_rf[4] = temp_data[0];
                tx_buf_rf[5] = temp_data[1];
                tx_buf_rf[6] = 0x0D;
                tx_buf_rf[7] = 0x0A;
                SMPL_Send(lid, tx_buf_rf, byteCountRf);
                byteCountRf = 0;
    break;
       
    case  GET_HUMIDITY:
                humidity = SHT75_medirHumedad();
                hum_data[0] = (humidity>>8) & 0xff; // most significant part first
                hum_data[1] = humidity & 0xff;
                byteCountRf  = len+2;
                tx_buf_rf[0] = byteCountRf - 3;
                tx_buf_rf[1] = lid;
                tx_buf_rf[2] = RESPONSE_FRAME;
                tx_buf_rf[3] = GET_TEMPERATURE;
                tx_buf_rf[4] = hum_data[0];
                tx_buf_rf[5] = hum_data[1];
                tx_buf_rf[6] = 0x0D;
                tx_buf_rf[7] = 0x0A;
                SMPL_Send(lid, tx_buf_rf, byteCountRf);
                byteCountRf = 0;
    break;
       
    case GET_NODE_LIST:
                byteCountRf = numNode+len+1;
                tx_buf_rf[0]= byteCountRf - 3;
                tx_buf_rf[1]= lid;
                tx_buf_rf[2]= RESPONSE_FRAME;
                tx_buf_rf[3]= GET_NODE_LIST;
                tx_buf_rf[4]= numNode;
                for(i=0; i<numNode; i++){
                     tx_buf_rf[i+5]=endDsIDs[i+1];
                   }  
                tx_buf_rf[4+numNode+1]= 0x0D;
                tx_buf_rf[4+numNode+2]= 0x0A; 
                SMPL_Send(lid, tx_buf_rf, byteCountRf);
                byteCountRf = 0;
    break;
            
    case  TOGGLE_LED:
                 byteCountRf  = len-1;
                 tx_buf_rf[0] = byteCountRf - 3;
                 tx_buf_rf[1] = lid;
                 tx_buf_rf[2] = RESPONSE_FRAME;
                 tx_buf_rf[3] = TOGGLE_LED;
                 tx_buf_rf[4] = 0x0D;
                 tx_buf_rf[5] = 0x0A;
                 SMPL_Send(lid, tx_buf_rf, byteCountRf);
                 byteCountRf = 0;
          
              switch(msg[4])
              {
              case TOGGLE_LED_1:
                  toggleLED(1);
                break;
              case TOGGLE_LED_2:
                  toggleLED(2);
                break;
              default:
                break;        
              }
    break;
         
    case  CLEAR_LED:
                 byteCountRf  = len-1;
                 tx_buf_rf[0] = byteCountRf - 3;
                 tx_buf_rf[1] = lid;
                 tx_buf_rf[2] = RESPONSE_FRAME;
                 tx_buf_rf[3] = CLEAR_LED;
                 tx_buf_rf[4] = 0x0D;
                 tx_buf_rf[5] = 0x0A;
                 SMPL_Send(lid, tx_buf_rf, byteCountRf);
                 byteCountRf = 0;
             
              switch(msg[4])
              {
              case CLEAR_LED_1:
                   if (BSP_LED1_IS_ON())
                      {
                      toggleLED(1);
                      }
                break;
              case CLEAR_LED_2:
                  if (BSP_LED2_IS_ON())
                      {
                      toggleLED(2);
                      }
                break;
              default:
                break;
               }
    break;
         
    case  SET_LED:
                 byteCountRf  = len-1;
                 tx_buf_rf[0] = byteCountRf - 3;
                 tx_buf_rf[1] = lid;
                 tx_buf_rf[2] = RESPONSE_FRAME;
                 tx_buf_rf[3] = SET_LED;
                 tx_buf_rf[4] = 0x0D;
                 tx_buf_rf[5] = 0x0A;
                 SMPL_Send(lid, tx_buf_rf, byteCountRf);
                 byteCountRf = 0;
             
              switch(msg[4])
              {
              case SET_LED_1:
                   if (!BSP_LED1_IS_ON())
                      {
                      toggleLED(1);
                      }
                break;
              case SET_LED_2:
                  if (!BSP_LED2_IS_ON())
                      {
                      toggleLED(2);
                      }
                break;
              default:
                break;
               }
    break;
         
    case  SET_ALL_LEDS:
              if (!BSP_LED1_IS_ON())
                 {
                 toggleLED(1);
                 }
              if (!BSP_LED2_IS_ON())
                 {
                 toggleLED(2);
                 }
              byteCountRf  = len;
                 tx_buf_rf[0] = byteCountRf - 3;
                 tx_buf_rf[1] = lid;
                 tx_buf_rf[2] = RESPONSE_FRAME;
                 tx_buf_rf[3] = SET_ALL_LEDS;
                 tx_buf_rf[4] = 0x0D;
                 tx_buf_rf[5] = 0x0A;
                 SMPL_Send(lid, tx_buf_rf, byteCountRf);
                 byteCountRf = 0;
    break;
         
         case  CLEAR_ALL_LEDS:
              if (BSP_LED1_IS_ON())
                 {
                 toggleLED(1);
                 }
              if (BSP_LED2_IS_ON())
                 {
                 toggleLED(2);
                 }
              byteCountRf  = len;
                 tx_buf_rf[0] = byteCountRf - 3;
                 tx_buf_rf[1] = lid;
                 tx_buf_rf[2] = RESPONSE_FRAME;
                 tx_buf_rf[3] = CLEAR_ALL_LEDS;
                 tx_buf_rf[4] = 0x0D;
                 tx_buf_rf[5] = 0x0A;
                 SMPL_Send(lid, tx_buf_rf, byteCountRf);
                 byteCountRf = 0;
    break;
         
    case  TOGGLE_ALL_LEDS:
                 toggleLED(1);
                 toggleLED(2);
                 byteCountRf  = len;
                 tx_buf_rf[0] = byteCountRf - 3;
                 tx_buf_rf[1] = lid;
                 tx_buf_rf[2] = RESPONSE_FRAME;
                 tx_buf_rf[3] = TOGGLE_ALL_LEDS;
                 tx_buf_rf[4] = 0x0D;
                 tx_buf_rf[5] = 0x0A;
                 SMPL_Send(lid, tx_buf_rf, byteCountRf);
                 byteCountRf = 0;
    break;
                
    default:
    break;
    }        
}


void toggleLED(uint8_t which)
{
  if (1 == which)
  {
    BSP_TOGGLE_LED1();
  }
  else if (2 == which)
  {
    BSP_TOGGLE_LED2();
  }

  return;
}

/* Runs in ISR context. Reading the frame should be done in the */
/* application thread not in the ISR thread. */
static uint8_t sCB(linkID_t lid)
{
  if (lid)
  {
    sPeerFrameSem++;
    sBlinky = 0;
  }
  else
  {
    sJoinSem++;
  }

  /* leave frame to be read by application. */
  return 0;
}

static void changeChannel(void)
{
#ifdef FREQUENCY_AGILITY
  freqEntry_t freq;

  if (++sChannel >= NWK_FREQ_TBL_SIZE)
  {
    sChannel = 0;
  }
  freq.logicalChan = sChannel;
  SMPL_Ioctl(IOCTL_OBJ_FREQ, IOCTL_ACT_SET, &freq);
  BSP_TURN_OFF_LED1();
  BSP_TURN_OFF_LED2();
  sBlinky = 1;
#endif
  return;
}

/* implement auto-channel-change policy here... */
static void  checkChangeChannel(void)
{
#ifdef FREQUENCY_AGILITY
  int8_t dbm, inARow = 0;

  uint8_t i;

  memset(sSample, 0x0, SSIZE);
  for (i=0; i<SSIZE; ++i)
  {
    /* quit if we need to service an app frame */
    if (sPeerFrameSem || sJoinSem)
    {
      return;
    }
    NWK_DELAY(1);
    SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RSSI, (void *)&dbm);
    sSample[i] = dbm;

    if (dbm > INTERFERNCE_THRESHOLD_DBM)
    {
      if (++inARow == IN_A_ROW)
      {
        changeChannel();
        break;
      }
    }
    else
    {
      inARow = 0;
    }
  }
#endif
  return;
}

