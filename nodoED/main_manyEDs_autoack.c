#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "nwk_pll.h"

#include "PGN_protocol.h"

#include "bsp_sensors.h"
#include "bsp_digio.h"

static void linkTo(void);

void toggleLED(uint8_t);

//static uint8_t tx_buf_rf[NUM_CONNECTIONS];
static uint8_t tx_buf_rf[10];
static uint8_t byteCountRf = 0;

void rf_processMessage(uint8_t * code, uint8_t length);
//void processReceivedFrameRf(uint8_t * buf, uint8_t len)

static uint8_t description[NUMBER_OF_SENSORS + REST_BYTES];

static uint8_t rf_txMessage[MAX_APP_PAYLOAD];


/* Callback handler */
static uint8_t sRxCallback(linkID_t);

static volatile uint8_t  sPeerFrameSem = 0;
static          linkID_t sLinkID1 = 0;  /*  Access Point Link ID*/
//static          linkID_t sLinkID1 = 1;  /*  Access Point Link ID*/
static volatile uint8_t  sSemaphore = 0;
static volatile uint8_t  nameED = 0xB3; /* Nombre del nodo*/

#define SPIN_ABOUT_A_SECOND   NWK_DELAY(1000)
#define SPIN_ABOUT_A_QUARTER_SECOND   NWK_DELAY(250)

/* How many times to try a Tx and miss an acknowledge before doing a scan */
#define MISSES_IN_A_ROW  2

void main (void)
{
  BSP_Init();
  /*  Initialize node's description */
//  initNode(description);

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

  /* Keep trying to join (a side effect of successful initialization) until
   * successful. Toggle LEDS to indicate that joining has not occurred.
   */
  while (SMPL_SUCCESS != SMPL_Init(sRxCallback))
  {
    toggleLED(1);
    toggleLED(2);
    SPIN_ABOUT_A_SECOND; /* calls nwk_pllBackgrounder for us */
  }

  /* LEDs on solid to indicate successful join. */
  if (!BSP_LED2_IS_ON())
  {
    toggleLED(2);
  }
  if (!BSP_LED1_IS_ON())
  {
    toggleLED(1);
  }


  /* Unconditional link to AP which is listening due to successful join. */
  linkTo();

  while (1)
    FHSS_ACTIVE( nwk_pllBackgrounder( false ) );
}



static void linkTo()
{


  /* Keep trying to link... */
  while (SMPL_SUCCESS != SMPL_Link(&sLinkID1))
  {
    toggleLED(1);
    toggleLED(2);
    SPIN_ABOUT_A_SECOND; /* calls nwk_pllBackgrounder for us */
  }

  /* we're linked. turn off red LED. received messages will toggle the green LED. */
  if (BSP_LED2_IS_ON())
  {
    toggleLED(2);
  }
  if (!BSP_LED1_IS_ON())
  {
    toggleLED(1);
  }

  /* turn on RX. default is RX off. */
  SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);


  while (1)
  {

  if(sSemaphore>0){

    sSemaphore--;
//    rf_processMessage(sSemaphore,1);
    sSemaphore = 0;

    }
    
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


/* handle received messages */
static uint8_t sRxCallback(linkID_t port)
{
  uint8_t msg[MAX_APP_PAYLOAD], len;
  /* is the callback for the link ID we want to handle? */
  if (port == sLinkID1)
  {


    /* yes. go get the frame. we know this call will succeed. */
     if((SMPL_SUCCESS == SMPL_Receive(sLinkID1, msg, &len)) && len){
     
      /*  Process the received frame, which is only a 1-byte command...  */
      /*  Store this byte in the flag  */
       rf_processMessage(msg,len);
      sSemaphore = msg[0]+1;    
      
    }

    /* Post to the semaphore to let application know so it processes 
     * and sends the reply
     */

    return 1;
  }
  return 0;
}


static void rf_processMessage( uint8_t *msg, uint8_t len)
{
  linkID_t lid = sLinkID1;
  uint8_t i;
  int temperature, humidity;
  uint8_t temp_data[2];
  uint8_t hum_data[2];

  switch(msg[3])
    {
      
       case  GET_ED_ID:
                 byteCountRf  = len+1;
                 tx_buf_rf[0] = byteCountRf - 3;
                 tx_buf_rf[1] = lid;
                 tx_buf_rf[2] = RESPONSE_FRAME;
                 tx_buf_rf[3] = GET_ED_ID;
                 tx_buf_rf[4] = nameED;
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

