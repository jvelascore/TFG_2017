//******************************************************************************
//   MSP430F5529LP:  simpleUsbBackchannel example
//
//   Description: 	Demonstrates simple sending over USB, as well as the F5529's
//                      backchannel UART.
//
//   Texas Instruments Inc.
//   August 2013
//******************************************************************************

// Basic MSP430 and driverLib #includes
#include "msp430.h"
#include "driverlib/MSP430F5xx_6xx/wdt_a.h"
#include "driverlib/MSP430F5xx_6xx/ucs.h"
#include "driverlib/MSP430F5xx_6xx/pmm.h"
#include "driverlib/MSP430F5xx_6xx/sfr.h"

#include "types.h"

// Application #includes
#include "BCUart.h"           // Include the backchannel UART "library"
#include "hal.h"              // Modify hal.h to select your hardware
#include "PGN_protocol.h"     // PGN protocol codes and defines

// Prototypes
void processReceivedFrameUART(uint8_t * buf, uint8_t len);
//void *copy_buffer( void *dst, const void *src, UINT len);

void processReceivedFrameRf(uint8_t * buf, uint8_t len);

// Hardcoded responses
//const BYTE rsp_get_node_list[DEFAULT_RSP_GET_NODE_LIST_LEN] = DEFAULT_RSP_GET_NODE_LIST;
//const BYTE rsp_toggle_led[DEFAULT_RSP_TOGGLE_LED_LEN] = DEFAULT_RSP_TOGGLE_LED;

// Global variables uart
WORD rxByteCount;                        // Momentarily stores the number of bytes received
BYTE rx_buf_bcuart[BC_RXBUF_SIZE];       // Same size as the UART's rcv buffer
BYTE tx_buf_bcuart[BC_RXBUF_SIZE];
// Global variables rf
WORD txByteCountRf;
WORD rxByteCountRf;
static volatile uint8_t  contTX = 0;
BYTE rx_buf_rf[BC_RXBUF_SIZE];
BYTE tx_buf_rf[BC_RXBUF_SIZE];
//BYTE ID_red[BC_RXBUF_SIZE];

static uint8_t  numNodeRed = 0;
static uint8_t IDsRed[NUM_CONNECTIONS] = {0};
/*----------------------------------------------------------------------------
 *  Demo Application for SimpliciTI
 *
 *  L. Friedman
 *  Texas Instruments, Inc.
 *---------------------------------------------------------------------------- */

#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "nwk_pll.h"

#include "bsp_sensors.h"
#include "bsp_digio.h"

static void linkTo(void);

void toggleLED(uint8_t);

/* Callback handler */
static uint8_t sRxCallback(linkID_t);

static volatile uint8_t  sPeerFrameSem = 0;
static          linkID_t sLinkID1 = 0;  /*  Access Point Link ID*/
static volatile uint8_t  sSemaphore = 0;

#define SPIN_ABOUT_A_SECOND   NWK_DELAY(1000)
#define SPIN_ABOUT_A_QUARTER_SECOND   NWK_DELAY(250)

/* How many times to try a Tx and miss an acknowledge before doing a scan */
#define MISSES_IN_A_ROW  2

void main (void)
{
    BSP_Init();
    WDTCTL = WDTPW + WDTHOLD;		           // Halt the dog, detener el perro
                                                   // MSP430 USB requires a Vcore setting of at least 2.  2 is high enough
                                                   // for 8MHz MCLK, below.
    PMM_setVCore(PMM_BASE, PMM_CORE_LEVEL_2);
    //initPorts();                                 // Config all the GPIOS for low-power (output low)
    initClocks(8000000);                           // Config clocks. MCLK=SMCLK=FLL=8MHz; ACLK=REFO=32kHz
    bcUartInitPGN();                               // Init the back-channel UART for PGN (9600 bps).
    __enable_interrupt();                          // Enable interrupts globally
  
  
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
  uint8_t msg[MAX_APP_PAYLOAD], len, i;
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
       // Look for rcv'ed command on the backchannel UART. If any, process.
       rxByteCount = bcUartReadCommandTimeout(rx_buf_bcuart);
       //rxByteCount = bcUartReadCommand(rx_buf_bcuart);
       //rxByteCount = bcUartReceiveBytesInBuffer(rx_buf_bcuart);
       if(rxByteCount != -1){     
         //toggleLED(1);
         processReceivedFrameUART(rx_buf_bcuart, rxByteCount);
       }
       else // Timeout
       {
         // Do other things
         //tx_buf_bcuart[0] = 0xCC;
         //tx_buf_bcuart[1] = 0xDD;
         //bcUartSend(tx_buf_bcuart, 2);
       }
     
  if(sSemaphore>0){
     sSemaphore--;       
//     rf_processMessage(sSemaphore,1);
     sSemaphore = 0;
    }
  }
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
      processReceivedFrameRf(msg, len);       
      //Lo dejo, pero igual no haría falta
      sSemaphore = msg[0]+1;    
       
     }
    
    /* Post to the semaphore to let application know so it processes 
     * and sends the reply
     */

    return 1;
  }
  return 0;
}

// Process received frame from radio and send the appropiate answer
void processReceivedFrameRf(uint8_t * buf, uint8_t len)
{    uint8_t i;

      switch(buf[2])  // 3rd byte corresponds to the 1st byte of the code
      {
      case RESPONSE_FRAME:
          bcUartSend(buf, len);
      case ACK_FRAME:
        // Do something
        break;
      default:
        // Do nothing
        break;
      }      
}

// Process received frame from UART and send the appropiate answer
void processReceivedFrameUART(uint8_t * buf, uint8_t len)
{
    uint8_t i;
    // 1st byte is the length not counting the two bytes of the EOF and
    // the length byte itself. Check
    if (buf[0] == len-3)
    {
      switch(buf[2])  // 3rd byte corresponds to the 1st byte of the code
      {
        
      case REQUEST_FRAME:
       
          for (i = 0; i < len; i++){
            tx_buf_rf[i] = buf[i];
            contTX++;
            }
          SMPL_Send(sLinkID1, tx_buf_rf, contTX);
            contTX = 0;
           toggleLED(1);
           toggleLED(2);
          break;
        
      case ACK_FRAME:
        // Do something
        break;
        
      default:
        // Do nothing
        break;
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

