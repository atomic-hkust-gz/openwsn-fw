/**
\brief This program shows the use of the "spi" bsp module.

Since the bsp modules for different platforms have the same declaration, you
can use this project with any platform.

This program was written to communicate with the AT86RF231 radio chip. It will
run regardless of your radio, but might not return anything useful.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2014.
*/

#include "stdint.h"
#include "board.h"
#include "spi.h"

//=========================== defines =========================================

//=========================== variables =======================================

typedef struct {
   uint8_t    txBuf[5];
   uint8_t    rxBuf[5];
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
   
   memset(&app_vars,0,sizeof(app_vars));
   
   // initialize  
   board_init();

   // prepare buffer to send over SPI
   app_vars.txBuf[0]     =  0x1D;           // Read register LLCC68
   app_vars.txBuf[1]     =  0x06;           // NODEADDRESS MSB
   app_vars.txBuf[2]     =  0xCD;           // NODEADDRESS LSB
   app_vars.txBuf[3]     =  0x00;           // dummy
   app_vars.txBuf[4]     =  0x00;           // dummy
   
   // Expected rxBuf
   // rxBuf[0] = 0xa1                       // status: Mode = standby
   // rxBuf[1] = 0xa1                       
   // rxBuf[2] = 0xa1
   // rxBuf[3] = 0x00                       // deafult node address = 0x00   

   while(1) {
      spi_txrx(
         app_vars.txBuf,
         sizeof(app_vars.txBuf),
         SPI_BUFFER,
         app_vars.rxBuf,
         sizeof(app_vars.rxBuf),
         SPI_FIRST,
         SPI_LAST
      );
   }
}
