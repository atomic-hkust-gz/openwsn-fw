/**
\brief This program shows the use of the "llcc68" bsp module.

\author Tengfei Chang <tengfeichang@hkust-gz.edu.cn>, August 2025.
*/

#include "stdint.h"
#include "board.h"
#include "spi.h"
#include "llcc68.h"

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
   
   // retrieve radio manufacturer ID over SPI
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
