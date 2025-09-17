/**
\brief LLCC68-specific library.

\author Tengfei Chang <tengfeichang@hkust-gz.edu.cn>, August 2025.

*/

#include "board.h"
#include "nrf52840.h"
#include "llcc68.h"
#include "spi.h"
#include "debugpins.h"
#include "leds.h"
#include "radio.h"


//========================== define ===========================================

#define DEVICE_ERRORS_LEN 5
#define LEN_OPCODE        1
#define LEN_OFFSET        1
#define LEN_STATUS        1
#define LEN_REG           2
//#define MAX_BUFFER_SIZE   255
#define MAX_BUFFER_SIZE   128

#define LLCC68_BUSY_PIN   NRF_GPIO_PIN_MAP(1,7)   // P1.07
#define LLCC68_MAX_TSW    15                      // 600 ns @64MHz cpu clock
//=========================== variables =======================================

typedef struct { 
    llcc68_busy_t busy;
}llcc68_var_t;

llcc68_var_t llcc68_var;
//========================== prototype ========================================

static inline void llcc68_busy_wait(void);

//=========================== public ==========================================

void llcc68_spiWriteReg(uint16_t reg, uint8_t regValueToWrite) {

    uint8_t spi_rx_buffer[LEN_OPCODE+LEN_REG+1];
    uint8_t spi_tx_buffer[LEN_OPCODE+LEN_REG+1];

    spi_tx_buffer[0]     = WRITEREGISTER;                   // OPCODE: write register
    spi_tx_buffer[1]     = (uint8_t)((reg >> 8) & 0xFF);    // High byte of reg
    spi_tx_buffer[2]     = (uint8_t)(reg & 0xFF);           // Low byte of reg
    spi_tx_buffer[3]     = regValueToWrite;                 // Value to write
    
    llcc68_busy_wait();

    spi_txrx(
        spi_tx_buffer,                      // bufTx
        LEN_OPCODE+LEN_REG+1,               // lenbufTx
        SPI_FIRSTBYTE,                      // returnType
        spi_rx_buffer,                      // bufRx
        LEN_OPCODE+LEN_REG+1,               // maxLenBufRx
        SPI_FIRST,                          // isFirst
        SPI_LAST                            // isLast
    );
}

uint8_t llcc68_spiReadReg(uint16_t reg) {

    uint8_t spi_rx_buffer[LEN_OPCODE+LEN_REG+LEN_STATUS+1];
    uint8_t spi_tx_buffer[LEN_OPCODE+LEN_REG+LEN_STATUS+1];

    spi_tx_buffer[0]     = READREGISTER;                    // OPCODE: read register
    spi_tx_buffer[1]     = (uint8_t)((reg >> 8) & 0xFF);    // High byte of reg
    spi_tx_buffer[2]     = (uint8_t)(reg & 0xFF);           // Low byte of reg
    spi_tx_buffer[3]     = 0;                               // dummy byte (status)
    spi_tx_buffer[4]     = 0;                               // data byte expected here

    llcc68_busy_wait();

    spi_txrx(
        spi_tx_buffer,                      // bufTx
        LEN_OPCODE+LEN_REG+LEN_STATUS+1,    // lenbufTx
        SPI_FIRSTBYTE,                      // returnType
        spi_rx_buffer,                      // bufRx
        LEN_OPCODE+LEN_REG+LEN_STATUS+1,    // maxLenBufRx
        SPI_FIRST,                          // isFirst
        SPI_LAST                            // isLast
    );

    return spi_rx_buffer[4];
}

// offset: from the txStartBufferPointer
// default Tx base address = 0x00 (256 byte max)
void llcc68_txBufferWrite(uint8_t offset, uint8_t* buffer, uint8_t len) {
    
    uint8_t spi_rx_buffer[LEN_OPCODE+LEN_OFFSET+MAX_BUFFER_SIZE];
    uint8_t spi_tx_buffer[LEN_OPCODE+LEN_OFFSET+MAX_BUFFER_SIZE];

    if (len > MAX_BUFFER_SIZE) {
        len = MAX_BUFFER_SIZE;
    }

    if (len == 0) {
        return; 
    }

    spi_tx_buffer[0]     = WRITEBUFFER;     // OPCODE: write buffer
    spi_tx_buffer[1]     = offset;          // offset from buffer base address
    memcpy(&spi_tx_buffer[2], buffer, len);

    llcc68_busy_wait();

    spi_txrx(
        spi_tx_buffer,                      // bufTx
        LEN_OPCODE+LEN_OFFSET+len,          // lenbufTx
        SPI_FIRSTBYTE,                      // returnType
        spi_rx_buffer,                      // bufRx
        LEN_OPCODE+LEN_OFFSET+len,          // maxLenBufRx
        SPI_FIRST,                          // isFirst
        SPI_LAST                            // isLast
    );
}

// offset: from the rxStartBufferPointer
// default Rx base address = 0x00 (256 byte max)
void llcc68_rxBufferRead(uint8_t offset, uint8_t* buffer, uint8_t len) {
    
    uint8_t spi_rx_buffer[LEN_OPCODE+LEN_OFFSET+LEN_STATUS+MAX_BUFFER_SIZE];
    uint8_t spi_tx_buffer[LEN_OPCODE+LEN_OFFSET+LEN_STATUS+MAX_BUFFER_SIZE];

    if (len > MAX_BUFFER_SIZE) {
        len = MAX_BUFFER_SIZE;
    }

    if (len == 0) {
        return; 
    }
        
    spi_tx_buffer[0]     = READBUFFER;                      // OPCODE: read buffer
    spi_tx_buffer[1]     = offset;                          // offset from buffer base address
    memset(&spi_tx_buffer[2], 0x00, LEN_STATUS+len);        // dummy bytes

    llcc68_busy_wait();

    spi_txrx(
        spi_tx_buffer,                                      // bufTx
        LEN_OPCODE+LEN_OFFSET+LEN_STATUS+len,               // lenbufTx
        SPI_FIRSTBYTE,                                      // returnType
        spi_rx_buffer,                                      // bufRx
        LEN_OPCODE+LEN_OFFSET+LEN_STATUS+len,               // maxLenBufRx
        SPI_FIRST,                                          // isFirst
        SPI_LAST                                            // isLast
    );

    // spi_rx_buffer[2] = status byte
    // spi_rx_buffer[3] = start of data
    memcpy(buffer, &spi_rx_buffer[3], len);
}

void llcc68_noAddress_opcode(uint8_t opcode, 
                 type_t rw, 
                 uint8_t* buffer, 
                 uint8_t len) {

    uint8_t spi_rx_buffer[MAX_BUFFER_SIZE];
    uint8_t spi_tx_buffer[MAX_BUFFER_SIZE];
    
    // load OPCODE
    spi_tx_buffer[0]    = opcode;  

    switch(rw){
      case TYPE_READ:
          // dummy bytes
          memset(&spi_tx_buffer[LEN_OPCODE+LEN_STATUS], 0x00, LEN_STATUS+len);
          
          llcc68_busy_wait();
               
          spi_txrx(
              spi_tx_buffer,                  // bufTx
              LEN_OPCODE+LEN_STATUS+len,      // lenbufTx
              SPI_FIRSTBYTE,                  // returnType
              spi_rx_buffer,                  // bufRx
              LEN_OPCODE+LEN_STATUS+len,      // maxLenBufRx
              SPI_FIRST,                      // isFirst
              SPI_LAST                        // isLast
          );
          memcpy(buffer, &spi_rx_buffer[LEN_OPCODE+LEN_STATUS], len);
      break;
      case TYPE_WRITE:
          memcpy(&spi_tx_buffer[LEN_OPCODE], buffer, len);

          llcc68_busy_wait();

          spi_txrx(
              spi_tx_buffer,                  // bufTx
              LEN_OPCODE+len,                 // lenbufTx
              SPI_FIRSTBYTE,                  // returnType
              spi_rx_buffer,                  // bufRx
              LEN_OPCODE+len,                 // maxLenBufRx
              SPI_FIRST,                      // isFirst
              SPI_LAST                        // isLast
          ); 
      break;
    }
}

void llcc68_init(void){
    memset(&llcc68_var, 0, sizeof(llcc68_var));
    nrf_gpio_cfg_input(LLCC68_BUSY_PIN);
}

//=========================== private =========================================

static inline void llcc68_busy_wait(void){
    
    volatile uint32_t i; 
    uint32_t gpio_pin;
    
    // assuming port 1
    gpio_pin = LLCC68_BUSY_PIN & 0x1f; 
    
    // wait ~600 ns (T_sw)(64 MHz CPU)
    for (i = 0; i < LLCC68_MAX_TSW; i++) { __NOP(); }

    // wait for BUSY pin to go low
    do {
      llcc68_var.busy = (llcc68_busy_t)((NRF_P1->IN >> gpio_pin) & 1UL);
      // for debug
      if (llcc68_var.busy == LLCC68_BUSY){
        __NOP();
      }
    } while(llcc68_var.busy != LLCC68_FREE);
}

//=========================== callbacks =======================================

// BUSY pin is going low (falling edge) and ready to communicate 
//void llcc68_busy_cb(void){
//    llcc68_var.busy = LLCC68_FREE;
//}
