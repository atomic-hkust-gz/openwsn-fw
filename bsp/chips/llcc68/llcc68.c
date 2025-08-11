/**
\brief LLCC68-specific library.

\author Tengfei Chang <tengfeichang@hkust-gz.edu.cn>, August 2025.

*/

#include "board.h"
#include "llcc68.h"
#include "spi.h"
#include "debugpins.h"
#include "leds.h"
#include "radio.h"

// ========================== define ==========================================

#define DEVICE_ERRORS_LEN 5
#define LEN_OPCODE        1
#define LEN_REG           2
#define MAXIMUM_LEN       256

// ========================== prototype =======================================

void llcc68_spiWriteReg(uint16_t reg, uint8_t regValueToWrite){

    uint8_t spi_rx_buffer[LEN_OPCODE+LEN_REG+1];
    uint8_t spi_tx_buffer[LEN_OPCODE+LEN_REG+1];

    spi_tx_buffer[0]     = WRITEREGISTER;                   // OPCODE: write register
    spi_tx_buffer[1]     = (uint8_t)((reg >> 8) & 0xFF);    // High byte of reg
    spi_tx_buffer[2]     = (uint8_t)(reg & 0xFF);           // Low byte of reg
    spi_tx_buffer[3]     = regValueToWrite;                 // Value to write

    spi_txrx(
        spi_tx_buffer,              // bufTx
        LEN_OPCODE+LEN_REG+1,       // lenbufTx
        SPI_FIRSTBYTE,              // returnType
        spi_rx_buffer,              // bufRx
        LEN_OPCODE+LEN_REG+1,       // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );
}

uint8_t llcc68_spiReadReg(uint16_t reg){

    uint8_t spi_rx_buffer[LEN_OPCODE+LEN_REG+1];
    uint8_t spi_tx_buffer[LEN_OPCODE+LEN_REG+1];

    spi_tx_buffer[0]     = READREGISTER;                    // OPCODE: read register
    spi_tx_buffer[1]     = (uint8_t)((reg >> 8) & 0xFF);    // High byte of reg
    spi_tx_buffer[2]     = (uint8_t)(reg & 0xFF);           // Low byte of reg
    spi_tx_buffer[3]     = 0;                  // Value to read

    spi_txrx(
        spi_tx_buffer,              // bufTx
        LEN_OPCODE+LEN_REG+1,       // lenbufTx
        SPI_FIRSTBYTE,              // returnType
        spi_rx_buffer,              // bufRx
        LEN_OPCODE+LEN_REG+1,       // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );
    return spi_rx_buffer[3];
}

void llcc68_multipleBytesRead(uint16_t reg, uint8_t* buffer, uint8_t len){

    uint8_t spi_rx_buffer[MAXIMUM_LEN];
    uint8_t spi_tx_buffer[MAXIMUM_LEN];
    
    spi_tx_buffer[0]     = READBUFFER;                     // OPCODE: read buffer
    spi_tx_buffer[1]     = (uint8_t)((reg >> 8) & 0xFF);    // High byte of reg
    spi_tx_buffer[2]     = (uint8_t)(reg & 0xFF);           // Low byte of reg

    spi_txrx(
        spi_tx_buffer,              // bufTx
        LEN_OPCODE+LEN_REG+len,     // lenbufTx
        SPI_FIRSTBYTE,              // returnType
        spi_rx_buffer,              // bufRx
        LEN_OPCODE+LEN_REG+len,     // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );

    memcpy(buffer, &spi_rx_buffer[3], len);
}

void llcc68_multipleBytesWrite(uint16_t reg, uint8_t* buffer, uint8_t len) {

    uint8_t spi_rx_buffer[MAXIMUM_LEN];
    uint8_t spi_tx_buffer[MAXIMUM_LEN];
    
    spi_tx_buffer[0]     = WRITEBUFFER;                     // OPCODE: write buffer
    spi_tx_buffer[1]     = (uint8_t)((reg >> 8) & 0xFF);    // High byte of reg
    spi_tx_buffer[2]     = (uint8_t)(reg & 0xFF);           // Low byte of reg
    memcpy(&spi_tx_buffer[3], buffer, len);

    spi_txrx(
        spi_tx_buffer,              // bufTx
        LEN_OPCODE+LEN_REG+len,     // lenbufTx
        SPI_FIRSTBYTE,              // returnType
        spi_rx_buffer,              // bufRx
        LEN_OPCODE+LEN_REG+len,     // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );
}


// Set using OPCODE

void llcc68_noAddress_opcode(uint8_t opcode, type_t rw, uint8_t* buffer, uint8_t len){

    uint8_t spi_rx_buffer[MAXIMUM_LEN];
    uint8_t spi_tx_buffer[MAXIMUM_LEN];
    
    spi_tx_buffer[0]     = opcode;  // OPCODE

    switch(rw){
    case TYPE_READ:
        spi_txrx(
            spi_tx_buffer,              // bufTx
            len+LEN_OPCODE,             // lenbufTx
            SPI_FIRSTBYTE,              // returnType
            spi_rx_buffer,              // bufRx
            len+LEN_OPCODE,             // maxLenBufRx
            SPI_FIRST,                  // isFirst
            SPI_LAST                    // isLast
        );
        memcpy(buffer, &spi_rx_buffer[LEN_OPCODE], len);
    break;
    case TYPE_WRITE:
        memcpy(&spi_tx_buffer[LEN_OPCODE], buffer, len);
        spi_txrx(
            spi_tx_buffer,              // bufTx
            len+LEN_OPCODE,             // lenbufTx
            SPI_FIRSTBYTE,              // returnType
            spi_rx_buffer,              // bufRx
            len+LEN_OPCODE,             // maxLenBufRx
            SPI_FIRST,                  // isFirst
            SPI_LAST                    // isLast
        );
    break;
    }
}
