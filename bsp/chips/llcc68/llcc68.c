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

#define DEVICE_ERRORS_LEN 5
#define REG_LEN 4

void llcc68_init(void){
  // Initialize to standby mode
  llcc68_mode_standby();

  // Set packet type to LoRa
  llcc68_opcode(SETPACKETTYPE, 0x01); // 0x00 = FSK mode, 0x01 = LoRa mode

  // Set RF Frequency
  //llcc68_opcode(SETRFFREQUENCY,);

  // Set power amplifier configuration
  // Set TX parameters
  // Set buffer base addresses
}

void llcc68_spiWriteReg(uint16_t reg, uint8_t regValueToWrite){
    uint8_t spi_rx_buffer[REG_LEN];
    uint8_t spi_tx_buffer[REG_LEN];

    spi_tx_buffer[0]     = WRITEREGISTER;                   // OPCODE: write register
    spi_tx_buffer[1]     = (uint8_t)((reg >> 8) & 0xFF);    // High byte of reg
    spi_tx_buffer[2]     = (uint8_t)(reg & 0xFF);           // Low byte of reg
    spi_tx_buffer[3]     = regValueToWrite;                 // Value to write

    spi_txrx(
        spi_tx_buffer,              // bufTx
        REG_LEN,                    // lenbufTx
        SPI_FIRSTBYTE,              // returnType
        spi_rx_buffer,              // bufRx
        REG_LEN,                    // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );
}

uint8_t llcc68_spiReadReg(uint16_t reg, uint8_t regValueToRead){
    uint8_t spi_rx_buffer[REG_LEN];
    uint8_t spi_tx_buffer[REG_LEN];

    spi_tx_buffer[0]     = READREGISTER;                    // OPCODE: read register
    spi_tx_buffer[1]     = (uint8_t)((reg >> 8) & 0xFF);    // High byte of reg
    spi_tx_buffer[2]     = (uint8_t)(reg & 0xFF);           // Low byte of reg
    spi_tx_buffer[3]     = regValueToRead;                  // Value to read

    spi_txrx(
        spi_tx_buffer,              // bufTx
        REG_LEN,                    // lenbufTx
        SPI_FIRSTBYTE,              // returnType
        spi_rx_buffer,              // bufRx
        REG_LEN,                    // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );
    return spi_tx_buffer[3];
}

void llcc68_GetDeviceErrors(uint8_t* error, uint8_t arrayLength){
    if (error == NULL || arrayLength < DEVICE_ERRORS_LEN) {
        // Handle invalid buffer (could set an error code or return early)
        return;
    }
    
    uint8_t spi_rx_buffer[DEVICE_ERRORS_LEN];
    uint8_t spi_tx_buffer[DEVICE_ERRORS_LEN];

    spi_tx_buffer[0] = GETDEVICEERRORS;
    spi_tx_buffer[1] = 0x00;
    spi_tx_buffer[2] = 0x00;
    spi_tx_buffer[3] = 0x00;
    spi_tx_buffer[4] = 0x00;

    spi_txrx(
        spi_tx_buffer,              // bufTx
        DEVICE_ERRORS_LEN,          // lenbufTx
        SPI_BUFFER,                 // returnType
        spi_rx_buffer,              // bufRx
        DEVICE_ERRORS_LEN,          // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );
    
    // Copy rxBuffer
    for (int i = 0; i < DEVICE_ERRORS_LEN; i++) {
        error[i] = spi_rx_buffer[i];
    }
}

void llcc68_mode_standby(void){
    uint8_t spi_rx_buffer[2];
    uint8_t spi_tx_buffer[2];

    spi_tx_buffer[0]     = SETSTANDBY;                      // OPCODE: MODE = standby
    spi_tx_buffer[1]     = 0x01;                            // 0x00 = RC 13MHz, 0x01 = XTAL 32MHz, 

    spi_txrx(
        spi_tx_buffer,              // bufTx
        2,                          // lenbufTx
        SPI_FIRSTBYTE,              // returnType
        spi_rx_buffer,              // bufRx
        2,                          // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );
}
    //data[0] = (freq_val >> 24) & 0xFF;
    //data[1] = (freq_val >> 16) & 0xFF;
    //data[2] = (freq_val >> 8) & 0xFF;
    //data[3] = freq_val & 0xFF;
// Set using OPCODE
void llcc68_opcode(uint8_t opcode, uint8_t value){
    uint8_t spi_rx_buffer[2];
    uint8_t spi_tx_buffer[2];

    spi_tx_buffer[0]     = opcode;                          // OPCODE
    spi_tx_buffer[1]     = value;                           // Value to write

    spi_txrx(
        spi_tx_buffer,              // bufTx
        2,                          // lenbufTx
        SPI_FIRSTBYTE,              // returnType
        spi_rx_buffer,              // bufRx
        2,                          // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );
}

/*
void llcc68_opcode (uint8_t opcode, uint8_t* value, uint8_t len){
    uint8_t spi_tx_buffer[256]; 
    spi_tx_buffer[0] = opcode;
    for (int i = 0; i < len; i++) {
        tx_buffer[i + 1] = value[i];
    }
    
    spi_txrx(
        spi_tx_buffer,              // bufTx
        len + 1,                    // lenbufTx
        SPI_FIRSTBYTE,              // returnType
        NULL,                       // bufRx
        0,                          // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );
}
*/



/*
void at86rf215_spiStrobe(uint8_t strobe, uint8_t type) {
    uint8_t  spi_tx_buffer[3];
    uint16_t register_cmd;

    switch(type){
    case FREQ_SUGHZ:
        register_cmd = RG_RF09_CMD;
        break;
    case FREQ_24GHZ:
        register_cmd = RG_RF24_CMD;
        break;
    default:
        return;
    }


    spi_tx_buffer[0]     = ((FLAG_WRITE) | (uint8_t)(register_cmd/256));
    spi_tx_buffer[1]     = (uint8_t)(register_cmd%256);
    spi_tx_buffer[2]     = strobe;

    spi_txrx(
        spi_tx_buffer,              // bufTx
        sizeof(spi_tx_buffer),      // lenbufTx
        SPI_FIRSTBYTE,              // returnType
        NULL,                       // bufRx
        1,                          // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );
}

void at86rf215_spiWriteReg(uint16_t reg, uint8_t regValueToWrite) {

    uint8_t spi_rx_buffer[3];
    uint8_t spi_tx_buffer[3];

    spi_tx_buffer[0]     = (FLAG_WRITE | (uint8_t)((reg)/256));
    spi_tx_buffer[1]     = (uint8_t)((reg)%256);
    spi_tx_buffer[2]     = regValueToWrite;

    spi_txrx(
        spi_tx_buffer,              // bufTx
        3,                          // lenbufTx
        SPI_FIRSTBYTE,              // returnType
        spi_rx_buffer,              // bufRx
        3,                          // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );
}

uint8_t at86rf215_spiReadReg(uint16_t regAddr16) {

    uint8_t              spi_tx_buffer[3];
    uint8_t              spi_rx_buffer[3];

    spi_tx_buffer[0] = (FLAG_READ | (uint8_t)(regAddr16 >> 8));
    spi_tx_buffer[1] = (uint8_t)(regAddr16 & 0xFF);
    spi_tx_buffer[2] = 0x00;

    spi_txrx(
        spi_tx_buffer,              // bufTx
        sizeof(spi_tx_buffer),      // lenbufTx
        SPI_BUFFER,                 // returnType
        spi_rx_buffer,              // bufRx
        sizeof(spi_rx_buffer),      // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );
    return spi_rx_buffer[2];
}

void at86rf215_spiWriteFifo(uint8_t* bufToWrite, uint16_t len, uint8_t type) {
    uint8_t              spi_tx_buffer[4];
    uint16_t             register_bbc_txfll;
    uint16_t             register_bbc_fbtxs;

    switch(type){
    case FREQ_SUGHZ:
        register_bbc_txfll = RG_BBC0_TXFLL;
        register_bbc_fbtxs = RG_BBC0_FBTXS;
        break;
    case FREQ_24GHZ:
        register_bbc_txfll = RG_BBC1_TXFLL;
        register_bbc_fbtxs = RG_BBC1_FBTXS;
        break;
    default:
        return;
    }

    // step 1. send packet length.
    spi_tx_buffer[0]     = (FLAG_WRITE | (uint8_t)(register_bbc_txfll >> 8));
    spi_tx_buffer[1]     = (uint8_t)(register_bbc_txfll & 0xFF);
    spi_tx_buffer[2]     = (uint8_t)(len & 0xFF);       // low byte of packet-length -> TXFLL
    spi_tx_buffer[3]     = (uint8_t)(len >> 8) & 0x07;  // high byte of packet-length -> TXFLH

    spi_txrx(
        spi_tx_buffer,              // bufTx
        sizeof(spi_tx_buffer),      // lenbufTx
        SPI_BUFFER,                 // returnType
        NULL,                       // bufRx
        1,                          // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );

    spi_tx_buffer[0]     = (FLAG_WRITE | (uint8_t)(register_bbc_fbtxs >> 8));
    spi_tx_buffer[1]     = (uint8_t)(register_bbc_fbtxs& 0xFF);

    // step 2. send the address to the Tx buffer.
    spi_txrx(
        spi_tx_buffer,             // bufTx
        2,                         // lenbufTx
        SPI_FIRSTBYTE,             // returnType
        NULL,                      // bufRx
        1,                         // maxLenBufRx
        SPI_FIRST,                 // isFirst
        SPI_NOTLAST                // isLast
    );

    // step 3. send the payload.
    spi_txrx(
        bufToWrite,                 // bufTx
        len - LENGTH_CRC,    // lenbufTx
        SPI_LASTBYTE,               // returnType
        NULL,                       // bufRx
        1,                          // maxLenBufRx
        SPI_NOTFIRST,               // isFirst
        SPI_LAST                    // isLast
    );

}

void at86rf215_spiReadRxFifo( uint8_t* pBufRead, uint16_t* lenRead, uint8_t type, uint16_t maxBuffLength) {

    uint8_t spi_tx_buffer[4];
    uint8_t spi_rx_buffer[4];
    uint16_t length;
    uint16_t             register_bbc_rxfll;
    uint16_t             register_bbc_fbrxs;

    switch(type){
    case FREQ_SUGHZ:
        register_bbc_rxfll = RG_BBC0_RXFLL;
        register_bbc_fbrxs = BASE_ADDR_BBC0_FB0;
        break;
    case FREQ_24GHZ:
        register_bbc_rxfll = RG_BBC1_RXFLL;
        register_bbc_fbrxs = BASE_ADDR_BBC1_FB1;
        break;
    default:
        return;
    }

    // step 1 - read packet length from RG_BBC0_RXFLL/_RXFLH:
    spi_tx_buffer[0] = (FLAG_READ | (uint8_t)(register_bbc_rxfll >> 8));
    spi_tx_buffer[1] = (uint8_t)(register_bbc_rxfll & 0xFF);
    spi_tx_buffer[2] = 0x00;
    spi_tx_buffer[3] = 0x00;

    //read FIFO length
    spi_txrx(
        spi_tx_buffer,              // bufTx
        4,                          // lenbufTx
        SPI_BUFFER,                 // returnType
        spi_rx_buffer,              // bufRx
        4,                          // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                   // isLast
    );
    length = (uint16_t)spi_rx_buffer[2]             // RXFLL
                    | ((uint16_t)(spi_rx_buffer[3] & 0x07) << 8);  // RXFLH

    if (length<=maxBuffLength){
        spi_tx_buffer[0]    = (FLAG_READ | (uint8_t)(register_bbc_fbrxs/256));
        spi_tx_buffer[1]    = (uint8_t)(register_bbc_fbrxs%256);
        //read FIFO
        spi_txrx(
            spi_tx_buffer,              // bufTx
            2,                          // size of the address
            SPI_BUFFER,                 // returnType
            spi_rx_buffer,              // bufRx
            2,                          // maxLenBufRx
            SPI_FIRST,                  // isFirst
            SPI_NOTLAST                 // isLast
        );

            spi_txrx(
            NULL,                        // bufTx
            length,                     // lenbufTx
            SPI_BUFFER,                 // returnType
            pBufRead,                  // bufRx
            length,                     // maxLenBufRx
            SPI_NOTFIRST,               // isFirst
            SPI_LAST                    // isLast
        );
    }
    *(lenRead) = length;
}

uint8_t at86rf215_status (uint8_t type){
    uint8_t spi_tx_buffer[3];
    uint8_t spi_rx_buffer[3];
    uint16_t register_state;

    switch(type){
    case FREQ_SUGHZ:
        register_state = RG_RF09_STATE;
        break;
    case FREQ_24GHZ:
        register_state = RG_RF24_STATE;
        break;
    default:
        return 0;
    }

    spi_tx_buffer[0] = (FLAG_READ | (uint8_t)(register_state/256));
    spi_tx_buffer[1] = (uint8_t)(register_state%256);
    spi_tx_buffer[2] = 0x00;

    spi_txrx(
        spi_tx_buffer,              // bufTx
        3,                          // lenbufTx
        SPI_BUFFER,                 // returnType
        spi_rx_buffer,              // bufRx
        3,                          // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );

    return spi_rx_buffer[2];
}

void at86rf215_read_isr (uint8_t* rf09_isr, uint8_t type){
    uint8_t spi_tx[6];
    uint8_t spi_rx[6];
    uint16_t register_irqs;

    switch(type){
    case FREQ_SUGHZ:
        register_irqs = RG_RF09_IRQS;
        break;
    case FREQ_24GHZ:
        register_irqs = RG_RF24_IRQS;
        break;
    default:
        return;
    }

    memset(&spi_tx[0],0,sizeof(spi_tx));
    memset(&spi_rx[0],0,sizeof(spi_rx));
    spi_tx[0] = (FLAG_READ | (uint8_t)(register_irqs >> 8));
    spi_tx[1] = (uint8_t)(register_irqs & 0xFF);

    spi_txrx(
        spi_tx,                     // bufTx
        6,                          // lenbufTx
        SPI_BUFFER,                 // returnType
        spi_rx,                      // bufRx
        6,                          // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );
    *(rf09_isr)     = spi_rx[2];
    *(rf09_isr+1)   = spi_rx[3];
    *(rf09_isr+2)   = spi_rx[4];
    *(rf09_isr+3)   = spi_rx[5];
}



void at86rf215_readBurst(uint16_t reg, uint8_t* regValueRead, uint16_t size){
    uint8_t spi_tx[2049];

    memset(&spi_tx[0],0,sizeof(spi_tx));

    spi_tx[0] = (uint8_t)(reg/256);
    spi_tx[1] = (uint8_t)(reg%256);

    spi_txrx(
        spi_tx,                     // bufTx
        sizeof(spi_tx),             // lenbufTx
        SPI_BUFFER,                 // returnType
        regValueRead,               // bufRx
        sizeof(spi_tx),             // maxLenBufRx
        SPI_FIRST,                  // isFirst
        SPI_LAST                    // isLast
    );

}
*/
