/**
\brief This program shows the use of the "radio" bsp module.

*/

#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "uart.h"
#include "radio_llcc68.h"

//=========================== defines =========================================

#define LENGTH_PACKET   125+LENGTH_CRC  ///< maximum length is 127 bytes
#define LEN_PKT_TO_SEND 20+LENGTH_CRC
#define CHANNEL         11             ///< 11=2.405GHz
#define TIMER_PERIOD    (0xffff>>4)    ///< 0xffff = 2s@32kHz
#define ID              0x99           ///< byte sent in the packets

#define MAX_BUFFER_SIZE   10

uint8_t stringToSend[]  = "+002 Ptest.24.00.12.-010\n";

//=========================== variables =======================================

enum {
    APP_FLAG_START_FRAME = 0x01,
    APP_FLAG_END_FRAME   = 0x02,
    APP_FLAG_TIMER       = 0x04,
};

typedef enum {
    APP_STATE_TX         = 0x01,
    APP_STATE_RX         = 0x02,
} app_state_t;

typedef struct {
    uint8_t              num_startFrame;
    uint8_t              num_endFrame;
    uint8_t              num_timer;
    
    uint8_t              num_rx_startFrame;
    uint8_t              num_rx_endFrame;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
    volatile    uint8_t         uartDone;
    volatile    uint8_t         uartSendNow;
                uint8_t         uart_lastTxByteIndex;

                uint8_t         flags;
                app_state_t     state;
                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_timer(void);

void     cb_uart_tx_done(void);
uint8_t  cb_uart_rx(void);

void llcc68_function_test(void);
//=========================== main ============================================


int mote_main(void){
    radioModulationParams_t loraModParams;
    packetParams_t packetParams;
    uint8_t packet[MAX_BUFFER_SIZE];
    
    memset(&loraModParams, 0, sizeof(loraModParams));
    memset(&packetParams, 0, sizeof(packetParams));

    board_init();
    radio_llcc68_init();
    
    //llcc68_function_test();

    radio_llcc68_loadPacket(uint8_t offset, uint8_t* buffer, uint8_t len);
    //radio_llcc68_setModulation(radioModulationParams_t modParams);
    //radio_llcc68_setPacketParams(packetParams_t packetParams);
    radio_llcc68_txNow(radioTimeout_t timeout);

    //SetDioIrqParams
    //Define Sync Word value: use the command WriteReg(...)
    //SetTx()
    //wait for irq TxDone or Timeout
    //clear IRQ TxDone

    while(1);

}
void llcc68_function_test(void){
    uint8_t packet[4];
    uint8_t opcodeDate[2];
    uint8_t txPayload[MAX_BUFFER_SIZE];
    uint8_t rxPayload[MAX_BUFFER_SIZE];

    memset(&packet, 0, sizeof(packet));
    memset(&opcodeDate, 0, sizeof(opcodeDate));
    memset(&txPayload, 0, sizeof(txPayload));
    memset(&rxPayload, 0, sizeof(rxPayload));
    
    // function testing

    // llcc68_spiReadingReg()
    packet[0] = llcc68_spiReadReg(CRCMSB); // Default Value = 0x1D
    packet[1] = llcc68_spiReadReg(CRCLSB); // Default Value = 0x0F

    // llcc68_spiReadingReg() 
    llcc68_spiWriteReg(CRCMSB,0x12);
    llcc68_spiWriteReg(CRCLSB,0x34);
    packet[2] = llcc68_spiReadReg(CRCMSB); // expected value = 0x12
    packet[3] = llcc68_spiReadReg(CRCLSB); // Expected value = 0x34
    
    // write opcode
    opcodeDate[0] = 0x01;     // LORA packet type
    llcc68_noAddress_opcode(SETPACKETTYPE, 
        TYPE_WRITE, &opcodeDate[0], sizeof(opcodeDate[0]));

    // read opcode
    llcc68_noAddress_opcode(GETPACKETTYPE, 
        TYPE_READ, &opcodeDate[1], sizeof(opcodeDate[1]));


    // llcc68_writeBuffer()
    opcodeDate[0] = 0x00;   // tx base address
    opcodeDate[0] = 0x80;   // rx base address
    llcc68_noAddress_opcode(SETBUFFERBASEADDRESS, 
        TYPE_WRITE, opcodeDate, sizeof(opcodeDate));

    for(int i = 0; i < MAX_BUFFER_SIZE; i++){
        txPayload[i] = (uint8_t)i;
    }
    llcc68_txBufferWrite((uint8_t)0x00, txPayload, sizeof(txPayload));
    rxPayload[0] = llcc68_spiReadReg(0x0080);
    rxPayload[1] = llcc68_spiReadReg(0x0040);
    rxPayload[2] = llcc68_spiReadReg(0x0020);
    rxPayload[3] = llcc68_spiReadReg(0x00C0);
    radio_llcc68_get_opError();
    radio_llcc68_get_status();
    // llcc68_readBuffer()
    //llcc68_rxBufferRead((uint8_t)0x00, rxPayload, sizeof(rxPayload));
};


    //llcc68_noAddress_opcode(SETFS, 
    //    TYPE_WRITE, (uint8_t*)&value, sizeof(value));
    //llcc68_noAddress_opcode(SETFS, TYPE_WRITE, packet, sizeof(packet));
/*
int mote_main(void) {
    uint8_t i;

    uint8_t freq_offset;
    uint8_t sign;
    uint8_t read;

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    // initialize board
    board_init();

    // setup UART
    uart_setCallbacks(cb_uart_tx_done,cb_uart_rx);
    uart_enableInterrupts();

    app_vars.uartDone = 1;

    // add callback functions radio
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    // prepare packet
    app_vars.packet_len = sizeof(app_vars.packet);
    for (i=0;i<app_vars.packet_len;i++) {
        app_vars.packet[i] = ID;
    }

    // start bsp timer
    sctimer_set_callback(cb_timer);
    sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
    sctimer_enable();

    // prepare radio
    radio_rfOn();
    // freq type only effects on scum port
    radio_setFrequency(CHANNEL, FREQ_RX);

    // switch in RX by default
    radio_rxEnable();
    app_vars.state = APP_STATE_RX;

    // start by a transmit
    app_vars.flags |= APP_FLAG_TIMER;

    while (1) {

        // sleep while waiting for at least one of the flags to be set
        while (app_vars.flags==0x00) {
            board_sleep();
        }

        // handle and clear every flag
        while (app_vars.flags) {


            //==== APP_FLAG_START_FRAME (TX or RX)

            if (app_vars.flags & APP_FLAG_START_FRAME) {
                // start of frame

                switch (app_vars.state) {
                    case APP_STATE_RX:
                        // started receiving a packet

                        // led
                        leds_error_on();
                        break;
                    case APP_STATE_TX:
                        // started sending a packet

                        // led
                        leds_sync_on();
                    break;
                }

                // clear flag
                app_vars.flags &= ~APP_FLAG_START_FRAME;
            }

            //==== APP_FLAG_END_FRAME (TX or RX)

            if (app_vars.flags & APP_FLAG_END_FRAME) {
                // end of frame

                switch (app_vars.state) {

                    case APP_STATE_RX:

                        // done receiving a packet
                        app_vars.packet_len = sizeof(app_vars.packet);

                        // get packet from radio
                        radio_getReceivedFrame(
                            app_vars.packet,
                            &app_vars.packet_len,
                            sizeof(app_vars.packet),
                            &app_vars.rxpk_rssi,
                            &app_vars.rxpk_lqi,
                            &app_vars.rxpk_crc
                        );

                        freq_offset = radio_getFrequencyOffset();
                        sign = (freq_offset & 0x80) >> 7;
                        if (sign){
                            read = 0xff - (uint8_t)(freq_offset) + 1;
                        } else {
                            read = freq_offset;
                        }

                        i = 0;
                        if (sign) {
                            stringToSend[i++] = '-';
                        } else {
                            stringToSend[i++] = '+';
                        }
                        stringToSend[i++] = '0'+read/100;
                        stringToSend[i++] = '0'+read/10;
                        stringToSend[i++] = '0'+read%10;
                        stringToSend[i++] = ' ';

                        stringToSend[i++] = 'P';
                        memcpy(&stringToSend[i],&app_vars.packet[0],14);
                        i += 14;

                        sign = (app_vars.rxpk_rssi & 0x80) >> 7;
                        if (sign){
                            read = 0xff - (uint8_t)(app_vars.rxpk_rssi) + 1;
                        } else {
                            read = app_vars.rxpk_rssi;
                        }

                        if (sign) {
                            stringToSend[i++] = '-';
                        } else {
                            stringToSend[i++] = '+';
                        }
                        stringToSend[i++] = '0'+read/100;
                        stringToSend[i++] = '0'+read/10;
                        stringToSend[i++] = '0'+read%10;

                        stringToSend[sizeof(stringToSend)-2] = '\r';
                        stringToSend[sizeof(stringToSend)-1] = '\n';

                        // send string over UART
                        if (app_vars.uartDone == 1) {
                            app_vars.uartDone              = 0;
                            app_vars.uart_lastTxByteIndex  = 0;
                            uart_writeByte(stringToSend[app_vars.uart_lastTxByteIndex]);
                        }

                        // led
                        leds_error_off();
                        break;
                    case APP_STATE_TX:
                        // done sending a packet

                        // switch to RX mode
                        radio_rxEnable();
                        radio_rxNow();
                        app_vars.state = APP_STATE_RX;

                        // led
                        leds_sync_off();
                        break;
                }
                // clear flag
                app_vars.flags &= ~APP_FLAG_END_FRAME;
            }

            //==== APP_FLAG_TIMER

            if (app_vars.flags & APP_FLAG_TIMER) {
                // timer fired

                if (app_vars.state==APP_STATE_RX) {
                    // stop listening
                    radio_rfOff();

                    // prepare packet
                    app_vars.packet_len = sizeof(app_vars.packet);
                    i = 0;
                    app_vars.packet[i++] = 't';
                    app_vars.packet[i++] = 'e';
                    app_vars.packet[i++] = 's';
                    app_vars.packet[i++] = 't';
                    app_vars.packet[i++] = CHANNEL;
                    while (i<app_vars.packet_len) {
                        app_vars.packet[i++] = ID;
                    }

                    // start transmitting packet
                    radio_loadPacket(app_vars.packet,LEN_PKT_TO_SEND);
                    radio_txEnable();
                    radio_txNow();

                    app_vars.state = APP_STATE_TX;
                }

                // clear flag
                app_vars.flags &= ~APP_FLAG_TIMER;
            }
        }
    }
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    // set flag
    app_vars.flags |= APP_FLAG_START_FRAME;

    // update debug stats
    app_dbg.num_startFrame++;

    if (app_vars.state == APP_STATE_RX) {
        app_dbg.num_rx_startFrame++;
    }
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
    // set flag
    app_vars.flags |= APP_FLAG_END_FRAME;

    // update debug stats
    app_dbg.num_endFrame++;

    if (app_vars.state == APP_STATE_RX) {
        app_dbg.num_rx_endFrame++;
    }
}

void cb_timer(void) {
    // set flag
    app_vars.flags |= APP_FLAG_TIMER;

    // update debug stats
    app_dbg.num_timer++;

    sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
}

void cb_uart_tx_done(void) {
    app_vars.uart_lastTxByteIndex++;
    if (app_vars.uart_lastTxByteIndex<sizeof(stringToSend)) {
        uart_writeByte(stringToSend[app_vars.uart_lastTxByteIndex]);
    } else {
        app_vars.uartDone = 1;
    }
}

uint8_t cb_uart_rx(void) {
    uint8_t byte;

    // toggle LED
    leds_error_toggle();

    // read received byte
    byte = uart_readByte();

    // echo that byte over serial
    uart_writeByte(byte);

    return 0;
}
*/