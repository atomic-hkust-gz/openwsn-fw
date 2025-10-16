/**
\brief This program shows the use of the "radio" bsp module.

*/

#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "uart.h"
#include "radio_llcc68.h"
#include "gpio_irq.h"

//=========================== defines =========================================

#define LENGTH_PACKET   125+LENGTH_CRC    ///< maximum length is 127 bytes
#define LEN_PKT_TO_SEND 20+LENGTH_CRC     ///< temp max packet length
#define RFFREQUENCY     470500000         ///< 470.5 MHz
#define TIMER_PERIOD    (0xfffff>>4)       ///< 0xfffff = 4s@32kHz
#define ID              0x99              ///< byte sent in the packets

// lora configuration
#define LORA_BANDWIDTH              0x04      // 0x04 = 125 kHz
#define LORA_SPREADING_FACTOR       0x07      // sf7
#define LORA_CODINGRATE             0x01      // cr 4/5
#define LORA_PREAMBLE_LENGTH        0x0080
#define LORA_PAYLOAD_LENGTH         0x80      // 128
// for testing
#define MAX_BUFFER_SIZE             0x80
#define CHANNEL_NUM                 2


// radio interrupt configuration
#define IRQ_CHANNEL                 0             ///< gpiote interrupt channel number
#define IRQ_NRF_PORT                PORT1         ///< p1.06 nrf interrupt port assignment
#define IRQ_NRF_PIN                 6             ///< nrf interrupt pin assignment
#define IRQ_RISING_EDGE             GPIOTE_LOTOHI ///< rising edge triggered

uint8_t stringToSend[]  = "+002 Ptest.24.00.12.-010\n";

static const uint8_t TXRXOFFSET =   0x00; 
//                                  { MSB,    , LSB}                                       
static const uint8_t TIMEOUT[3] =   {0x13,0x88,0x00};  ///< 20 s = 1,280,000 * 15.625 us

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
                irqStatus_t     irqStatus;
                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;
                packetStats_t   packetStats;

} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void      cb_startFrame(PORT_TIMER_WIDTH timestamp);
void      cb_endFrame(PORT_TIMER_WIDTH timestamp);
void      cb_gpio_irq(void);
void      cb_timer(void);

void      cb_uart_tx_done(void);
uint8_t   cb_uart_rx(void);

void      llcc68_function_test(void);
void      llcc68_irq_test(void);

//=========================== main ============================================


int mote_main(void){

    //radioModulationParams_t loraModParams;
    //packetParams_t packetParams;
    //radioTimeout_t radioTimeout;   
    
    // initialize board & radio
    board_init();
    radio_llcc68_init();

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));
    ////memset(&loraModParams, 0, sizeof(loraModParams));
    ////memset(&packetParams, 0, sizeof(packetParams));
    ////memset(&radioTimeout, 0, sizeof(radioTimeout));

   app_vars.packet_len = LORA_PAYLOAD_LENGTH;
   llcc68_irq_test();

} 

 

    
   

    


    //// setup UART
    //uart_setCallbacks(cb_uart_tx_done,cb_uart_rx);
    //uart_enableInterrupts();
    //app_vars.uartDone = 1;
    
    //// P1.06 as radio trigger (rising edge detect)
    //// set callback function
    ////gpio_irq_config(IRQ_CHANNEL, 
    ////                IRQ_NRF_PORT, 
    ////                IRQ_NRF_PIN, 
    ////                IRQ_RISING_EDGE, 
    ////                cb_gpio_irq);
    ////gpio_irq_enable(IRQ_CHANNEL);

    //// prepare packet
    
    ////app_vars.packet_len = sizeof(app_vars.packet);

    


    


    //  for (int i = 0; i < app_vars.packet_len; i++) {
    //      app_vars.packet[i] = ID;
    //}




    //for (int i = 0; i < app_vars.packet_len; i++){
    //    app_vars.packet[i] = (uint8_t)i;
    //}

    //// radio config
    //radio_llcc68_setFrequency(RFFREQUENCY);

    //loraModParams = (radioModulationParams_t){
    //    .SpreadingFactor      = LORA_SF7,
    //    .Bandwidth            = LORA_BW_125,
    //    .CodingRate           = LORA_CR_4_5,
    //    .LowDataRateOptimize  = LDRO_ON
    //};
    //radio_llcc68_setModulation(loraModParams);
    //packetParams = (packetParams_t){
    //    .PreambleLength       = LORA_PREAMBLE_LENGTH,
    //    .HeaderType           = FIXED_LENGTH_PACKET,
    //    .PayloadLength        = LENGTH_PACKET,
    //    .CrcType              = CRC_ON,
    //    .InvertIq             = STD_IQ
    //};
    //radio_llcc68_setPacketParams(packetParams);

    //// start bsp timer
    //sctimer_set_callback(cb_timer);
    //sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
    //sctimer_enable();
    
    //// switch in RX by default
    ////radio_rxEnable();
    //app_vars.state = APP_STATE_RX;

    //// start by a transmit
    //app_vars.flags |= APP_FLAG_TIMER;

    ////radio_llcc68_loadPacket(offset, buffer, sizeof(packet));

    ////memcpy(radioTimeout, RADIOTIMEOUT, sizeof(radioTimeout));
    ////radio_llcc68_txNow(radioTimeout_t timeout);
    //while (1){

    //  //if (app_vars.flags & APP_FLAG_TIMER){

    //    radio_llcc68_loadPacket(TXRXOFFSET, app_vars.packet, LEN_PKT_TO_SEND);
    //    memcpy(radioTimeout.timeout, TIMEOUT, sizeof(radioTimeout.timeout));
    //    radio_llcc68_txNow(radioTimeout);
    //    radio_llcc68_get_status();
    //    radio_llcc68_get_opError();
        
    //    //app_vars.flags &= ~APP_FLAG_TIMER;
    //  //}
    //  for(int i = 0; i < 10000; i++){};
    
    //}




    //SetDioIrqParams
    //Define Sync Word value: use the command WriteReg(...)
    //SetTx()
    //wait for irq TxDone or Timeout
    //clear IRQ TxDone

    //while (1) {

    //    // sleep while waiting for at least one of the flags to be set
    //    while (app_vars.flags==0x00) {
    //        board_sleep();
    //    }

    //    // handle and clear every flag
    //    while (app_vars.flags) {


    //        //==== APP_FLAG_START_FRAME (TX or RX)

    //        if (app_vars.flags & APP_FLAG_START_FRAME) {
    //            // start of frame

    //            switch (app_vars.state) {
    //                case APP_STATE_RX:
    //                    // started receiving a packet

    //                    // led
    //                    leds_error_on();
    //                    break;
    //                case APP_STATE_TX:
    //                    // started sending a packet

    //                    // led
    //                    leds_sync_on();
    //                break;
    //            }

    //            // clear flag
    //            app_vars.flags &= ~APP_FLAG_START_FRAME;
    //        }

    //        //==== APP_FLAG_END_FRAME (TX or RX)

    //        if (app_vars.flags & APP_FLAG_END_FRAME) {
    //            // end of frame

    //            switch (app_vars.state) {

    //                case APP_STATE_RX:

    //                    // done receiving a packet
    //                    app_vars.packet_len = sizeof(app_vars.packet);

    //                    // get packet from radio
    //                    radio_getReceivedFrame(
    //                        app_vars.packet,
    //                        &app_vars.packet_len,
    //                        sizeof(app_vars.packet),
    //                        &app_vars.rxpk_rssi,
    //                        &app_vars.rxpk_lqi,
    //                        &app_vars.rxpk_crc
    //                    );

    //                    i = 0;
    //                    memcpy(&stringToSend[i],&app_vars.packet[0], 
    //                        app_vars.packet_len);
    //                    i += app_vars.packet_len + 1;

    //                    stringToSend[sizeof(stringToSend)-2] = '\r';
    //                    stringToSend[sizeof(stringToSend)-1] = '\n';

    //                    // send string over UART
    //                    if (app_vars.uartDone == 1) {
    //                        app_vars.uartDone              = 0;
    //                        app_vars.uart_lastTxByteIndex  = 0;
    //                        uart_writeByte(stringToSend[app_vars.uart_lastTxByteIndex]);
    //                    }

    //                    // led
    //                    leds_error_off();
    //                    break;
    //                case APP_STATE_TX:
    //                    // done sending a packet

    //                    // switch to RX mode
    //                    radio_rxEnable();
    //                    radio_rxNow();
    //                    app_vars.state = APP_STATE_RX;

    //                    // led
    //                    leds_sync_off();
    //                    break;
    //            }
    //            // clear flag
    //            app_vars.flags &= ~APP_FLAG_END_FRAME;
    //        }

    //        //==== APP_FLAG_TIMER

    //        if (app_vars.flags & APP_FLAG_TIMER) {
    //            // timer fired

    //            if (app_vars.state==APP_STATE_RX) {
    //                // radio mode = standby_RC
    //                radio_llcc68_rfOff();

    //                // prepare packet
    //                app_vars.packet_len = sizeof(app_vars.packet);
    //                i = 0;
    //                app_vars.packet[i++] = 't';
    //                app_vars.packet[i++] = 'e';
    //                app_vars.packet[i++] = 's';
    //                app_vars.packet[i++] = 't';
    //                while (i<app_vars.packet_len) {
    //                    app_vars.packet[i++] = ID;
    //                }
                    
    //                // start transmitting packet
    //                radio_llcc68_loadPacket(TXRXOFFSET,
    //                    app_vars.packet,LEN_PKT_TO_SEND);
    //                radio_llcc68_setFrequency(RFFREQUENCY);
    //                memcpy(radioTimeout, TIMEOUT, sizeof(radioTimeout));
    //                radio_llcc68_txNow(radioTimeout);

    //                app_vars.state = APP_STATE_TX;
    //            }

    //            // clear flag
    //            app_vars.flags &= ~APP_FLAG_TIMER;
    //        }
    //    }
    //}
   




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
}


void llcc68_irq_test(void){

    radio_llcc68_config_t loraConfig;
    radioTimeout_t radioTimeout;

    memset(&loraConfig, 0, sizeof(loraConfig));
 
    // P1.06 assigned to radio interrupt (DIO1)(rising edge detect)
    gpio_irq_config(IRQ_CHANNEL, 
                    IRQ_NRF_PORT, 
                    IRQ_NRF_PIN, 
                    IRQ_RISING_EDGE, 
                    cb_gpio_irq);
    gpio_irq_enable(IRQ_CHANNEL);
   
    app_vars.packet[0] = 't';
    app_vars.packet[1] = 'e';
    app_vars.packet[2] = 's';
    app_vars.packet[3] = 't';
    for (int i = 4; i < app_vars.packet_len; i++){
        app_vars.packet[i] = (uint8_t)i;
    }

    // basic Tx steps 1-6
    loraConfig.loraModParams  = (radioModulationParams_t){
        .spreadingFactor      = LORA_SF7,
        .bandwidth            = LORA_BW_125,
        .codingRate           = LORA_CR_4_5,
        .lowDataRateOptimize  = LDRO_OFF,
    };
    loraConfig.radioTxParams  = (radioTxParams_t){
        .txPowerDbm           = TX_P22_DBM,
        .txRampTime           = RAMP_200U,
    };
    loraConfig.packetParams   = (packetParams_t){
        .preambleLength       = LORA_PREAMBLE_LENGTH,
        .headerType           = FIXED_LENGTH_PACKET,
        .payloadLength        = MAX_BUFFER_SIZE,
        .crcType              = CRC_ON,
        .invertIq             = STD_IQ,
    };
    loraConfig.channel        = CHANNEL_NUM;
    loraConfig.syncword       = PRIVATESYNC;
    radio_llcc68_lora_config(loraConfig);
    
    // start bsp timer
    sctimer_set_callback(cb_timer);
    sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
    sctimer_enable();
    
    while(1){

      while(!(app_vars.flags & APP_FLAG_TIMER)){
        // wait
        __NOP();
      }
      app_vars.flags = 0;
      // basic Tx step 7
      radio_llcc68_loadPacket(TXRXOFFSET, app_vars.packet, app_vars.packet_len);
      memcpy(radioTimeout.timeout, TIMEOUT, sizeof(TIMEOUT));
      
      // basic Tx steps 8-12
      //radio_llcc68_txNow(radioTimeout);
      radio_llcc68_rxNow(radioTimeout);
      app_vars.flags != APP_FLAG_START_FRAME;

      //while((app_vars.irqStatus.txDone & 1) == 0){
      while((app_vars.irqStatus.rxDone & 1 | app_vars.irqStatus.timeout & 1) == 0){
        // basic Tx step 13
        board_sleep();
      }
      radio_llcc68_readPacket(app_vars.packet);
      radio_llcc68_getPacketStats(&app_vars.packetStats);
      app_vars.flags = APP_FLAG_END_FRAME;
      // basic Tx step 14
      // clear IRQ status
      radio_llcc68_irq_clear();
      app_vars.irqStatus = radio_llcc68_irq_status();
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

void cb_gpio_irq(void) {
    app_vars.irqStatus = radio_llcc68_irq_status();
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
