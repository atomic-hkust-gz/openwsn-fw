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
//#define MAX_BUFFER_SIZE             0x80
#define CHANNEL_NUM                 2


// radio interrupt configuration
#define IRQ_CHANNEL                 0             ///< gpiote interrupt channel number
#define IRQ_NRF_PORT                PORT1         ///< p1.06 nrf interrupt port assignment
#define IRQ_NRF_PIN                 6             ///< nrf interrupt pin assignment
#define IRQ_RISING_EDGE             GPIOTE_LOTOHI ///< rising edge triggered

static const uint8_t TXRXOFFSET =   0x00; 
//                                  { MSB,    , LSB}                                       
static const uint8_t TIMEOUT[3] =   {0x13,0x88,0x00};  ///< 20 s = 1,280,000 * 15.625 us

uint8_t stringToSend[]  = "testing....\n\r";

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
    radio_llcc68_irqStatus_t    irqStatus;
                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;
    radio_llcc68_packetStats_t  packetStats;

} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void      cb_startFrame(PORT_TIMER_WIDTH timestamp);
void      cb_endFrame(PORT_TIMER_WIDTH timestamp);
void      cb_gpio_irq(void);
void      cb_timer(void);

void      cb_uart_tx_done(void);
uint8_t   cb_uart_rx(void);

void      llcc68_irq_test(void);

//=========================== main ============================================


int mote_main(void){

    radio_llcc68_config_t loraConfig;
    radioTimeout_t radioTimeout; 
    uint8_t i;
    uint8_t sign; 
    uint8_t read;
   
    // initialize board & radio
    board_init();
    radio_llcc68_init();

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));
    memset(&loraConfig, 0, sizeof(radio_llcc68_config_t));
    memset(&radioTimeout, 0, sizeof(radioTimeout_t));

  
    // setup UART
    uart_setCallbacks(cb_uart_tx_done,cb_uart_rx);
    uart_enableInterrupts();
    app_vars.uartDone = 1;
 
    // P1.06 assigned to radio interrupt (DIO1)(rising edge detect)
    gpio_irq_config(IRQ_CHANNEL, 
                    IRQ_NRF_PORT, 
                    IRQ_NRF_PIN, 
                    IRQ_RISING_EDGE, 
                    cb_gpio_irq);
    gpio_irq_enable(IRQ_CHANNEL);

    // prepare packet
    app_vars.packet_len = sizeof(app_vars.packet);

    llcc68_irq_test();
    // -------------------------------------
    app_vars.packet[0] = 't';
    app_vars.packet[1] = 'e';
    app_vars.packet[2] = 's';
    app_vars.packet[3] = 't';

    for (int i = 4; i < app_vars.packet_len; i++){
        app_vars.packet[i] = (uint8_t)i;
    }
    
    // radio config
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
        .payloadLength        = MAX_PACKET_SIZE,
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

    // switch in RX by default
    app_vars.state = APP_STATE_RX;

    // start by a transmit
    app_vars.flags |= APP_FLAG_TIMER;
    
    while(1){
      // sleep while waiting for timer
      while(!(app_vars.flags & APP_FLAG_TIMER)){
        // wait
        board_sleep();
      }

      // handle and clear every flag
      while (app_vars.flags) {

        // APP_FLAG_START_FRAME  (TX or RX)
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

        // APP_FLAG_END_FRAME (TX or RX)
        if (app_vars.flags & APP_FLAG_END_FRAME) {
          // end of frame
          switch (app_vars.state) {
            case APP_STATE_RX:
              // done receiving a packet
              app_vars.packet_len = sizeof(app_vars.packet);
              // get packet from radio
              radio_llcc68_getReceivedFrame(
                  app_vars.packet,
                  &app_vars.packet_len,
                  &app_vars.packetStats
              );
              // clear IRQ status
              radio_llcc68_irq_clear();
              
              i = 0;
              sign = (app_vars.packetStats.rssiPkt & 0x80) >> 7;
              if (sign){
                  read = 0xff - (uint8_t)(app_vars.packetStats.rssiPkt) + 1;
                  stringToSend[i++] = '-';
              } else {
                  read = app_vars.packetStats.rssiPkt;
                  stringToSend[i++] = '+';
              }

              stringToSend[i++] = '0' + read;
              stringToSend[i++] = ' ';



              sign = (app_vars.packetStats.snrPkt & 0x80) >> 7;
              if (sign){
                  read = 0xff - (uint8_t)(app_vars.packetStats.snrPkt) + 1;
                  stringToSend[i++] = '-';
              } else {
                  read = app_vars.packetStats.snrPkt;
                  stringToSend[i++] = '+';
              }

              stringToSend[i++] = '0' + read;
              stringToSend[i++] = ' ';


              sign = (app_vars.packetStats.signalRssiPk & 0x80) >> 7;
              if (sign){
                  read = 0xff - (uint8_t)(app_vars.packetStats.signalRssiPk) + 1;
                  stringToSend[i++] = '-';
              } else {
                  read = app_vars.packetStats.rssiPkt;
                  stringToSend[i++] = '+';
              }

              stringToSend[i++] = '0' + read;
              stringToSend[i++] = ' ';
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
              memcpy(radioTimeout.timeout, TIMEOUT, sizeof(TIMEOUT));
              radio_llcc68_rxNow(radioTimeout);
              app_vars.state = APP_STATE_RX;

              // led
              leds_sync_off();
              break;
          }
          // clear flag
          app_vars.flags &= ~APP_FLAG_END_FRAME;
        }

        // APP_FLAG_TIMER
        if (app_vars.flags & APP_FLAG_TIMER) {
          
          // timer fired
          if (app_vars.state == APP_STATE_RX) {
              
              // stop listening
              //radio_rfOff();

              // prepare packet
              app_vars.packet_len = sizeof(app_vars.packet);
              i = 0;
              app_vars.packet[i++] = 't';
              app_vars.packet[i++] = 'e';
              app_vars.packet[i++] = 's';
              app_vars.packet[i++] = 't';
              app_vars.packet[i++] = CHANNEL_NUM;
              while (i<app_vars.packet_len) {
                  app_vars.packet[i++] = ID;
              }

              // start transmitting packet
              radio_llcc68_loadPacket(TXRXOFFSET, app_vars.packet, app_vars.packet_len);
              memcpy(radioTimeout.timeout, TIMEOUT, sizeof(TIMEOUT));
              radio_llcc68_txNow(radioTimeout);

              app_vars.state = APP_STATE_TX;
          }

          // clear flag
          app_vars.flags &= ~APP_FLAG_TIMER;
        }
      }
    }
  }

//==========================================
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
      radio_llcc68_getReceivedFrame(
                  app_vars.packet,
                  &app_vars.packet_len,
                  &app_vars.packetStats
              );
      app_vars.flags = APP_FLAG_END_FRAME;
      // basic Tx step 14
      // clear IRQ status
      radio_llcc68_irq_clear();
      app_vars.irqStatus = radio_llcc68_getIrqstatus();
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
    app_vars.irqStatus = radio_llcc68_getIrqstatus();
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
