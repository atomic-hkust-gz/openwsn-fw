/**
\brief This program shows the use of the "radio" bsp module.

*/

#include "stdlib.h"
#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "uart.h"
#include "radio_llcc68.h"
#include "gpio_irq.h"

//=========================== defines =========================================

// Mode
#define TXRXMODE APP_STATE_TX //APP_STATE_TX or APP_STATE_RX

#define LENGTH_PACKET   125+LENGTH_CRC    ///< maximum length is 127 bytes
#define LEN_PKT_TO_SEND 20+LENGTH_CRC     ///< temp max packet length
#define RFFREQUENCY     470500000         ///< 470.5 MHz
#define TIMER_PERIOD    (0xfffff>>4)       ///< 0xfffff = 4s@32kHz
#define ID              0x99              ///< byte sent in the packets

// lora configuration
#define LORA_BANDWIDTH              LORA_BW_125
#define LORA_SPREADING_FACTOR       LORA_SF7         
#define LORA_CODINGRATE             LORA_CR_4_5 
#define LORA_PREAMBLE_LENGTH        PREAMBLE_LENGTH_32
#define LORA_PAYLOAD_LENGTH         MAX_BUFFER_SIZE
// for testing
#define CHANNEL_NUM                 2

static const uint8_t TXRXOFFSET =   0x00; 
//                                  { MSB,    , LSB}                                       
static const uint8_t TIMEOUT[3] =   {0x13,0x88,0x00};  ///< 20 s = 1,280,000 * 15.625 us

uint8_t stringToSend[30];

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
    uint8_t              num_timer;
    uint8_t              num_tx_sent;
    uint8_t              num_rx_startFrame;
    uint8_t              num_rx_endFrame;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
    volatile    uint8_t         uartDone;
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

void      cb_gpio_irq(void);
void      cb_timer(void);

void      cb_uart_tx_done(void);
uint8_t   cb_uart_rx(void);

void      fill_packet_count(uint8_t count);
void      uart_string_fill(uint8_t packet_count, 
                            radio_llcc68_packetStats_t  stats);

//=========================== main ============================================


int mote_main(void){

    radio_llcc68_config_t loraConfig;
    radioTimeout_t radioTimeout;
    uint8_t i;
    uint8_t tempRead; 

   
    // initialize board & radio
    board_init();
    radio_llcc68_init(cb_gpio_irq);

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));
    memset(&loraConfig, 0, sizeof(radio_llcc68_config_t));
    memset(&radioTimeout, 0, sizeof(radioTimeout_t));

  
    // setup UART
    uart_setCallbacks(cb_uart_tx_done,cb_uart_rx);
    uart_enableInterrupts();
    app_vars.uartDone = 1;

    // prepare packet
    app_vars.packet_len = sizeof(app_vars.packet);
    app_vars.state = TXRXMODE;

    //llcc68_irq_test();
    // -------------------------------------      
    if (TXRXMODE == APP_STATE_TX) {
        app_vars.packet[0] = 'T';
        app_vars.packet[1] = 'x';
        app_vars.packet[2] = ' ';
        app_vars.packet[3] = 's';
        app_vars.packet[4] = 'e';
        app_vars.packet[5] = 'n';
        app_vars.packet[6] = 'd';
        app_vars.packet[7] = 'e';
        app_vars.packet[8] = 'r';
        for (i = 9; i < app_vars.packet_len; i++){
            app_vars.packet[i] = (uint8_t)i;
        }
    } else {
        if (TXRXMODE == APP_STATE_RX) {
            app_vars.packet[0] = 'R';
            app_vars.packet[1] = 'x';
            app_vars.packet[2] = ' ';
            app_vars.packet[3] = 't';
            app_vars.packet[4] = 'e';
            app_vars.packet[5] = 'm';
            app_vars.packet[6] = 'p';
            for (i = 7; i < app_vars.packet_len; i++){
                app_vars.packet[i] = (uint8_t)i;
            }
        } else { 
            return 0;
        }
    }

    // lora radio config
    loraConfig.loraModParams  = (radioModulationParams_t){
        .spreadingFactor      = LORA_SPREADING_FACTOR,
        .bandwidth            = LORA_BANDWIDTH,
        .codingRate           = LORA_CODINGRATE,
        .lowDataRateOptimize  = LDRO_OFF,
    };
    loraConfig.radioTxParams  = (radioTxParams_t){
        .txPowerDbm           = TX_P22_DBM,
        .txRampTime           = RAMP_200U,
    };
    loraConfig.packetParams   = (packetParams_t){
        .preambleLength       = LORA_PREAMBLE_LENGTH,
        .headerType           = VARIABLE_LENGTH_PACKET,
        .payloadLength        = LORA_PAYLOAD_LENGTH,
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
 
    while(1) {

        while(!(app_vars.flags & APP_FLAG_TIMER)) {
            // wait for periodic timer
            board_sleep ();
        }

        app_vars.flags = 0;
        memcpy(radioTimeout.timeout, TIMEOUT, sizeof(TIMEOUT));
      
        // tx node
        if (TXRXMODE == APP_STATE_TX) {
          // load tx packet
          fill_packet_count(app_dbg.num_tx_sent);
          radio_llcc68_loadPacket(TXRXOFFSET, app_vars.packet, app_vars.packet_len);

          radio_llcc68_txNow(radioTimeout);
          while((app_vars.irqStatus.txDone & 1 | 
                 app_vars.irqStatus.timeout & 1) == 0){
              board_sleep();
          }
        }
        // rx node
        else {
            radio_llcc68_rxNow(radioTimeout);
            while((app_vars.irqStatus.rxDone & 1 | 
                   app_vars.irqStatus.timeout & 1) == 0){
                board_sleep();
            }

            if (app_vars.irqStatus.rxDone & 1){
                radio_llcc68_getReceivedFrame(
                          app_vars.packet,
                          &app_vars.packet_len,
                          &app_vars.packetStats);
                uart_string_fill(app_dbg.num_rx_endFrame, app_vars.packetStats);
            }        
        }
        app_vars.irqStatus = radio_llcc68_getIrqstatus();
    }  
}

//=========================== callbacks =======================================

void cb_gpio_irq(void) {
    app_vars.irqStatus = radio_llcc68_getIrqstatus();
    radio_llcc68_irq_clear();
    // start frame
    if (app_vars.irqStatus.preambleDetect & 1){
      if (app_vars.state == APP_STATE_RX) {
        app_dbg.num_rx_startFrame++;
      }
      app_vars.flags |= APP_FLAG_START_FRAME;
    }
    
    // end frame
    else if (app_vars.irqStatus.txDone & 1 |
        app_vars.irqStatus.rxDone & 1)
    {
      if (app_vars.state == APP_STATE_RX) {
        app_dbg.num_rx_endFrame++;
      }

      else { // APP_STATE_TX
        app_dbg.num_tx_sent++;
      }
      app_vars.flags |= APP_FLAG_END_FRAME;
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

void fill_packet_count(uint8_t count){
    char temp[3];
    uint8_t j;

    for (j = 0; j < sizeof(temp); j++) {
      temp[j] = '0'+ count % 10; 
      count /= 10;
    }
    app_vars.packet[9] = temp[2];
    app_vars.packet[10] = temp[1];
    app_vars.packet[11] = temp[0];
}
void uart_string_fill(uint8_t packet_count, 
                      radio_llcc68_packetStats_t  stats){
    uint8_t i = 0; 
    uint8_t j;
    int8_t value;
    uint8_t count;
    char temp[3];

    // rssi of packet
    stringToSend[i++] = 'r';
    stringToSend[i++] = 's';
    stringToSend[i++] = 's';
    stringToSend[i++] = 'i';

    value = stats.rssiPkt;
    if (value < 0) {
      value = -value;
      stringToSend[i++] = '-';
    }
    else { 
      stringToSend[i++] = '+';
    }

    for (j = 0; j < sizeof(temp); j++) {
      temp[j] = '0'+ value % 10; 
      value /= 10;
    }
    stringToSend[i++] = temp[2];
    stringToSend[i++] = temp[1];
    stringToSend[i++] = temp[0];

    stringToSend[i++] = ' ';

    // snr of packet
    stringToSend[i++] = 's';
    stringToSend[i++] = 'n';
    stringToSend[i++] = 'r';

    value = stats.snrPkt;
    if (value < 0) {
      value = -value;
      stringToSend[i++] = '-';
    }
    else { 
      stringToSend[i++] = '+';
    }

    for (j = 0; j < sizeof(temp); j++) {
      temp[j] = '0'+ value % 10; 
      value /= 10;
    }
    stringToSend[i++] = temp[2];
    stringToSend[i++] = temp[1];
    stringToSend[i++] = temp[0];
    
    stringToSend[i++] = ' ';
    
    // rx packet count
    stringToSend[i++] = 'n';
    stringToSend[i++] = 'u';
    stringToSend[i++] = 'm';

    count = packet_count;
    for (j = 0; j < sizeof(temp); j++) {
      temp[j] = '0'+ count % 10; 
      count /= 10;
    }
    stringToSend[i++] = temp[2];
    stringToSend[i++] = temp[1];
    stringToSend[i++] = temp[0];

    stringToSend[sizeof(stringToSend)-2] = '\r';
    stringToSend[sizeof(stringToSend)-1] = '\n';

    // send string over UART
    if (app_vars.uartDone == 1) {
        app_vars.uartDone              = 0;
        app_vars.uart_lastTxByteIndex  = 0;
        uart_writeByte(stringToSend[app_vars.uart_lastTxByteIndex]);
    }
}
