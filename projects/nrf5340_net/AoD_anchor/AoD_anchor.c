/**
\brief This program is for AoD anchor
The AoD anchor will preiodicly send a packet to AoD nodes.

Use one timer to schedule a periodic slot.
Use another timer to schedule a short time which used for control
when send the packet in one slot.

\author Manjiang Cao <mcao999@connect.hkust-gz.edu.cn>, Nov. 2024.
*/

#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "uart.h"
#include "radio_df.h"
#include "radio_CHW_df.h"
#include "timer.h"

//=========================== defines =========================================

#define LENGTH_BLE_CRC  3
#define LENGTH_PACKET   125+LENGTH_BLE_CRC  ///< maximum length is 127 bytes
#define CHANNEL         0              ///< 0~39
#define TIMER_PERIOD    (0xffff>>3)     ///< 0xffff = 2s@32kHz
#define TXPOWER         0xD5            ///< 2's complement format, 0xD8 = -40dbm

#define NUM_SAMPLES     SAMPLE_MAXCNT
#define LEN_UART_BUFFER ((NUM_SAMPLES*4)+8)
#define LENGTH_SERIAL_FRAME  127              // length of the serial frame

#define ENABLE_DF       1

#define TIMER0_CC_SCHEDULE  0
#define TIMER0_CC_CAPTURE   1

#define TIMER1_CC_SCHEDULE  0
#define TIMER1_CC_CAPTURE   1

uint16_t length = 0;

const static uint8_t ble_device_addr[6] = { 
    0xaa, 0xbb, 0xcc, 0xcc, 0xbb, 0xaa
};

// get from https://openuuid.net/signin/:  a24e7112-a03f-4623-bb56-ae67bd653c73
const static uint8_t ble_uuid[16]       = {
    0xa2, 0x4e, 0x71, 0x12, 0xa0, 0x3f, 
    0x46, 0x23, 0xbb, 0x56, 0xae, 0x67,
    0xbd, 0x65, 0x3c, 0x73
};

//=========================== variables =======================================

typedef struct {
    uint8_t              num_startFrame;
    uint8_t              num_endFrame;
    uint8_t              num_timer;
    uint8_t              num_slot;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
                uint8_t         flags;
                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;
                uint16_t        num_samples;
                uint32_t        sample_buffer[NUM_SAMPLES];
                uint8_t         uart_buffer_to_send[LEN_UART_BUFFER];
                uint16_t        uart_lastTxByteIndex;
     volatile   uint8_t         uartDone;
                uint8_t         txpk_txNow;
                uint8_t         packet_counter;


                uint8_t         uart_txFrame[LENGTH_SERIAL_FRAME];
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_timer0(void);
void     cb_timer1(void);

void     cb_uartTxDone(void);
uint8_t  cb_uartRxCb(void);

void     assemble_ibeacon_packet(uint8_t);

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
    uint16_t i;

    uint8_t freq_offset;
    uint8_t sign;
    uint8_t read;
    
    uint8_t current_time;
    //slot_index == 0  --->  Tx 
    //slot_index == 1~4 ---> sleep                      

    uint8_t packet_counter;
    packet_counter = 0;

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    // initialize board
    board_init();

#if ENABLE_DF == 1
    antenna_CHW_tx_switch_init();
    radio_configure_direction_finding_CHW_antenna_switch();

#endif

    //antenna_CHW_switch_init();
    // set_antenna_CHW_switches();
    //uart_setCallbacks(cb_uartTxDone,cb_uartRxCb);
    //uart_enableInterrupts();

    // add callback functions radio
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    // prepare packet
    app_vars.packet_len = sizeof(app_vars.packet);
    
    //start timer
    timer0_init();        //timer0   NRF_TIMER0_NS    16MHz  used for schedule slot, each slot is 500us i.e. 8000 ticks
    timer1_init();        //timer1   NRF_TIMER1_NS    16MHz  used for schedule when start send a packet in Tx slot. When Tx slot passed 100us i.e. 1600 ticks start send a packet

    timer_clear(NRF_TIMER0_NS);
    timer_clear(NRF_TIMER1_NS);
    timer_schedule(NRF_TIMER0_NS, timer_getCapturedValue(NRF_TIMER0_NS, TIMER0_CC_SCHEDULE), 8000000);

    timer0_set_callback(TIMER0_CC_SCHEDULE, cb_timer0);
    timer1_set_callback(TIMER1_CC_SCHEDULE, cb_timer1);

    timer_start(NRF_TIMER0_NS);
    timer_start(NRF_TIMER1_NS);

    // prepare radio
    radio_rfOn();
    // freq type only effects on scum port
    radio_setFrequency(CHANNEL, FREQ_RX);

#if ENABLE_DF == 1
    radio_configure_direction_finding_manual_AoD();
#endif

    // switch in RX by default
    //radio_txEnable();
    //radio_rxNow();
    //app_vars.state = APP_STATE_RX;

    // start by a transmit
    //app_vars.flags |= APP_FLAG_TIMER;
  
    while (1){

        // sleep while waiting for at least one of the rxpk_done to be set
        app_vars.txpk_txNow==0;
        while (app_vars.txpk_txNow==0) {
            board_sleep();
        }
        
        leds_error_toggle();
        // prepare packet
        app_vars.packet_len = sizeof(app_vars.packet);
        
        assemble_ibeacon_packet(packet_counter);

        // start transmitting packet
        //radio_rfOn();
        //radio_setFrequency(CHANNEL, FREQ_RX);
        radio_loadPacket(app_vars.packet,LENGTH_PACKET);
        radio_txEnable();
        radio_txNow();
        packet_counter++;


    }
}
  

//=========================== private =========================================

void assemble_ibeacon_packet(uint8_t packet_counter) {

    uint8_t i;
    i=0;

    memset( app_vars.packet, 0x00, sizeof(app_vars.packet) );

    app_vars.packet[i++]  = 0x42;               // BLE ADV_NONCONN_IND (this is a must)
    app_vars.packet[i++]  = 0x21;               // Payload length
    app_vars.packet[i++]  = ble_device_addr[0]; // BLE adv address byte 0
    app_vars.packet[i++]  = ble_device_addr[1]; // BLE adv address byte 1
    app_vars.packet[i++]  = ble_device_addr[2]; // BLE adv address byte 2
    app_vars.packet[i++]  = ble_device_addr[3]; // BLE adv address byte 3
    app_vars.packet[i++]  = ble_device_addr[4]; // BLE adv address byte 4
    app_vars.packet[i++]  = ble_device_addr[5]; // BLE adv address byte 5

    app_vars.packet[i++]  = 0x1a;
    app_vars.packet[i++]  = 0xff;
    app_vars.packet[i++]  = 0x4c;
    app_vars.packet[i++]  = 0x00;

    app_vars.packet[i++]  = 0x02;
    app_vars.packet[i++]  = 0x15;
    memcpy(&app_vars.packet[i], &ble_uuid[0], 16);
    i                    += 16;
    app_vars.packet[i++]  = 0x00;               // major
    app_vars.packet[i++]  = 0xff;
    app_vars.packet[i++]  = 0x00;               // minor
    app_vars.packet[i++] = packet_counter & 0x7F;
    app_vars.packet[i++]  = TXPOWER;            // power level
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    // set flag
    //app_vars.flags |= APP_FLAG_START_FRAME;

    //leds_sync_on();
    // update debug stats
    app_dbg.num_startFrame++;

}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
    bool     expectedFrame;
    uint8_t  i;

    // set flag
    //app_vars.flags |= APP_FLAG_END_FRAME;

    // after send a packet, turn off the radio
    //app_vars.txpk_txNow = 0;
    //radio_rfOff();
}

void cb_timer0(void) {
      app_dbg.num_slot++;

      //Each loop contain 5 slots, slot 0 is Tx
      if (app_dbg.num_slot == 5) {
          app_dbg.num_slot = 0;
      }
      
      // if is 0 slot, timer1 schedule for Tx
      if (app_dbg.num_slot == 0) {
          timer_schedule(NRF_TIMER1_NS, TIMER1_CC_SCHEDULE, timer_getCapturedValue(NRF_TIMER1_NS, TIMER1_CC_SCHEDULE)+1600000);
          app_vars.txpk_txNow = 1;

      } 

    timer_schedule(NRF_TIMER0_NS, TIMER0_CC_SCHEDULE, timer_getCapturedValue(NRF_TIMER0_NS, TIMER0_CC_SCHEDULE)+8000000);
}

void cb_timer1(void) {

    //app_vars.txpk_txNow = 1;
    radio_txNow();

}

