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
#define BEACON_CHANNEL  20

#define NUM_SAMPLES     SAMPLE_MAXCNT
#define LEN_UART_BUFFER 5// ((NUM_SAMPLES*4)+8)
#define LENGTH_SERIAL_FRAME  127              // length of the serial frame

#define ENABLE_DF       1

#define UART_LENGTH     5

const static uint8_t ble_device_addr[6] = { 
    0xaa, 0xbb, 0xcc, 0xcc, 0xbb, 0xaa
};

// get from https://openuuid.net/signin/:  a24e7112-a03f-4623-bb56-ae67bd653c73
const static uint8_t ble_uuid[16]       = {
    0xa2, 0x4e, 0x71, 0x12, 0xa0, 0x3f, 
    0x46, 0x23, 0xbb, 0x56, 0xae, 0x67,
    0xbd, 0x65, 0x3c, 0x73
};

#define NUM_SLOTS       5
#define SLOT_DURATION   (32768/200)*20  // 5ms@ (32768/200)
#define SENDING_OFFSET  (32768/1000)*20 // 1ms@ (32768/1000)
#define TURNON_OFFSET   (32768/2000)*20 // 0.5ms@ (32768/2000)

//define debug GPIO
#define DEBUG_PORT           1
#define DEBUG_PIN0           10
#define DEBUG_RADIO_PIN      11 

//=========================== variables =======================================
typedef enum {
    APP_STATE_TX         = 0x01,
    APP_STATE_RX         = 0x02,
    APP_STATE_OFF        = 0x04,
} app_state_t;

typedef struct {
    uint8_t              num_startFrame;
    uint8_t              num_endFrame;
    uint8_t              num_timer;
    uint8_t              num_slot;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
                bool            isTargetPkt;
                bool            isSynced;
                uint8_t         slot_timerId;
                uint8_t         inner_rxtimerId;
                uint8_t         inner_txtimerId;
                uint8_t         target_tag_id;
                app_state_t     state;

                uint8_t         slot_offset;
                uint8_t         pkt_sqn;
                uint32_t        time_slotStartAt;

                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;
                
                uint8_t         rxpk_packet[LENGTH_PACKET];
                uint8_t         rxpk_packet_len;
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;
                uint16_t        num_samples;
                uint32_t        sample_buffer[NUM_SAMPLES];
                
                uint8_t         uart_buffer_to_send[UART_LENGTH];
                uint16_t        uart_lastTxByteIndex;
     volatile   uint8_t         uartDone;
                int8_t          estimate_angle;

                uint8_t         antenna_array_id;
                uint32_t        capture_time;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);

void     cb_slot_timer(void);
void     cb_inner_slot_rxtimer(void);
void     cb_inner_slot_txtimer(void);

void     cb_uartTxDone(void);
uint8_t  cb_uartRxCb(void);

void     assemble_ibeacon_packet(uint8_t);
void nrf_gpio_cfg_output(uint8_t port_number, uint32_t pin_number);
//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
    uint16_t i;

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    // initialize board
    board_init();

    // turn radio off
    radio_rfOff();
    app_vars.state = APP_STATE_OFF;
    
    //set antenna array id
    app_vars.antenna_array_id = 1;
    app_vars.target_tag_id = 1;
    
    uart_setCallbacks(cb_uartTxDone,cb_uartRxCb);
    uart_enableInterrupts();

#if ENABLE_DF == 1
    antenna_CHW_tx_switch_init();
    radio_configure_direction_finding_CHW_antenna_switch(app_vars.antenna_array_id);
    radio_configure_direction_finding_manual_AoD();
#endif

    // add callback functions radio
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    app_vars.slot_timerId = 0;
    app_vars.inner_rxtimerId = 1;
    app_vars.inner_txtimerId = 2;

    //initial debugs GPIO
    nrf_gpio_cfg_output(DEBUG_PORT, DEBUG_PIN0);
    nrf_gpio_cfg_output(DEBUG_PORT, DEBUG_RADIO_PIN);


    // start sctimer
    sctimer_set_callback(app_vars.slot_timerId, cb_slot_timer);
    sctimer_set_callback(app_vars.inner_rxtimerId, cb_inner_slot_rxtimer);    //in slot 0, set when to turn on the radio for receiving the sync pacekt
    sctimer_set_callback(app_vars.inner_txtimerId, cb_inner_slot_txtimer);    //in slot 1, set when to turn on the radio for sending a DF packet

    //app_vars.time_slotStartAt = sctimer_readCounter()+SLOT_DURATION;
    //sctimer_setCompare(app_vars.slot_timerId, app_vars.time_slotStartAt);
    
    //  The handhold device start with keep listening, until achieve sync.
    radio_rfOn();
    radio_setFrequency(CHANNEL, FREQ_RX);
    radio_rxEnable();
    app_vars.state = APP_STATE_RX;
    radio_rxNow();
    //sctimer_enable(app_vars.slot_timerId);

    // sleep
    while (1){
        //if (app_vars.slot_offset == 0) {
        //    NRF_P1_NS->OUTSET =  1 << DEBUG_PIN0;
        //}
        //else {
        //    NRF_P1_NS->OUTCLR =  1 << DEBUG_PIN0;
        //}

        //if (app_vars.state == APP_STATE_OFF) {
        //    NRF_P1_NS->OUTCLR =  1 << DEBUG_RADIO_PIN;
        //} else {
        //    NRF_P1_NS->OUTSET =  1 << DEBUG_RADIO_PIN;
        //}
        board_sleep();
    }
}
  

//=========================== private =========================================

void assemble_ibeacon_packet(uint8_t sqn) {

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
    app_vars.packet[i++]  = 1;                  //34 byte,  represent packet type, 1 means this packet is a DF packet
    app_vars.packet[i++]  = 1;      //app_vars.target_tag_id;   // represent which tag need to be find
    app_vars.packet[i++]  = 0x00;               // power level
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {

    if (app_vars.state == APP_STATE_RX) {
        app_vars.capture_time = timestamp;
    }
    app_dbg.num_startFrame++;
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {

    app_dbg.num_endFrame++;

    if (app_vars.state == APP_STATE_RX) {
        //received a ble packet
        app_vars.isTargetPkt = FALSE;

        radio_rfOff();
        app_vars.state = APP_STATE_OFF;

        radio_getReceivedFrame(
            app_vars.rxpk_packet,
            &app_vars.rxpk_packet_len,
            sizeof(app_vars.rxpk_packet),
            &app_vars.rxpk_rssi,
            &app_vars.rxpk_lqi,
            &app_vars.rxpk_crc
        );
        
        if (app_vars.rxpk_packet[0] == 0x42 & app_vars.rxpk_packet[1] == 0x21) {
            app_vars.isTargetPkt = TRUE;      //Check if received packet is a legal plast system packet
        }

        if  (app_vars.isTargetPkt) {
            switch (app_vars.rxpk_packet[33]) {
            case 0:
                // 0 represent this packet is a sync packet
                if (app_vars.isSynced) {
                    app_vars.time_slotStartAt = app_vars.capture_time + SLOT_DURATION - SENDING_OFFSET;
                    sctimer_setCompare(app_vars.slot_timerId, app_vars.time_slotStartAt);
                } else {
                    app_vars.slot_offset = 0;
                    app_vars.time_slotStartAt = app_vars.capture_time + SLOT_DURATION - SENDING_OFFSET;
                    sctimer_setCompare(app_vars.slot_timerId, app_vars.time_slotStartAt);
                    app_vars.isSynced = TRUE;
                }
            break;
            case 2:
                // 2 represent this packet is a broadcast packet pacekt
                
                // need to do
                // read the position information
                // send the information through serials

                app_vars.estimate_angle = app_vars.rxpk_packet[34];
                
                uint8_t i;
                i = 0;
                app_vars.uart_buffer_to_send[i++] = app_vars.target_tag_id;
                app_vars.uart_buffer_to_send[i++] = app_vars.estimate_angle;

                app_vars.uart_buffer_to_send[i++] = 0xff;
                app_vars.uart_buffer_to_send[i++] = 0xff;
                app_vars.uart_buffer_to_send[i++] = 0xff;

                app_vars.uart_lastTxByteIndex = 0;
                uart_writeByte(app_vars.uart_buffer_to_send[0]);

            break;
            }
            return;   //legal packet and task done, end the EoF interrupt
        }

        // if illegal pkt, keep listening; If illegal pkt but synced, do nothing
        if (app_vars.isSynced == FALSE) {
            radio_rfOn();
            app_vars.state = APP_STATE_RX;
            radio_rxEnable();
            radio_rxNow();
        }
        return;   //illegal pkt end the EoF interrupt
    }


    if (app_vars.state == APP_STATE_TX) {
        radio_rfOff();
        app_vars.state = APP_STATE_OFF;
        return;   // finished sending a DF pacekt, end the EoF interrupt
    }  
    
}

void cb_slot_timer(void) {

      leds_error_toggle();
      // update slot offset
      app_vars.slot_offset = (app_vars.slot_offset+1)%NUM_SLOTS;
      // schedule next slot
      app_vars.time_slotStartAt += SLOT_DURATION;
      sctimer_setCompare(app_vars.slot_timerId, app_vars.time_slotStartAt);

      
      // check which slotoffset is right now

      switch(app_vars.slot_offset) {
      case 0:
          // set when to turn on the radio for receiving a sync packet
          sctimer_setCompare(app_vars.inner_rxtimerId, app_vars.time_slotStartAt - SLOT_DURATION + SENDING_OFFSET - TURNON_OFFSET);
      break;
      case 1:
     
          // set when to send packet out
          radio_rfOff();
          sctimer_setCompare(app_vars.inner_txtimerId, app_vars.time_slotStartAt - SLOT_DURATION + SENDING_OFFSET);
          
          // prepare to send
          // prepare packet
          app_vars.packet_len = sizeof(app_vars.packet);
          assemble_ibeacon_packet(app_vars.pkt_sqn++);

          //prepare radio
          //radio_rfOn();
          //app_vars.state = APP_STATE_TX;
          radio_setFrequency(CHANNEL, FREQ_RX);
          radio_loadPacket(app_vars.packet,LENGTH_PACKET);
          //radio_txEnable();
      break;
      case 3:
          //turn on the radio for receiving the broadcast packet
          radio_rfOn();
          app_vars.state = APP_STATE_RX;
          radio_setFrequency(BEACON_CHANNEL, FREQ_RX);
          radio_rxEnable();
          radio_rxNow();
      break;
      default:
          radio_rfOff();
          app_vars.state = APP_STATE_OFF;
      break;
      }
   
}

void cb_inner_slot_rxtimer(void) {
    app_dbg.num_timer++;

    radio_rfOn();
    app_vars.state = APP_STATE_RX;
    radio_setFrequency(CHANNEL, FREQ_RX);
    radio_rxEnable();
    radio_rxNow();
}

void cb_inner_slot_txtimer(void) {
    radio_rfOn();
    //radio_setFrequency(CHANNEL, FREQ_RX);
    //radio_loadPacket(app_vars.packet,LENGTH_PACKET);
    radio_txEnable();
    radio_txNow();
}

void cb_uartTxDone(void) {

   app_vars.uart_lastTxByteIndex++;
   if (app_vars.uart_lastTxByteIndex<LEN_UART_BUFFER) {
      uart_writeByte(app_vars.uart_buffer_to_send[app_vars.uart_lastTxByteIndex]);
   } else {
      app_vars.uartDone = 1;
   }
}

uint8_t cb_uartRxCb(void) {
   uint8_t byte;
   
   // read received byte
   byte = uart_readByte();
   
   // echo that byte over serial
   uart_writeByte(byte);
   
   return 0;
}
