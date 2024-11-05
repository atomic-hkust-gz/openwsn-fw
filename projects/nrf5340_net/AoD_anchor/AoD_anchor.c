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

#define NUM_SAMPLES     SAMPLE_MAXCNT
#define LEN_UART_BUFFER ((NUM_SAMPLES*4)+8)
#define LENGTH_SERIAL_FRAME  127              // length of the serial frame

#define ENABLE_DF       1

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
#define SLOT_DURATION   (32768/200)  // 5ms@ (32768/200)
#define SENDING_OFFSET  (32768/1000) // 1ms@ (32768/200)

//=========================== variables =======================================

typedef struct {
    uint8_t              num_startFrame;
    uint8_t              num_endFrame;
    uint8_t              num_timer;
    uint8_t              num_slot;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {

                uint8_t         slot_timerId;
                uint8_t         inner_timerId;

                uint8_t         slot_offset;
                uint8_t         pkt_sqn;
                uint32_t        time_slotStartAt;

                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);

void     cb_slot_timer(void);
void     cb_inner_slot_timer(void);

void     assemble_ibeacon_packet(uint8_t);

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

#if ENABLE_DF == 1
    antenna_CHW_tx_switch_init();
    radio_configure_direction_finding_CHW_antenna_switch();
    radio_configure_direction_finding_manual_AoD();
#endif

    // add callback functions radio
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    app_vars.slot_timerId = 0;
    app_vars.inner_timerId = 1;

    // start sctimer
    sctimer_set_callback(app_vars.slot_timerId, cb_slot_timer);
    sctimer_set_callback(app_vars.inner_timerId, cb_inner_slot_timer);
    app_vars.time_slotStartAt = sctimer_readCounter()+SLOT_DURATION;
    sctimer_setCompare(app_vars.slot_timerId, app_vars.time_slotStartAt);

    sctimer_enable(app_vars.slot_timerId);

    // sleep
    while (1){
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
    app_vars.packet[i++]  = sqn;
    app_vars.packet[i++]  = 0x00;               // power level
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {

}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {

    radio_rfOff();
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
     
          // set when to send packet out
          sctimer_setCompare(app_vars.inner_timerId, app_vars.time_slotStartAt+SENDING_OFFSET);
          
          // prepare to send
          
          // prepare packet
          app_vars.packet_len = sizeof(app_vars.packet);
          assemble_ibeacon_packet(app_vars.pkt_sqn++);

          //prepare radio
          radio_rfOn();
          radio_setFrequency(CHANNEL, FREQ_RX);
          radio_loadPacket(app_vars.packet,LENGTH_PACKET);
          radio_txEnable();
      break;
      case 1:
      break;
      default:
      break;
      }

}

void cb_inner_slot_timer(void) {

    radio_txNow();
}

