/**
\brief This program shows the use of the pwm to control robot.

\author Tengfei Chang <tengfeichang@hkust-gz.edu.cn>, November 2025.
*/

#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "uart.h"
#include "motor.h"
#include "eui64.h"

//=========================== defines =========================================

#define LENGTH_PACKET   125+LENGTH_CRC  ///< maximum length is 127 bytes
#define LEN_PKT_TO_SEND 20+LENGTH_CRC
#define CHANNEL         25             ///< 11=2.405GHz
#define TIMER_PERIOD    (0xffff>>4)    ///< 0xffff = 2s@32kHz
#define RSSI_HISTORY_LEN 16
#define MOVING_RSSI_THRESHOLD 1        /// unit: dbm
#define TARGET_RSSI     -30            /// unit: dbm
#define TARGET_ID       0xeb

uint8_t stringToSend[]  = "00112233aabbccdd \r\n";
const uint8_t mask[] = "wsan-robot";

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

                uint8_t         robot_id[8];
                uint8_t         follow_neighbor_id[8];
                int8_t          rssi_history[RSSI_HISTORY_LEN];
                uint8_t         rssi_index;
                int8_t          rssi_avg;
                bool            has_neighbor;

                float           velocity;
                float           direction; 
                float           rotation_speed;

                uint8_t         timer_counts;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_timer(void);

void     cb_uart_tx_done(void);
uint8_t  cb_uart_rx(void);

void     moving_decision(
    float* moving_speed, 
    float* moving_direction, 
    float* rotation_speed
);

//=========================== helper ==========================================

char     byte_to_hexchar(uint8_t byte);

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
    uint8_t i, j;

    uint8_t freq_offset;
    uint8_t sign;
    uint8_t read;

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    // initialize board
    board_init();

    // get my id
    eui64_get(app_vars.robot_id);

    // setup UART
    uart_setCallbacks(cb_uart_tx_done,cb_uart_rx);
    uart_enableInterrupts();

    app_vars.uartDone = 1;

    // add callback functions radio
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    // start bsp timer
    sctimer_set_callback(cb_timer);
    sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
    sctimer_enable();

    pwm_multi_init();
    while(app_vars.timer_counts==0); 

    pwm_start(PWM_0);  
    pwm_start(PWM_1);

    app_vars.velocity       = 100;
    app_vars.direction      = 90 % 360;
    app_vars.rotation_speed = 0;

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

                        if (memcmp(app_vars.packet, mask, sizeof(mask)!=0)) {
                            // not target packet, no future process
                            leds_error_off();
                            break;
                        }

                        if (app_vars.has_neighbor) {
                            if (
                                memcmp(
                                    app_vars.follow_neighbor_id, 
                                    &app_vars.packet[sizeof(mask)], 
                                    8
                                ) == 0
                            ) {
                                app_vars.rssi_history[app_vars.rssi_index++] = app_vars.rxpk_rssi;
                            }
                        } else {
                            app_vars.has_neighbor = true;
                            memcpy(app_vars.follow_neighbor_id, &app_vars.packet[sizeof(mask)], 8);
                            app_vars.rssi_history[app_vars.rssi_index++] = app_vars.rxpk_rssi;
                        }

                        if (app_vars.rssi_index == RSSI_HISTORY_LEN) {
                            app_vars.rssi_index = 0;
                        }

                        memcpy(&stringToSend[0], &app_vars.packet[0], 16);
                        stringToSend[16] = app_vars.rxpk_rssi;
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
                    i   = 0;
                    memcpy(&app_vars.packet[i], mask, sizeof(mask));
                    i  += sizeof(mask);
                    memcpy(&app_vars.packet[i], app_vars.robot_id, 8);
                    i  += 8;
                    memcpy(&app_vars.packet[i], app_vars.follow_neighbor_id, 8);
                    i  += 8;

                    // start transmitting packet
                    radio_loadPacket(app_vars.packet,i+2);
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

    app_vars.timer_counts++;

    sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);

    if (app_vars.robot_id[7]==TARGET_ID) {
        // target node don't move
        return;
    }

    // check if robot needs to move
    
    //    only move every 16 times
    if (app_vars.timer_counts%RSSI_HISTORY_LEN != 0) {
        return;
    }

    moving_decision(&app_vars.velocity, &app_vars.direction, &app_vars.rotation_speed);
    car_control(app_vars.velocity, app_vars.direction, app_vars.rotation_speed);

    leds_debug_toggle();
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

void     moving_decision(
    float* moving_speed, 
    float* moving_direction, 
    float* rotation_speed
) {
    uint8_t i;

    int16_t rssi_sum;

    rssi_sum  = 0;
    for (i=0;i<RSSI_HISTORY_LEN;i++) {
        if (app_vars.rssi_history[i] == 0) {
            // not enough rssi samples yet, don't move in this case 
            *moving_speed     = 0;
            return;
        } else {
            rssi_sum += app_vars.rssi_history[i];
        }
    }

    if (app_vars.rssi_avg == 0) {
        app_vars.rssi_avg = rssi_sum/RSSI_HISTORY_LEN;
        // no history rssi_avg, don't move in this case
        *moving_speed     = 0;
        return;
    } else {
        if (rssi_sum/RSSI_HISTORY_LEN >= TARGET_RSSI) {
            // target reached stop moving
            *moving_speed     = 0;
            *moving_direction = 0;
        } else {
            if (rssi_sum/RSSI_HISTORY_LEN - app_vars.rssi_avg > MOVING_RSSI_THRESHOLD) {
                // this is the right direction, keep moving with current direction
                *moving_speed     = 100;
            } else {
                // change a direction and keep moving
                *moving_speed     = 100;
                *moving_direction += 90;    
                if (*moving_direction>=360) {
                    *moving_direction -= 360;
                }
            }
        }
    }
    app_vars.rssi_avg = rssi_sum/RSSI_HISTORY_LEN;
}


//=========================== helper ==========================================

/*
\input: a byte value < 16.
\output: return the hex format of the input in char type 
*/
char byte_to_hexchar(uint8_t byte) {

    if (byte>=0x10) {
        return 0;
    }

    char output;
    if (byte < 0x0a) {
        output = (byte        & 0x0f) + '0';
    } else {
        output = ((byte-0x0a) & 0x0f) + 'a';
    }

    return output;
}