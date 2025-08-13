#ifndef __LLCC668_RADIO_H
#define __LLCC668_RADIO_H

/**
\addtogroup BSP
\{
\addtogroup radio
\{

\brief Cross-platform declaration "radio" bsp module.

*/

#include "llcc68.h"

//=========================== define ==========================================

#define LENGTH_CRC 2

//=========================== typedef =========================================

typedef enum {
    LORA_SF5  = 0x05,
    LORA_SF6  = 0x06,
    LORA_SF7  = 0x07,
    LORA_SF8  = 0x08,
    LORA_SF9  = 0x09,
    LORA_SF10 = 0x0A, 
    LORA_SF11 = 0x0B,
}loraSpreadingFactor_t;

typedef enum {
    LORA_BW_125 = 0x04,
    LORA_BW_250 = 0x05,
    LORA_BW_500 = 0x06,
}loraBandwidth_t;

typedef enum {
    LORA_CR_4_5 = 0x01,
    LORA_CR_4_6 = 0x02,
    LORA_CR_4_7 = 0x03,
    LORA_CR_4_8 = 0x04,
}loraCodingRate_t;

// low data rate optimize (LDRO)
typedef enum {
    LDRO_OFF = 0x00,
    LDRO_ON  = 0x01,
}LoraLdro_t;

typedef enum {
    // -9 dBm (0xF7) to +22 dBm (0x16)
    TX_P22_DBM    = 0x16,
    TX_P10_DBM    = 0x0A,
    TX_N9_DBM     = 0xF7,
}radioPower_t;

typedef enum {
    RAMP_10U      = 0x00, // 10 us
    RAMP_20U      = 0x01, // 20 us
    RAMP_40U      = 0x02, // 40 us
    RAMP_80U      = 0x03, // 80 us
    RAMP_200U     = 0x04, // 200 us
    RAMP_800U     = 0x05, // 800 us
    RAMP_1700U    = 0x06, // 1700 us
    RAMP_3400U    = 0x07, // 3400 us
}radioRampUpTime_t;

typedef enum {
    VARIABLE_LENGTH_PACKET    = 0x00,
    FIXED_LENGTH_PACKET       = 0x01,
}headerType_t;

typedef enum {
    CRC_OFF       = 0x00,
    CRC_ON        = 0x01,
}crcType_t;

typedef enum {
    STD_IQ        = 0x00,
    INVERT_IQ     = 0x01,
}invertIq_t;

typedef struct {
    loraSpreadingFactor_t SpreadingFactor;
    loraBandwidth_t       Bandwidth;
    loraCodingRate_t      CodingRate;
    LoraLdro_t            LowDataRateOptimize;
}radioModulationParams_t;

typedef struct {
    radioPower_t      TxPowerDbm;
    radioRampUpTime_t TxRampTime;
}radioTxParams_t;

typedef struct {
    uint16_t          PreambleLength;
    headerType_t      HeaderType;
    uint8_t           PayloadLength;
    crcType_t         CrcType;
    invertIq_t        InvertIq;
}packetParams_t;

typedef struct {
    uint8_t           TxBaseAddress;
    uint8_t           RxBaseAddress;
}bufferBaseAddress_t;

// lora mode only irq status format: 
// bit: 15|14|13|12|11|10|9            |8         |7       |6        |...
//      na|na|na|na|na|na|rx/tx timeout|cad detect|cad done|crc error|...
//
// ---> 5           |4           |3 |2              |1      |0      |
// ---> header error|header valid|na|preamble detect|rx done|tx done|
typedef struct __attribute__((packed)) {
    // need to double check bit order
    uint8_t TxDone          : 1;
    uint8_t RxDone          : 1;
    uint8_t PreambleDetect  : 1;
    uint8_t SyncWordDetect  : 1;
    uint8_t HeaderValid     : 1;
    uint8_t HeaderError     : 1;
    uint8_t CrcError        : 1;
    uint8_t CadDone         : 1;

    uint8_t CadDetected     : 1;
    uint8_t Timeout         : 1;
    uint8_t reserved        : 6;
}irqStatus_t;

/*
typedef enum {
   RADIOSTATE_STOPPED             = 0x00,   ///< Completely stopped.
   RADIOSTATE_RFOFF               = 0x01,   ///< Listening for commands, but RF chain is off.
   RADIOSTATE_SETTING_FREQUENCY   = 0x02,   ///< Configuring the frequency.
   RADIOSTATE_FREQUENCY_SET       = 0x03,   ///< Done configuring the frequency.
   RADIOSTATE_LOADING_PACKET      = 0x04,   ///< Loading packet into the radio's TX buffer.
   RADIOSTATE_PACKET_LOADED       = 0x05,   ///< Packet is fully loaded in the radio's TX buffer.
   RADIOSTATE_ENABLING_TX         = 0x06,   ///< The RF TX chaing is being enabled (includes locking the PLL).
   RADIOSTATE_TX_ENABLED          = 0x07,   ///< Radio ready to transmit.
   RADIOSTATE_TRANSMITTING        = 0x08,   ///< Busy transmitting bytes.
   RADIOSTATE_ENABLING_RX         = 0x09,   ///< The RF RX chain is being enabled (includes locking the PLL).
   RADIOSTATE_LISTENING           = 0x0a,   ///< RF chain is on, listening, but no packet received yet.
   RADIOSTATE_RECEIVING           = 0x0b,   ///< Busy receiving bytes.
   RADIOSTATE_TXRX_DONE           = 0x0c,   ///< Frame has been sent/received completely.
   RADIOSTATE_TURNING_OFF         = 0x0d,   ///< Turning the RF chain off.
} radio_state_t;

typedef enum {
   FREQ_TX                        = 0x01,
   FREQ_RX                        = 0x02,
} radio_freq_t;

typedef void (*radio_capture_cbt)(PORT_TIMER_WIDTH timestamp);
*/
//=========================== variables =======================================

//=========================== prototypes ======================================

// admin
void          radio_llcc68_init(void);
//void     radio_llcc68_setStartFrameCb(radio_capture_cbt cb);
//void     radio_llcc68_setEndFrameCb(radio_capture_cbt cb);
// reset
void          radio_llcc68_reset(void);
// RF admin

uint8_t       radio_llcc68_get_status(void);
void          radio_llcc68_wait_on_busy(void);
irqStatus_t   radio_llcc68_irq_status(void);
//void     radio_llcc68_setFrequency(uint16_t channel, radio_freq_t tx_or_rx);
//void     radio_llcc68_rfOn(void);
//void     radio_llcc68_rfOff(void);
// TX
//void     radio_llcc68_loadPacket(uint8_t* packet, uint16_t len);
//void     radio_llcc68_txEnable(void);
//void     radio_llcc68_txNow(void);
// RX
//void     radio_llcc68_rxEnable(void);
//void     radio_llcc68_rxNow(void);
/*
void     radio_getReceivedFrame(uint8_t* bufRead,
                                uint16_t* lenRead,
                                uint16_t  maxBufLen,
                                 int8_t* rssi,
                                uint8_t* lqi,
                                   bool* crc);
                                   */

// interrupt handlers
void cb_compare(void);
//void    radio_isr(void);

/**
\}
\}
*/

#endif
