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

#define LENGTH_CRC      2
#define REGION_CHINA
// #define REGION_EUROPE
// #define REGION_USA

// LoRaWAN + region restrictions
#if defined(REGION_CHINA)
    #define REGION_ISM              FREQ_BAND_470_510 // image calibration
    #define REGION_MAX_DBM          PA_CONFIG_17_DBM 
    #define REGION_MAX_BW           LORA_BW_125
    #define REGION_UPLINK_CH_MAX    96    // total uplink channels
    #define REGION_DOWNLINK_CH_MAX  48    // total downlink channels
    #define REGION_UPLINK_START     470300000   // 470.3 MHz
    #define REGION_DOWNLINK_START   500300000   // 500.3 MHz
    #define REGION_CH_SPACING_UP    200000      // 200 kHz
    #define REGION_CH_SPACING_DOWN  200000      // 200 kHz
    #define REGION_MIN_SF_UPLINK    LORA_SF7    // radio min is SF 5
    #define REGION_MAX_SF_UPLINK    LORA_SF11   // 
    #define REGION_MIN_SF_DOWNLINK  LORA_SF7    // 
    #define REGION_MAX_SF_DOWNLINK  LORA_SF11   // actually max is SF12
                                                  // radio max is SF11                                            
#elif defined(REGION_EUROPE)
    #define REGION_ISM              FREQ_BAND_863_870 // image calibration
    #define REGION_MAX_DBM          PA_CONFIG_14_DBM  // 14 dBm (ERP) or 16 dBm (EIRP)
    #define REGION_MAX_BW           LORA_BW_125
    #define REGION_UPLINK_CH_MAX    3     // total uplink channels
    #define REGION_DOWNLINK_CH_MAX  3     // total downlink channels
    #define REGION_UPLINK_START     868100000   // 868.1 MHz
    #define REGION_DOWNLINK_START   868100000   // 868.1 MHz
    #define REGION_CH_SPACING_UP    200000      // 200 kHz
    #define REGION_CH_SPACING_DOWN  200000      // 200 kHz
    #define REGION_MIN_SF_UPLINK    LORA_SF7    // radio min is SF 5
    #define REGION_MAX_SF_UPLINK    LORA_SF11   // 
    #define REGION_MIN_SF_DOWNLINK  LORA_SF7    // 
    #define REGION_MAX_SF_DOWNLINK  LORA_SF11   // actually max is SF12
                                                  // radio max is SF11
#elif defined(REGION_USA)
    #define REGION_ISM              FREQ_BAND_902_928 // image calibration
    #define REGION_MAX_DBM          PA_CONFIG_22_DBM  // actually 30, radio max is 22 dBm 
    #define REGION_MAX_BW           LORA_BW_500
    #define REGION_UPLINK_CH_MAX    64    // total uplink channels
    #define REGION_DOWNLINK_CH_MAX  8     // total downlink channels
    #define REGION_UPLINK_START     902300000   // 902.3 MHz
    #define REGION_DOWNLINK_START   923300000   // 923.3 MHz
    #define REGION_CH_SPACING_UP    200000      // 200 kHz
    #define REGION_CH_SPACING_DOWN  600000      // 600 kHz
    #define REGION_MIN_SF_UPLINK    LORA_SF7    // radio min is SF 5
    #define REGION_MAX_SF_UPLINK    LORA_SF10   // SF 7-10 uplink
    #define REGION_MIN_SF_DOWNLINK  LORA_SF7    // SF 7-12 downlink
    #define REGION_MAX_SF_DOWNLINK  LORA_SF11   // actually max is SF12
                                                  // radio max is SF11
#else // china region settings
    #define REGION_ISM              FREQ_BAND_470_510 // image calibration
    #define REGION_MAX_DBM          PA_CONFIG_17_DBM 
    #define REGION_MAX_BW           LORA_BW_125
    #define REGION_UPLINK_CH_MAX    96    // total uplink channels
    #define REGION_DOWNLINK_CH_MAX  48    // total downlink channels
    #define REGION_UPLINK_START     470300000   // 470.3 MHz
    #define REGION_DOWNLINK_START   500300000   // 500.3 MHz
    #define REGION_CH_SPACING_UP    200000      // 200 kHz
    #define REGION_CH_SPACING_DOWN  200000      // 200 kHz
    #define REGION_MIN_SF_UPLINK    LORA_SF7    // radio min is SF 5
    #define REGION_MAX_SF_UPLINK    LORA_SF11   // 
    #define REGION_MIN_SF_DOWNLINK  LORA_SF7    // 
    #define REGION_MAX_SF_DOWNLINK  LORA_SF11   // actually max is SF12
                                                  // radio max is SF11
#endif
//=========================== typedef =========================================
#define MAX_PACKET_SIZE                 127
#define MAX_BUFFER_SIZE                0x80
#define PREAMBLE_LENGTH_32           0x0080

// radio info
typedef enum {
    LLCC68STATE_RESET               = 0x00,   ///< reset pin low/powering on.
    LLCC68STATE_STARTUP             = 0x01,   ///< chip still waking up.
    LLCC68STATE_STANDBY_RC          = 0x02,   ///< standby mode using RC oscillator.
    LLCC68STATE_STANDBY_XOSC        = 0x03,   ///< standby mode using XOSC oscillator.
    LLCC68STATE_FS                  = 0x04,   ///< fs mode, set the radio frequency.
    LLCC68STATE_TX                  = 0x05,   ///< tx mode, transmitting.
    LLCC68STATE_RX                  = 0x06,   ///< rx mode, receiving.
    LLCC68STATE_SLEEP               = 0x07,   ///< sleep mode, using RTC timer in low power mode.
    LLCC68STATE_ENABLE_CALIBRATING  = 0x08,   ///< begin calibration of all clocks.
    LLCC68STATE_CALIBRATION_DONE    = 0x09,   ///< all clocks finished calibrating.
    LLCC68STATE_ENABLE_IMAGE_CAL    = 0x0a,   ///< begin image calibration (ISM bands).
    LLCC68STATE_FREQUENCY_SET       = 0x0b,   ///< done configuring the frequency.
    LLCC68STATE_CONFIG_SET          = 0x0b,   ///< done configuring radio.
    LLCC68STATE_PACKET_LOADED       = 0x0c,   ///< packet loaded into radio FIFO
    LLCC68STATE_PACKET_READ         = 0x0d,   ///< packet retrieved from radio FIFO
} radio_llcc68_state_t;

// lora parameters
typedef enum {
    LORA_SF5      = 0x05,
    LORA_SF6      = 0x06,
    LORA_SF7      = 0x07,
    LORA_SF8      = 0x08,
    LORA_SF9      = 0x09,
    LORA_SF10     = 0x0A, 
    LORA_SF11     = 0x0B,
} loraSpreadingFactor_t;

typedef enum {
    LORA_BW_125   = 0x04,
    LORA_BW_250   = 0x05,
    LORA_BW_500   = 0x06,
} loraBandwidth_t;

typedef enum {
    LORA_CR_4_5   = 0x01,
    LORA_CR_4_6   = 0x02,
    LORA_CR_4_7   = 0x03,
    LORA_CR_4_8   = 0x04,
} loraCodingRate_t;

// low data rate optimize (LDRO)
typedef enum {
    LDRO_OFF      = 0x00,
    LDRO_ON       = 0x01,
} LoraLdro_t;

typedef enum {
    // -9 dBm (0xF7) to +22 dBm (0x16)
    TX_P22_DBM    = 0x16,
    TX_P10_DBM    = 0x0A,
    TX_N9_DBM     = 0xF7,
} radioPower_t;

typedef enum {
    RAMP_10U      = 0x00, // 10 us
    RAMP_20U      = 0x01, // 20 us
    RAMP_40U      = 0x02, // 40 us
    RAMP_80U      = 0x03, // 80 us
    RAMP_200U     = 0x04, // 200 us
    RAMP_800U     = 0x05, // 800 us
    RAMP_1700U    = 0x06, // 1700 us
    RAMP_3400U    = 0x07, // 3400 us
} radioRampUpTime_t;

typedef enum {
    LDO_MODE      = 0x00, // only ldo
    DC_DC_MODE    = 0x01, // dc + ldo
} regulatorMode_t;

typedef enum {
    VARIABLE_LENGTH_PACKET    = 0x00,
    FIXED_LENGTH_PACKET       = 0x01,
} headerType_t;

typedef enum {
    CRC_OFF       = 0x00,
    CRC_ON        = 0x01,
} crcType_t;

typedef enum {
    STD_IQ        = 0x00,
    INVERT_IQ     = 0x01,
} invertIq_t;

typedef enum {
    PUBLICSYNC    = PUBLICNETWORK,
    PRIVATESYNC   = PRIVATENETWORK,
} syncword_t;

// status bytes
typedef enum {
    UNUSED        = 0x00,
    RESERVED      = 0x01,
    STBY_RC       = 0x02,
    STBY_XOSC     = 0x03,
    MODE_FS       = 0x04,
    MODE_RX       = 0x05,
    MODE_TX       = 0x06, 
} chipMode_t;

typedef enum {
    RESERVED0     = 0x00,
    RESERVED1     = 0x01,
    DATA_TO_HOST  = 0x02,
    CMD_TIMEOUT   = 0x03,
    PROCESS_ERROR = 0x04,
    EXE_ERROR     = 0x05,
    TX_DONE       = 0x06,
} commandStatus_t;

typedef struct __attribute__((packed)){
    loraSpreadingFactor_t spreadingFactor;    // 0x05(SF7) to 0x0B(SF11)
    loraBandwidth_t       bandwidth;          // 0x04(125KHz) to 0x06(500kHz)
    loraCodingRate_t      codingRate;         // 0x01(4/5) to 0x04(4/8)
    LoraLdro_t            lowDataRateOptimize;// 0x00(off) 0x01(on)
} radioModulationParams_t;

typedef struct __attribute__((packed)){
    radioPower_t      txPowerDbm;       // 9(0xF7) to +22(0x16)
    radioRampUpTime_t txRampTime;       // 0x00(10us) to 0x07(3400us)
} radioTxParams_t;

typedef struct __attribute__((packed)){
    uint16_t          preambleLength;   // 0x0001 to 0xFFFF
    headerType_t      headerType;       // 0x00 Variable length
                                        // 0x01 Fixed length packet
    uint8_t           payloadLength;    // 0x00 to 0xFF   
    crcType_t         crcType;          // 0x00(crc OFF) or 0x01(crc on)
    invertIq_t        invertIq;         // 0x00(standard IQ setup) or
                                        // 0x01(inverted IQ setup)
} packetParams_t;

typedef struct __attribute__((packed)){
    uint8_t           txBaseAddress;    // 0x00
    uint8_t           rxBaseAddress;    // 0x80
} bufferBaseAddress_t;

typedef struct __attribute__((packed)){
   // Timeout Duration = timeout[] * 15.625 us
   uint8_t timeout[3];
} radioTimeout_t;

// same format as irqStatus
typedef struct __attribute__((packed)) {
    uint16_t irqMask;
    uint16_t dio1Mask;
    uint16_t dio2Mask;
    uint16_t dio3Mask;
} irqParams_t;

// lora mode only irq status format: 
// bit: 15|14|13|12|11|10|9            |8         |7       |6        |...
//      na|na|na|na|na|na|rx/tx timeout|cad detect|cad done|crc error|...
//
// ---> 5           |4           |3 |2              |1      |0      |
// ---> header error|header valid|na|preamble detect|rx done|tx done|
typedef struct __attribute__((packed)) {
    // MSB
    uint8_t cadDetected     : 1;
    uint8_t timeout         : 1;
    uint8_t reserved        : 6;

    // LSB
    uint8_t txDone          : 1;
    uint8_t rxDone          : 1;
    uint8_t preambleDetect  : 1;
    uint8_t syncWordDetect  : 1;
    uint8_t headerValid     : 1;
    uint8_t headerError     : 1;
    uint8_t crcError        : 1;
    uint8_t cadDone         : 1;
} radio_llcc68_irqStatus_t;

typedef struct __attribute__((packed)) {
    uint8_t         reserved1       : 1;
    commandStatus_t commandStatus   : 3;
    chipMode_t      chipMode        : 3;
    uint8_t         reserved7       : 1;
} radio_llcc68_status_t;

typedef struct {
    int8_t                  rssiPkt;
    int8_t                  snrPkt;
    int8_t                  signalRssiPk;
} radio_llcc68_packetStats_t;

typedef struct __attribute__((packed)) {
    // MSB
    uint8_t paPamp          : 1;
    uint8_t reserved9_15    : 7;
    
    // LSB
    uint8_t rc64Cal         : 1;
    uint8_t rc13MCal        : 1;
    uint8_t pllCal          : 1;
    uint8_t adcCal          : 1;
    uint8_t imgCal          : 1;
    uint8_t xoscStart       : 1;
    uint8_t pllLock         : 1;
    uint8_t reserved7       : 1;

} radio_llcc68_opError_t;

typedef struct {
    radioModulationParams_t loraModParams;
    radioTxParams_t         radioTxParams; //no rxParams needed
    packetParams_t          packetParams;
    uint8_t                 channel;
    syncword_t              syncword;
} radio_llcc68_config_t;

//=========================== variables =======================================

//=========================== prototypes ======================================

// radio init
void          radio_llcc68_init(void);
// radio reset
void          radio_llcc68_reset(void);
// radio control
void          radio_llcc68_loadPacket(uint8_t offset, 
                                      uint8_t* buffer, 
                                      uint8_t len);
void          radio_llcc68_lora_config(radio_llcc68_config_t radio);
void          radio_llcc68_txNow(radioTimeout_t timeout);
void          radio_llcc68_rxNow(radioTimeout_t timeout);
void          radio_llcc68_irq_clear(void);
void          radio_llcc68_getReceivedFrame(uint8_t* pBufRead,
                                            uint8_t* pLenRead,
                         radio_llcc68_packetStats_t* packetStatts);
// radio info
radio_llcc68_status_t      radio_llcc68_getStatus(void);
radio_llcc68_opError_t     radio_llcc68_getOpError(void);
radio_llcc68_irqStatus_t   radio_llcc68_getIrqstatus(void);
// private 
void          lorawan_channel_mapping(void);



//void     radio_llcc68_setStartFrameCb(radio_capture_cbt cb);
//void     radio_llcc68_setEndFrameCb(radio_capture_cbt cb);

// RX

/*
void     radio_getReceivedFrame(uint8_t* bufRead,
                                uint16_t* lenRead,
                                uint16_t  maxBufLen,
                                 int8_t* rssi,
                                uint8_t* lqi,
                                   bool* crc);
                                   */

// interrupt handlers
//void cb_compare(void);

#endif
