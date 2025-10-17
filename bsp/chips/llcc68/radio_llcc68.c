/**
 * brief LLCC68 Lora definition of the "radio" bsp module. 
 *
*/

#include "nrf52840.h"
#include "nrf52840_bitfields.h"
#include "board_info.h"
#include "sctimer.h"
#include "debugpins.h"
#include "leds.h"
#include "llcc68.h"
#include "radio_llcc68.h"
#include "gpio_irq.h"


//=========================== defines =========================================
#define DUBUG

#define NRF_GPIO_PIN_MAP(port, pin) (((port) << 5) | ((pin) & 0x1F))
// Pin assignments
#define LLCC68_RESET_PIN NRF_GPIO_PIN_MAP(1,8)    // P1.08

#define IRQMASK                     LSB_FIRST_16(0xFFFF)
#define IRQTXDONE                   LSB_FIRST_16(0x0001)  // tx done 
#define IRQRXDONE                   LSB_FIRST_16(0x0002)  // rx done
#define IRQPREAMBLEDETECTED         LSB_FIRST_16(0x0004)  // preambleDetected 
#define IRQHEADERVALID              LSB_FIRST_16(0x0010)  // headerValid
#define IRQHEADERERROR              LSB_FIRST_16(0x0020)  // headerErr
#define IRQCRCERROR                 LSB_FIRST_16(0x0040)  // crcErr
#define IRQCADDONE                  LSB_FIRST_16(0x0080)  // cadDone 
#define IRQCADDETECTED              LSB_FIRST_16(0x0100)  // cadDetected
#define IRQTIMEOUT                  LSB_FIRST_16(0x0200)  // rx/tx timeout
#define DIO2MASK                    0x0000
#define DIO3MASK                    0x0000
#define LORA_TX_BASE_ADDR             0x00
#define LORA_RX_BASE_ADDR             0x80    // max packet size = 128
//=========================== variables =======================================

typedef struct {
    //radio_capture_cbt     startFrame_cb;
    //radio_capture_cbt     endFrame_cb;
    radio_llcc68_status_t     status;
    #if defined(DEBUG)
      radio_llcc68_state_t    state; 
      radio_llcc68_opError_t  opError;
    #endif

    //uint8_t               payload[1+MAX_PACKET_SIZE] __attribute__ ((aligned));
    //bool                  hfc_started;
} radio_llcc68_vars_t;


typedef struct {
    uint32_t frequency;   // frequency in Hz
} lorawan_channel_t;

static radio_llcc68_vars_t radio_vars;

#if defined(REGION_EUROPE)
    static lorawan_channel_t lorawan_ch_map[REGION_UPLINK_CH_MAX]; 
#else
    static lorawan_channel_t lorawan_ch_map[REGION_UPLINK_CH_MAX +
                                            REGION_DOWNLINK_CH_MAX];
#endif


//=========================== public ==========================================

// radio init
void radio_llcc68_init(void) { 
    radio_llcc68_irqStatus_t irqStatus;
    uint8_t value;
    uint8_t mulitParam[4];

    // clear internal variables
    memset(&radio_vars, 0, sizeof(radio_vars));
    memset(&lorawan_ch_map, 0, sizeof(lorawan_ch_map));
    memset(&irqStatus, 0, sizeof(irqStatus));
    memset(&mulitParam, 0, sizeof(mulitParam));
    
    #if defined(DEBUG)
      radio_vars.state = LLCC68STATE_STANDBY_RC; 
    #endif
    
    // configure LLCC68 BUSY pin
    // P1.07 assigned to BUSY pin
    llcc68_init();

    // init loRaWAN channel mapping
    lorawan_channel_mapping();

    // reset llcc68
    // nrf pin configure
    nrf_gpio_cfg_output(LLCC68_RESET_PIN);
    radio_llcc68_reset();

    // tx clamp config
    // data sheet section 15.2.2 workaround
    // bits 4-1 must be set to “1111” (0x1E)
    value = llcc68_spiReadReg(TXCLAMPCONFIG);
    value = value | TX_CLAMP_WORKAROUND;
    llcc68_spiWriteReg(TXCLAMPCONFIG, TX_CLAMP_WORKAROUND);
    
    // clock calibration 
    value = CALIBRATE_ALL;
    llcc68_noAddress_opcode(CALIBRATE, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));
    
    #if defined(DEBUG)
      radio_vars.state = LLCC68STATE_ENABLE_CALIBRATING;
    #endif

    // wait for calibration to finish (typically 3.5 ms)
    do {
        radio_vars.status = radio_llcc68_getStatus();
    } while(radio_vars.status.chipMode != STBY_RC);
    
    #if defined(DEBUG)
      radio_vars.state = LLCC68STATE_CALIBRATION_DONE;
    #endif

     // image calibration for ISM band
     memcpy(mulitParam, REGION_ISM, sizeof(REGION_ISM));
     llcc68_noAddress_opcode(CALIBRATEIMAGE, 
        TYPE_WRITE, (uint8_t*)&mulitParam, sizeof(REGION_ISM));
    
    #if defined(DEBUG)
      radio_vars.state = LLCC68STATE_ENABLE_IMAGE_CAL;
    #endif

    // set to standby mode and use RC_13 MHz clock reference 
    value = RC_13MHz;
    llcc68_noAddress_opcode(SETSTANDBY, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));
    
    #if defined(DEBUG)
      radio_vars.state = LLCC68STATE_STANDBY_RC;
    #endif

    // clear IRQ status
    memset(&irqStatus, IRQMASK, sizeof(irqStatus));
    llcc68_noAddress_opcode(CLEARIRQSTATUS, 
        TYPE_WRITE, (uint8_t*)&irqStatus, sizeof(irqStatus));

    // enable DIO2 as RF switch ctrl
    value = 0x01;
    llcc68_noAddress_opcode(SETDIO2ASRFSWITCHCTRL, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));
    
    #if defined(DEBUG)
      // check for device errors 
      radio_vars.status = radio_llcc68_getStatus();
      radio_vars.opError = radio_llcc68_getOpError();
    #endif
}

// radio reset
void radio_llcc68_reset(void) {
    
    uint16_t i;
    
    // reset pin low
    NRF_P1->OUTCLR = (1UL << (LLCC68_RESET_PIN & 0x1F));

    #if defined(DEBUG)
      radio_vars.state = LLCC68STATE_RESET;
    #endif

    // wait > 100us
    for(i = 0; i < 0xffff; i++);

    // reset pin high
    NRF_P1->OUTSET = (1UL << (LLCC68_RESET_PIN & 0x1F));

    #if defined(DEBUG)
      radio_vars.state = LLCC68STATE_STARTUP;
    #endif

    // wait for chip to go to standby mode
    do {
      radio_vars.status = radio_llcc68_getStatus();
    } while (radio_vars.status.chipMode != STBY_RC);

    #if defined(DEBUG)
      // check for device errors 
      radio_vars.opError = radio_llcc68_getOpError();

      radio_vars.state = LLCC68STATE_STANDBY_RC;
    #endif
}

// radio control
void radio_llcc68_loadPacket(uint8_t offset, 
                              uint8_t* buffer, 
                              uint8_t len) {

    llcc68_txBufferWrite(offset, buffer, len);

    #if defined(DEBUG)
      // check for device errors 
      radio_vars.status = radio_llcc68_getStatus();
      radio_vars.opError = radio_llcc68_getOpError();

      radio_vars.state = LLCC68STATE_PACKET_LOADED;
    #endif
}

void radio_llcc68_lora_config(radio_llcc68_config_t radio){

    uint8_t value;
    uint8_t mulitParam[4];
    uint32_t freqReg;
    bufferBaseAddress_t bufferBaseAddress;
    radio_llcc68_irqStatus_t irqStatus;
    irqParams_t irqParams;

    int i;
    
    memset(&bufferBaseAddress, 0, sizeof(bufferBaseAddress));
    memset(&irqStatus, 0, sizeof(irqStatus));
    memset(&irqParams, 0, sizeof(irqParams));

    // set standby mode
    value = RC_13MHz;
    llcc68_noAddress_opcode(SETSTANDBY, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));
     
    #if defined(DEBUG)
      radio_vars.state = LLCC68STATE_STANDBY_RC;
    #endif

    // set packet type
    value = PACKET_TYPE_LORA;
    llcc68_noAddress_opcode(SETPACKETTYPE, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));

    // check if valid channel number
    // need to add channel mask
    #if defined(REGION_EUROPE)
      if (radio.channel >= REGION_UPLINK_CH_MAX) {
        // invalid channel number
        // default to channel 0
        freqReg = (uint32_t)(((uint64_t)lorawan_ch_map[0] * 
                                        (1ULL << 25)) / 32000000ULL);
      }
      else {
        // freqReg = frequency(Hz) * 32 MHz(F_XTAL) / 2^25
        freqReg = (uint32_t)(((uint64_t)lorawan_ch_map[radio.channel] *
                                         (1ULL << 25)) / 32000000ULL);
      }
    #else
      if (radio.channel >= REGION_UPLINK_CH_MAX + REGION_DOWNLINK_CH_MAX) {
        // invalid channel number
        // default to channel 0
        freqReg = (uint32_t)(((uint64_t)lorawan_ch_map[0].frequency * 
                                        (1ULL << 25)) / 32000000ULL);
      }
      else {
        // freqReg = frequency(Hz) * 32 MHz(F_XTAL) / 2^25
        freqReg = (uint32_t)(((uint64_t)lorawan_ch_map[radio.channel].frequency * 
                                        (1ULL << 25)) / 32000000ULL);
      }
    #endif

    // set RF frequency
    //memcpy(mulitParam, &freqReg, sizeof(freqReg));
    mulitParam[0] = 0x89;
    mulitParam[1] = 0xFD;
    mulitParam[3] = 0xE8;
    mulitParam[4] = 0x1B;
    llcc68_noAddress_opcode(SETRFFREQUENCY, 
                            TYPE_WRITE,
                            (uint8_t*)&mulitParam, 
                            sizeof(mulitParam));
    #if defined(DEBUG)
      radio_vars.state = LLCC68STATE_FREQUENCY_SET;
    #endif

    // set power amplifier configuration
    memcpy(mulitParam, REGION_MAX_DBM, sizeof(REGION_MAX_DBM));
    llcc68_noAddress_opcode(SETPACONFIG, 
                            TYPE_WRITE, 
                            (uint8_t*)&mulitParam, 
                            sizeof(REGION_MAX_DBM));

    llcc68_noAddress_opcode(SETTXPARAMS, 
                            TYPE_WRITE, 
                            (uint8_t*)&radio.radioTxParams, 
                            sizeof(radio.radioTxParams));

    bufferBaseAddress = (bufferBaseAddress_t){
        .txBaseAddress    = LORA_TX_BASE_ADDR,
        .rxBaseAddress    = LORA_RX_BASE_ADDR,
    };
    llcc68_noAddress_opcode(SETBUFFERBASEADDRESS, 
                            TYPE_WRITE, 
                            (uint8_t*)&bufferBaseAddress, 
                            sizeof(bufferBaseAddress));

    llcc68_noAddress_opcode(SETMODULATIONPARAMS, 
                            TYPE_WRITE, 
                            (uint8_t*)&radio.loraModParams, 
                            sizeof(radio.loraModParams));

    // data sheet section 15.1.2 Workaround
    value = llcc68_spiReadReg(TXMODULATION);
    
    if (radio.loraModParams.bandwidth == LORA_BW_500){
        // bit #2 set low
        value = value & 0xFB;
    }
    else{
        // bit #2 set high
        value = value | 0x04; 
    }
    llcc68_spiWriteReg(TXMODULATION, value);

    radio.packetParams.preambleLength = LSB_FIRST_16(
                             radio.packetParams.preambleLength);
    llcc68_noAddress_opcode(SETPACKETPARAMS, 
                            TYPE_WRITE, 
                            (uint8_t*)&radio.packetParams, 
                            sizeof(radio.packetParams));
    
    // clear IRQ status
    memset(&irqStatus, IRQMASK, sizeof(irqStatus));
    llcc68_noAddress_opcode(CLEARIRQSTATUS, 
                            TYPE_WRITE,
                            (uint8_t*)&irqStatus, 
                            sizeof(irqStatus));
    
    // set IRQ/DIO params
    // IrqMask: irqStatus flag info enabled
    // DioXMask: trigger interrupt when flag triggered
    irqParams.irqMask         = IRQTXDONE | IRQRXDONE | IRQPREAMBLEDETECTED |
                                IRQHEADERVALID | IRQHEADERERROR |IRQCRCERROR |
                                IRQCADDONE |IRQCADDETECTED | IRQTIMEOUT;
    irqParams.dio1Mask        = IRQTXDONE | IRQRXDONE | IRQTIMEOUT;
    irqParams.dio2Mask        = DIO2MASK;
    irqParams.dio3Mask        = DIO3MASK;
    llcc68_noAddress_opcode(SETDIOIRQPARAMS, 
                            TYPE_WRITE, 
                            (uint8_t*)&irqParams, 
                            sizeof(irqParams));

    // set sync word (private/public) 
    // set MSB
    value = (radio.syncword >> 8) & 0xFF;
    llcc68_spiWriteReg(LORASYNCWORDMSB,value);
    // set LSB
    value = radio.syncword & 0xFF;
    llcc68_spiWriteReg(LORASYNCWORDLSB,value);

    #if defined(DEBUG)
      // check for device errors 
      radio_vars.status = radio_llcc68_getStatus();
      radio_vars.opError = radio_llcc68_getOpError();
      
      radio_vars.state = LLCC68STATE_CONFIG_SET;
    #endif
}

// Timeout Duration = timeout[] * 15.625 us
void radio_llcc68_txNow(radioTimeout_t txMax){
    // set Tx mode
    llcc68_noAddress_opcode(SETTX, 
                            TYPE_WRITE, 
                            (uint8_t*)&txMax.timeout, 
                            sizeof(txMax.timeout));
    #if defined(DEBUG)
      // check for device errors 
      radio_vars.status = radio_llcc68_getStatus();
      radio_vars.opError = radio_llcc68_getOpError();

      radio_vars.state = LLCC68STATE_TX;
    #endif
} 

// Timeout Duration = timeout[] * 15.625 us
void radio_llcc68_rxNow(radioTimeout_t rxMax){
    // set Rx mode
    llcc68_noAddress_opcode(SETRX, 
                            TYPE_WRITE, 
                            (uint8_t*)&rxMax.timeout, 
                            sizeof(rxMax.timeout));

    #if defined(DEBUG)
      // check for device errors 
      radio_vars.status = radio_llcc68_getStatus();
      radio_vars.opError = radio_llcc68_getOpError();

      radio_vars.state = LLCC68STATE_RX;
    #endif
}

void radio_llcc68_getReceivedFrame(uint8_t* pBufRead,
                                   uint8_t* pLenRead,
                radio_llcc68_packetStats_t* packetStats)
{

    uint8_t rx_buf[3];
    uint8_t len;
    uint8_t pntBuf;

    // get packet info
    llcc68_noAddress_opcode(GETRXBUFFERSTATUS, TYPE_READ, rx_buf, sizeof(uint8_t) * 2);
    // [0] = payloadLengthRx 
    // [1] = rxStartBufferPointer
    len = rx_buf[0];
    pntBuf = rx_buf[1];

    if (len == 0) {
        return; 
    }
    // rxBuffer has looped
    if (len > MAX_PACKET_SIZE) { 
        len = MAX_PACKET_SIZE; 
    }

    // save packet
    llcc68_rxBufferRead(rx_buf[1], pBufRead, rx_buf[0]);
    
    // save packet length
    *pLenRead = rx_buf[0];

    // get packet stats
    llcc68_noAddress_opcode(GETPACKETSTATUS, 
        TYPE_READ, rx_buf, sizeof(rx_buf));
    
    rx_buf[0] = -1 * rx_buf[0] >> 1;   // rssiPkt (dBm)
    rx_buf[1] =      rx_buf[1] >> 2;   // snrPkt (dB)
    rx_buf[2] = -1 * rx_buf[2] >> 1;   // signalRssiPk (dB)
    
    memcpy(packetStats, rx_buf, sizeof(radio_llcc68_packetStats_t));

    #if defined(DEBUG)
      // check for device errors 
      radio_vars.status = radio_llcc68_getStatus();
      radio_vars.opError = radio_llcc68_getOpError();

      radio_vars.state = LLCC68STATE_PACKET_READ;
    #endif
}

void radio_llcc68_irq_clear(void) {
    
    radio_llcc68_irqStatus_t irqStatus;

    // clear IRQ status
    memset(&irqStatus, IRQMASK, sizeof(irqStatus));
    llcc68_noAddress_opcode(CLEARIRQSTATUS, 
        TYPE_WRITE, (uint8_t*)&irqStatus, sizeof(irqStatus));
}

// radio info
radio_llcc68_status_t radio_llcc68_getStatus(void) {
    
    uint8_t rx_buf;
    radio_llcc68_status_t status;

    llcc68_noAddress_opcode(GETSTATUS, 
        TYPE_READ, &rx_buf, sizeof(rx_buf));

    memcpy(&status, &rx_buf, sizeof(status));
    return status;
}

radio_llcc68_opError_t radio_llcc68_getOpError(void) {
    
    uint8_t rx_buf[2];

    llcc68_noAddress_opcode(GETDEVICEERRORS, 
        TYPE_READ, rx_buf, sizeof(rx_buf));
    
    return *(radio_llcc68_opError_t *)rx_buf;
}

radio_llcc68_irqStatus_t radio_llcc68_getIrqstatus(void) {
    
    uint8_t rx_buf[2];

    llcc68_noAddress_opcode(GETIRQSTATUS, 
        TYPE_READ, rx_buf, sizeof(rx_buf));

    return *(radio_llcc68_irqStatus_t *)rx_buf;
}

//=========================== private =========================================
// channel mapping
void lorawan_channel_mapping(void){
    uint8_t channel;

    // uplink: spacing every 200 kHz for most regions
    for (channel = 0; channel < REGION_UPLINK_CH_MAX; channel++) {
        lorawan_ch_map[channel].frequency =
                REGION_UPLINK_START + channel * REGION_CH_SPACING_UP;
    }

    #if defined(REGION_EUROPE)
        // uplink/downlink are on the same frequency
    #else
        // downlink: spacing every 200 kHz for most regions (except USA)
        for (channel = REGION_UPLINK_CH_MAX;
             channel < (REGION_UPLINK_CH_MAX + REGION_DOWNLINK_CH_MAX); 
             channel++) {
            lorawan_ch_map[channel].frequency =
                REGION_DOWNLINK_START + (channel - REGION_UPLINK_CH_MAX) * REGION_CH_SPACING_DOWN;
        }
    #endif
}

//=========================== callbacks =======================================

