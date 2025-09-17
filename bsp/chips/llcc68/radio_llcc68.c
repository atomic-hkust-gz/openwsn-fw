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

#define IRQMASK                     0x03FF//0x03F7//0x03FF
#define IRQTXDONE                   0x0001    // tx done 
#define IRQRXDONE                   0x0002    // rx done
#define IRQTIMEOUT                  0x0200    // rx/tx timeout
#define DIO2MASK                    0x0000
#define DIO3MASK                    0x0000
#define LORA_TX_BASE_ADDR             0x00
#define LORA_RX_BASE_ADDR             0x80    // max packet size = 128
//=========================== variables =======================================

typedef struct {
    //radio_capture_cbt     startFrame_cb;
    //radio_capture_cbt     endFrame_cb;
    radio_llcc68_state_t    state; 
    radio_llcc68_status_t   status;
    radio_llcc68_opError_t  opError;   
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
    irqStatus_t irqStatus;
    uint8_t value;
    uint8_t mulitParam[4];

    // clear internal variables
    memset(&radio_vars, 0, sizeof(radio_vars));
    memset(&lorawan_ch_map, 0, sizeof(lorawan_ch_map));
    memset(&irqStatus, 0, sizeof(irqStatus));
    memset(&mulitParam, 0, sizeof(mulitParam));

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

    // wait for calibration to finish (typically 3.5 ms)
    do {
        radio_llcc68_get_status();
    } while(radio_vars.status.chipMode != STBY_RC);

     // image calibration for ISM band
     memcpy(mulitParam, REGION_ISM, sizeof(REGION_ISM));
     llcc68_noAddress_opcode(CALIBRATEIMAGE, 
        TYPE_WRITE, (uint8_t*)&mulitParam, sizeof(REGION_ISM));

    // set to standby mode and use RC_13 MHz clock reference 
    value = RC_13MHz;
    //value = XTAL_32MHz;
    llcc68_noAddress_opcode(SETSTANDBY, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));
    radio_vars.state = LLCC68STATE_STANDBY_RC;

    // clear IRQ status
    memset(&irqStatus, IRQMASK, sizeof(irqStatus));
    llcc68_noAddress_opcode(CLEARIRQSTATUS, 
        TYPE_WRITE, (uint8_t*)&irqStatus, sizeof(irqStatus));

    // enable DIO2 as RF switch ctrl
    value = llcc68_spiReadReg(SETDIO2ASRFSWITCHCTRL);
    value = value | 0x01;
    llcc68_spiWriteReg(SETDIO2ASRFSWITCHCTRL,value);
    
    #if defined(DEBUG)
      // check for device errors 
      radio_llcc68_get_status();
      radio_llcc68_get_opError();
    #endif
}

// radio reset
void radio_llcc68_reset(void) {
    
    uint16_t i;
    
    // reset pin low
    NRF_P1->OUTCLR = (1UL << (LLCC68_RESET_PIN & 0x1F));
    // wait > 100us
    for(i = 0; i < 0xffff; i++);
    // reset pin high
    NRF_P1->OUTSET = (1UL << (LLCC68_RESET_PIN & 0x1F));

    // wait for chip to go to standby mode
    do {
      radio_llcc68_get_status();
    } while (radio_vars.status.chipMode != STBY_RC);

    radio_llcc68_get_opError();
    radio_vars.state = LLCC68STATE_STANDBY_RC;
}

// radio control
void radio_llcc68_loadPacket(uint8_t offset, 
                              uint8_t* buffer, 
                              uint8_t len) {

    llcc68_txBufferWrite(offset, buffer, len);

    #if defined(DEBUG)
      // check for device errors 
      radio_llcc68_get_status();
      radio_llcc68_get_opError();
    #endif

    radio_vars.state = LLCC68STATE_PACKET_LOADED;
}

void radio_llcc68_lora_config(radio_llcc68_config_t radio){

    uint8_t value;
    uint8_t mulitParam[4];
    uint32_t freqReg;
    bufferBaseAddress_t bufferBaseAddress;
    irqStatus_t irqStatus;
    irqParams_t irqParams;

    int i;
    
    memset(&bufferBaseAddress, 0, sizeof(bufferBaseAddress));
    memset(&irqStatus, 0, sizeof(irqStatus));
    memset(&irqParams, 0, sizeof(irqParams));

    // set standby mode
    value = RC_13MHz;
    llcc68_noAddress_opcode(SETSTANDBY, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));
    for(i = 0; i < 0xfff; i++){};
    // set packet type
    value = PACKET_TYPE_LORA;
    llcc68_noAddress_opcode(SETPACKETTYPE, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));
    for(i = 0; i < 0xfff; i++){};
    // check if valid channel number
    // need to add channel mask
    #if defined(REGION_EUROPE)
      if (radio.channel >= REGION_UPLINK_CH_MAX) {
        // invalid channel number
        // default to channel 0
        freqReg = (uint32_t)(((uint64_t)lorawan_ch_map[0] * 
                                        32000000) / (1ULL << 25));
      }
      else {
        // freqReg = frequency(Hz) * 32 MHz(F_XTAL) / 2^25
        freqReg = (uint32_t)(((uint64_t)lorawan_ch_map[radio.channel] *
                                         32000000) / (1ULL << 25));
      }
    #else
      if (radio.channel >= REGION_UPLINK_CH_MAX + REGION_DOWNLINK_CH_MAX) {
        // invalid channel number
        // default to channel 0
        freqReg = (uint32_t)(((uint64_t)lorawan_ch_map[0].frequency * 
                                        32000000) / (1ULL << 25));
      }
      else {
        // freqReg = frequency(Hz) * 32 MHz(F_XTAL) / 2^25
        freqReg = (uint32_t)(((uint64_t)lorawan_ch_map[radio.channel].frequency * 
                                        32000000) / (1ULL << 25));
      }
    #endif

    // set RF frequency
    memcpy(mulitParam, &freqReg, sizeof(freqReg));
    llcc68_noAddress_opcode(SETRFFREQUENCY, 
                            TYPE_WRITE,
                            (uint8_t*)&mulitParam, 
                            sizeof(mulitParam));
    for(i = 0; i < 0xfff; i++){};
    // set power amplifier configuration
    memcpy(mulitParam, REGION_MAX_DBM, sizeof(REGION_MAX_DBM));
    llcc68_noAddress_opcode(SETPACONFIG, 
                            TYPE_WRITE, 
                            (uint8_t*)&mulitParam, 
                            sizeof(REGION_MAX_DBM));
    for(i = 0; i < 0xfff; i++){};
    llcc68_noAddress_opcode(SETTXPARAMS, 
                            TYPE_WRITE, 
                            (uint8_t*)&radio.radioTxParams, 
                            sizeof(radio.radioTxParams));
    for(i = 0; i < 0xfff; i++){};
    bufferBaseAddress = (bufferBaseAddress_t){
        .txBaseAddress    = LORA_TX_BASE_ADDR,
        .rxBaseAddress    = LORA_RX_BASE_ADDR,
    };
    llcc68_noAddress_opcode(SETBUFFERBASEADDRESS, 
                            TYPE_WRITE, 
                            (uint8_t*)&bufferBaseAddress, 
                            sizeof(bufferBaseAddress));
    //debug temp
    uint8_t temp[8] = {0,1,2,3,4,5,6,7};
    //radio_llcc68_loadPacket(0x00,(uint8_t*)&temp, sizeof(temp));
    llcc68_txBufferWrite(0x00,(uint8_t*)&temp, sizeof(temp));
    //radio_llcc68_get_status();
    //radio_llcc68_get_opError();
    for(i = 0; i < 0xffff; i++){__NOP();};
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
    for(i = 0; i < 0xfff; i++){};
    llcc68_noAddress_opcode(SETPACKETPARAMS, 
                            TYPE_WRITE, 
                            (uint8_t*)&radio.packetParams, 
                            sizeof(radio.packetParams));
    for(i = 0; i < 0xfff; i++){};
    
    // clear IRQ status
    memset(&irqStatus, IRQMASK, sizeof(irqStatus));
    llcc68_noAddress_opcode(CLEARIRQSTATUS, 
                            TYPE_WRITE,
                            (uint8_t*)&irqStatus, 
                            sizeof(irqStatus));
    for(i = 0; i < 0xfff; i++){};
    // set IRQ/DIO params
    // IrqMask needs to match DioXMask to be valid
    irqParams.irqMask         = IRQTXDONE | IRQTIMEOUT; 
    irqParams.dio1Mask        = IRQTXDONE | IRQTIMEOUT;
    irqParams.dio2Mask        = DIO2MASK;
    irqParams.dio3Mask        = DIO3MASK;
    llcc68_noAddress_opcode(SETDIOIRQPARAMS, 
                            TYPE_WRITE, 
                            (uint8_t*)&irqParams, 
                            sizeof(irqParams));

    // set sync word (private/public) 
    // set MSB
    value = (radio.syncword >> 8) & 0xFF;
    for(i = 0; i < 0xfff; i++){};
    llcc68_spiWriteReg(LORASYNCWORDMSB,value);
    // set LSB
    value = radio.syncword & 0xFF;
    for(i = 0; i < 0xfff; i++){};
    llcc68_spiWriteReg(LORASYNCWORDLSB,value);

    #if defined(DEBUG)
      // check for device errors 
      radio_llcc68_get_status();
      radio_llcc68_get_opError();
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
      radio_llcc68_get_status();
      radio_llcc68_get_opError();
    #endif
} 

// Timeout Duration = timeout[] * 15.625 us
void radio_llcc68_rxNow(radioTimeout_t rxMax){

    irqStatus_t irqStatus;
    irqParams_t irqParams;

    memset(&irqStatus, 0, sizeof(irqStatus));
    memset(&irqParams, 0, sizeof(irqParams));
    
    // clear IRQ status
    memset(&irqStatus, IRQMASK, sizeof(irqStatus));
    llcc68_noAddress_opcode(CLEARIRQSTATUS, 
                            TYPE_WRITE, 
                            (uint8_t*)&irqStatus,
                            sizeof(irqStatus));

    // set IRQ/DIO params
    // IrqMask needs to match DioXMask to be valid
    irqParams.irqMask         = IRQRXDONE | IRQTIMEOUT; 
    irqParams.dio1Mask        = IRQRXDONE | IRQTIMEOUT;
    irqParams.dio2Mask        = DIO2MASK;
    irqParams.dio3Mask        = DIO3MASK;
    llcc68_noAddress_opcode(SETDIOIRQPARAMS, 
                            TYPE_WRITE, 
                            (uint8_t*)&irqParams, 
                            sizeof(irqParams));
    
    // set Rx mode
    llcc68_noAddress_opcode(SETRX, 
                            TYPE_WRITE, 
                            (uint8_t*)&rxMax.timeout, 
                            sizeof(rxMax.timeout));
}

// radio info
void radio_llcc68_get_status(void) {
    uint8_t rx_buf;
    llcc68_noAddress_opcode(GETSTATUS, TYPE_READ, &rx_buf, sizeof(rx_buf));

    memcpy(&radio_vars.status, &rx_buf, sizeof(radio_vars.status));
}

void radio_llcc68_get_opError(void) {
    uint8_t rx_buf[2];
    llcc68_noAddress_opcode(GETDEVICEERRORS, TYPE_READ, rx_buf, sizeof(rx_buf));
    
    memcpy(&radio_vars.opError, &rx_buf, sizeof(radio_vars.opError));
}

irqStatus_t radio_llcc68_irq_status(void) {
    uint8_t rx_buf[2];
    llcc68_noAddress_opcode(GETIRQSTATUS, TYPE_READ, rx_buf, sizeof(rx_buf));

    return *(irqStatus_t *)rx_buf;
}

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


/*
void radio_llcc68_getReceivedFrame(uint8_t* pBufRead,
                            uint8_t* pLenRead,
                            uint8_t  maxBufLen,
                             int8_t* pRssi,
                            uint8_t* pLqi,
                               bool* pCrc)
{
    // check for length parameter; if too long, payload won't fit into memory
    uint8_t len;

    len = radio_vars.payload[0];

    if (len == 0) {
        return; 
    }

    if (len > MAX_PACKET_SIZE) { 
        len = MAX_PACKET_SIZE; 
    }

    if (len > maxBufLen) { 
        len = maxBufLen; 
    }

    // copy payload
    memcpy(pBufRead, &radio_vars.payload[1], len);

    // store other parameters
    *pLenRead = len;
    *pLqi = radio_vars.payload[radio_vars.payload[0]-1];

    // For the RSSI calculation, see 
    //
    // - http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.nrf52840.ps/radio.html?cp=2_0_0_5_19_11_6#ieee802154_rx and
    // - https://www.metageek.com/training/resources/understanding-rssi.html
    //
    // Our RSSI will be in the range -91 dB (worst) to 0 dB (best)
    *pRssi = (*pLqi > 91)?(0):(((int8_t) *pLqi) - 91);

    *pCrc = (NRF_RADIO->CRCSTATUS == 1U);
}
*/
//=========================== private =========================================



//=========================== callbacks =======================================

