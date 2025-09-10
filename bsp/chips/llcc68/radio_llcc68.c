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


//=========================== defines =========================================
#define DUBUG         1

#define NRF_GPIO_PIN_MAP(port, pin) (((port) << 5) | ((pin) & 0x1F))
// Pin assignments
#define LLCC68_RESET_PIN NRF_GPIO_PIN_MAP(1,8)    // P1.08
#define LLCC68_BUSY_PORT  PORT1                   // P1.07
#define LLCC68_BUSY_PIN   7  
#define BUSY_IRQ_CHANNEL  7


#define TIMER_PERIOD                0x2000 // @32kHz = 0.25 s

#define MAX_PACKET_SIZE             128 // 256 actual max
#define IRQMASK                     0x03FF
#define IRQTXDONE                   0x0001    // TxDone 
#define IRQRXDONE                   0x0002    // RxDone
#define IRQTIMEOUT                  0x0200    // Rx/Tx timeout
#define DIO2MASK                    0x0000
#define DIO3MASK                    0x0000


// lora configuration
#define RF_FREQUENCY                490000000 // 490 MHz
#define LORA_BANDWIDTH              0x04      // 0x04 = 125 kHz
#define LORA_SPREADING_FACTOR       0x07      // SF7
#define LORA_CODINGRATE             0x01      // CR 4/5
#define LORA_PREAMBLE_LENGTH        0x08
#define LORA_PAYLOAD_LENGTH         0x80      // 128
#define LORA_TX_BASE_ADDR           0x00
#define LORA_RX_BASE_ADDR           0x80      // Max packet size = 128
//#define LORA_TX_POWER_DBM           14

//=========================== variables =======================================

typedef struct {
    //radio_capture_cbt     startFrame_cb;
    //radio_capture_cbt     endFrame_cb;
    radio_llcc68_state_t    state; 
    radio_llcc68_status_t   status;
    radio_llcc68_opError_t  opError;
    radio_llcc68_busy_t     busy;
    
    //uint8_t               payload[1+MAX_PACKET_SIZE] __attribute__ ((aligned));
    //bool                  hfc_started;
} radio_llcc68_vars_t;

static radio_llcc68_vars_t radio_vars;

//=========================== prototypes ======================================

void radio_llcc68_busy_cb(void);

//=========================== public ==========================================

void radio_llcc68_init(void) { 
    irqStatus_t irqStatus;
    uint8_t value;
    uint8_t mulitParam[4];

    // clear internal variables
    memset(&radio_vars, 0, sizeof(radio_vars));
    memset(&irqStatus, 0, sizeof(irqStatus));
    memset(&mulitParam, 0, sizeof(mulitParam));

    // configure LLCC68 BUSY pin
    // P1.07 assigned to BUSY pin (falling edge detect)
    gpio_irq_config(BUSY_IRQ_CHANNEL, 
                    LLCC68_BUSY_PORT, 
                    LLCC68_BUSY_PIN, 
                    GPIOTE_HITOLO, 
                    radio_llcc68_busy_cb);
    gpio_irq_enable(IRQ_CHANNEL);

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
    radio_llcc68_busy_wait();
    
    // clock calibration 
    value = CALIBRATE_ALL;
    llcc68_noAddress_opcode(CALIBRATE, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));
    radio_llcc68_busy_wait();

    // wait for calibration to finish (typically 3.5 ms)
    do {
        radio_llcc68_get_status();
    } while(radio_vars.status.ChipMode != STBY_RC);

     // image calibration for ISM band
     memcpy(mulitParam, REGION_ISM, sizeof(REGION_ISM));
     llcc68_noAddress_opcode(CALIBRATEIMAGE, 
        TYPE_WRITE, (uint8_t*)&mulitParam, sizeof(REGION_ISM));
     radio_llcc68_busy_wait();

    // set to standby mode and use RC_13 MHz clock reference 
    value = RC_13MHz;
    //value = XTAL_32MHz;
    llcc68_noAddress_opcode(SETSTANDBY, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));
    radio_llcc68_busy_wait();
    radio_vars.state = LLCC68STATE_STANDBY_RC;

    // clear IRQ status
    memset(&irqStatus, IRQMASK, sizeof(irqStatus));
    llcc68_noAddress_opcode(CLEARIRQSTATUS, 
        TYPE_WRITE, (uint8_t*)&irqStatus, sizeof(irqStatus));
    radio_llcc68_busy_wait();
    
    if (DUBUG) {
    // check for device errors 
    radio_llcc68_get_status();
    radio_llcc68_get_opError();
    }
}

void radio_llcc68_setFrequency(uint32_t frequencyHz) {
    uint32_t freqReg;
    uint8_t RF_frequency[4];

    // freqReg = frequency(Hz) * 2^20 / 1,000,000
    // equivalent to 
    // freqReg = freqeuncy(Hz) * 32 MHz(F_XTAL) / 2^25
    freqReg = (uint32_t)(((uint64_t)frequencyHz * (1UL << 20)) / 1000000UL);

    memcpy(RF_frequency, &freqReg, sizeof(freqReg));
    llcc68_noAddress_opcode(SETRFFREQUENCY, TYPE_WRITE, RF_frequency, sizeof(RF_frequency));
    radio_llcc68_busy_wait();

    if (DUBUG) {
    // check for device errors 
    radio_llcc68_get_status();
    radio_llcc68_get_opError();
    }
}

void radio_llcc68_loadPacket(uint8_t offset, uint8_t* buffer, uint8_t len){
    llcc68_txBufferWrite(offset, buffer, len);
    radio_llcc68_busy_wait();

    if(DUBUG){
    // check for device errors 
    radio_llcc68_get_status();
    radio_llcc68_get_opError();
    }

    radio_vars.state = LLCC68STATE_PACKET_LOADED;
}

void radio_llcc68_setModulation(radioModulationParams_t modParams){
    llcc68_noAddress_opcode(SETMODULATIONPARAMS, 
        TYPE_WRITE, (uint8_t*)&modParams, sizeof(modParams));

    if (DUBUG) {
    // check for device errors 
    radio_llcc68_get_status();
    radio_llcc68_get_opError();
    }
}

void radio_llcc68_setPacketParams(packetParams_t packetParams){
    llcc68_noAddress_opcode(SETPACKETPARAMS, 
        TYPE_WRITE, (uint8_t*)&packetParams, sizeof(packetParams));

    if (DUBUG) {
    // check for device errors 
    radio_llcc68_get_status();
    radio_llcc68_get_opError();
    }
}
void radio_llcc68_lora_config(radio_llcc68_config_t radio){

    uint8_t value;
    uint8_t mulitParam[4];

    // set standby mode
    value = RC_13MHz;
    llcc68_noAddress_opcode(SETSTANDBY, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));
    radio_llcc68_busy_wait();
    
    // set packet type
    value = PACKET_TYPE_LORA;
    llcc68_noAddress_opcode(SETPACKETTYPE, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));
    radio_llcc68_busy_wait();

    /*
    // set RF frequency
    memcpy(mulitParam, REGION_ISM, sizeof(REGION_ISM));
    llcc68_noAddress_opcode(SETRFFREQUENCY, 
        TYPE_WRITE, (uint8_t*)&mulitParam, sizeof(REGION_ISM));
    radio_llcc68_busy_wait();
    */

    // set power amplifier configuration
    memcpy(mulitParam, PA_CONFIG_17_DBM, sizeof(PA_CONFIG_17_DBM));
    llcc68_noAddress_opcode(SETPACONFIG, 
        TYPE_WRITE, (uint8_t*)&mulitParam, sizeof(PA_CONFIG_17_DBM));
    radio_llcc68_busy_wait();

}
void radio_llcc68_txEnable(void) {



    for(i = 0; i < 0xffff; i++);

    // set Tx parameters
    radioTxParams = (radioTxParams_t){
        .TxPowerDbm           = TX_P22_DBM,
        .TxRampTime           = RAMP_200U
    };
    llcc68_noAddress_opcode(SETTXPARAMS, 
        TYPE_WRITE, (uint8_t*)&radioTxParams, sizeof(radioTxParams));

    for(i = 0; i < 0xffff; i++);

    // set buffer base addresses
    bufferBaseAddress = (bufferBaseAddress_t){
        .TxBaseAddress        = LORA_TX_BASE_ADDR,
        .RxBaseAddress        = LORA_RX_BASE_ADDR
    };
    llcc68_noAddress_opcode(SETBUFFERBASEADDRESS, 
        TYPE_WRITE, (uint8_t*)&bufferBaseAddress, sizeof(bufferBaseAddress));

    for(i = 0; i < 0xffff; i++);
    
    // enable DIO2 as RF switch ctrl
    value = llcc68_spiReadReg(SETDIO2ASRFSWITCHCTRL);
    value = value | 0xFF;
    llcc68_spiWriteReg(SETDIO2ASRFSWITCHCTRL,value);

    for(i = 0; i < 0xffff; i++);

    if (DUBUG) {
    // check for device errors 
    radio_llcc68_get_status();
    radio_llcc68_get_opError();
    }
}

// Timeout Duration = timeout[] * 15.625 us
void radio_llcc68_txNow(radioTimeout_t txMax){
    radioModulationParams_t loraModParams;
    packetParams_t packetParams;
    irqStatus_t irqStatus;
    irqParams_t irqParams;
    uint8_t value;
    uint16_t i;
    
    memset(&loraModParams, 0, sizeof(loraModParams));
    memset(&packetParams, 0, sizeof(packetParams));
    memset(&irqStatus, 0, sizeof(irqStatus));
    memset(&irqParams, 0, sizeof(irqParams));

    radio_llcc68_get_status();

    
    // set modulation parameters
    loraModParams = (radioModulationParams_t){
        .SpreadingFactor      = LORA_SF7,
        .Bandwidth            = LORA_BW_125,
        .CodingRate           = LORA_CR_4_5,
        .LowDataRateOptimize  = LDRO_OFF
    };
    llcc68_noAddress_opcode(SETMODULATIONPARAMS, 
        TYPE_WRITE, (uint8_t*)&loraModParams, sizeof(loraModParams));
    
    // data sheet section 15.1.2 Workaround
    value = llcc68_spiReadReg(TXMODULATION);

    radio_llcc68_get_status();
    for(i = 0; i < 0xffff; i++);
    
    if (loraModParams.Bandwidth == LORA_BW_500){
        // bit #2 set low
        value = value & 0xFB;
    }
    else{
        // bit #2 set high
        value = value | 0x04; 
    }
    llcc68_spiWriteReg(TXMODULATION,value);

    radio_llcc68_get_status();
    for(i = 0; i < 0xffff; i++);

    // set packet parameters
    packetParams = (packetParams_t){
        .PreambleLength       = LORA_PREAMBLE_LENGTH,
        .HeaderType           = FIXED_LENGTH_PACKET,
        .PayloadLength        = LORA_PAYLOAD_LENGTH,
        .CrcType              = CRC_ON,
        .InvertIq             = STD_IQ
    };
    llcc68_noAddress_opcode(SETPACKETPARAMS, 
        TYPE_WRITE, (uint8_t*)&packetParams, sizeof(packetParams));

    radio_llcc68_get_status();
    for(i = 0; i < 0xffff; i++);
    
    // clear IRQ status
    memset(&irqStatus, IRQMASK, sizeof(irqStatus));
    llcc68_noAddress_opcode(CLEARIRQSTATUS, 
        TYPE_WRITE, (uint8_t*)&irqStatus, sizeof(irqStatus));
    
    for(i = 0; i < 0xffff; i++);

    // set IRQ/DIO params
    // IrqMask needs to match DioXMask to be valid
    irqParams.IrqMask         = IRQTXDONE | 0x100; //IRQTXDONE | IRQTIMEOUT; 
    irqParams.Dio1Mask        = IRQTXDONE | 0x100;//IRQTXDONE | IRQTIMEOUT;
    irqParams.Dio2Mask        = DIO2MASK;
    irqParams.Dio3Mask        = DIO3MASK;
    llcc68_noAddress_opcode(SETDIOIRQPARAMS, 
        TYPE_WRITE, (uint8_t*)&irqParams, sizeof(irqParams));

    radio_llcc68_get_status();
    for(i = 0; i < 0xffff; i++);

    // set sync word (private/public) 
    value = (PRIVATENETWORK >> 8) & 0xFF;
    llcc68_spiWriteReg(LORASYNCWORDMSB,value);
    for(i = 0; i < 0xffff; i++);
    value = PRIVATENETWORK & 0xFF;
    llcc68_spiWriteReg(LORASYNCWORDLSB,value);
    
    // set Tx mode
    llcc68_noAddress_opcode(SETTX, 
        TYPE_WRITE, (uint8_t*)&txMax.Timeout, sizeof(txMax.Timeout));
} 



// Timeout Duration = timeout[] * 15.625 us
void radio_llcc68_rxNow(radioTimeout_t rxMax){
    llcc68_noAddress_opcode(SETRX, 
        TYPE_WRITE, (uint8_t*)&rxMax.Timeout, sizeof(rxMax.Timeout));

}

// set radio to standby_RC mode
void radio_llcc68_rfOff(void){
    uint8_t config = RC_13MHz;
    llcc68_noAddress_opcode(SETSTANDBY, 
        TYPE_WRITE, (uint8_t*)&config, sizeof(config));

    radio_vars.state = LLCC68STATE_STANDBY_RC;
}

// Gets the chip's status byte
void radio_llcc68_get_status(void) {
    uint8_t rx_buf;
    llcc68_noAddress_opcode(GETSTATUS, TYPE_READ, &rx_buf, sizeof(rx_buf));

    memcpy(&radio_vars.status, &rx_buf, sizeof(radio_vars.status));
    radio_llcc68_busy_wait();
}

void radio_llcc68_get_opError(void) {
    uint8_t rx_buf[2];
    llcc68_noAddress_opcode(GETDEVICEERRORS, TYPE_READ, rx_buf, sizeof(rx_buf));
    
    memcpy(&radio_vars.opError, &rx_buf, sizeof(radio_vars.opError));
    radio_llcc68_busy_wait();
}

irqStatus_t radio_llcc68_irq_status(void) {
    uint8_t rx_buf[2];
    llcc68_noAddress_opcode(GETIRQSTATUS, TYPE_READ, rx_buf, sizeof(rx_buf));

    return *(irqStatus_t *)rx_buf;
}

void radio_llcc68_reset(void) {
    
    uint16_t i;
    
    // reset pin low
    NRF_P1->OUTCLR = (1UL << (LLCC68_RESET_PIN & 0x1F));
    // wait > 100us
    for(i = 0; i < 0xffff; i++);
    // reset pin high
    NRF_P1->OUTSET = (1UL << (LLCC68_RESET_PIN & 0x1F));
    
    // wait for chip to go to standby mode
    radio_llcc68_busy_wait();
    do {
      radio_llcc68_get_status();
    } while (radio_vars.status.ChipMode != STBY_RC);

    radio_llcc68_get_opError();
    radio_vars.state = LLCC68STATE_STANDBY_RC;
}

void radio_llcc68_busy_wait(void){
    // wait for BUSY pin to go low
    radio_vars.busy = LLCC68_BUSY;
    while(radio_vars.busy != LLCC68_FREE){
      board_sleep();
    }
}

//=========================== callbacks =======================================
// BUSY pin is going low (falling edge) and ready to communicate 
void radio_llcc68_busy_cb(void){
    radio_vars.busy = LLCC68_FREE;
    
    // Clear event
    //NRF_GPIOTE->EVENTS_IN[channel] = 0;
}


/*





void radio_llcc68_rfOn(void) {
    // power on radio
    NRF_RADIO->POWER = ((uint32_t)(1)) << 0;

    radio_vars.state = RADIOSTATE_STOPPED;
}

void radio_llcc68_rfOff(void) {

    radio_vars.state  = RADIOSTATE_TURNING_OFF;

    radio_vars.state  = RADIOSTATE_RFOFF;
}


void radio_llcc68_loadPacket(uint8_t* packet, uint16_t len) {

    radio_vars.state  = RADIOSTATE_LOADING_PACKET;

    ///< note: 1st byte should be the payload size (for Nordic), and
    ///   the two last bytes are used by the MAC layer for CRC
    if ((len > 0) && (len <= MAX_PACKET_SIZE)) {
        radio_vars.payload[0]= len;
        memcpy(&radio_vars.payload[1], packet, len);
    }

    radio_vars.state  = RADIOSTATE_PACKET_LOADED;
}

void radio_llcc68_txEnable(void) {

    radio_vars.state  = RADIOSTATE_ENABLING_TX;

    NRF_RADIO->EVENTS_READY = (uint32_t)0;

    NRF_RADIO->TASKS_TXEN = (uint32_t)1;
    while(NRF_RADIO->EVENTS_READY==0);


    // wiggle debug pin
    debugpins_radio_set();
    leds_radio_on();

    radio_vars.state  = RADIOSTATE_TX_ENABLED;
}


void radio_llcc68_txNow(void) {

    NRF_RADIO->TASKS_START = (uint32_t)1;

    radio_vars.state = RADIOSTATE_TRANSMITTING;
}


void radio_llcc68_rxEnable(void) {

    radio_vars.state = RADIOSTATE_ENABLING_RX;

    if (NRF_RADIO->STATE != STATE_RX) {

        // turn off radio first
        radio_rfOff();

        NRF_RADIO->EVENTS_READY = (uint32_t)0;

        NRF_RADIO->TASKS_RXEN  = (uint32_t)1;

        while(NRF_RADIO->EVENTS_READY==0);
    }
}


void radio_llcc68_rxNow(void) {

    NRF_RADIO->TASKS_START = (uint32_t)1;

    debugpins_radio_set();
    leds_radio_on();

    radio_vars.state  = RADIOSTATE_LISTENING;
}


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

//=========================== private =========================================



//=========================== callbacks =======================================

