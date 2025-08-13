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
#define NRF_GPIO_PIN_MAP(port, pin) (((port) << 5) | ((pin) & 0x1F))
// Pin assignments
#define LLCC68_IRQ_PIN   NRF_GPIO_PIN_MAP(1,6)   // P1.06
#define LLCC68_BUSY_PIN  NRF_GPIO_PIN_MAP(1,7)   // P1.07
#define LLCC68_RESET_PIN NRF_GPIO_PIN_MAP(1,8)   // P1.08

#define SCTIMER_PERIOD              32 // @32kHz = 1ms

#define MAX_PACKET_SIZE             128 // 255 actual max
#define IRQ_MASK                    0xFF


// lora configuration
#define RF_FREQUENCY                490000000 // 490 MHz
#define LORA_BANDWIDTH              0x04      // 0x04 = 125 kHz
#define LORA_SPREADING_FACTOR       0X07      // SF7
#define LORA_CODINGRATE             0X01      // CR 4/5
#define LORA_PREAMBLE_LENGTH        0X08
#define LORA_PAYLOAD_LENGTH         0X80      // 128
#define LORA_TX_BASE_ADDR           0x00
#define LORA_RX_BASE_ADDR           0x80      // Max packet size = 128
//#define LORA_TX_POWER_DBM           14

//=========================== variables =======================================
/*
typedef struct {
    radio_capture_cbt   startFrame_cb;
    radio_capture_cbt   endFrame_cb;
    radio_state_t       state; 
    //This payload length will effect ble rx
    uint8_t             payload[1+MAX_PACKET_SIZE] __attribute__ ((aligned));
    bool                hfc_started;
} radio_vars_t;

static radio_vars_t radio_vars;
*/
//=========================== prototypes ======================================

//static uint32_t swap_bits(uint32_t inp);
//static uint32_t bytewise_bitswap(uint32_t inp);

//=========================== public ==========================================

void radio_llcc68_init(void) {
    
    radioModulationParams_t loraModParams;
    radioTxParams_t radioTxParams;
    packetParams_t packetParams;
    bufferBaseAddress_t bufferBaseAddress;
    irqStatus_t irqStatus;
    uint8_t value;

    // clear internal variables
    memset(&loraModParams, 0, sizeof(loraModParams));
    memset(&radioTxParams, 0, sizeof(radioTxParams));
    memset(&packetParams, 0, sizeof(packetParams));
    memset(&bufferBaseAddress, 0, sizeof(bufferBaseAddress));
    memset(&irqStatus, 0, sizeof(irqStatus));
    
    // nrf pin configure
    nrf_gpio_cfg_input(LLCC68_IRQ_PIN);
    nrf_gpio_cfg_input(LLCC68_BUSY_PIN);
    nrf_gpio_cfg_output(LLCC68_RESET_PIN);
    // set busy pin to pulldown
    NRF_P1->PIN_CNF[LLCC68_BUSY_PIN & 0x1F] =
      (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

    // reset
    radio_llcc68_reset();

    // tx clamp config
    // workaround: must be set to 0x1E
    llcc68_spiWriteReg(TXCLAMPCONFIG, TX_CLAMP_WORKAROUND);

    // clock calibration 
    value = CALIBRATE_ALL;
    llcc68_noAddress_opcode(CALIBRATE, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));
    radio_llcc68_wait_on_busy();

    // set to standby mode and use XTAL_32 MHz clock reference 
    value = XTAL_32MHz;
    llcc68_noAddress_opcode(SETSTANDBY, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));

    // set packet type
    value = PACKET_TYPE_LORA;
    llcc68_noAddress_opcode(SETPACKETTYPE, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));

    // set RF Frequency
    value = RF_FREQ_490_MHZ;
    llcc68_noAddress_opcode(SETRFFREQUENCY, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));

    // set power amplifier configuration
    value = PA_CONFIG_17_DBM;
    llcc68_noAddress_opcode(SETPACONFIG, 
        TYPE_WRITE, (uint8_t*)&value, sizeof(value));

    // set Tx parameters
    radioTxParams = (radioTxParams_t){
        .TxPowerDbm           = TX_P22_DBM,
        .TxRampTime           = RAMP_200U
    };
    llcc68_noAddress_opcode(SETTXPARAMS, 
        TYPE_WRITE, (uint8_t*)&radioTxParams, sizeof(radioTxParams));

    // set modulation parameters
    loraModParams = (radioModulationParams_t){
        .SpreadingFactor      = LORA_SF7,
        .Bandwidth            = LORA_BW_125,
        .CodingRate           = LORA_CR_4_5,
        .LowDataRateOptimize  = LDRO_ON
    };
    llcc68_noAddress_opcode(SETMODULATIONPARAMS, 
        TYPE_WRITE, (uint8_t*)&loraModParams, sizeof(loraModParams));

    // set packet parameters
    packetParams = (packetParams_t){
        .PreambleLength       = LORA_PREAMBLE_LENGTH,
        .HeaderType           = FIXED_LENGTH_PACKET,
        .PayloadLength        = LORA_PAYLOAD_LENGTH,
        .CrcType              = CRC_ON,
        .InvertIq             = STD_IQ
    };
    llcc68_noAddress_opcode(SETMODULATIONPARAMS, 
        TYPE_WRITE, (uint8_t*)&packetParams, sizeof(packetParams));

    // set buffer base addresses
    bufferBaseAddress = (bufferBaseAddress_t){
        .TxBaseAddress        = LORA_TX_BASE_ADDR,
        .RxBaseAddress        = LORA_RX_BASE_ADDR
    };
    llcc68_noAddress_opcode(SETBUFFERBASEADDRESS, 
        TYPE_WRITE, (uint8_t*)&bufferBaseAddress, sizeof(bufferBaseAddress));

    // clear IRQ status
    memset(&irqStatus, IRQ_MASK, sizeof(irqStatus));
    llcc68_noAddress_opcode(CLEARIRQSTATUS, 
        TYPE_WRITE, (uint8_t*)&irqStatus, sizeof(irqStatus));

    // WriteBuffer
    // SetTx
    // GetIrqStatus
    // GetRxBufferStatus
    // ReadBuffer
    // GetStatus
  
}
// Gets the chip's status byte
uint8_t radio_llcc68_get_status(void) {
    uint8_t rx_buf;
    llcc68_noAddress_opcode(GETSTATUS, TYPE_READ, rx_buf, sizeof(rx_buf));

    // After the command, the chip returns the status byte as the first data byte
    return rx_buf;
}

// Waits until the chip is no longer busy
void radio_llcc68_wait_on_busy(void) {
    // wait for busy pin to go low
    while ((NRF_P1->IN & (1UL << (LLCC68_BUSY_PIN & 0x1F))) != 0) {
        board_sleep();
    }
}


irqStatus_t radio_llcc68_irq_status(void) {
    uint8_t rx_buf[] = {0,0};
    llcc68_noAddress_opcode(GETIRQSTATUS, TYPE_READ, rx_buf, sizeof(rx_buf));

    return *(irqStatus_t *)rx_buf;
}

void radio_llcc68_reset(void) {

    // reset pin low
    NRF_P1->OUTCLR = (1UL << (LLCC68_RESET_PIN & 0x1F));

    // wait 1 ms (>100 us)
    sctimer_set_callback(cb_compare);
    sctimer_setCompare(sctimer_readCounter()+SCTIMER_PERIOD);
    while (1) {
        board_sleep();
    }

    // reset pin high
    NRF_P1->OUTSET = (1UL << (LLCC68_RESET_PIN & 0x1F));

    radio_llcc68_wait_on_busy();
}

/*
void radio_llcc68_setStartFrameCb(radio_capture_cbt cb) {

    radio_vars.startFrame_cb  = cb;
}


void radio_llcc68_setEndFrameCb(radio_capture_cbt cb) {

    radio_vars.endFrame_cb = cb;
}





void radio_llcc68_setFrequency(uint8_t frequency, radio_freq_t tx_or_rx) {

    NRF_RADIO->FREQUENCY = FREQUENCY_STEP*(frequency-FREQUENCY_OFFSET);

    radio_vars.state     = RADIOSTATE_FREQUENCY_SET;
}

int8_t radio_llcc68_getFrequencyOffset(void){
  
    return 0; 
}

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

static uint32_t swap_bits(uint32_t inp) {

    uint32_t i;
    uint32_t retval = 0;

    inp = (inp & 0x000000FFUL);

    for (i = 0; i < 8; i++) {
        retval |= ((inp >> i) & 0x01) << (7 - i);
    }

    return retval;
}


static uint32_t bytewise_bitswap(uint32_t inp) {

    return (swap_bits(inp >> 24) << 24)
          | (swap_bits(inp >> 16) << 16)
          | (swap_bits(inp >> 8) << 8)
          | (swap_bits(inp));
}


//=========================== callbacks =======================================

//=========================== interrupt handlers ==============================

void RADIO_IRQHandler(void) {

    if (NRF_RADIO->EVENTS_ADDRESS) {

        NRF_RADIO->EVENTS_ADDRESS = 0;

        if (radio_vars.startFrame_cb) {
            radio_vars.startFrame_cb(sctimer_readCounter());
        }
    }

    if (NRF_RADIO->EVENTS_END) {

        NRF_RADIO->EVENTS_END = 0;

        if (radio_vars.endFrame_cb) {
            radio_vars.endFrame_cb(sctimer_readCounter());
        }
    }
}
*/

void cb_compare(void) {   
   // toggle error led
   leds_error_toggle();
      
   // schedule again
   sctimer_setCompare(sctimer_readCounter()+SCTIMER_PERIOD);
}