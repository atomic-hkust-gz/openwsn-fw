#ifndef __LLCC68_H
#define __LLCC68_H

//=========================== typedef =========================================

typedef enum {
    TYPE_READ   = 0,
    TYPE_WRITE  = 1,
}type_t;

typedef enum {
    LLCC68_FREE   = 0x00,
    LLCC68_BUSY   = 0x01,
}llcc68_busy_t;

//========================== defines ==========================================
#define LSB_FIRST_16(x) (uint16_t)((((x) & 0xFF) << 8) | (((x) >> 8) & 0xFF))

// operating modes
#define SETSLEEP              0x84
#define SETSTANDBY            0x80    // set RC or XTAL clock source
#define SETFS                 0xC1
#define SETTX                 0x83
#define SETRX                 0x82
#define STOPTIMERONPREAMBLE   0x9F
#define SETRXDUTYCYCLE        0x94
#define SETCAD                0xC5
#define SETTXCONTINUOUSWAVE   0xD1
#define SETTXINFINITEPREAMBLE 0xD2
#define SETREGULATORMODE      0x96
#define CALIBRATE             0x89
#define CALIBRATEIMAGE        0x98
#define SETPACONFIG           0x95
#define SETRXTXFALLBACKMODE   0X93

// register access
#define WRITEREGISTER         0x0D
#define READREGISTER          0x1D
#define WRITEBUFFER           0x0E
#define READBUFFER            0x1E

// IRQ and DIO
#define SETDIOIRQPARAMS       0x08
#define GETIRQSTATUS          0x12
#define CLEARIRQSTATUS        0x02
#define SETDIO2ASRFSWITCHCTRL 0x9D
#define SETDIO3ASTCXOCTRL     0x97

// RF and packet settings
#define SETRFFREQUENCY        0x86
#define SETPACKETTYPE         0x8A
#define GETPACKETTYPE         0x11
#define SETTXPARAMS           0x8E
#define SETMODULATIONPARAMS   0x8B
#define SETPACKETPARAMS       0x8C
#define SETCADPARAMS          0x88
#define SETBUFFERBASEADDRESS  0x8F
#define SETLORASYMBNUMTIMEOUT 0xA0

// radio status
#define GETSTATUS             0xC0
#define GETRSSILNST           0x15
#define GETRXBUFFERSTATUS     0x13
#define GETPACKETSTATUS       0x14
#define GETDEVICEERRORS       0x17
#define CLEARDEVICEERRORS     0x07
#define GETSTATS              0x10
#define RESETSTATS            0x00

// register table
#define DIOXOUTPUTENABLE      LSB_FIRST_16(0x0580)
#define DIOXINPUTENABLE       LSB_FIRST_16(0x0583)
#define DIOXPULLUPCTRL        LSB_FIRST_16(0x0584)
#define DIOXPULLDOWNCTRL      LSB_FIRST_16(0x0585)
#define WHITENINGMSB          LSB_FIRST_16(0x06B8)
#define WHITENINGLSB          LSB_FIRST_16(0x06B9)
#define CRCMSB                LSB_FIRST_16(0x06BC)
#define CRCLSB                LSB_FIRST_16(0x06BD)
#define CRCMSBPOLY            LSB_FIRST_16(0x06BE)
#define CRCLSBPOLY            LSB_FIRST_16(0x06BF)
#define SYNCWORD0             LSB_FIRST_16(0x06C0)
#define SYNCWORD1             LSB_FIRST_16(0x06C1)
#define SYNCWORD2             LSB_FIRST_16(0x06C2)
#define SYNCWORD3             LSB_FIRST_16(0x06C3)
#define SYNCWORD4             LSB_FIRST_16(0x06C4)
#define SYNCWORD5             LSB_FIRST_16(0x06C5)
#define SYNCWORD6             LSB_FIRST_16(0x06C6)
#define SYNCWORD7             LSB_FIRST_16(0x06C7)
#define NODEADDRESS           LSB_FIRST_16(0x06CD)
#define BROADCASTADDRESS      LSB_FIRST_16(0x06CE)
#define IQPOLARITY            LSB_FIRST_16(0x0736)
// set to PUBLICNETWORK or PRIVATENETWORK
#define LORASYNCWORDMSB       LSB_FIRST_16(0x0740) 
#define LORASYNCWORDLSB       LSB_FIRST_16(0x0741)
#define PUBLICNETWORK         0x3444
#define PRIVATENETWORK        0x1424
#define RANDOMNUMBERGEN0      LSB_FIRST_16(0x0819)
#define RANDOMNUMBERGEN1      LSB_FIRST_16(0x081A)
#define RANDOMNUMBERGEN2      LSB_FIRST_16(0x081B)
#define RANDOMNUMBERGEN3      LSB_FIRST_16(0x081C)
#define TXMODULATION          LSB_FIRST_16(0x0889)
#define RXGAIN                LSB_FIRST_16(0x08AC)
#define TXCLAMPCONFIG         LSB_FIRST_16(0x08D8)
#define OCPCONFIG             LSB_FIRST_16(0x08E7)
#define RTCCTRL               LSB_FIRST_16(0x0902)
#define XTATRIM               LSB_FIRST_16(0x0911)
#define XTBTRIM               LSB_FIRST_16(0x0912)
#define DIO3OUTPUTVOLTAGE     LSB_FIRST_16(0x0920)
#define EVENTMASK             LSB_FIRST_16(0x0944)

//========================== prototypes ======================================

void llcc68_init(void);
void llcc68_spiWriteReg(uint16_t reg, uint8_t regValueToWrite);
uint8_t llcc68_spiReadReg(uint16_t reg);
void llcc68_txBufferWrite(uint8_t offset, uint8_t* buffer, uint8_t len);
void llcc68_rxBufferRead(uint8_t offset, uint8_t* buffer, uint8_t len);
void llcc68_noAddress_opcode(uint8_t opcode, type_t rw, uint8_t* buffer, uint8_t len);

//=========================== callbacks =======================================

//void llcc68_busy_cb(void);

//========================== settings =========================================

// standby mode set
#define RC_13MHz            0x00 
#define XTAL_32MHz          0x01

// set packet type
#define PACKET_TYPE_GFSK    0x00
#define PACKET_TYPE_LORA    0x01

#define TX_CLAMP_WORKAROUND 0x1E
#define CALIBRATE_ALL       0x7F

// Optimal PA settings according to the datasheet
// format: { paDutyCycle, hpMax, deviceSel (0), paLut (1) }
static const uint8_t PA_CONFIG_22_DBM[] = { 0x04, 0x07, 0x00, 0x01 };
static const uint8_t PA_CONFIG_20_DBM[] = { 0x03, 0x05, 0x00, 0x01 };
static const uint8_t PA_CONFIG_17_DBM[] = { 0x02, 0x03, 0x00, 0x01 }; // 17 dBm for China
static const uint8_t PA_CONFIG_14_DBM[] = { 0x02, 0x02, 0x00, 0x01 }; // 14 dBm for Japan/ Europe

// RF_frequency configuration 
// freq_val = (uint32_t)((double)RF_FREQUENCY / (double)32000000 * (double)(1 << 25));
// buffer[0] = (freq_val >> 24) & 0xFF;
// buffer[1] = (freq_val >> 16) & 0xFF;
// buffer[2] = (freq_val >> 8) & 0xFF;
// buffer[3] = freq_val & 0xFF;
// (RF_FREQ/F_XTAL)*2^25                 {  LSB,     ,     ,  MSB }
static const uint8_t RF_FREQ_434_MHZ[] = { 0x00, 0x00, 0x20, 0x1B }; // (434 MHz/32 MHz)*2^25 | or 434*2^20
static const uint8_t RF_FREQ_490_MHZ[] = { 0x00, 0x00, 0xA0, 0x1E }; // (490 MHz/32 MHz)*2^25 | or 490*2^20
static const uint8_t RF_FREQ_868_MHZ[] = { 0x00, 0x00, 0x40, 0x36 }; // (868 MHz/32 MHz)*2^25 | or 868*2^20
static const uint8_t RF_FREQ_915_MHZ[] = { 0x00, 0x00, 0x30, 0x39 }; // (915 MHz/32 MHz)*2^25 | or 915*2^20

// image calibration over the ISM bands
// Frequency band (MHz) = 0x Freq1(1byte) Freq2(1byte)
static const uint8_t FREQ_BAND_430_440[] = { 0x6B, 0x6F }; // 
static const uint8_t FREQ_BAND_470_510[] = { 0x75, 0x81 }; // China
static const uint8_t FREQ_BAND_779_787[] = { 0xC1, 0xC5 }; // 
static const uint8_t FREQ_BAND_863_870[] = { 0xD7, 0xDB }; // Europe
static const uint8_t FREQ_BAND_902_928[] = { 0xE1, 0xE9 }; // USA


#endif