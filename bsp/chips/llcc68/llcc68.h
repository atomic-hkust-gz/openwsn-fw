/* This file is auto-generated. Do not modify manually! */

#ifndef LLCC68_H
#define LLCC68_H

//=========================== typedef =========================================

typedef enum {
    TYPE_READ   = 0,
    TYPE_WRITE  = 1,
}type_t;

typedef struct { 
    uint16_t  addr;
    uint8_t   data;
}registerSetting_t;

typedef struct {
    uint16_t    channel_spacing;
    uint32_t    frequency_0;
    uint16_t    channel;
}frequencySetting_t;

// ========================== defines =========================================

// LoRa Configuration Constants
#define RF_FREQUENCY                490000000 // 490 MHz
#define LORA_BANDWIDTH              0x04      // 0x04 = 125 kHz
#define LORA_SPREADING_FACTOR       7         // SF7
#define LORA_CODINGRATE             1         // CR 4/5
#define LORA_PREAMBLE_LENGTH        8
#define LORA_TX_POWER_DBM           14

// Operating modes
#define SETSLEEP            0x84
#define SETSTANDBY          0x80
#define SETFS               0xC1
#define SETTX               0x83
#define SETRX               0x82
#define STOPTIMERONPREAMBLE 0x8F
#define SETRXDUTYCYCLE      0x9F
#define SETCAD              0xC5
#define SETTXCONTINUOUSWAVE 0xD1
#define SETREGULATORMODE    0x96
#define CALIBRATE           0x89
#define CALIBRATEIMAGE      0x98
#define SETPACONFIG         0x95
#define SETRXTXFALLBACKMODE 0X93

// Register access
#define WRITEREGISTER       0x0D
#define READREGISTER        0x1D
#define WRITEBUFFER         0x0E
#define READBUFFER          0x1E

// IRQ and DIO
#define SETDIOIRQPARAMS     0x08
#define GETIRQSTATUS        0x12
#define CLEARIRQSTATUS      0x02
#define SETDIO2ASRFSWITCHCTRL 0x9D
#define SETDIO3ASTCXOCTRL   0x97

// RF and packet settings
#define SETRFFREQUENCY      0x86
#define SETPACKETTYPE       0x8A
#define GETPACKETTYPE       0x11
#define SETTXPARAMS         0x8E
#define SETMODULATIONPARAMS 0x8B
#define SETPACKETPARAMS     0x8C
#define SETCADPARAMS        0x88
#define SETBUFFERBASEADDRESS 0x8F
#define SETLORASYMBNUMTIMEOUT 0xA0

// Radio status
#define GETSTATUS           0xC0
#define GETRSSILNST         0x15
#define GETRXBUFFERSTATUS   0x13
#define GETPACKETSTATUS     0x14
#define GETDEVICEERRORS     0x17
#define GETSTATS            0x10
#define RESETSTATS          0x00

// Register Table
#define DIOXOUTPUTENABLE    0x0580
#define DIOXINPUTENABLE     0x0583
#define DIOXPULLUPCTRL      0x0584
#define DIOXPULLDOWNCTRL    0x0585
#define WHITENINGMSB        0x06B8
#define WHITENINGLSB        0x06B9
#define CRCMSB              0x06BC
#define CRCLSB              0x06BD
#define CRCMSBPOLY          0x06BE
#define CRCLSBPOLY          0x06BF
#define SYNCWORD0           0x06C0
#define SYNCWORD1           0x06C1
#define SYNCWORD2           0x06C2
#define SYNCWORD3           0x06C3
#define SYNCWORD4           0x06C4
#define SYNCWORD5           0x06C5
#define SYNCWORD6           0x06C6
#define SYNCWORD7           0x06C7
#define NODEADDRESS         0x06CD
#define BROADCASTADDRESS    0x06CD
#define IQPOLARITY          0x0736
#define LORASYNCWORDMSB     0x0740  // set to PUBLICNETWORK or PRIVATENETWORK
#define LORASYNCWORDLSB     0x0741
#define PUBLICNETWORK       0x3444
#define PRIVATENETWORK      0x1424
#define RANDOMNUMBERGEN0    0x0819
#define RANDOMNUMBERGEN1    0x081A
#define RANDOMNUMBERGEN2    0x081B
#define RANDOMNUMBERGEN3    0x081C
#define TXMODULATION        0x0889
#define RXGAIN              0x08AC
#define TXCLAMPCONFIG       0x08AC
#define OCPCONFIG           0x08E7
#define RTCCTRL             0x0902
#define XTATRIM             0x0911
#define XTBTRIM             0x0912
#define DIO3OUTPUTVOLTAGE   0x920
#define EVENTMASK           0x944

#define RC_13MHz            0x00 
#define XTAL_32MHz          0x01

#define MODE_FSK            0x00
#define MODE_LORA           0x01

/* ========================== prototypes =================================== */

void llcc68_spiWriteReg(uint16_t reg, uint8_t regValueToWrite);
uint8_t llcc68_spiReadReg(uint16_t reg);
void llcc68_multipleBytesRead(uint16_t reg, uint8_t* buffer, uint8_t len);
void llcc68_multipleBytesWrite(uint16_t reg, uint8_t* buffer, uint8_t len) ;
void llcc68_noAddress_opcode(uint8_t opcode, type_t rw, uint8_t* buffer, uint8_t len);


#endif