/**
 * Author: Jacob Louie (jlouie475@connect.hkust-gz.edu.cn)
 * Company: HKUST(GZ)
 * Date:   Aug 2025
 * Description: nRF52840-specific definition of the SPI module
 */

#include "nrf52840.h"
#include "nrf52840_bitfields.h"
#include "spi.h"

//=========================== defines =========================================
#define NRF_GPIO_PIN_MAP(port, pin) (((port) << 5) | ((pin) & 0x1F))
// Pin assignments
#define SPI_SS_PIN   NRF_GPIO_PIN_MAP(0,0)   // P0.00
#define SPI_MOSI_PIN NRF_GPIO_PIN_MAP(0,1)   // P0.01
#define SPI_MISO_PIN NRF_GPIO_PIN_MAP(0,5)   // P0.05
#define SPI_SCK_PIN  NRF_GPIO_PIN_MAP(0,6)   // P0.06
//#define SPI_IRQ_PIN  NRF_GPIO_PIN_MAP(0,7)   // P0.07

// SPI instance
#define SPIM NRF_SPIM0

// SPI freqeuncy
#define SPI_FREQ       SPIM_FREQUENCY_FREQUENCY_M4  // 4 Mbps

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== public ==========================================

// Configure GPIOs and SPIM0
void spi_init(void) {
    // Configure SPI pins
    nrf_gpio_cfg_output(SPI_SCK_PIN);
    nrf_gpio_cfg_output(SPI_MOSI_PIN);
    nrf_gpio_cfg_input(SPI_MISO_PIN);
    nrf_gpio_cfg_output(SPI_SS_PIN);

    // De-assert SS
    if (SPI_SS_PIN < 32) {
        NRF_P0->OUTSET = (1UL << SPI_SS_PIN);
    } else {
        NRF_P1->OUTSET = (1UL << (SPI_SS_PIN & 0x1F));
    }

    // Disable SPIM0 before configuration
    NRF_SPIM0->ENABLE = (SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos);

    // Assign SPI pins (no automatic SS pin used)
    NRF_SPIM0->PSEL.SCK  = SPI_SCK_PIN;
    NRF_SPIM0->PSEL.MOSI = SPI_MOSI_PIN;
    NRF_SPIM0->PSEL.MISO = SPI_MISO_PIN;

    // Set SPI frequency
    NRF_SPIM0->FREQUENCY = SPI_FREQ;

    // Configure mode 0: CPOL=0, CPHA=0, MSB first
    NRF_SPIM0->CONFIG = 
          (SPIM_CONFIG_ORDER_MsbFirst   << SPIM_CONFIG_ORDER_Pos)
        | (SPIM_CONFIG_CPHA_Leading     << SPIM_CONFIG_CPHA_Pos)
        | (SPIM_CONFIG_CPOL_ActiveHigh  << SPIM_CONFIG_CPOL_Pos);

    // Enable SPIM0
    NRF_SPIM0->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
}

// Blocking SPI transfer using EasyDMA
void spi_transfer(uint8_t* tx_buf, uint8_t* rx_buf, uint32_t length) {
    // Assert SS (active low)
    if (SPI_SS_PIN < 32) {
        NRF_P0->OUTCLR = (1UL << SPI_SS_PIN);
    } else {
        NRF_P1->OUTCLR = (1UL << (SPI_SS_PIN & 0x1F));
    }

    // Set up DMA pointers
    NRF_SPIM0->TXD.PTR    = (uint32_t)tx_buf;
    NRF_SPIM0->TXD.MAXCNT = length;

    NRF_SPIM0->RXD.PTR    = (uint32_t)rx_buf;
    NRF_SPIM0->RXD.MAXCNT = length;

    // Clear previous events
    NRF_SPIM0->EVENTS_END = 0;

    // Start transfer
    NRF_SPIM0->TASKS_START = 1;

    // Wait for completion
    while (NRF_SPIM0->EVENTS_END == 0);

    // De-assert SS
    if (SPI_SS_PIN < 32) {
        NRF_P0->OUTSET = (1UL << SPI_SS_PIN);
    } else {
        NRF_P1->OUTSET = (1UL << (SPI_SS_PIN & 0x1F));
    }
}

//=========================== private =========================================

void nrf_gpio_cfg_input(uint32_t pin_number) {
    NRF_GPIO_Type* port = (pin_number < 32) ? NRF_P0 : NRF_P1;
    uint32_t pin = pin_number & 0x1F;

    port->PIN_CNF[pin] =
          (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)
        | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
        | (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

void nrf_gpio_cfg_output(uint32_t pin_number) {
    NRF_GPIO_Type* port = (pin_number < 32) ? NRF_P0 : NRF_P1;
    uint32_t pin = pin_number & 0x1F;

    port->PIN_CNF[pin] =
          (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos)
        | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)
        | (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}