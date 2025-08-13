/**
 * Author: Jacob Louie (jlouie475@connect.hkust-gz.edu.cn)
 * Company: HKUST(GZ)
 * Date:   Aug 2025
 * Description: nRF52840-specific definition of the SPI module
 */

#include "board.h"
#include "nrf52840.h"
#include "nrf52840_bitfields.h"
#include "spi.h"

//=========================== defines =========================================
#define NRF_GPIO_PIN_MAP(port, pin) (((port) << 5) | ((pin) & 0x1F))
// Pin assignments
#define SPI_SS_PIN    NRF_GPIO_PIN_MAP(1,2)   // P1.02
#define SPI_MOSI_PIN  NRF_GPIO_PIN_MAP(1,3)   // P1.03
#define SPI_MISO_PIN  NRF_GPIO_PIN_MAP(1,4)   // P1.04
#define SPI_SCK_PIN   NRF_GPIO_PIN_MAP(1,5)   // P1.05


// SPI instance
#define SPIM NRF_SPIM0

// SPI freqeuncy
#define SPI_FREQ       SPIM_FREQUENCY_FREQUENCY_M4  // 4 Mbps

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== public ==========================================

// configure GPIOs and SPIM0
void spi_init(void) {

    // configure SPI pins
    nrf_gpio_cfg_output(SPI_SCK_PIN);
    nrf_gpio_cfg_output(SPI_MOSI_PIN);
    nrf_gpio_cfg_input(SPI_MISO_PIN);
    nrf_gpio_cfg_output(SPI_SS_PIN);

    // de-assert SS
    NRF_P1->OUTSET = (1UL << (SPI_SS_PIN & 0x001F));

    // disable SPIM0 before configuration
    NRF_SPIM0->ENABLE = (SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos);

    // assign SPI pins (no automatic SS pin used)
    NRF_SPIM0->PSEL.SCK  = SPI_SCK_PIN;
    NRF_SPIM0->PSEL.MOSI = SPI_MOSI_PIN;
    NRF_SPIM0->PSEL.MISO = SPI_MISO_PIN;

    // set SPI frequency
    NRF_SPIM0->FREQUENCY = SPI_FREQ;

    // configure mode 0: CPOL=0, CPHA=0, MSB first
    NRF_SPIM0->CONFIG = 
          (SPIM_CONFIG_ORDER_MsbFirst   << SPIM_CONFIG_ORDER_Pos)
        | (SPIM_CONFIG_CPHA_Leading     << SPIM_CONFIG_CPHA_Pos)
        | (SPIM_CONFIG_CPOL_ActiveHigh  << SPIM_CONFIG_CPOL_Pos);

    // Enable SPIM0
    NRF_SPIM0->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
}

// blocking SPI transfer using EasyDMA
void spi_txrx(uint8_t*     bufTx,
                 uint16_t     lenbufTx,
                 spi_return_t returnType,
                 uint8_t*     bufRx,
                 uint16_t     maxLenBufRx,
                 spi_first_t  isFirst,
                 spi_last_t   isLast) {

    // assert SS (active low)
    NRF_P1->OUTCLR = (1UL << (SPI_SS_PIN & 0x1F));

    // set up DMA pointers
    NRF_SPIM0->TXD.PTR    = (uint32_t)bufTx;
    NRF_SPIM0->TXD.MAXCNT = lenbufTx;

    NRF_SPIM0->RXD.PTR    = (uint32_t)bufRx;
    NRF_SPIM0->RXD.MAXCNT = maxLenBufRx;

    // clear previous events
    NRF_SPIM0->EVENTS_END = 0;

    // start transfer
    NRF_SPIM0->TASKS_START = 1;

    // wait for completion
    while (NRF_SPIM0->EVENTS_END == 0);

    // de-assert SS
    NRF_P1->OUTSET = (1UL << (SPI_SS_PIN & 0x1F));
}

//=========================== private =========================================