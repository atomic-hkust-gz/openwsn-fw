#incldue "radio_llcc68.h"
#include "radio.h"
#include "radio_wrapper.h"



//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== public ==========================================

//=========================== private =========================================

void radio_phy_init(radio_phy_type phy, gpioIrq_cbt cb) {
    if (phy == IEEE802154) {
      radio_init();
    }

    else if (phy == LORA) {
      radio_llcc68_init(cb);
    }

    else { break();}

}

void radio_phy_on(radio_phy_type phy) {
    if (phy == IEEE802154) {
      radio_rfOn();
    }

    else if (phy == LORA) {

    }

    else { break();}
};

void radio_phy_off(radio_phy_type phy) {
    if (phy == IEEE802154) {
      radio_rfOff();
    }

    else if (phy == LORA) {

    }

    else { break();}
}

void radio_phy_txEnable(radio_phy_type phy) {
    if (phy == IEEE802154) {
      radio_txEnable();
      radio_txNow();
    }

    else if (phy == LORA) {

    }

    else { break();}
};

void radio_phy_rxEnable(radio_phy_type phy) {
    if (phy == IEEE802154) {
      radio_rxEnable();
      radio_rxNow();
    }

    else if (phy == LORA) {

    }

    else { break();}
}

void radio_phy_setFrameCb(radio_phy_type phy) {
    if (phy == IEEE802154) {


    }

    else if (phy == LORA) {

    }

    else { break();}

}

void radio_phy_loadPacket(radio_phy_type phy, uint8_t* packet, uint16_t len) {
    if (phy == IEEE802154) {


    }

    else if (phy == LORA) {

    }

    else { break();}
}






