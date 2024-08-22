/**
\brief A CoAP resource which allows an application to GET/SET the state of the
   error LED.
*/

#include "config.h"
#define OPENWSN_CCRAZYFLIE_C 1

#if OPENWSN_CCRAZYFLIE_C

#include "opendefs.h"
#include "ccrazyflie.h"
#include "coap.h"
#include "packetfunctions.h"
// #include "leds.h"
#include "cf_multiranger.h"
#include "openqueue.h"

//=========================== variables =======================================

ccrazyflie_vars_t ccrazyflie_vars;

const uint8_t ccrazyflie_path0[] = "f";

//=========================== prototypes ======================================

owerror_t ccrazyflie_receive(OpenQueueEntry_t *msg,
                       coap_header_iht *coap_header,
                       coap_option_iht *coap_incomingOptions,
                       coap_option_iht *coap_outgoingOptions,
                       uint8_t *coap_outgoingOptionsLen);

void ccrazyflie_sendDone(OpenQueueEntry_t *msg, owerror_t error);

//=========================== public ==========================================

void ccrazyflie_init(void) {

    // prepare the resource descriptor for the /l path
    ccrazyflie_vars.desc.path0len = sizeof(ccrazyflie_path0) - 1;
    ccrazyflie_vars.desc.path0val = (uint8_t * )(&ccrazyflie_path0);
    ccrazyflie_vars.desc.path1len = 0;
    ccrazyflie_vars.desc.path1val = NULL;

    ccrazyflie_vars.desc.componentID = COMPONENT_CCRAZYFLIE;
    ccrazyflie_vars.desc.securityContext = NULL;
    ccrazyflie_vars.desc.discoverable = TRUE;
    ccrazyflie_vars.desc.callbackRx = &ccrazyflie_receive;
    ccrazyflie_vars.desc.callbackSendDone = &ccrazyflie_sendDone;

    // register with the CoAP module
    coap_register(&ccrazyflie_vars.desc);
}

//=========================== private =========================================

/**
\brief Called when a CoAP message is received for this resource.

\param[in] msg          The received message. CoAP header and options already
   parsed.
\param[in] coap_header  The CoAP header contained in the message.
\param[in] coap_options The CoAP options contained in the message.

\return Whether the response is prepared successfully.
*/
owerror_t ccrazyflie_receive(OpenQueueEntry_t *msg,
                       coap_header_iht *coap_header,
                       coap_option_iht *coap_incomingOptions,
                       coap_option_iht *coap_outgoingOptions,
                       uint8_t *coap_outgoingOptionsLen) {
    owerror_t outcome;

    switch (coap_header->Code) {
        case COAP_CODE_REQ_GET:
            // reset packet payload
            msg->payload = &(msg->packet[127]);
            msg->length = 0;

            // add CoAP payload
            if (packetfunctions_reserveHeader(&msg, 1) == E_FAIL) {
                openqueue_freePacketBuffer(msg);
                return E_FAIL;
            }

            // if (leds_error_isOn() == 1) {
            //     msg->payload[0] = '1';
            // } else {
            //     msg->payload[0] = '0';
            // }

            if (mutiranger_up_isClose()) {
                msg->payload[0] = '1';
            } else {
                msg->payload[0] = '0';
            }

            // set the CoAP header
            coap_header->Code = COAP_CODE_RESP_CONTENT;

            outcome = E_SUCCESS;
            break;

        case COAP_CODE_REQ_PUT:

            //// change the LED's state
            //if (msg->payload[0] == '1') {
            //    leds_error_on();
            //} else if (msg->payload[0] == '2') {
            //    leds_error_toggle();
            //} else {
            //    leds_error_off();
            //}

            // reset packet payload
            msg->payload = &(msg->packet[127]);
            msg->length = 0;

            // set the CoAP header
            coap_header->Code = COAP_CODE_RESP_CHANGED;

            outcome = E_SUCCESS;
            break;

        default:
            outcome = E_FAIL;
            break;
    }

    return outcome;
}

/**
\brief The stack indicates that the packet was sent.

\param[in] msg The CoAP message just sent.
\param[in] error The outcome of sending it.
*/
void ccrazyflie_sendDone(OpenQueueEntry_t *msg, owerror_t error) {
    openqueue_freePacketBuffer(msg);
}

#endif /* OPENWSN_CCRAZYFLIE_C */
