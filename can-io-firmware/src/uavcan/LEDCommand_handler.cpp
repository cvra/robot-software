/* For accessing the GPIO functions */
#include <hal.h>

/* UAVCAN interface */
#include <uavcan/uavcan.hpp>

/* Code generated from our .uavcan file */
#include <cvra/io/LEDCommand.hpp>

/* This function is called every time we receive a request for our newly
 * defined LedCommand service.
 */
void LEDCommand_handler(
    const uavcan::ReceivedDataStructure<cvra::io::LEDCommand::Request>& req,
    cvra::io::LEDCommand::Response& rsp)
{
    /*  req contains the incoming data, while rsp will be sent back as a reply
     *  to this request. */

    /* Set the LED status to the requested value */
    if (req.led_status) {
        palSetPad(GPIOA, GPIOA_LED);
    } else {
        palClearPad(GPIOA, GPIOA_LED);
    }

    /* Reply with an example data. */
    rsp.data = 42;
}
