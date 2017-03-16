#include <ch.h>
#include <hal.h>
#include "DigitalInput_pub.hpp"
#include <cvra/io/DigitalInput.hpp>

uavcan::INode *node;

void read_inputs(bool *result)
{
    // Workaround the lack of inputs
    palSetPadMode(GPIOB, GPIOB_DEBUG_RX, PAL_STM32_MODE_INPUT);

    result[0] = palReadPad(GPIOA, GPIOA_PIN0);
    result[1] = palReadPad(GPIOA, GPIOA_PIN1);
    result[2] = palReadPad(GPIOA, GPIOA_PIN2);
    result[3] = palReadPad(GPIOA, GPIOA_PIN3);

    result[4] = palReadPad(GPIOB, GPIOB_SWO);
    result[5] = palReadPad(GPIOB, GPIOB_PIN4);
    result[6] = palReadPad(GPIOB, GPIOB_PIN5);
    result[7] = palReadPad(GPIOB, GPIOB_DEBUG_RX);
}

void digital_input_thread(void *p)
{
    uavcan::INode *node = (uavcan::INode *)p;

    uavcan::Publisher<cvra::io::DigitalInput> pub(*node);

    while (true) {
        bool values[8];

        read_inputs(values);

        cvra::io::DigitalInput msg = cvra::io::DigitalInput();

        for (int i = 0; i < 8; i++) {
            msg.pin[i] = values[i];
        }

        pub.broadcast(msg);

        chThdSleepMilliseconds(100);
    }
}

void DigitalInput_pub_start(uavcan::INode &node)
{
    static THD_WORKING_AREA(wa, 700);
    chThdCreateStatic(wa, sizeof(wa), LOWPRIO, digital_input_thread, &node);
}
