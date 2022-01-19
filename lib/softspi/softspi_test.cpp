#include <cstring>
#include <cstdint>
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "softspi/softspi.h"

extern "C" void softspi_sck_set(softspi_t* dev, int status)
{
    mock("softspi").actualCall("set_sck").withBoolParameter("state", status);
}

extern "C" void softspi_mosi_set(softspi_t* dev, int data)
{
    mock("softspi").actualCall("set_mosi").withBoolParameter("data", data);
}

extern "C" int softspi_miso_get(softspi_t* dev)
{
    return mock("softspi").actualCall("get_miso").returnIntValueOrDefault(0);
}

TEST_GROUP (SoftSpiTestGroup) {
    static constexpr int bufsize = 10;
    softspi_t spi;
    uint8_t txbuf[bufsize];
    uint8_t rxbuf[bufsize];

    void setup() override
    {
        memset(txbuf, 0, sizeof txbuf);
        mock("softspi").strictOrder();
    }
};

TEST(SoftSpiTestGroup, InitPutsClockLow)
{
    mock("softspi").expectOneCall("set_sck").withBoolParameter("state", false);
    softspi_init(&spi);
}

TEST(SoftSpiTestGroup, CanSendSpi)
{
    // First 4 bits transmitted will be '1' (MSB first)
    txbuf[0] = 0xf0;
    for (auto i = 0; i < 4; i++) {
        mock("softspi").expectOneCall("set_mosi").withBoolParameter("data", true);
        mock("softspi").expectOneCall("set_sck").withBoolParameter("state", true);
        mock("softspi").expectOneCall("set_sck").withBoolParameter("state", false);
    }
    // All the followup bits are '0'
    for (auto i = 0; i < bufsize * 8 - 4; i++) {
        mock("softspi").expectOneCall("set_mosi").withBoolParameter("data", false);
        mock("softspi").expectOneCall("set_sck").withBoolParameter("state", true);
        mock("softspi").expectOneCall("set_sck").withBoolParameter("state", false);
    }
    mock("softspi").ignoreOtherCalls();
    softspi_send(&spi, txbuf, rxbuf, bufsize);
}

TEST(SoftSpiTestGroup, CanReceiveSpi)
{
    // The first received byte will be 0xf0, and all the rest will be zero
    for (auto i = 0; i < 4; i++) {
        mock("softspi").expectOneCall("set_sck").withBoolParameter("state", true);
        mock("softspi").expectOneCall("get_miso").andReturnValue(1);
        mock("softspi").expectOneCall("set_sck").withBoolParameter("state", false);
    }
    for (auto i = 0; i < bufsize * 8 - 4; i++) {
        mock("softspi").expectOneCall("set_sck").withBoolParameter("state", true);
        mock("softspi").expectOneCall("get_miso").andReturnValue(0);
        mock("softspi").expectOneCall("set_sck").withBoolParameter("state", false);
    }
    mock("softspi").ignoreOtherCalls();
    softspi_send(&spi, txbuf, rxbuf, bufsize);

    CHECK_EQUAL_TEXT(0xf0, rxbuf[0], "Invalid RX data");

    for (auto i = 1; i < 8; i++) {
        CHECK_EQUAL_TEXT(0x0, rxbuf[i], "Invalid RX data");
    }
}
