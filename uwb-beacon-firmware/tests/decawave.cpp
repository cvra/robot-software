#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"

extern "C" int readfromspi(uint16 headerLength, const uint8* headerBuffer, uint32 readlength, uint8* readBuffer)
{
    (void)readlength;
    mock("spi").actualCall("read").withMemoryBufferParameter("header", (const unsigned char*)headerBuffer, headerLength).withOutputParameter("buf", (unsigned char*)readBuffer);

    return DWT_SUCCESS;
}

extern "C" int writetospi(uint16 headerLength, const uint8* headerBuffer, uint32 bodyLength, const uint8* bodyBuffer)
{
    return DWT_SUCCESS;
}

extern "C" void deca_sleep(unsigned int time_ms)
{
}

extern "C" decaIrqStatus_t decamutexon(void)
{
    return 0;
}

extern "C" void decamutexoff(decaIrqStatus_t s)
{
}

TEST_GROUP (Decawave) {
};

/* little sanity check to make sure everything is working as expected. */
TEST(Decawave, ReadId)
{
    uint8_t header[] = {0x00};
    uint8_t reply[] = {0x30, 0x01, 0xca, 0xde};
    mock("spi").expectOneCall("read").withMemoryBufferParameter("header", header, sizeof(header)).withOutputParameterReturning("buf", reply, sizeof(reply));

    auto id = dwt_readdevid();

    CHECK_EQUAL(DWT_DEVICE_ID, id);
}
