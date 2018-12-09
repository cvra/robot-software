#include "hal.h"
#include <CppUTestExt/MockSupport.h>

void chThdSleepMilliseconds(int ms)
{
    mock("ch").actualCall("ThdSleepMilliseconds").withIntParameter("ms", ms);
}

void spiSelect(SPIDriver* p)
{
    mock("spi").actualCall("select").withPointerParameter("drv", p);
}

void spiUnselect(SPIDriver* p)
{
    mock("spi").actualCall("unselect").withPointerParameter("drv", p);
}

void spiSend(SPIDriver* p, size_t n, const void* buf)
{
    mock("spi").actualCall("send").withPointerParameter("drv", p).withMemoryBufferParameter("buf", (const unsigned char*)buf, n);
}

void spiReceive(SPIDriver* p, size_t n, void* buf)
{
    mock("spi").actualCall("receive").withPointerParameter("drv", p).withParameter("n", n).withOutputParameter("buf", (unsigned char*)buf);
}
