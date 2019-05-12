#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "parameter_flash_storage/flash.h"
#include <cstring>

extern "C" {

void flash_lock(void)
{
    mock("flash").actualCall("lock");
}

void flash_unlock(void)
{
    mock("flash").actualCall("unlock");
}

void flash_sector_erase(void* p)
{
    mock("flash").actualCall("erase").withParameter("sector", p);

    /* At least invalid any checksum in that block. */
    memset(p, 0, 1);
}

void flash_write(void* addr, const void* data, size_t len)
{
    memcpy(addr, data, len);
    mock("flash").actualCall("write");
}

uint8_t flash_addr_to_sector(void* addr)
{
    (void)addr;
    return mock("flash").actualCall("addr_to_sector").returnIntValueOrDefault(0);
}

void flash_sector_erase_number(uint8_t number)
{
    mock("flash").actualCall("erase").withParameter("sector_number", number);
}
}

TEST_GROUP (FlashMockTestCase) {
};

TEST(FlashMockTestCase, TestFlashLock)
{
    mock("flash").expectOneCall("lock");
    flash_lock();
}

TEST(FlashMockTestCase, TestFlashUnlock)
{
    mock("flash").expectOneCall("unlock");
    flash_unlock();
}

TEST(FlashMockTestCase, TestFlashErase)
{
    int a;
    mock("flash").expectOneCall("erase").withParameter("sector", &a);
    flash_sector_erase(&a);
}

TEST(FlashMockTestCase, TestFlashWrite)
{
    int a;
    int b = 42;

    mock("flash").expectOneCall("write");

    flash_write(&a, &b, sizeof a);

    CHECK_EQUAL(b, a);
}
