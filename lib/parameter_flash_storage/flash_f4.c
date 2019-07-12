#include <stdint.h>
#include <stddef.h>
#include <parameter_flash_storage/flash.h>

/* Flash registers. Copied here to avoid dependencies on either libopencm3 or
 * ChibiOS. */
#define MMIO32(addr) (*(volatile uint32_t*)(addr))
#define FLASH_MEM_INTERFACE_BASE 0x40023C00
#define FLASH_ACR MMIO32(FLASH_MEM_INTERFACE_BASE + 0x00)
#define FLASH_KEYR MMIO32(FLASH_MEM_INTERFACE_BASE + 0x04)
#define FLASH_OPTKEYR MMIO32(FLASH_MEM_INTERFACE_BASE + 0x08)
#define FLASH_SR MMIO32(FLASH_MEM_INTERFACE_BASE + 0x0C)
#define FLASH_CR MMIO32(FLASH_MEM_INTERFACE_BASE + 0x10)

#define FLASH_KEY1 0x45670123
#define FLASH_KEY2 0xCDEF89AB
#define FLASH_CR_SNB_POS 3
#define FLASH_CR_LOCK (1 << 31)
#define FLASH_CR_PSIZE ((uint32_t)0x03 << 8)
#define FLASH_CR_PG (1 << 0)
#define FLASH_CR_SNB ((uint32_t)0x000000F8)
#define FLASH_CR_SER ((uint32_t)0x00000002)
#define FLASH_CR_STRT ((uint32_t)0x00010000)

#define FLASH_SR_BSY (1 << 16)

static void flash_sector_erase_number(uint8_t sector);
static uint8_t flash_addr_to_sector(void* p);

static uint8_t flash_addr_to_sector(void* p)
{
    uint32_t addr = (uint32_t)p;
    uint8_t sector;
    uint32_t offset = addr & 0xFFFFFF;
    if (offset < 0x10000) {
        sector = offset / 0x4000; // 16K sectors, 0x08000000 to 0x0800FFFF
    } else if (offset < 0x20000) {
        sector = 3 + offset / 0x10000; // 64K sector, 0x08010000 to 0x0801FFFF
    } else {
        sector = 4 + offset / 0x20000; // 128K sectors, 0x08010000 to 0x080FFFFF
    }
    if (addr >= 0x08100000) {
        // Bank 2, same layout, starting at 0x08100000
        sector += 12;
    }
    return sector;
}

void flash_lock(void)
{
    FLASH_CR |= FLASH_CR_LOCK;
}

void flash_unlock(void)
{
    // ensure cleared state
    FLASH_CR |= FLASH_CR_LOCK;

    // unlock flash access
    FLASH_KEYR = FLASH_KEY1;
    FLASH_KEYR = FLASH_KEY2;
}

static void flash_set_parallelism_8x(void)
{
    // parallelism 8x, one byte write/erase
    FLASH_CR &= ~FLASH_CR_PSIZE;
}

static void flash_wait_while_busy(void)
{
    while ((FLASH_SR & FLASH_SR_BSY) != 0) {
    }
}

static void flash_enable_programming(void)
{
    FLASH_CR |= FLASH_CR_PG;
}

static void flash_disable_programming(void)
{
    FLASH_CR &= ~FLASH_CR_PG;
}

void flash_write(void* addr, const void* data, size_t len)
{
    flash_wait_while_busy();

    flash_enable_programming();

    /* Note:
     * This is a possible point of optimization of energy/time consumption
     * Different parallelism size of erase and write operations can be chosen
     * depending on the supply voltage. */
    flash_set_parallelism_8x();

    uint8_t* dst = (uint8_t*)addr;
    uint8_t* src = (uint8_t*)data;

    size_t i;
    for (i = 0; i < len; i++) {
        dst[i] = src[i];
        flash_wait_while_busy();
    }

    flash_disable_programming();
}

void flash_sector_erase(void* addr)
{
    uint32_t sector = flash_addr_to_sector(addr);

    flash_sector_erase_number(sector);
}

static void flash_sector_erase_number(uint8_t sector)
{
    flash_set_parallelism_8x();

    flash_wait_while_busy();

    FLASH_CR &= ~FLASH_CR_SNB;
    FLASH_CR |= (sector << FLASH_CR_SNB_POS) & FLASH_CR_SNB;
    FLASH_CR |= FLASH_CR_SER;
    FLASH_CR |= FLASH_CR_STRT;

    flash_wait_while_busy();
}
