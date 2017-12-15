#include <stdint.h>
#include "flash.h"

/* Flash registers. Copied here to avoid dependencies on either libopencm3 or
 * ChibiOS. */
#define MMIO32(addr) (*(volatile uint32_t *)(addr))
#define FLASH_MEM_INTERFACE_BASE 0x40022000
#define FLASH_KEYR               MMIO32(FLASH_MEM_INTERFACE_BASE + 0x04)
#define FLASH_SR                 MMIO32(FLASH_MEM_INTERFACE_BASE + 0x0C)
#define FLASH_CR                 MMIO32(FLASH_MEM_INTERFACE_BASE + 0x10)
#define FLASH_AR                 MMIO32(FLASH_MEM_INTERFACE_BASE + 0x14)

#define FLASH_CR_PG              (1 << 0)
#define FLASH_CR_PER             (1 << 1)
#define FLASH_CR_STRT            (1 << 6)
#define FLASH_CR_LOCK            (1 << 7)

#define FLASH_SR_BSY             (1 << 0)
#define FLASH_SR_EOP             (1 << 5)

#define FLASH_KEYR_KEY1          ((uint32_t)0x45670123)
#define FLASH_KEYR_KEY2          ((uint32_t)0xcdef89ab)

static void flash_wait_for_last_operation(void)
{
    while ((FLASH_SR & FLASH_SR_BSY) == FLASH_SR_BSY) {
    }
}

static void flash_write_half_word(uint16_t *flash, uint16_t half_word)
{
    /* select flash programming */
    FLASH_CR |= FLASH_CR_PG;

    /* perform half-word write */
    *flash = half_word;

    flash_wait_for_last_operation();
}


void flash_lock(void)
{
    FLASH_CR |= FLASH_CR_LOCK;
}

void flash_unlock(void)
{
    /* Clear the unlock sequence state. */
    FLASH_CR |= FLASH_CR_LOCK;

    /* Authorize the FPEC access. */
    FLASH_KEYR = FLASH_KEYR_KEY1;
    FLASH_KEYR = FLASH_KEYR_KEY2;
}

void flash_write(void *page, const void *data, size_t len)
{
    uint8_t *bytes = (uint8_t *) data;
    uint16_t *flash = (uint16_t *) page;
    uint16_t half_word;

    flash_wait_for_last_operation();

    size_t count;
    for (count = len; count > 1; count -= 2) {

        half_word = *bytes++;
        half_word |= (uint16_t)*bytes++ << 8;

        flash_write_half_word(flash++, half_word);
    }

    if (count == 1) {
        half_word = *bytes;
        /* preserve value of adjacent byte */
        half_word |= (uint16_t)(*(uint8_t *)flash) << 8;

        flash_write_half_word(flash, half_word);
    }

    /* reset flags */
    FLASH_CR &= ~FLASH_CR_PG;
    FLASH_SR |= FLASH_SR_EOP;
}

void flash_sector_erase(void *page)
{
    flash_wait_for_last_operation();

    FLASH_CR |= FLASH_CR_PER;
    FLASH_AR = (uint32_t) page;
    FLASH_CR |= FLASH_CR_STRT;

    flash_wait_for_last_operation();

    FLASH_CR &= ~FLASH_CR_PER;
}
