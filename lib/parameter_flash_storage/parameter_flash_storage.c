#include <string.h>
#include "parameter_flash_storage.h"
#include "parameter_flash_storage_private.h"
#include "flash.h"
#include <parameter/parameter_msgpack.h>
#include <cmp/cmp.h>
#include <cmp_mem_access/cmp_mem_access.h>
#include <crc/crc32.h>

/* We cannot use a CRC start value of 0 because CRC(0, 0xffffffff) = 0xffffffff
 * which makes empty flash pages valid. */
#define CRC_INITIAL_VALUE 0xdeadbeef

static size_t cmp_flash_writer(struct cmp_ctx_s* ctx, const void* data, size_t len)
{
    cmp_mem_access_t* mem = (cmp_mem_access_t*)ctx->buf;
    if (mem->index + len <= mem->size) {
        flash_write(&mem->buf[mem->index], data, len);
        mem->index += len;
        return len;
    } else {
        return 0;
    }
}

void parameter_flash_storage_erase(void* dst)
{
    flash_unlock();
    flash_sector_erase(dst);
    flash_lock();
}

static void err_mark_false(void* arg, const char* id, const char* err)
{
    (void)id;
    (void)err;

    bool* b = (bool*)arg;
    *b = false;
}

void parameter_flash_storage_save(void* dst, size_t dst_len, parameter_namespace_t* ns)
{
    cmp_ctx_t cmp;
    cmp_mem_access_t mem;
    size_t len;
    bool success = true;

    void* orig_dst = dst;

    flash_unlock();

    /* If there is no valid block, erase flash just to start from a pristine
     * state. */
    if (parameter_flash_storage_block_find_last_used(dst) == NULL) {
        flash_sector_erase(dst);
    }

    /* Find first available flash block. */
    dst = parameter_flash_storage_block_find_first_free(dst);

    len = dst_len - (dst - orig_dst);

    /* If the destination is too small to fit even the header, erase the block. */
    if (len <= PARAMETER_FLASH_STORAGE_HEADER_SIZE) {
        flash_sector_erase(orig_dst);
        dst = orig_dst;
        len = dst_len;
    }

    cmp_mem_access_init(&cmp,
                        &mem,
                        dst + PARAMETER_FLASH_STORAGE_HEADER_SIZE,
                        len - PARAMETER_FLASH_STORAGE_HEADER_SIZE);

    /* Replace the RAM writer with the special writer for flash. */
    cmp.write = cmp_flash_writer;

    /* Tries to write the config. If there is an error the callback will set
     * success to false. */
    parameter_msgpack_write_cmp(ns, &cmp, err_mark_false, &success);

    if (success == false) {
        flash_sector_erase(orig_dst);
        flash_lock();
        return parameter_flash_storage_save(orig_dst, dst_len, ns);
    }

    len = cmp_mem_access_get_pos(&mem);

    // On STM32F3s we have to make sure we wrote a multiple of 16 bits because
    // the flash controller does not support single byte writes.
    if (len % 2 == 1) {
        uint8_t* end = dst + len + PARAMETER_FLASH_STORAGE_HEADER_SIZE + 1;
        uint8_t data = 0x00;
        flash_write(end, &data, 1);
        len++;
    }

    parameter_flash_storage_write_block_header(dst, len);

    flash_lock();
}

bool parameter_flash_storage_load(parameter_namespace_t* ns, void* src)
{
    int res;
    uint32_t src_len;

    src = parameter_flash_storage_block_find_last_used(src);

    /* If no valid block was found signal an error. */
    if (src == NULL) {
        return false;
    }

    src_len = parameter_flash_storage_block_get_length(src);

    bool successful = true;
    res = parameter_msgpack_read(ns, src + PARAMETER_FLASH_STORAGE_HEADER_SIZE, src_len,
                                 err_mark_false, &successful);

    if (res != 0) {
        return false;
    }

    return successful;
}

bool parameter_flash_storage_block_is_valid(void* p)
{
    uint8_t* block = (uint8_t*)p;
    uint32_t crc, length;
    size_t offset = 0;

    /* Extract length checksum. */
    memcpy(&crc, &block[offset], sizeof(crc));
    offset += sizeof(crc);

    /* Extract length. */
    memcpy(&length, &block[offset], sizeof(length));
    offset += sizeof(length);

    /* Check that the length is valid. */
    if (crc != crc32(CRC_INITIAL_VALUE, &length, sizeof(length))) {
        return false;
    }

    /* Extract data checksum. */
    memcpy(&crc, &block[offset], sizeof(crc));
    offset += sizeof(crc);

    /* Check that the data checksum is valid. */
    if (crc != crc32(CRC_INITIAL_VALUE, &block[offset], length)) {
        return false;
    }

    return true;
}

void parameter_flash_storage_write_block_header(void* dst, uint32_t len)
{
    uint32_t crc;
    size_t offset = 0;

    /* First write length checksum. */
    crc = crc32(CRC_INITIAL_VALUE, &len, sizeof(uint32_t));
    flash_write(dst + offset, &crc, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    /* Then write the length itself. */
    flash_write(dst + offset, &len, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    /* Then write the data checksum. */
    crc = crc32(CRC_INITIAL_VALUE, dst + PARAMETER_FLASH_STORAGE_HEADER_SIZE, len);
    flash_write(dst + offset, &crc, sizeof(uint32_t));
}

uint32_t parameter_flash_storage_block_get_length(void* block)
{
    uint32_t header[2];
    memcpy(header, block, sizeof header);

    return header[1];
}

void* parameter_flash_storage_block_find_last_used(void* p)
{
    uint8_t* block = (uint8_t*)p;
    uint8_t* last = NULL;

    while (parameter_flash_storage_block_is_valid(block)) {
        last = block;
        block += parameter_flash_storage_block_get_length(block);
        block += PARAMETER_FLASH_STORAGE_HEADER_SIZE;
    }

    return last;
}

void* parameter_flash_storage_block_find_first_free(void* p)
{
    uint8_t* block = (uint8_t*)p;

    while (parameter_flash_storage_block_is_valid(block)) {
        block += parameter_flash_storage_block_get_length(block);
        block += PARAMETER_FLASH_STORAGE_HEADER_SIZE;
    }

    return block;
}
