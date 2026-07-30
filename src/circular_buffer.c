#include "circular_buffer.h"

#include "esp_crc.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define MAGIC 0x5B15B1

typedef struct cb_header {
    uint32_t magic;
    size_t front;
    uint32_t record_num;
    uint32_t sequence;
    uint32_t crc;
} cb_header;

struct cb_header_crc_data {
    uint32_t magic;
    size_t front;
    uint32_t record_num;
    uint32_t sequence;
};

static size_t records_in_sec(CircularBuffer *cb) { return cb->sector_size / cb->record_size; }

static size_t secs_for_one_header(CircularBuffer *cb) {
    size_t sec_size = cb->sector_size;
    return (sizeof(struct cb_header) + sec_size - 1) / sec_size;
}

static size_t secs_for_header(CircularBuffer *cb) {
    return 2 * secs_for_one_header(cb);
}

static uint32_t sec_num(CircularBuffer *cb) { return cb->total_size / cb->sector_size - secs_for_header(cb); }

static size_t header_offset(CircularBuffer *cb) { return secs_for_header(cb) * cb->sector_size; }

/**
 * @return Capacity of the circular buffer
 */

size_t circular_buffer_get_max_records(CircularBuffer *cb) { return sec_num(cb) * records_in_sec(cb); }

static size_t get_record_addr(CircularBuffer *cb, size_t index) {
    size_t sec_size = cb->sector_size;
    uint32_t remaining_capacity_in_front_sector = (sec_size - (cb->front % sec_size)) / cb->record_size;
    if (remaining_capacity_in_front_sector > index) { return cb->front + (index * cb->record_size); }
    else {
        uint32_t remaining_records = index - remaining_capacity_in_front_sector;
        uint32_t full_secs = remaining_records / records_in_sec(cb);
        uint32_t front_sec = cb->front / sec_size;
        uint32_t record_sec = (front_sec + full_secs + 1) % sec_num(cb);
        size_t record_offset_in_sec = (remaining_records % records_in_sec(cb)) * cb->record_size;
        return record_sec * sec_size + record_offset_in_sec;
    }
}

static size_t get_back(CircularBuffer *cb) {
    return get_record_addr(cb, cb->record_num);
}

static bool is_all_ff(const void *ptr, size_t len) {
    const uint8_t *p = (const uint8_t *)ptr;
    size_t i;
    for (i = 0; i < len; i++) {
        if (p[i] != 0xFF) return false;
    }
    return true;
}

static uint32_t header_crc(const cb_header *hdr) {
    struct cb_header_crc_data crc_data;
    memset(&crc_data, 0, sizeof(crc_data));
    crc_data.magic = hdr->magic;
    crc_data.front = hdr->front;
    crc_data.record_num = hdr->record_num;
    crc_data.sequence = hdr->sequence;
    return esp_crc32_le(0, (const uint8_t*)&crc_data, sizeof(crc_data));
}

static void update_crc(cb_header *hdr) {
    hdr->crc = header_crc(hdr);
}

static bool check_header(const cb_header *hdr) {
    return header_crc(hdr) == hdr->crc && hdr->magic == MAGIC;
}

static circular_buffer_err_t write_header(CircularBuffer *cb) {
    cb_header header;
    memset(&header, 0, sizeof(header));
    header.magic = MAGIC;
    header.front = cb->front;
    header.record_num = cb->record_num;
    header.sequence = ++cb->sequence;
    update_crc(&header);
    size_t addr = (cb->sequence % 2) * secs_for_one_header(cb) * cb->sector_size;
    circular_buffer_err_t err = cb->erase_range(cb->storage_ctx, addr, secs_for_one_header(cb) * cb->sector_size);
    if (err != CIRCULAR_BUFFER_OK) { return err; }
    return cb->write(cb->storage_ctx, addr, &header, sizeof(header));
}

/**
 * Initializes circular buffer
 * @param sector_size erase sector size of the storage backend
 * @param read reads bytes from the storage backend
 * @param erase_range erases a sector-aligned range in the storage backend
 * @param write writes bytes to the storage backend
 * @param total_size total size of the storage backend
 * @param storage_ctx user-owned context passed to read, erase_range, and write
 * @param record_size  size of every record in circular buffer
 * @param overwrite whether overwrite feature should be turned on
 * @param recovery_mode use backup header if header was correupted (this may cause in at most one corrupted record)
 * @return CIRCULAR_BUFFER_OK if ok
 */
circular_buffer_err_t circular_buffer_init(CircularBuffer *cb,
                               size_t sector_size,
                               circular_buffer_read_fn read,
                               circular_buffer_erase_range_fn erase_range,
                               circular_buffer_write_fn write,
                               size_t total_size,
                               void *storage_ctx,
                               size_t record_size,
                               int overwrite,
                               int recovery_mode) {
    circular_buffer_err_t err = CIRCULAR_BUFFER_OK;
    size_t sec_size = sector_size;

    if (cb == NULL || read == NULL || erase_range == NULL || write == NULL) { return CIRCULAR_BUFFER_ERR_INVALID_ARG; }
    if (sector_size == 0 || total_size == 0 || record_size == 0) { return CIRCULAR_BUFFER_ERR_INVALID_SIZE; }
    if (total_size % sector_size != 0) { return CIRCULAR_BUFFER_ERR_INVALID_SIZE; }
    if (record_size > sector_size) { return CIRCULAR_BUFFER_ERR_INVALID_SIZE; }

    cb->sector_size = sector_size;
    cb->total_size = total_size;
    cb->storage_ctx = storage_ctx;
    cb->read = read;
    cb->erase_range = erase_range;
    cb->write = write;
    cb->record_size = record_size;
    cb->overwrite = overwrite;

    if (sec_num(cb) == 0 || records_in_sec(cb) == 0) { return CIRCULAR_BUFFER_ERR_INVALID_SIZE; }

    cb_header header1;
    err = cb->read(cb->storage_ctx, 0, &header1, sizeof(cb_header));
    if (err != CIRCULAR_BUFFER_OK) { return err; }

    cb_header header2;
    err = cb->read(cb->storage_ctx, secs_for_one_header(cb) * sec_size, &header2, sizeof(cb_header));
    if (err != CIRCULAR_BUFFER_OK) { return err; }

    bool header1_valid = check_header(&header1);
    bool header2_valid = check_header(&header2);

    void *next = NULL;

    if (header1_valid && header2_valid) {
        if (header1.sequence > header2.sequence || (header1.sequence == 0 && header2.sequence == UINT32_MAX)) {
            cb->front = header1.front;
            cb->record_num = header1.record_num;
            cb->sequence = header1.sequence;
        } else {
            cb->front = header2.front;
            cb->record_num = header2.record_num;
            cb->sequence = header2.sequence;
        }
    } else if (recovery_mode && (header1_valid ^ header2_valid)) {
        if (header1_valid) {
            cb->front = header1.front;
            cb->record_num = header1.record_num;
            cb->sequence = header1.sequence;
        } else {
            cb->front = header2.front;
            cb->record_num = header2.record_num;
            cb->sequence = header2.sequence;
        }
        size_t back = get_back(cb);
        if (back % sec_size != 0) {
            next = malloc(record_size);
            if (next == NULL) { return CIRCULAR_BUFFER_ERR_NO_MEM; }
            err = cb->read(cb->storage_ctx, header_offset(cb) + back, next, record_size);
            if (err != CIRCULAR_BUFFER_OK) { goto cleanup; }
            if (!is_all_ff(next, record_size)) {
                ++cb->record_num;
                err = write_header(cb);
                if (err != CIRCULAR_BUFFER_OK) { goto cleanup; }
            }
        }
    } else {
        cb->front = 0;
        cb->record_num = 0;
        cb->sequence = -1;
        err = write_header(cb);
        if (err != CIRCULAR_BUFFER_OK) { return err; }
    }

cleanup:
    free(next);
    return err;
}

/**
 * Pushes data to the back of circular buffer
 * @param src source of data
 * @return CIRCULAR_BUFFER_OK if ok
 */
circular_buffer_err_t circular_buffer_push_back(CircularBuffer *cb, void* src) {
    circular_buffer_err_t err;
    size_t sec_size;
    size_t back;

    if (cb == NULL || src == NULL) { return CIRCULAR_BUFFER_ERR_INVALID_ARG; }
    sec_size = cb->sector_size;

    uint32_t remaining_capacity_in_front_sector = (sec_size - (cb->front % sec_size)) / cb->record_size;
    if (remaining_capacity_in_front_sector > cb->record_num) { back = cb->front + (cb->record_num * cb->record_size); }
    else {
        uint32_t remaining_records = cb->record_num - remaining_capacity_in_front_sector;
        uint32_t full_secs = remaining_records / records_in_sec(cb);
        uint32_t front_sec = cb->front / sec_size;
        uint32_t back_sec = (front_sec + full_secs + 1) % sec_num(cb);
        if (back_sec == front_sec) {
            if (cb->overwrite) {
                cb->front = ((front_sec + 1) % sec_num(cb)) * sec_size;
                cb->record_num -= remaining_capacity_in_front_sector;
            }
            else { return CIRCULAR_BUFFER_ERR_NO_MEM; }
        }
        size_t back_offset_in_sec = (remaining_records % records_in_sec(cb)) * cb->record_size;
        back = back_sec * sec_size + back_offset_in_sec;
    }
    if (back % sec_size == 0) {
        err = cb->erase_range(cb->storage_ctx, back + header_offset(cb), sec_size);
        if (err != CIRCULAR_BUFFER_OK) { return err; }
    }
    err = cb->write(cb->storage_ctx, back + header_offset(cb), src, cb->record_size);
    if (err != CIRCULAR_BUFFER_OK) { return err; }
    cb->record_num++;
    return write_header(cb);
}

/**
 * Retrieves data from the front of the circular buffer
 * @param dest destination of data
 * @return CIRCULAR_BUFFER_OK if ok
 */
circular_buffer_err_t circular_buffer_peek_front(CircularBuffer *cb, void* dest) {
    return circular_buffer_peek_at(cb, 0, dest);
}

/**
 * Retrieves data from the circular buffer without deleting it
 * @param index record index from the front
 * @param dest destination of data
 * @return CIRCULAR_BUFFER_OK if ok
 */
circular_buffer_err_t circular_buffer_peek_at(CircularBuffer *cb, size_t index, void* dest) {
    size_t addr;

    if (cb == NULL || dest == NULL) { return CIRCULAR_BUFFER_ERR_INVALID_ARG; }
    if (index >= cb->record_num) { return CIRCULAR_BUFFER_ERR_NOT_FOUND; }
    addr = get_record_addr(cb, index);
    return cb->read(cb->storage_ctx, addr + header_offset(cb), dest, cb->record_size);
}

/**
 * Retrieves data from the front of the circular buffer and deletes it
 * @param dest destination of data
 * @return CIRCULAR_BUFFER_OK if ok
 */
circular_buffer_err_t circular_buffer_pop_front(CircularBuffer *cb, void* dest) {
    circular_buffer_err_t err = circular_buffer_peek_front(cb, dest);
    if (err != CIRCULAR_BUFFER_OK) { return err; }
    return circular_buffer_delete_front(cb);
}

/**
 * @return Number of records currently in the circular buffer
 */
uint32_t circular_buffer_get_record_num(CircularBuffer *cb) { return cb->record_num; }

/**
 * Deletes one record from the front of the circular buffer
 * @return CIRCULAR_BUFFER_OK if ok
 */
circular_buffer_err_t circular_buffer_delete_front(CircularBuffer *cb) {
    if (cb == NULL) { return CIRCULAR_BUFFER_ERR_INVALID_ARG; }
    if (cb->record_num == 0) { return CIRCULAR_BUFFER_ERR_NOT_FOUND; }
    size_t sec_size = cb->sector_size;
    if (sec_size - (cb->front % sec_size) >= 2 * cb->record_size) { cb->front += cb->record_size; }
    else { cb->front = ((cb->front / sec_size) + 1) % sec_num(cb) * sec_size; }
    cb->record_num--;
    return write_header(cb);
}
