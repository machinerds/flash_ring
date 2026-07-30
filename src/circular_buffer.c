#include "circular_buffer.h"

#include "esp_crc.h"

#include <stdbool.h>
#include <string.h>

#define MAGIC 0x5B15B1
#define RECORD_COMMIT_MAGIC 0x0B
#define RECORD_COMMIT_MASK 0x0F
#define RECORD_FLAGS_MASK 0xF0
#define RECORD_FLAG_COUNT 4
#define INVALID_RECORD_INDEX ((size_t)-1)

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

static size_t record_slot_size(CircularBuffer *cb) { return cb->record_size + 1; }

static size_t records_in_sec(CircularBuffer *cb) { return cb->sector_size / record_slot_size(cb); }

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
    size_t slot_size = record_slot_size(cb);
    uint32_t remaining_capacity_in_front_sector = (sec_size - (cb->front % sec_size)) / slot_size;
    if (remaining_capacity_in_front_sector > index) { return cb->front + (index * slot_size); }
    else {
        uint32_t remaining_records = index - remaining_capacity_in_front_sector;
        uint32_t full_secs = remaining_records / records_in_sec(cb);
        uint32_t front_sec = cb->front / sec_size;
        uint32_t record_sec = (front_sec + full_secs + 1) % sec_num(cb);
        size_t record_offset_in_sec = (remaining_records % records_in_sec(cb)) * slot_size;
        return record_sec * sec_size + record_offset_in_sec;
    }
}

static size_t get_back(CircularBuffer *cb) {
    return get_record_addr(cb, cb->record_num);
}

static bool is_record_committed(uint8_t commit) {
    return (commit & RECORD_COMMIT_MASK) == RECORD_COMMIT_MAGIC;
}

static uint8_t flag_mask(size_t flag) {
    return (uint8_t)(1u << (flag + 4));
}

static bool is_record_flag_set(uint8_t commit, size_t flag) {
    return (commit & flag_mask(flag)) != 0;
}

static circular_buffer_err_t read_record_commit_raw(CircularBuffer *cb, size_t index, uint8_t *commit) {
    size_t addr = get_record_addr(cb, index);
    return cb->read(cb->storage_ctx, addr + header_offset(cb) + cb->record_size, commit, sizeof(*commit));
}

static circular_buffer_err_t read_record_commit(CircularBuffer *cb, size_t index, uint8_t *commit) {
    circular_buffer_err_t err = read_record_commit_raw(cb, index, commit);
    if (err != CIRCULAR_BUFFER_OK) { return err; }
    if (!is_record_committed(*commit)) { return CIRCULAR_BUFFER_ERR_INVALID_RECORD; }
    return CIRCULAR_BUFFER_OK;
}

static circular_buffer_err_t write_record_commit(CircularBuffer *cb, size_t index, uint8_t commit) {
    size_t addr = get_record_addr(cb, index);
    return cb->write(cb->storage_ctx, addr + header_offset(cb) + cb->record_size, &commit, sizeof(commit));
}

static void clear_first_flagged_records(CircularBuffer *cb) {
    size_t i;
    for (i = 0; i < RECORD_FLAG_COUNT; ++i) {
        cb->first_flagged_record[i] = INVALID_RECORD_INDEX;
    }
}

static circular_buffer_err_t find_first_flagged_record(CircularBuffer *cb, size_t flag, size_t start, size_t *record_index) {
    size_t index;

    for (index = start; index < cb->record_num; ++index) {
        uint8_t commit;
        circular_buffer_err_t err = read_record_commit_raw(cb, index, &commit);
        if (err != CIRCULAR_BUFFER_OK) { return err; }
        if (is_record_committed(commit) && is_record_flag_set(commit, flag)) {
            *record_index = index;
            return CIRCULAR_BUFFER_OK;
        }
    }

    *record_index = INVALID_RECORD_INDEX;
    return CIRCULAR_BUFFER_OK;
}

static circular_buffer_err_t init_first_flagged_records(CircularBuffer *cb) {
    size_t index;
    size_t found = 0;

    clear_first_flagged_records(cb);
    for (index = 0; index < cb->record_num && found < RECORD_FLAG_COUNT; ++index) {
        uint8_t commit;
        size_t flag;
        circular_buffer_err_t err = read_record_commit_raw(cb, index, &commit);
        if (err != CIRCULAR_BUFFER_OK) { return err; }
        if (!is_record_committed(commit)) { continue; }
        for (flag = 0; flag < RECORD_FLAG_COUNT; ++flag) {
            if (cb->first_flagged_record[flag] == INVALID_RECORD_INDEX && is_record_flag_set(commit, flag)) {
                cb->first_flagged_record[flag] = index;
                ++found;
            }
        }
    }
    return CIRCULAR_BUFFER_OK;
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
    if (record_size >= sector_size) { return CIRCULAR_BUFFER_ERR_INVALID_SIZE; }

    cb->sector_size = sector_size;
    cb->total_size = total_size;
    cb->storage_ctx = storage_ctx;
    cb->read = read;
    cb->erase_range = erase_range;
    cb->write = write;
    cb->record_size = record_size;
    cb->overwrite = overwrite;
    clear_first_flagged_records(cb);

    if (sec_num(cb) == 0 || records_in_sec(cb) == 0) { return CIRCULAR_BUFFER_ERR_INVALID_SIZE; }

    cb_header header1;
    err = cb->read(cb->storage_ctx, 0, &header1, sizeof(cb_header));
    if (err != CIRCULAR_BUFFER_OK) { return err; }

    cb_header header2;
    err = cb->read(cb->storage_ctx, secs_for_one_header(cb) * sec_size, &header2, sizeof(cb_header));
    if (err != CIRCULAR_BUFFER_OK) { return err; }

    bool header1_valid = check_header(&header1);
    bool header2_valid = check_header(&header2);

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
            uint8_t commit;
            err = cb->read(cb->storage_ctx, header_offset(cb) + back + cb->record_size, &commit, sizeof(commit));
            if (err != CIRCULAR_BUFFER_OK) { return err; }
            if (is_record_committed(commit)) {
                ++cb->record_num;
                err = write_header(cb);
                if (err != CIRCULAR_BUFFER_OK) { return err; }
            }
        }
    } else {
        cb->front = 0;
        cb->record_num = 0;
        cb->sequence = -1;
        err = write_header(cb);
        if (err != CIRCULAR_BUFFER_OK) { return err; }
    }

    if (cb->record_num != 0) { err = init_first_flagged_records(cb); }
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
    size_t slot_size;
    size_t back;
    size_t dropped_records = 0;
    uint8_t commit = RECORD_FLAGS_MASK | RECORD_COMMIT_MAGIC;

    if (cb == NULL || src == NULL) { return CIRCULAR_BUFFER_ERR_INVALID_ARG; }
    sec_size = cb->sector_size;
    slot_size = record_slot_size(cb);

    uint32_t remaining_capacity_in_front_sector = (sec_size - (cb->front % sec_size)) / slot_size;
    if (remaining_capacity_in_front_sector > cb->record_num) { back = cb->front + (cb->record_num * slot_size); }
    else {
        uint32_t remaining_records = cb->record_num - remaining_capacity_in_front_sector;
        uint32_t full_secs = remaining_records / records_in_sec(cb);
        uint32_t front_sec = cb->front / sec_size;
        uint32_t back_sec = (front_sec + full_secs + 1) % sec_num(cb);
        if (back_sec == front_sec) {
            if (cb->overwrite) {
                cb->front = ((front_sec + 1) % sec_num(cb)) * sec_size;
                cb->record_num -= remaining_capacity_in_front_sector;
                dropped_records = remaining_capacity_in_front_sector;
            }
            else { return CIRCULAR_BUFFER_ERR_NO_MEM; }
        }
        size_t back_offset_in_sec = (remaining_records % records_in_sec(cb)) * slot_size;
        back = back_sec * sec_size + back_offset_in_sec;
    }
    if (back % sec_size == 0) {
        err = cb->erase_range(cb->storage_ctx, back + header_offset(cb), sec_size);
        if (err != CIRCULAR_BUFFER_OK) { return err; }
    }
    err = cb->write(cb->storage_ctx, back + header_offset(cb), src, cb->record_size);
    if (err != CIRCULAR_BUFFER_OK) { return err; }
    err = cb->write(cb->storage_ctx, back + header_offset(cb) + cb->record_size, &commit, sizeof(commit));
    if (err != CIRCULAR_BUFFER_OK) { return err; }
    cb->record_num++;
    err = write_header(cb);
    if (err != CIRCULAR_BUFFER_OK) { return err; }
    {
        size_t flag;
        size_t new_index = cb->record_num - 1;
        for (flag = 0; flag < RECORD_FLAG_COUNT; ++flag) {
            if (dropped_records != 0 && cb->first_flagged_record[flag] != INVALID_RECORD_INDEX) {
                if (cb->first_flagged_record[flag] < dropped_records) {
                    err = find_first_flagged_record(cb, flag, 0, &cb->first_flagged_record[flag]);
                    if (err != CIRCULAR_BUFFER_OK) { return err; }
                    continue;
                }
                cb->first_flagged_record[flag] -= dropped_records;
            }
            if (cb->first_flagged_record[flag] == INVALID_RECORD_INDEX) {
                cb->first_flagged_record[flag] = new_index;
            }
        }
    }
    return CIRCULAR_BUFFER_OK;
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
    uint8_t commit;
    circular_buffer_err_t err;

    if (cb == NULL || dest == NULL) { return CIRCULAR_BUFFER_ERR_INVALID_ARG; }
    if (index >= cb->record_num) { return CIRCULAR_BUFFER_ERR_NOT_FOUND; }
    addr = get_record_addr(cb, index);
    err = cb->read(cb->storage_ctx, addr + header_offset(cb) + cb->record_size, &commit, sizeof(commit));
    if (err != CIRCULAR_BUFFER_OK) { return err; }
    if (!is_record_committed(commit)) { return CIRCULAR_BUFFER_ERR_INVALID_RECORD; }
    return cb->read(cb->storage_ctx, addr + header_offset(cb), dest, cb->record_size);
}

/**
 * Retrieves the indexed record with a flag set without deleting it
 * @param index flagged record index, among records whose flag is true
 * @param flag flag index, 0 to 3
 * @param dest destination of data
 * @param record_index absolute record index from the front
 * @return CIRCULAR_BUFFER_OK if ok
 */
circular_buffer_err_t circular_buffer_peek_flagged(CircularBuffer *cb, size_t index, size_t flag, void* dest, size_t *record_index) {
    size_t matched = 0;
    size_t start;

    if (cb == NULL || dest == NULL || record_index == NULL) { return CIRCULAR_BUFFER_ERR_INVALID_ARG; }
    if (flag >= RECORD_FLAG_COUNT) { return CIRCULAR_BUFFER_ERR_INVALID_ARG; }
    if (cb->first_flagged_record[flag] == INVALID_RECORD_INDEX) { return CIRCULAR_BUFFER_ERR_NOT_FOUND; }

    start = cb->first_flagged_record[flag];
    while (start < cb->record_num) {
        circular_buffer_err_t err = find_first_flagged_record(cb, flag, start, record_index);
        if (err != CIRCULAR_BUFFER_OK) { return err; }
        if (*record_index == INVALID_RECORD_INDEX) { return CIRCULAR_BUFFER_ERR_NOT_FOUND; }
        if (matched == index) {
            return circular_buffer_peek_at(cb, *record_index, dest);
        }
        ++matched;
        start = *record_index + 1;
    }

    return CIRCULAR_BUFFER_ERR_NOT_FOUND;
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
    circular_buffer_err_t err;
    if (cb == NULL) { return CIRCULAR_BUFFER_ERR_INVALID_ARG; }
    if (cb->record_num == 0) { return CIRCULAR_BUFFER_ERR_NOT_FOUND; }
    size_t sec_size = cb->sector_size;
    size_t slot_size = record_slot_size(cb);
    if (sec_size - (cb->front % sec_size) >= 2 * slot_size) { cb->front += slot_size; }
    else { cb->front = ((cb->front / sec_size) + 1) % sec_num(cb) * sec_size; }
    cb->record_num--;
    err = write_header(cb);
    if (err != CIRCULAR_BUFFER_OK) { return err; }
    {
        size_t flag;
        for (flag = 0; flag < RECORD_FLAG_COUNT; ++flag) {
            if (cb->first_flagged_record[flag] == INVALID_RECORD_INDEX) { continue; }
            if (cb->first_flagged_record[flag] == 0) {
                err = find_first_flagged_record(cb, flag, 0, &cb->first_flagged_record[flag]);
                if (err != CIRCULAR_BUFFER_OK) { return err; }
            } else {
                --cb->first_flagged_record[flag];
            }
        }
    }
    return CIRCULAR_BUFFER_OK;
}

circular_buffer_err_t circular_buffer_clear_flag(CircularBuffer *cb, size_t index, size_t flag) {
    uint8_t commit;
    circular_buffer_err_t err;

    if (cb == NULL) { return CIRCULAR_BUFFER_ERR_INVALID_ARG; }
    if (flag >= RECORD_FLAG_COUNT) { return CIRCULAR_BUFFER_ERR_INVALID_ARG; }
    if (index >= cb->record_num) { return CIRCULAR_BUFFER_ERR_NOT_FOUND; }

    err = read_record_commit(cb, index, &commit);
    if (err != CIRCULAR_BUFFER_OK) { return err; }
    commit &= (uint8_t)~flag_mask(flag);
    err = write_record_commit(cb, index, commit);
    if (err != CIRCULAR_BUFFER_OK) { return err; }

    if (cb->first_flagged_record[flag] == index) {
        err = find_first_flagged_record(cb, flag, index + 1, &cb->first_flagged_record[flag]);
        if (err != CIRCULAR_BUFFER_OK) { return err; }
    }
    return CIRCULAR_BUFFER_OK;
}
