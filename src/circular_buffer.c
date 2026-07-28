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

static size_t records_in_sec(CircularBuffer *cb) { return wl_sector_size(cb->wl_handle) / cb->record_size; }

static size_t secs_for_one_header(CircularBuffer *cb) {
    size_t sec_size = wl_sector_size(cb->wl_handle);
    return (sizeof(struct cb_header) + sec_size - 1) / sec_size;
}

static size_t secs_for_header(CircularBuffer *cb) {
    return 2 * secs_for_one_header(cb);
}

static uint32_t sec_num(CircularBuffer *cb) { return wl_size(cb->wl_handle) / wl_sector_size(cb->wl_handle) - secs_for_header(cb); }

static size_t header_offset(CircularBuffer *cb) { return secs_for_header(cb) * wl_sector_size(cb->wl_handle); }

/**
 * @return Capacity of the circular buffer
 */

size_t circular_buffer_get_max_records(CircularBuffer *cb) { return sec_num(cb) * records_in_sec(cb); }

static size_t get_back(CircularBuffer *cb) {
    size_t sec_size = wl_sector_size(cb->wl_handle);
    uint32_t remaining_capacity_in_front_sector = (sec_size - (cb->front % sec_size)) / cb->record_size;
    if (remaining_capacity_in_front_sector > cb->record_num) { return cb->front + (cb->record_num * cb->record_size); }
    else {
        uint32_t remaining_records = cb->record_num - remaining_capacity_in_front_sector;
        uint32_t full_secs = remaining_records / records_in_sec(cb);
        uint32_t front_sec = cb->front / sec_size;
        uint32_t back_sec = (front_sec + full_secs + 1) % sec_num(cb);
        size_t back_offset_in_sec = (remaining_records % records_in_sec(cb)) * cb->record_size;
        return back_sec * sec_size + back_offset_in_sec;
    }
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

static esp_err_t write_header(CircularBuffer *cb) {
    cb_header header;
    memset(&header, 0, sizeof(header));
    header.magic = MAGIC;
    header.front = cb->front;
    header.record_num = cb->record_num;
    header.sequence = ++cb->sequence;
    update_crc(&header);
    size_t addr = (cb->sequence % 2) * secs_for_one_header(cb) * wl_sector_size(cb->wl_handle);
    esp_err_t err = wl_erase_range(cb->wl_handle, addr, secs_for_one_header(cb) * wl_sector_size(cb->wl_handle));
    if (err != ESP_OK) { return err; }
    return wl_write(cb->wl_handle, addr, &header, sizeof(header));
}

/**
 * Initializes circular buffer
 * @param partition_name name of partition in which circular buffer is going to be initialized
 * @param record_size  size of every record in circular buffer
 * @param overwrite whether overwrite feature should be turned on
 * @param recovery_mode use backup header if header was correupted (this may cause in at most one corrupted record)
 * @return ESP_OK if ok
 */
esp_err_t circular_buffer_init(CircularBuffer *cb, char *partition_name, size_t record_size, int overwrite, int recovery_mode) {
    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_ANY,
        partition_name
    );

    if (partition == NULL) { return ESP_ERR_NOT_FOUND; }

    esp_err_t err = wl_mount(partition, &cb->wl_handle);
    if (err != ESP_OK) { return err; }

    size_t sec_size = wl_sector_size(cb->wl_handle);

    if (record_size > sec_size) { return ESP_ERR_INVALID_SIZE; }

    cb->record_size = record_size;
    cb->overwrite = overwrite;

    cb_header header1;
    err = wl_read(cb->wl_handle, 0, &header1, sizeof(cb_header));
    if (err != ESP_OK) { return err; }

    cb_header header2;
    err = wl_read(cb->wl_handle, secs_for_one_header(cb) * sec_size, &header2, sizeof(cb_header));
    if (err != ESP_OK) { return err; }

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
            void* next = malloc(record_size);
            wl_read(cb->wl_handle, header_offset(cb) + back, next, record_size);
            if (!is_all_ff(next, record_size)) {
                ++cb->record_num;
                write_header(cb);
            }
            free(next);
        }
    } else {
        cb->front = 0;
        cb->record_num = 0;
        cb->sequence = -1;
        write_header(cb);
    }

    return ESP_OK;
}

/**
 * Pushes data to the back of circular buffer
 * @param src source of data
 * @return ESP_OK if ok
 */
esp_err_t circular_buffer_push_back(CircularBuffer *cb, void* src) {
    size_t sec_size = wl_sector_size(cb->wl_handle);
    size_t back;
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
            else { return ESP_ERR_NO_MEM; }
        }
        size_t back_offset_in_sec = (remaining_records % records_in_sec(cb)) * cb->record_size;
        back = back_sec * sec_size + back_offset_in_sec;
    }
    if (back % sec_size == 0) { wl_erase_range(cb->wl_handle, back + header_offset(cb), sec_size); }
    esp_err_t err = wl_write(cb->wl_handle, back + header_offset(cb), src, cb->record_size);
    if (err != ESP_OK) { return err; }
    cb->record_num++;
    return write_header(cb);
}

/**
 * Retrieves data from the front of the circular buffer
 * @param dest destination of data
 * @return ESP_OK if ok
 */
esp_err_t circular_buffer_peek_front(CircularBuffer *cb, void* dest) {
    if (cb->record_num == 0) { return ESP_ERR_NOT_FOUND; }
    return wl_read(cb->wl_handle, cb->front + header_offset(cb), dest, cb->record_size);
}

/**
 * Retrieves data from the front of the circular buffer and deletes it
 * @param dest destination of data
 * @return ESP_OK if ok
 */
esp_err_t circular_buffer_pop_front(CircularBuffer *cb, void* dest) {
    esp_err_t err = circular_buffer_peek_front(cb, dest);
    if (err != ESP_OK) { return err; }
    return circular_buffer_delete_front(cb);
}

/**
 * @return Number of records currently in the circular buffer
 */
uint32_t circular_buffer_get_record_num(CircularBuffer *cb) { return cb->record_num; }

/**
 * Deletes one record from the front of the circular buffer
 * @return ESP_OK if ok
 */
esp_err_t circular_buffer_delete_front(CircularBuffer *cb) {
    if (cb->record_num == 0) { return ESP_ERR_NOT_FOUND; }
    size_t sec_size = wl_sector_size(cb->wl_handle);
    if (sec_size - (cb->front % sec_size) >= 2 * cb->record_size) { cb->front += cb->record_size; }
    else { cb->front = ((cb->front / sec_size) + 1) % sec_num(cb) * sec_size; }
    cb->record_num--;
    return write_header(cb);
}
