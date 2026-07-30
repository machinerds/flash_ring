#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CIRCULAR_BUFFER_OK = 0,
    CIRCULAR_BUFFER_FAIL = -1,
    CIRCULAR_BUFFER_ERR_NO_MEM = -2,
    CIRCULAR_BUFFER_ERR_INVALID_ARG = -3,
    CIRCULAR_BUFFER_ERR_INVALID_SIZE = -4,
    CIRCULAR_BUFFER_ERR_NOT_FOUND = -5,
    CIRCULAR_BUFFER_ERR_INVALID_RECORD = -6,
} circular_buffer_err_t;

typedef circular_buffer_err_t (*circular_buffer_read_fn)(void *ctx, size_t src_addr, void *dest, size_t size);
typedef circular_buffer_err_t (*circular_buffer_erase_range_fn)(void *ctx, size_t start_addr, size_t size);
typedef circular_buffer_err_t (*circular_buffer_write_fn)(void *ctx, size_t dest_addr, const void *src, size_t size);

typedef struct {
    size_t front;
    size_t record_size;
    size_t record_num;
    size_t sector_size;
    size_t total_size;
    uint32_t sequence;
    void *storage_ctx;
    circular_buffer_read_fn read;
    circular_buffer_erase_range_fn erase_range;
    circular_buffer_write_fn write;
    int overwrite;
} CircularBuffer;

circular_buffer_err_t circular_buffer_init(CircularBuffer *cb,
                                           size_t sector_size,
                                           circular_buffer_read_fn read,
                                           circular_buffer_erase_range_fn erase_range,
                                           circular_buffer_write_fn write,
                                           size_t total_size,
                                           void *storage_ctx,
                                           size_t record_size,
                                           int overwrite,
                                           int recovery_mode);
circular_buffer_err_t circular_buffer_push_back(CircularBuffer *cb, void *src);
circular_buffer_err_t circular_buffer_peek_at(CircularBuffer *cb, size_t index, void *dest);
circular_buffer_err_t circular_buffer_peek_front(CircularBuffer *cb, void *dest);
circular_buffer_err_t circular_buffer_pop_front(CircularBuffer *cb, void *dest);
circular_buffer_err_t circular_buffer_delete_front(CircularBuffer *cb);
uint32_t circular_buffer_get_record_num(CircularBuffer *cb);
size_t circular_buffer_get_max_records(CircularBuffer *cb);

#ifdef __cplusplus
}
#endif

#endif
