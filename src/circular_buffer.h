#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include "wear_levelling.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    size_t front;
    size_t record_size;
    size_t record_num;
    uint32_t sequence;
    wl_handle_t wl_handle;
    int overwrite;
} CircularBuffer;

esp_err_t circular_buffer_init(CircularBuffer *cb, char *partition_name, size_t record_size, int overwrite, int recovery_mode);
esp_err_t circular_buffer_push_back(CircularBuffer *cb, void *src);
esp_err_t circular_buffer_peek_front(CircularBuffer *cb, void *dest);
esp_err_t circular_buffer_pop_front(CircularBuffer *cb, void *dest);
esp_err_t circular_buffer_delete_front(CircularBuffer *cb);
uint32_t circular_buffer_get_record_num(CircularBuffer *cb);
size_t circular_buffer_get_max_records(CircularBuffer *cb);

#ifdef __cplusplus
}
#endif

#endif
