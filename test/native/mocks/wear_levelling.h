#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t wl_handle_t;

#define WL_INVALID_HANDLE -1

typedef struct {
    const char* label;
} esp_partition_t;

#define ESP_PARTITION_TYPE_DATA 1
#define ESP_PARTITION_SUBTYPE_ANY 0

const esp_partition_t* esp_partition_find_first(int type, int subtype, const char* label);

esp_err_t wl_mount(const esp_partition_t *partition, wl_handle_t *out_handle);
esp_err_t wl_unmount(wl_handle_t handle);
esp_err_t wl_erase_range(wl_handle_t handle, size_t start_addr, size_t size);
esp_err_t wl_write(wl_handle_t handle, size_t dest_addr, const void *src, size_t size);
esp_err_t wl_read(wl_handle_t handle, size_t src_addr, void *dest, size_t size);
size_t wl_size(wl_handle_t handle);
size_t wl_sector_size(wl_handle_t handle);

void wl_mock_reset_flash(void);
void wl_mock_set_reset_on_mount(int reset);
void wl_mock_set_save_snapshots(int save);
esp_err_t wl_mock_overwrite(size_t addr, const void *src, size_t size);

#ifdef __cplusplus
}
#endif
