#include "circular_buffer.h"
#include "wear_levelling.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK_OK(expr) check_ok((expr), #expr, __LINE__)
#define CHECK_TRUE(expr) check_true((expr), #expr, __LINE__)
#define CHECK_EQ(actual, expected) check_eq((size_t)(actual), (size_t)(expected), #actual, #expected, __LINE__)

static const size_t SECTOR_SIZE = 4096;
static wl_handle_t wl_handle = WL_INVALID_HANDLE;

static void fail(const char *message, int line) {
    fprintf(stderr, "FAIL line %d: %s\n", line, message);
    exit(1);
}

static void check_ok(circular_buffer_err_t err, const char *expr, int line) {
    if (err != CIRCULAR_BUFFER_OK) {
        char message[160];
        snprintf(message, sizeof(message), "%s returned %d", expr, err);
        fail(message, line);
    }
}

static void check_true(int value, const char *expr, int line) {
    if (!value) fail(expr, line);
}

static void check_eq(size_t actual, size_t expected, const char *actual_expr, const char *expected_expr, int line) {
    if (actual != expected) {
        char message[200];
        snprintf(message, sizeof(message), "%s == %zu, expected %s == %zu", actual_expr, actual, expected_expr, expected);
        fail(message, line);
    }
}

static circular_buffer_err_t map_esp_err(esp_err_t err) {
    switch (err) {
        case ESP_OK: return CIRCULAR_BUFFER_OK;
        case ESP_FAIL: return CIRCULAR_BUFFER_FAIL;
        case ESP_ERR_NO_MEM: return CIRCULAR_BUFFER_ERR_NO_MEM;
        case ESP_ERR_INVALID_ARG: return CIRCULAR_BUFFER_ERR_INVALID_ARG;
        case ESP_ERR_INVALID_SIZE: return CIRCULAR_BUFFER_ERR_INVALID_SIZE;
        case ESP_ERR_NOT_FOUND: return CIRCULAR_BUFFER_ERR_NOT_FOUND;
        default: return CIRCULAR_BUFFER_FAIL;
    }
}

static void fill_record(uint8_t *record, size_t record_size, uint32_t value) {
    size_t i;

    memset(record, 0, record_size);
    memcpy(record, &value, sizeof(value));
    for (i = sizeof(value); i < record_size; ++i) {
        record[i] = (uint8_t)(value + i);
    }
}

static uint32_t read_record_id(const uint8_t *record) {
    uint32_t value;
    memcpy(&value, record, sizeof(value));
    return value;
}

static circular_buffer_err_t storage_read(void *ctx, size_t src_addr, void *dest, size_t size) {
    return map_esp_err(wl_read(*(wl_handle_t *)ctx, src_addr, dest, size));
}

static circular_buffer_err_t storage_erase_range(void *ctx, size_t start_addr, size_t size) {
    return map_esp_err(wl_erase_range(*(wl_handle_t *)ctx, start_addr, size));
}

static circular_buffer_err_t storage_write(void *ctx, size_t dest_addr, const void *src, size_t size) {
    return map_esp_err(wl_write(*(wl_handle_t *)ctx, dest_addr, src, size));
}

static void mount_mock_storage(void) {
    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_ANY,
        "mock"
    );

    CHECK_TRUE(partition != NULL);
    CHECK_OK(wl_mount(partition, &wl_handle));
}

static circular_buffer_err_t init_mock_buffer(CircularBuffer *cb, size_t record_size, int overwrite, int recovery_mode) {
    return circular_buffer_init(cb,
                                wl_sector_size(wl_handle),
                                storage_read,
                                storage_erase_range,
                                storage_write,
                                wl_size(wl_handle),
                                &wl_handle,
                                record_size,
                                overwrite,
                                recovery_mode);
}

static int all_ff(const uint8_t *record, size_t record_size) {
    size_t i;

    for (i = 0; i < record_size; ++i) {
        if (record[i] != 0xFF) return 0;
    }
    return 1;
}

static void fresh_buffer(CircularBuffer *cb, size_t record_size, int overwrite, int recovery_mode) {
    wl_mock_set_reset_on_mount(1);
    wl_mock_reset_flash();
    mount_mock_storage();
    CHECK_OK(init_mock_buffer(cb, record_size, overwrite, recovery_mode));
    wl_mock_set_reset_on_mount(0);
}

static void test_delete_front_keeps_last_record_in_sector(void) {
    const size_t record_size = SECTOR_SIZE / 4;
    CircularBuffer cb;
    uint8_t input[SECTOR_SIZE / 4];
    uint8_t output[SECTOR_SIZE / 4];
    uint32_t i;

    fresh_buffer(&cb, record_size, 1, 0);

    for (i = 0; i < 4; ++i) {
        fill_record(input, record_size, i);
        CHECK_OK(circular_buffer_push_back(&cb, input));
    }

    for (i = 0; i < 3; ++i) {
        CHECK_OK(circular_buffer_pop_front(&cb, output));
        CHECK_EQ(read_record_id(output), i);
    }

    CHECK_OK(circular_buffer_peek_front(&cb, output));
    CHECK_TRUE(!all_ff(output, record_size));
    CHECK_EQ(read_record_id(output), 3);

    CHECK_OK(circular_buffer_pop_front(&cb, output));
    CHECK_EQ(read_record_id(output), 3);
    CHECK_EQ(circular_buffer_get_record_num(&cb), 0);
    CHECK_EQ(circular_buffer_delete_front(&cb), CIRCULAR_BUFFER_ERR_NOT_FOUND);
    CHECK_EQ(circular_buffer_pop_front(&cb, output), CIRCULAR_BUFFER_ERR_NOT_FOUND);
    CHECK_OK(wl_unmount(0));
}

static void test_delete_front_crosses_partial_sector(void) {
    const size_t record_size = 1500;
    CircularBuffer cb;
    uint8_t input[1500];
    uint8_t output[1500];
    uint32_t i;

    fresh_buffer(&cb, record_size, 1, 0);

    for (i = 0; i < 5; ++i) {
        fill_record(input, record_size, i + 10);
        CHECK_OK(circular_buffer_push_back(&cb, input));
    }

    for (i = 0; i < 5; ++i) {
        CHECK_OK(circular_buffer_pop_front(&cb, output));
        CHECK_TRUE(!all_ff(output, record_size));
        CHECK_EQ(read_record_id(output), i + 10);
    }
    CHECK_OK(wl_unmount(0));
}

static void test_peek_at_reads_index_without_deleting(void) {
    const size_t record_size = 1500;
    CircularBuffer cb;
    uint8_t input[1500];
    uint8_t output[1500];
    uint32_t i;

    fresh_buffer(&cb, record_size, 1, 0);

    for (i = 0; i < 5; ++i) {
        fill_record(input, record_size, i + 10);
        CHECK_OK(circular_buffer_push_back(&cb, input));
    }

    CHECK_OK(circular_buffer_delete_front(&cb));

    for (i = 0; i < 4; ++i) {
        CHECK_OK(circular_buffer_peek_at(&cb, i, output));
        CHECK_TRUE(!all_ff(output, record_size));
        CHECK_EQ(read_record_id(output), i + 11);
    }

    CHECK_EQ(circular_buffer_peek_at(&cb, 4, output), CIRCULAR_BUFFER_ERR_NOT_FOUND);
    CHECK_EQ(circular_buffer_peek_at(NULL, 0, output), CIRCULAR_BUFFER_ERR_INVALID_ARG);
    CHECK_EQ(circular_buffer_peek_at(&cb, 0, NULL), CIRCULAR_BUFFER_ERR_INVALID_ARG);
    CHECK_EQ(circular_buffer_get_record_num(&cb), 4);

    CHECK_OK(circular_buffer_pop_front(&cb, output));
    CHECK_EQ(read_record_id(output), 11);
    CHECK_OK(wl_unmount(0));
}

static void test_read_rejects_uncommitted_record(void) {
    const size_t record_size = 64;
    CircularBuffer cb;
    uint8_t input[64];
    uint8_t output[64];
    uint8_t corrupt_commit = 0xFF;

    fresh_buffer(&cb, record_size, 1, 0);

    fill_record(input, record_size, 500);
    CHECK_OK(circular_buffer_push_back(&cb, input));

    CHECK_OK(wl_mock_overwrite((2 * SECTOR_SIZE) + record_size, &corrupt_commit, sizeof(corrupt_commit)));
    CHECK_EQ(circular_buffer_peek_front(&cb, output), CIRCULAR_BUFFER_ERR_INVALID_RECORD);
    CHECK_EQ(circular_buffer_get_record_num(&cb), 1);

    CHECK_OK(wl_unmount(0));
}

static void test_flagged_records_can_be_found_and_cleared(void) {
    const size_t record_size = 64;
    CircularBuffer cb;
    uint8_t input[64];
    uint8_t output[64];
    size_t record_index;
    uint32_t i;

    fresh_buffer(&cb, record_size, 1, 0);

    for (i = 0; i < 5; ++i) {
        fill_record(input, record_size, i + 700);
        CHECK_OK(circular_buffer_push_back(&cb, input));
    }

    CHECK_EQ(cb.first_flagged_record[2], 0);
    CHECK_OK(circular_buffer_peek_flagged(&cb, 2, 2, output, &record_index));
    CHECK_EQ(record_index, 2);
    CHECK_EQ(read_record_id(output), 702);

    CHECK_OK(circular_buffer_clear_flag(&cb, 0, 2));
    CHECK_OK(circular_buffer_clear_flag(&cb, 0, 2));
    CHECK_EQ(cb.first_flagged_record[2], 1);

    CHECK_OK(circular_buffer_clear_flag(&cb, 1, 2));
    CHECK_EQ(cb.first_flagged_record[2], 2);
    CHECK_OK(circular_buffer_peek_flagged(&cb, 0, 2, output, &record_index));
    CHECK_EQ(record_index, 2);
    CHECK_EQ(read_record_id(output), 702);

    CHECK_OK(circular_buffer_delete_front(&cb));
    CHECK_EQ(cb.first_flagged_record[2], 1);
    CHECK_OK(circular_buffer_peek_flagged(&cb, 0, 2, output, &record_index));
    CHECK_EQ(record_index, 1);
    CHECK_EQ(read_record_id(output), 702);

    CHECK_EQ(circular_buffer_peek_flagged(&cb, 3, 2, output, &record_index), CIRCULAR_BUFFER_ERR_NOT_FOUND);
    CHECK_EQ(circular_buffer_peek_flagged(&cb, 0, 4, output, &record_index), CIRCULAR_BUFFER_ERR_INVALID_ARG);
    CHECK_EQ(circular_buffer_clear_flag(&cb, 4, 2), CIRCULAR_BUFFER_ERR_NOT_FOUND);
    CHECK_EQ(circular_buffer_clear_flag(&cb, 0, 4), CIRCULAR_BUFFER_ERR_INVALID_ARG);

    CHECK_OK(wl_unmount(0));
}

static void test_overwrite_wraps_without_ff_records(void) {
    const size_t record_size = SECTOR_SIZE / 4;
    CircularBuffer cb;
    uint8_t input[SECTOR_SIZE / 4];
    uint8_t output[SECTOR_SIZE / 4];
    uint32_t max_records;
    uint32_t records_per_sector;
    uint32_t i;
    uint32_t expected;

    fresh_buffer(&cb, record_size, 1, 0);
    max_records = (uint32_t)circular_buffer_get_max_records(&cb);
    records_per_sector = (uint32_t)(SECTOR_SIZE / (record_size + 1));

    for (i = 0; i < max_records + records_per_sector; ++i) {
        fill_record(input, record_size, i);
        CHECK_OK(circular_buffer_push_back(&cb, input));
    }

    CHECK_EQ(circular_buffer_get_record_num(&cb), max_records);
    for (expected = records_per_sector; expected < max_records + records_per_sector; ++expected) {
        CHECK_OK(circular_buffer_pop_front(&cb, output));
        CHECK_TRUE(!all_ff(output, record_size));
        CHECK_EQ(read_record_id(output), expected);
    }
    CHECK_EQ(circular_buffer_get_record_num(&cb), 0);
    CHECK_OK(wl_unmount(0));
}

static void test_no_overwrite_reports_full_and_preserves_data(void) {
    const size_t record_size = SECTOR_SIZE / 2;
    CircularBuffer cb;
    uint8_t input[SECTOR_SIZE / 2];
    uint8_t output[SECTOR_SIZE / 2];
    uint32_t max_records;
    uint32_t i;

    fresh_buffer(&cb, record_size, 0, 0);
    max_records = (uint32_t)circular_buffer_get_max_records(&cb);

    for (i = 0; i < max_records; ++i) {
        fill_record(input, record_size, i + 1000);
        CHECK_OK(circular_buffer_push_back(&cb, input));
    }

    fill_record(input, record_size, 999999);
    CHECK_EQ(circular_buffer_push_back(&cb, input), CIRCULAR_BUFFER_ERR_NO_MEM);
    CHECK_EQ(circular_buffer_get_record_num(&cb), max_records);

    CHECK_OK(circular_buffer_pop_front(&cb, output));
    CHECK_EQ(read_record_id(output), 1000);
    CHECK_OK(wl_unmount(0));
}

static void test_remount_preserves_header_and_records(void) {
    const size_t record_size = 64;
    CircularBuffer cb;
    uint8_t input[64];
    uint8_t output[64];
    uint32_t i;
    uint32_t expected;

    fresh_buffer(&cb, record_size, 1, 0);

    for (i = 0; i < 8; ++i) {
        fill_record(input, record_size, i + 200);
        CHECK_OK(circular_buffer_push_back(&cb, input));
    }

    for (i = 0; i < 3; ++i) {
        CHECK_OK(circular_buffer_pop_front(&cb, output));
        CHECK_EQ(read_record_id(output), i + 200);
    }
    CHECK_OK(wl_unmount(0));

    mount_mock_storage();
    CHECK_OK(init_mock_buffer(&cb, record_size, 1, 0));
    CHECK_EQ(circular_buffer_get_record_num(&cb), 5);

    for (expected = 203; expected < 208; ++expected) {
        CHECK_OK(circular_buffer_pop_front(&cb, output));
        CHECK_TRUE(!all_ff(output, record_size));
        CHECK_EQ(read_record_id(output), expected);
    }
    CHECK_OK(wl_unmount(0));
}

static void test_recovery_mode_revives_record_after_latest_header_corruption(void) {
    const size_t record_size = 64;
    CircularBuffer cb;
    uint8_t input[64];
    uint8_t output[64];
    uint8_t corrupt[32];
    uint32_t i;
    uint32_t expected;

    fresh_buffer(&cb, record_size, 1, 0);

    for (i = 0; i < 3; ++i) {
        fill_record(input, record_size, i + 300);
        CHECK_OK(circular_buffer_push_back(&cb, input));
    }
    memset(corrupt, 0x00, sizeof(corrupt));
    CHECK_OK(wl_mock_overwrite(SECTOR_SIZE, corrupt, sizeof(corrupt)));
    CHECK_OK(wl_unmount(0));

    mount_mock_storage();
    CHECK_OK(init_mock_buffer(&cb, record_size, 1, 1));
    CHECK_EQ(circular_buffer_get_record_num(&cb), 3);

    for (expected = 300; expected < 303; ++expected) {
        CHECK_OK(circular_buffer_pop_front(&cb, output));
        CHECK_TRUE(!all_ff(output, record_size));
        CHECK_EQ(read_record_id(output), expected);
    }
    CHECK_OK(wl_unmount(0));
}

static uint32_t next_rand(uint32_t *state) {
    *state = *state * 1664525u + 1013904223u;
    return *state;
}

static void model_pop_front(uint32_t *expected, size_t max_records, size_t *expected_front, size_t *expected_count) {
    *expected_front = (*expected_front + 1) % max_records;
    (*expected_count)--;
    (void)expected;
}

static uint32_t model_front(uint32_t *expected, size_t expected_front) {
    return expected[expected_front];
}

static void model_push_back(uint32_t *expected, size_t max_records, size_t expected_front, size_t expected_count, uint32_t value) {
    expected[(expected_front + expected_count) % max_records] = value;
}

static void run_randomized_model(size_t record_size, int overwrite) {
    CircularBuffer cb;
    uint8_t *input;
    uint8_t *output;
    uint32_t *expected;
    uint32_t rng = overwrite ? 0xC0FFEEu : 0x51A7EEDu;
    uint32_t next_value = 10000;
    size_t max_records;
    size_t records_per_sector;
    size_t sector_count;
    size_t model_front_slot = 0;
    size_t expected_front = 0;
    size_t expected_count = 0;
    size_t step;

    fresh_buffer(&cb, record_size, overwrite, 0);

    input = (uint8_t *)malloc(record_size);
    output = (uint8_t *)malloc(record_size);
    max_records = circular_buffer_get_max_records(&cb);
    expected = (uint32_t *)malloc(max_records * sizeof(uint32_t));
    if (input == NULL || output == NULL || expected == NULL) fail("malloc failed", __LINE__);

    records_per_sector = SECTOR_SIZE / (record_size + 1);
    sector_count = max_records / records_per_sector;

    for (step = 0; step < 6000; ++step) {
        uint32_t op = next_rand(&rng) % 100;

        if (op < 45) {
            circular_buffer_err_t err;
            fill_record(input, record_size, next_value);
            err = circular_buffer_push_back(&cb, input);
            if (expected_count == max_records) {
                if (!overwrite) {
                    CHECK_EQ(err, CIRCULAR_BUFFER_ERR_NO_MEM);
                    CHECK_EQ(circular_buffer_get_record_num(&cb), expected_count);
                    ++next_value;
                    continue;
                } else {
                    size_t front_sector = model_front_slot / records_per_sector;
                    size_t drop_count = records_per_sector - (model_front_slot % records_per_sector);
                    size_t i;

                    for (i = 0; i < drop_count; ++i) {
                        model_pop_front(expected, max_records, &expected_front, &expected_count);
                    }
                    model_front_slot = ((front_sector + 1) % sector_count) * records_per_sector;
                }
            }

            CHECK_OK(err);
            model_push_back(expected, max_records, expected_front, expected_count, next_value);
            expected_count++;
            ++next_value;
        } else if (op < 70) {
            circular_buffer_err_t err = circular_buffer_pop_front(&cb, output);
            if (expected_count == 0) {
                CHECK_EQ(err, CIRCULAR_BUFFER_ERR_NOT_FOUND);
            } else {
                CHECK_OK(err);
                CHECK_TRUE(!all_ff(output, record_size));
                CHECK_EQ(read_record_id(output), model_front(expected, expected_front));
                model_pop_front(expected, max_records, &expected_front, &expected_count);
                model_front_slot = (model_front_slot + 1) % max_records;
            }
        } else if (op < 85) {
            circular_buffer_err_t err = circular_buffer_peek_front(&cb, output);
            if (expected_count == 0) {
                CHECK_EQ(err, CIRCULAR_BUFFER_ERR_NOT_FOUND);
            } else {
                CHECK_OK(err);
                CHECK_TRUE(!all_ff(output, record_size));
                CHECK_EQ(read_record_id(output), model_front(expected, expected_front));
            }
        } else {
            if (expected_count != 0) {
                CHECK_OK(circular_buffer_delete_front(&cb));
                model_pop_front(expected, max_records, &expected_front, &expected_count);
                model_front_slot = (model_front_slot + 1) % max_records;
            }
        }

        CHECK_EQ(circular_buffer_get_record_num(&cb), expected_count);
    }

    while (expected_count != 0) {
        CHECK_OK(circular_buffer_pop_front(&cb, output));
        CHECK_TRUE(!all_ff(output, record_size));
        CHECK_EQ(read_record_id(output), model_front(expected, expected_front));
        model_pop_front(expected, max_records, &expected_front, &expected_count);
    }

    CHECK_EQ(circular_buffer_get_record_num(&cb), 0);
    free(expected);
    free(output);
    free(input);
    CHECK_OK(wl_unmount(0));
}

static void test_randomized_against_model(void) {
    run_randomized_model(SECTOR_SIZE / 4, 0);
    run_randomized_model(SECTOR_SIZE / 4, 1);
    run_randomized_model(SECTOR_SIZE / 8, 1);
}

int main(void) {
    wl_mock_set_save_snapshots(0);

    test_delete_front_keeps_last_record_in_sector();
    test_delete_front_crosses_partial_sector();
    test_peek_at_reads_index_without_deleting();
    test_read_rejects_uncommitted_record();
    test_flagged_records_can_be_found_and_cleared();
    test_overwrite_wraps_without_ff_records();
    test_no_overwrite_reports_full_and_preserves_data();
    test_remount_preserves_header_and_records();
    test_recovery_mode_revives_record_after_latest_header_corruption();
    test_randomized_against_model();

    printf("All circular buffer tests passed.\n");
    return 0;
}
