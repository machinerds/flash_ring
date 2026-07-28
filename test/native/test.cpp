#include "circular_buffer.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>

#include "esp_crc.h"

#define CHECK_OK(expr) check_ok((expr), #expr, __LINE__)
#define CHECK_TRUE(expr) check_true((expr), #expr, __LINE__)
#define CHECK_EQ(actual, expected) check_eq((actual), (expected), #actual, #expected, __LINE__)

static const size_t SECTOR_SIZE = 4096;

static void fail(const char *message, int line) {
    std::fprintf(stderr, "FAIL line %d: %s\n", line, message);
    std::exit(1);
}

static void check_ok(esp_err_t err, const char *expr, int line) {
    if (err != ESP_OK) {
        char message[160];
        std::snprintf(message, sizeof(message), "%s returned %d", expr, err);
        fail(message, line);
    }
}

static void check_true(bool value, const char *expr, int line) {
    if (!value) fail(expr, line);
}

static void check_eq(size_t actual, size_t expected, const char *actual_expr, const char *expected_expr, int line) {
    if (actual != expected) {
        char message[200];
        std::snprintf(message, sizeof(message), "%s == %zu, expected %s == %zu", actual_expr, actual, expected_expr, expected);
        fail(message, line);
    }
}

static void fill_record(uint8_t *record, size_t record_size, uint32_t value) {
    std::memset(record, 0, record_size);
    std::memcpy(record, &value, sizeof(value));
    for (size_t i = sizeof(value); i < record_size; ++i) {
        record[i] = (uint8_t)(value + i);
    }
}

static uint32_t read_record_id(const uint8_t *record) {
    uint32_t value;
    std::memcpy(&value, record, sizeof(value));
    return value;
}

static bool all_ff(const uint8_t *record, size_t record_size) {
    for (size_t i = 0; i < record_size; ++i) {
        if (record[i] != 0xFF) return false;
    }
    return true;
}

static CircularBuffer fresh_buffer(size_t record_size, bool overwrite, bool recovery_mode = false) {
    wl_mock_set_reset_on_mount(1);
    wl_mock_reset_flash();
    CircularBuffer cb;
    CHECK_OK(cb.init((char*)"mock", record_size, overwrite, recovery_mode));
    wl_mock_set_reset_on_mount(0);
    return cb;
}

static void test_delete_front_keeps_last_record_in_sector() {
    const size_t record_size = SECTOR_SIZE / 4;
    CircularBuffer cb = fresh_buffer(record_size, true);
    uint8_t input[record_size];
    uint8_t output[record_size];

    for (uint32_t i = 0; i < 4; ++i) {
        fill_record(input, record_size, i);
        CHECK_OK(cb.push_back(input));
    }

    for (uint32_t i = 0; i < 3; ++i) {
        CHECK_OK(cb.pop_front(output));
        CHECK_EQ(read_record_id(output), i);
    }

    CHECK_OK(cb.peek_front(output));
    CHECK_TRUE(!all_ff(output, record_size));
    CHECK_EQ(read_record_id(output), 3);

    CHECK_OK(cb.pop_front(output));
    CHECK_EQ(read_record_id(output), 3);
    CHECK_EQ(cb.get_record_num(), 0);
    CHECK_EQ(cb.delete_front(), ESP_ERR_NOT_FOUND);
    CHECK_EQ(cb.pop_front(output), ESP_ERR_NOT_FOUND);
    CHECK_OK(wl_unmount(0));
}

static void test_delete_front_crosses_partial_sector() {
    const size_t record_size = 1500;
    CircularBuffer cb = fresh_buffer(record_size, true);
    uint8_t input[record_size];
    uint8_t output[record_size];

    for (uint32_t i = 0; i < 5; ++i) {
        fill_record(input, record_size, i + 10);
        CHECK_OK(cb.push_back(input));
    }

    for (uint32_t i = 0; i < 5; ++i) {
        CHECK_OK(cb.pop_front(output));
        CHECK_TRUE(!all_ff(output, record_size));
        CHECK_EQ(read_record_id(output), i + 10);
    }
    CHECK_OK(wl_unmount(0));
}

static void test_overwrite_wraps_without_ff_records() {
    const size_t record_size = SECTOR_SIZE / 4;
    CircularBuffer cb = fresh_buffer(record_size, true);
    uint8_t input[record_size];
    uint8_t output[record_size];
    const uint32_t max_records = (uint32_t)cb.get_max_records();

    for (uint32_t i = 0; i < max_records + 4; ++i) {
        fill_record(input, record_size, i);
        CHECK_OK(cb.push_back(input));
    }

    CHECK_EQ(cb.get_record_num(), max_records);
    for (uint32_t expected = 4; expected < max_records + 4; ++expected) {
        CHECK_OK(cb.pop_front(output));
        CHECK_TRUE(!all_ff(output, record_size));
        CHECK_EQ(read_record_id(output), expected);
    }
    CHECK_EQ(cb.get_record_num(), 0);
    CHECK_OK(wl_unmount(0));
}

static void test_no_overwrite_reports_full_and_preserves_data() {
    const size_t record_size = SECTOR_SIZE / 2;
    CircularBuffer cb = fresh_buffer(record_size, false);
    uint8_t input[record_size];
    uint8_t output[record_size];
    const uint32_t max_records = (uint32_t)cb.get_max_records();

    for (uint32_t i = 0; i < max_records; ++i) {
        fill_record(input, record_size, i + 1000);
        CHECK_OK(cb.push_back(input));
    }

    fill_record(input, record_size, 999999);
    CHECK_EQ(cb.push_back(input), ESP_ERR_NO_MEM);
    CHECK_EQ(cb.get_record_num(), max_records);

    CHECK_OK(cb.pop_front(output));
    CHECK_EQ(read_record_id(output), 1000);
    CHECK_OK(wl_unmount(0));
}

static void test_remount_preserves_header_and_records() {
    const size_t record_size = 64;
    {
        CircularBuffer cb = fresh_buffer(record_size, true);
        uint8_t input[record_size];
        uint8_t output[record_size];

        for (uint32_t i = 0; i < 8; ++i) {
            fill_record(input, record_size, i + 200);
            CHECK_OK(cb.push_back(input));
        }

        for (uint32_t i = 0; i < 3; ++i) {
            CHECK_OK(cb.pop_front(output));
            CHECK_EQ(read_record_id(output), i + 200);
        }
        CHECK_OK(wl_unmount(0));
    }

    CircularBuffer cb;
    CHECK_OK(cb.init((char*)"mock", record_size, true, false));
    CHECK_EQ(cb.get_record_num(), 5);

    uint8_t output[record_size];
    for (uint32_t expected = 203; expected < 208; ++expected) {
        CHECK_OK(cb.pop_front(output));
        CHECK_TRUE(!all_ff(output, record_size));
        CHECK_EQ(read_record_id(output), expected);
    }
    CHECK_OK(wl_unmount(0));
}

static void test_recovery_mode_revives_record_after_latest_header_corruption() {
    const size_t record_size = 64;
    {
        CircularBuffer cb = fresh_buffer(record_size, true);
        uint8_t input[record_size];

        for (uint32_t i = 0; i < 3; ++i) {
            fill_record(input, record_size, i + 300);
            CHECK_OK(cb.push_back(input));
        }
        uint8_t corrupt[32];
        std::memset(corrupt, 0x00, sizeof(corrupt));
        CHECK_OK(wl_mock_overwrite(SECTOR_SIZE, corrupt, sizeof(corrupt)));
        CHECK_OK(wl_unmount(0));
    }

    CircularBuffer cb;
    CHECK_OK(cb.init((char*)"mock", record_size, true, true));
    CHECK_EQ(cb.get_record_num(), 3);

    uint8_t output[record_size];
    for (uint32_t expected = 300; expected < 303; ++expected) {
        CHECK_OK(cb.pop_front(output));
        CHECK_TRUE(!all_ff(output, record_size));
        CHECK_EQ(read_record_id(output), expected);
    }
    CHECK_OK(wl_unmount(0));
}

static uint32_t next_rand(uint32_t *state) {
    *state = *state * 1664525u + 1013904223u;
    return *state;
}

static void run_randomized_model(size_t record_size, bool overwrite) {
    CircularBuffer cb = fresh_buffer(record_size, overwrite);
    uint8_t input[record_size];
    uint8_t output[record_size];
    std::deque<uint32_t> expected;
    uint32_t rng = overwrite ? 0xC0FFEEu : 0x51A7EEDu;
    uint32_t next_value = 10000;
    const size_t records_per_sector = SECTOR_SIZE / record_size;
    const size_t sector_count = cb.get_max_records() / records_per_sector;
    size_t model_front_slot = 0;

    for (size_t step = 0; step < 6000; ++step) {
        uint32_t op = next_rand(&rng) % 100;

        if (op < 45) {
            fill_record(input, record_size, next_value);
            esp_err_t err = cb.push_back(input);
            if (expected.size() == cb.get_max_records()) {
                if (!overwrite) {
                    CHECK_EQ(err, ESP_ERR_NO_MEM);
                    CHECK_EQ(cb.get_record_num(), expected.size());
                    ++next_value;
                    continue;
                }

                size_t front_sector = model_front_slot / records_per_sector;
                size_t drop_count = records_per_sector - (model_front_slot % records_per_sector);
                for (size_t i = 0; i < drop_count; ++i) expected.pop_front();
                model_front_slot = ((front_sector + 1) % sector_count) * records_per_sector;
            }

            CHECK_OK(err);
            expected.push_back(next_value);
            ++next_value;
        } else if (op < 70) {
            esp_err_t err = cb.pop_front(output);
            if (expected.empty()) {
                CHECK_EQ(err, ESP_ERR_NOT_FOUND);
            } else {
                CHECK_OK(err);
                CHECK_TRUE(!all_ff(output, record_size));
                CHECK_EQ(read_record_id(output), expected.front());
                expected.pop_front();
                model_front_slot = (model_front_slot + 1) % cb.get_max_records();
            }
        } else if (op < 85) {
            esp_err_t err = cb.peek_front(output);
            if (expected.empty()) {
                CHECK_EQ(err, ESP_ERR_NOT_FOUND);
            } else {
                CHECK_OK(err);
                CHECK_TRUE(!all_ff(output, record_size));
                CHECK_EQ(read_record_id(output), expected.front());
            }
        } else {
            if (expected.empty()) continue;
            CHECK_OK(cb.delete_front());
            expected.pop_front();
            model_front_slot = (model_front_slot + 1) % cb.get_max_records();
        }

        CHECK_EQ(cb.get_record_num(), expected.size());
    }

    while (!expected.empty()) {
        CHECK_OK(cb.pop_front(output));
        CHECK_TRUE(!all_ff(output, record_size));
        CHECK_EQ(read_record_id(output), expected.front());
        expected.pop_front();
    }

    CHECK_EQ(cb.get_record_num(), 0);
    CHECK_OK(wl_unmount(0));
}

static void test_randomized_against_model() {
    run_randomized_model(SECTOR_SIZE / 4, false);
    run_randomized_model(SECTOR_SIZE / 4, true);
    run_randomized_model(SECTOR_SIZE / 8, true);
}

int main() {
    crc32_init();
    wl_mock_set_save_snapshots(0);

    test_delete_front_keeps_last_record_in_sector();
    test_delete_front_crosses_partial_sector();
    test_overwrite_wraps_without_ff_records();
    test_no_overwrite_reports_full_and_preserves_data();
    test_remount_preserves_header_and_records();
    test_recovery_mode_revives_record_after_latest_header_corruption();
    test_randomized_against_model();

    std::printf("All circular buffer tests passed.\n");
    return 0;
}
