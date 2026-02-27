#include <unity.h>
#include "robot_logic.h"

void setUp(void) {}
void tearDown(void) {}

void test_extract_N_field(void) {
    TEST_ASSERT_EQUAL(7, getJsonInt("{\"N\":7,\"D1\":128}", "\"N\":"));
}

void test_extract_D1_field(void) {
    TEST_ASSERT_EQUAL(128, getJsonInt("{\"N\":7,\"D1\":128,\"D2\":64}", "\"D1\":"));
}

void test_extract_D2_field(void) {
    TEST_ASSERT_EQUAL(64, getJsonInt("{\"N\":7,\"D1\":128,\"D2\":64}", "\"D2\":"));
}

void test_extract_D3_field(void) {
    TEST_ASSERT_EQUAL(255, getJsonInt("{\"N\":20,\"D1\":100,\"D2\":200,\"D3\":255}", "\"D3\":"));
}

void test_missing_field_returns_default(void) {
    TEST_ASSERT_EQUAL(0, getJsonInt("{\"N\":7}", "\"D1\":"));
}

void test_missing_field_returns_custom_default(void) {
    TEST_ASSERT_EQUAL(-1, getJsonInt("{\"N\":7}", "\"D1\":", -1));
}

void test_negative_value(void) {
    TEST_ASSERT_EQUAL(-150, getJsonInt("{\"N\":7,\"D1\":-150}", "\"D1\":"));
}

void test_zero_value(void) {
    TEST_ASSERT_EQUAL(0, getJsonInt("{\"N\":7,\"D1\":0}", "\"D1\":"));
}

void test_large_value(void) {
    TEST_ASSERT_EQUAL(1000, getJsonInt("{\"N\":102,\"D1\":1000}", "\"D1\":"));
}

void test_command_ping(void) {
    TEST_ASSERT_EQUAL(1, getJsonInt("{\"N\":1}", "\"N\":"));
}

void test_command_stop(void) {
    TEST_ASSERT_EQUAL(6, getJsonInt("{\"N\":6}", "\"N\":"));
}

void test_command_all_sensors(void) {
    TEST_ASSERT_EQUAL(100, getJsonInt("{\"N\":100}", "\"N\":"));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_extract_N_field);
    RUN_TEST(test_extract_D1_field);
    RUN_TEST(test_extract_D2_field);
    RUN_TEST(test_extract_D3_field);
    RUN_TEST(test_missing_field_returns_default);
    RUN_TEST(test_missing_field_returns_custom_default);
    RUN_TEST(test_negative_value);
    RUN_TEST(test_zero_value);
    RUN_TEST(test_large_value);
    RUN_TEST(test_command_ping);
    RUN_TEST(test_command_stop);
    RUN_TEST(test_command_all_sensors);
    return UNITY_END();
}
