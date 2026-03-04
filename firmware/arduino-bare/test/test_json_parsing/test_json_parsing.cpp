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
void test_missing_field_returns_default(void) {
    TEST_ASSERT_EQUAL(0, getJsonInt("{\"N\":7}", "\"D1\":"));
}
void test_missing_field_returns_custom_default(void) {
    TEST_ASSERT_EQUAL(-1, getJsonInt("{\"N\":7}", "\"D1\":", -1));
}
void test_negative_value(void) {
    TEST_ASSERT_EQUAL(-150, getJsonInt("{\"N\":7,\"D1\":-150}", "\"D1\":"));
}
void test_empty_string_returns_default(void) {
    TEST_ASSERT_EQUAL(0, getJsonInt("", "\"N\":"));
}
void test_field_at_end_no_value(void) {
    TEST_ASSERT_EQUAL(0, getJsonInt("{\"N\":", "\"N\":"));
}
void test_zero_value_explicit(void) {
    TEST_ASSERT_EQUAL(0, getJsonInt("{\"N\":0,\"D1\":5}", "\"N\":"));
}
void test_large_value(void) {
    TEST_ASSERT_EQUAL(32000, getJsonInt("{\"D1\":32000}", "\"D1\":"));
}
void test_whitespace_before_value(void) {
    TEST_ASSERT_EQUAL(7, getJsonInt("{\"N\": 7}", "\"N\":"));
}
void test_extract_D2_field(void) {
    TEST_ASSERT_EQUAL(64, getJsonInt("{\"N\":7,\"D1\":128,\"D2\":64}", "\"D2\":"));
}
void test_extract_D3_field(void) {
    TEST_ASSERT_EQUAL(255, getJsonInt("{\"N\":20,\"D1\":0,\"D2\":128,\"D3\":255}", "\"D3\":"));
}
void test_partial_field_name_match(void) {
    TEST_ASSERT_EQUAL(5, getJsonInt("{\"D1\":5,\"D11\":99}", "\"D1\":"));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_extract_N_field);
    RUN_TEST(test_extract_D1_field);
    RUN_TEST(test_missing_field_returns_default);
    RUN_TEST(test_missing_field_returns_custom_default);
    RUN_TEST(test_negative_value);
    RUN_TEST(test_empty_string_returns_default);
    RUN_TEST(test_field_at_end_no_value);
    RUN_TEST(test_zero_value_explicit);
    RUN_TEST(test_large_value);
    RUN_TEST(test_whitespace_before_value);
    RUN_TEST(test_extract_D2_field);
    RUN_TEST(test_extract_D3_field);
    RUN_TEST(test_partial_field_name_match);
    return UNITY_END();
}
