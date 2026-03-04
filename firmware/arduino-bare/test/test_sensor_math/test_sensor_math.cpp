#include <unity.h>
#include "robot_logic.h"

void setUp(void) {}
void tearDown(void) {}

// --- adcToBatteryVoltage tests ---
void test_battery_mid_range(void) {
    // 512/1023 * 5.0 * 2.0 ≈ 5.01V
    float voltage = adcToBatteryVoltage(512, 2.0);
    TEST_ASSERT_FLOAT_WITHIN(0.05, 5.01, voltage);
}
void test_battery_zero_adc(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.0, adcToBatteryVoltage(0, 2.0));
}
void test_battery_full_scale(void) {
    // 1023/1023 * 5.0 * 2.0 = 10.0V
    float voltage = adcToBatteryVoltage(1023, 2.0);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 10.0, voltage);
}

// --- pulseToDistanceCm tests ---
void test_distance_typical(void) {
    // 1160us / 58 = 20cm
    TEST_ASSERT_EQUAL(20, pulseToDistanceCm(1160));
}
void test_distance_close(void) {
    TEST_ASSERT_EQUAL(2, pulseToDistanceCm(116));
}
void test_distance_far(void) {
    TEST_ASSERT_EQUAL(100, pulseToDistanceCm(5800));
}
void test_distance_timeout_returns_negative_one(void) {
    TEST_ASSERT_EQUAL(-1, pulseToDistanceCm(0));
}

// --- median3 tests ---
void test_median3_sorted(void) {
    TEST_ASSERT_EQUAL(2, median3(1, 2, 3));
}
void test_median3_reverse(void) {
    TEST_ASSERT_EQUAL(2, median3(3, 2, 1));
}
void test_median3_same(void) {
    TEST_ASSERT_EQUAL(5, median3(5, 5, 5));
}
void test_median3_two_timeouts(void) {
    TEST_ASSERT_EQUAL(-1, median3(-1, 20, -1));
}
void test_median3_one_timeout(void) {
    TEST_ASSERT_EQUAL(15, median3(-1, 15, 20));
}
void test_battery_custom_ratio(void) {
    float voltage = adcToBatteryVoltage(512, 3.5);
    TEST_ASSERT_FLOAT_WITHIN(0.05, 8.76, voltage);
}
void test_battery_minimum_reading(void) {
    float voltage = adcToBatteryVoltage(1, 2.0);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0098, voltage);
}
void test_battery_3v3_reference(void) {
    float voltage = adcToBatteryVoltage(512, 2.0, 3.3f, 1023);
    TEST_ASSERT_FLOAT_WITHIN(0.05, 3.30, voltage);
}
void test_distance_minimum(void) {
    TEST_ASSERT_EQUAL(1, pulseToDistanceCm(58));
}
void test_distance_sub_minimum(void) {
    TEST_ASSERT_EQUAL(0, pulseToDistanceCm(57));
}
void test_distance_max_range(void) {
    TEST_ASSERT_EQUAL(400, pulseToDistanceCm(23200));
}
void test_distance_negative_pulse(void) {
    TEST_ASSERT_TRUE(pulseToDistanceCm(-100) < 0);
}
void test_median3_two_same_low(void) {
    TEST_ASSERT_EQUAL(5, median3(5, 5, 10));
}
void test_median3_two_same_high(void) {
    TEST_ASSERT_EQUAL(10, median3(10, 5, 10));
}
void test_median3_all_negative(void) {
    TEST_ASSERT_EQUAL(-2, median3(-1, -2, -3));
}
void test_median3_large_spread(void) {
    TEST_ASSERT_EQUAL(100, median3(0, 100, 30000));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_battery_mid_range);
    RUN_TEST(test_battery_zero_adc);
    RUN_TEST(test_battery_full_scale);
    RUN_TEST(test_battery_custom_ratio);
    RUN_TEST(test_battery_minimum_reading);
    RUN_TEST(test_battery_3v3_reference);
    RUN_TEST(test_distance_typical);
    RUN_TEST(test_distance_close);
    RUN_TEST(test_distance_far);
    RUN_TEST(test_distance_timeout_returns_negative_one);
    RUN_TEST(test_distance_minimum);
    RUN_TEST(test_distance_sub_minimum);
    RUN_TEST(test_distance_max_range);
    RUN_TEST(test_distance_negative_pulse);
    RUN_TEST(test_median3_sorted);
    RUN_TEST(test_median3_reverse);
    RUN_TEST(test_median3_same);
    RUN_TEST(test_median3_two_timeouts);
    RUN_TEST(test_median3_one_timeout);
    RUN_TEST(test_median3_two_same_low);
    RUN_TEST(test_median3_two_same_high);
    RUN_TEST(test_median3_all_negative);
    RUN_TEST(test_median3_large_spread);
    return UNITY_END();
}
