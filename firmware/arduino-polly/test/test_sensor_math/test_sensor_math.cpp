#include <unity.h>
#include <math.h>
#include "robot_logic.h"

void setUp(void) {}
void tearDown(void) {}

// --- adcToBatteryVoltage tests ---

void test_battery_mid_range(void) {
    // ADC 512 ≈ 2.5V at pin → 5.0V with ratio 2.0
    float voltage = adcToBatteryVoltage(512, 2.0);
    TEST_ASSERT_FLOAT_WITHIN(0.05, 5.01, voltage);
}

void test_battery_full(void) {
    // Full battery ~8.4V: ADC reading ≈ 860 (8.4V / 2.0 ratio = 4.2V at pin, 4.2/5*1023 ≈ 860)
    float voltage = adcToBatteryVoltage(860, 2.0);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 8.4, voltage);
}

void test_battery_empty(void) {
    // Empty battery ~6.0V: ADC ≈ 614 (6.0V / 2.0 = 3.0V at pin, 3.0/5*1023 ≈ 614)
    float voltage = adcToBatteryVoltage(614, 2.0);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 6.0, voltage);
}

void test_battery_zero_adc(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.0, adcToBatteryVoltage(0, 2.0));
}

void test_battery_custom_ratio(void) {
    // With calibrated ratio 2.5: ADC 512 ≈ 2.5V at pin → 6.25V
    float voltage = adcToBatteryVoltage(512, 2.5);
    TEST_ASSERT_FLOAT_WITHIN(0.05, 6.26, voltage);
}

// --- rawToTempC tests ---

void test_temp_room_temperature(void) {
    // Room temp ~25°C: raw ≈ (25 - 36.53) * 340 = -3920
    float temp = rawToTempC(-3920);
    TEST_ASSERT_FLOAT_WITHIN(0.5, 25.0, temp);
}

void test_temp_zero_raw(void) {
    // raw=0 → 36.53°C
    TEST_ASSERT_FLOAT_WITHIN(0.01, 36.53, rawToTempC(0));
}

void test_temp_positive_raw(void) {
    // raw=3400 → (3400/340) + 36.53 = 10 + 36.53 = 46.53
    TEST_ASSERT_FLOAT_WITHIN(0.01, 46.53, rawToTempC(3400));
}

void test_temp_negative_raw(void) {
    // raw=-6800 → (-6800/340) + 36.53 = -20 + 36.53 = 16.53
    TEST_ASSERT_FLOAT_WITHIN(0.01, 16.53, rawToTempC(-6800));
}

// --- pulseToDistanceCm tests ---

void test_distance_typical(void) {
    // 1160 µs ≈ 20cm (1160 / 58 = 20)
    TEST_ASSERT_EQUAL(20, pulseToDistanceCm(1160));
}

void test_distance_close(void) {
    // 116 µs ≈ 2cm
    TEST_ASSERT_EQUAL(2, pulseToDistanceCm(116));
}

void test_distance_far(void) {
    // 5800 µs ≈ 100cm
    TEST_ASSERT_EQUAL(100, pulseToDistanceCm(5800));
}

void test_distance_max_range(void) {
    // 10000 µs timeout ≈ 172cm
    TEST_ASSERT_EQUAL(172, pulseToDistanceCm(10000));
}

void test_distance_timeout(void) {
    // 0 µs = timeout, should return 0
    TEST_ASSERT_EQUAL(0, pulseToDistanceCm(0));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_battery_mid_range);
    RUN_TEST(test_battery_full);
    RUN_TEST(test_battery_empty);
    RUN_TEST(test_battery_zero_adc);
    RUN_TEST(test_battery_custom_ratio);
    RUN_TEST(test_temp_room_temperature);
    RUN_TEST(test_temp_zero_raw);
    RUN_TEST(test_temp_positive_raw);
    RUN_TEST(test_temp_negative_raw);
    RUN_TEST(test_distance_typical);
    RUN_TEST(test_distance_close);
    RUN_TEST(test_distance_far);
    RUN_TEST(test_distance_max_range);
    RUN_TEST(test_distance_timeout);
    return UNITY_END();
}
