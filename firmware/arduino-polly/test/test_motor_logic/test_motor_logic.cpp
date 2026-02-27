#include <unity.h>
#include "robot_logic.h"

void setUp(void) {}
void tearDown(void) {}

// --- rampSpeed tests ---

void test_ramp_from_zero_to_positive(void) {
    // Starting at 0, target 100, maxAccel 20 → should move to 20
    TEST_ASSERT_EQUAL(20, rampSpeed(0, 100, 20));
}

void test_ramp_from_zero_to_negative(void) {
    // Starting at 0, target -100, maxAccel 20 → should move to -20
    TEST_ASSERT_EQUAL(-20, rampSpeed(0, -100, 20));
}

void test_ramp_reaches_target_when_close(void) {
    // Starting at 90, target 100, maxAccel 20 → should reach 100 (delta=10 < maxAccel)
    TEST_ASSERT_EQUAL(100, rampSpeed(90, 100, 20));
}

void test_ramp_already_at_target(void) {
    TEST_ASSERT_EQUAL(150, rampSpeed(150, 150, 20));
}

void test_ramp_decelerate(void) {
    // Starting at 200, target 0, maxAccel 20 → should move to 180
    TEST_ASSERT_EQUAL(180, rampSpeed(200, 0, 20));
}

void test_ramp_decelerate_negative(void) {
    // Starting at -200, target 0, maxAccel 20 → should move to -180
    TEST_ASSERT_EQUAL(-180, rampSpeed(-200, 0, 20));
}

void test_ramp_reverse_direction(void) {
    // Starting at 100, target -100, maxAccel 20 → should decrease by 20
    TEST_ASSERT_EQUAL(80, rampSpeed(100, -100, 20));
}

void test_ramp_small_max_accel(void) {
    TEST_ASSERT_EQUAL(1, rampSpeed(0, 255, 1));
}

// --- clampMotorSpeed tests ---

void test_clamp_no_bias(void) {
    TEST_ASSERT_EQUAL(150, clampMotorSpeed(150, 1.0));
}

void test_clamp_with_bias_increase(void) {
    // 150 * 1.1 = 165
    TEST_ASSERT_EQUAL(165, clampMotorSpeed(150, 1.1));
}

void test_clamp_with_bias_decrease(void) {
    // 150 * 0.9 = 135
    TEST_ASSERT_EQUAL(135, clampMotorSpeed(150, 0.9));
}

void test_clamp_overflow_positive(void) {
    // 250 * 1.2 = 300, should clamp to 255
    TEST_ASSERT_EQUAL(255, clampMotorSpeed(250, 1.2));
}

void test_clamp_overflow_negative(void) {
    // -250 * 1.2 = -300, should clamp to -255
    TEST_ASSERT_EQUAL(-255, clampMotorSpeed(-250, 1.2));
}

void test_clamp_zero(void) {
    TEST_ASSERT_EQUAL(0, clampMotorSpeed(0, 1.5));
}

void test_clamp_negative_speed_no_bias(void) {
    TEST_ASSERT_EQUAL(-128, clampMotorSpeed(-128, 1.0));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_ramp_from_zero_to_positive);
    RUN_TEST(test_ramp_from_zero_to_negative);
    RUN_TEST(test_ramp_reaches_target_when_close);
    RUN_TEST(test_ramp_already_at_target);
    RUN_TEST(test_ramp_decelerate);
    RUN_TEST(test_ramp_decelerate_negative);
    RUN_TEST(test_ramp_reverse_direction);
    RUN_TEST(test_ramp_small_max_accel);
    RUN_TEST(test_clamp_no_bias);
    RUN_TEST(test_clamp_with_bias_increase);
    RUN_TEST(test_clamp_with_bias_decrease);
    RUN_TEST(test_clamp_overflow_positive);
    RUN_TEST(test_clamp_overflow_negative);
    RUN_TEST(test_clamp_zero);
    RUN_TEST(test_clamp_negative_speed_no_bias);
    return UNITY_END();
}
