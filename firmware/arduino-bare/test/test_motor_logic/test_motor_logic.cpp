#include <unity.h>
#include "robot_logic.h"

void setUp(void) {}
void tearDown(void) {}

// --- rampSpeed tests ---
void test_ramp_from_zero_to_positive(void) {
    TEST_ASSERT_EQUAL(20, rampSpeed(0, 100, 20));
}
void test_ramp_from_zero_to_negative(void) {
    TEST_ASSERT_EQUAL(-20, rampSpeed(0, -100, 20));
}
void test_ramp_reaches_target_when_close(void) {
    TEST_ASSERT_EQUAL(100, rampSpeed(90, 100, 20));
}
void test_ramp_already_at_target(void) {
    TEST_ASSERT_EQUAL(150, rampSpeed(150, 150, 20));
}
void test_ramp_decelerate(void) {
    TEST_ASSERT_EQUAL(180, rampSpeed(200, 0, 20));
}
void test_ramp_decelerate_negative(void) {
    TEST_ASSERT_EQUAL(-180, rampSpeed(-200, 0, 20));
}
void test_ramp_reverse_direction(void) {
    TEST_ASSERT_EQUAL(80, rampSpeed(100, -100, 20));
}

// --- clampMotorSpeed tests ---
void test_clamp_no_bias(void) {
    TEST_ASSERT_EQUAL(150, clampMotorSpeed(150, 1.0));
}
void test_clamp_overflow_positive(void) {
    TEST_ASSERT_EQUAL(255, clampMotorSpeed(250, 1.2));
}
void test_clamp_overflow_negative(void) {
    TEST_ASSERT_EQUAL(-255, clampMotorSpeed(-250, 1.2));
}
void test_clamp_zero(void) {
    TEST_ASSERT_EQUAL(0, clampMotorSpeed(0, 1.5));
}
void test_ramp_zero_accel(void) {
    TEST_ASSERT_EQUAL(100, rampSpeed(100, 200, 0));
}
void test_ramp_single_step(void) {
    TEST_ASSERT_EQUAL(101, rampSpeed(100, 200, 1));
}
void test_ramp_large_accel(void) {
    TEST_ASSERT_EQUAL(200, rampSpeed(100, 200, 1000));
}
void test_ramp_negative_toward_zero(void) {
    TEST_ASSERT_EQUAL(-80, rampSpeed(-100, 0, 20));
}
void test_ramp_negative_decelerate(void) {
    TEST_ASSERT_EQUAL(-120, rampSpeed(-100, -200, 20));
}
void test_clamp_bias_reduces(void) {
    TEST_ASSERT_EQUAL(120, clampMotorSpeed(150, 0.8));
}
void test_clamp_negative_with_bias(void) {
    TEST_ASSERT_EQUAL(-180, clampMotorSpeed(-150, 1.2));
}
void test_clamp_max_speed_no_bias(void) {
    TEST_ASSERT_EQUAL(255, clampMotorSpeed(255, 1.0));
}
void test_clamp_min_speed_no_bias(void) {
    TEST_ASSERT_EQUAL(-255, clampMotorSpeed(-255, 1.0));
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
    RUN_TEST(test_ramp_zero_accel);
    RUN_TEST(test_ramp_single_step);
    RUN_TEST(test_ramp_large_accel);
    RUN_TEST(test_ramp_negative_toward_zero);
    RUN_TEST(test_ramp_negative_decelerate);
    RUN_TEST(test_clamp_no_bias);
    RUN_TEST(test_clamp_overflow_positive);
    RUN_TEST(test_clamp_overflow_negative);
    RUN_TEST(test_clamp_zero);
    RUN_TEST(test_clamp_bias_reduces);
    RUN_TEST(test_clamp_negative_with_bias);
    RUN_TEST(test_clamp_max_speed_no_bias);
    RUN_TEST(test_clamp_min_speed_no_bias);
    return UNITY_END();
}
