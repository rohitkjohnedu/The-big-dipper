#include <unity.h>
#include <stdint.h>
#include "state_machine.h"

// =============================================================================
// test_state_machine.cpp
// Run with: pio test -e native
// =============================================================================

static StateMachine sm;

void setUp() {
    sm = StateMachine();   // fresh instance before each test
}

void tearDown() {}

// -----------------------------------------------------------------------------
// Construction
// -----------------------------------------------------------------------------

void test_initial_state_is_idle() {
    TEST_ASSERT_EQUAL(SystemState::IDLE, sm.getState());
}

void test_initial_phase_is_none() {
    TEST_ASSERT_EQUAL(RunPhase::NONE, sm.getPhase());
}

void test_initial_error_is_none() {
    TEST_ASSERT_EQUAL(ErrorCode::NONE, sm.getError());
}

void test_initial_homed_false() {
    TEST_ASSERT_FALSE(sm.isHomed());
}

// -----------------------------------------------------------------------------
// Transitions
// -----------------------------------------------------------------------------

void test_to_homing_sets_state() {
    sm.toHoming();
    TEST_ASSERT_EQUAL(SystemState::HOMING, sm.getState());
}

void test_to_homing_clears_phase() {
    sm.toRunning();
    sm.setPhase(RunPhase::DESCENDING);
    sm.toHoming();
    TEST_ASSERT_EQUAL(RunPhase::NONE, sm.getPhase());
}

void test_to_homing_clears_error() {
    sm.toError(ErrorCode::SOFT_LIMIT_EXCEEDED);
    sm.toHoming();
    TEST_ASSERT_EQUAL(ErrorCode::NONE, sm.getError());
}

void test_to_ready_sets_state() {
    sm.toHoming();
    sm.toReady();
    TEST_ASSERT_EQUAL(SystemState::READY, sm.getState());
}

void test_to_ready_sets_homed() {
    sm.toHoming();
    sm.toReady();
    TEST_ASSERT_TRUE(sm.isHomed());
}

void test_to_ready_clears_phase() {
    sm.toHoming();
    sm.toReady();
    TEST_ASSERT_EQUAL(RunPhase::NONE, sm.getPhase());
}

void test_to_running_sets_state() {
    sm.toHoming();
    sm.toReady();
    sm.toRunning();
    TEST_ASSERT_EQUAL(SystemState::RUNNING, sm.getState());
}

void test_to_paused_sets_state() {
    sm.toHoming();
    sm.toReady();
    sm.toRunning();
    sm.toPaused();
    TEST_ASSERT_EQUAL(SystemState::PAUSED, sm.getState());
}

void test_to_idle_sets_state() {
    sm.toIdle();
    TEST_ASSERT_EQUAL(SystemState::IDLE, sm.getState());
}

void test_to_idle_clears_phase() {
    sm.toHoming();
    sm.toReady();
    sm.toRunning();
    sm.setPhase(RunPhase::ASCENDING);
    sm.toIdle();
    TEST_ASSERT_EQUAL(RunPhase::NONE, sm.getPhase());
}

void test_to_error_sets_state() {
    sm.toError(ErrorCode::ENDSTOP_TRIGGERED_UNEXPECTEDLY);
    TEST_ASSERT_EQUAL(SystemState::ERROR, sm.getState());
}

void test_to_error_stores_code() {
    sm.toError(ErrorCode::SEG_BUFFER_OVERFLOW);
    TEST_ASSERT_EQUAL(ErrorCode::SEG_BUFFER_OVERFLOW, sm.getError());
}

void test_to_error_clears_phase() {
    sm.toHoming();
    sm.toReady();
    sm.toRunning();
    sm.setPhase(RunPhase::DWELL_BOTTOM);
    sm.toError(ErrorCode::SOFT_LIMIT_EXCEEDED);
    TEST_ASSERT_EQUAL(RunPhase::NONE, sm.getPhase());
}

void test_error_recovery_via_homing() {
    sm.toError(ErrorCode::SOFT_LIMIT_EXCEEDED);
    sm.toHoming();
    TEST_ASSERT_EQUAL(SystemState::HOMING, sm.getState());
    TEST_ASSERT_EQUAL(ErrorCode::NONE, sm.getError());
}

// -----------------------------------------------------------------------------
// Phase transitions
// -----------------------------------------------------------------------------

void test_set_phase_descending() {
    sm.toHoming();
    sm.toReady();
    sm.toRunning();
    sm.setPhase(RunPhase::DESCENDING);
    TEST_ASSERT_EQUAL(RunPhase::DESCENDING, sm.getPhase());
}

void test_set_phase_dwell_bottom() {
    sm.toHoming();
    sm.toReady();
    sm.toRunning();
    sm.setPhase(RunPhase::DWELL_BOTTOM);
    TEST_ASSERT_EQUAL(RunPhase::DWELL_BOTTOM, sm.getPhase());
}

void test_set_phase_ascending() {
    sm.toHoming();
    sm.toReady();
    sm.toRunning();
    sm.setPhase(RunPhase::ASCENDING);
    TEST_ASSERT_EQUAL(RunPhase::ASCENDING, sm.getPhase());
}

void test_set_phase_dwell_top() {
    sm.toHoming();
    sm.toReady();
    sm.toRunning();
    sm.setPhase(RunPhase::DWELL_TOP);
    TEST_ASSERT_EQUAL(RunPhase::DWELL_TOP, sm.getPhase());
}

// -----------------------------------------------------------------------------
// Guard functions
// -----------------------------------------------------------------------------

void test_can_run_true_only_in_ready() {
    TEST_ASSERT_FALSE(sm.canRun());           // IDLE

    sm.toHoming();
    TEST_ASSERT_FALSE(sm.canRun());           // HOMING

    sm.toReady();
    TEST_ASSERT_TRUE(sm.canRun());            // READY ← only valid state

    sm.toRunning();
    TEST_ASSERT_FALSE(sm.canRun());           // RUNNING

    sm.toPaused();
    TEST_ASSERT_FALSE(sm.canRun());           // PAUSED

    sm.toError(ErrorCode::NONE);
    TEST_ASSERT_FALSE(sm.canRun());           // ERROR
}

void test_can_pause_true_only_in_running() {
    TEST_ASSERT_FALSE(sm.canPause());         // IDLE

    sm.toHoming();
    TEST_ASSERT_FALSE(sm.canPause());         // HOMING

    sm.toReady();
    TEST_ASSERT_FALSE(sm.canPause());         // READY

    sm.toRunning();
    TEST_ASSERT_TRUE(sm.canPause());          // RUNNING ← only valid state

    sm.toPaused();
    TEST_ASSERT_FALSE(sm.canPause());         // PAUSED
}

void test_can_resume_true_only_in_paused() {
    TEST_ASSERT_FALSE(sm.canResume());        // IDLE

    sm.toHoming();
    sm.toReady();
    sm.toRunning();
    TEST_ASSERT_FALSE(sm.canResume());        // RUNNING

    sm.toPaused();
    TEST_ASSERT_TRUE(sm.canResume());         // PAUSED ← only valid state

    sm.toReady();
    TEST_ASSERT_FALSE(sm.canResume());        // READY
}

void test_can_jog_true_only_in_ready() {
    TEST_ASSERT_FALSE(sm.canJog());           // IDLE

    sm.toHoming();
    TEST_ASSERT_FALSE(sm.canJog());           // HOMING

    sm.toReady();
    TEST_ASSERT_TRUE(sm.canJog());            // READY ← only valid state

    sm.toRunning();
    TEST_ASSERT_FALSE(sm.canJog());           // RUNNING
}

// -----------------------------------------------------------------------------
// String representations
// -----------------------------------------------------------------------------

void test_state_string_idle() {
    TEST_ASSERT_EQUAL_STRING("IDLE", sm.stateString());
}

void test_state_string_homing() {
    sm.toHoming();
    TEST_ASSERT_EQUAL_STRING("HOMING", sm.stateString());
}

void test_state_string_ready() {
    sm.toHoming();
    sm.toReady();
    TEST_ASSERT_EQUAL_STRING("READY", sm.stateString());
}

void test_state_string_running() {
    sm.toHoming();
    sm.toReady();
    sm.toRunning();
    TEST_ASSERT_EQUAL_STRING("RUNNING", sm.stateString());
}

void test_state_string_paused() {
    sm.toHoming();
    sm.toReady();
    sm.toRunning();
    sm.toPaused();
    TEST_ASSERT_EQUAL_STRING("PAUSED", sm.stateString());
}

void test_state_string_error() {
    sm.toError(ErrorCode::NONE);
    TEST_ASSERT_EQUAL_STRING("ERROR", sm.stateString());
}

void test_phase_string_none() {
    TEST_ASSERT_EQUAL_STRING("NONE", sm.phaseString());
}

void test_phase_string_descending() {
    sm.toHoming();
    sm.toReady();
    sm.toRunning();
    sm.setPhase(RunPhase::DESCENDING);
    TEST_ASSERT_EQUAL_STRING("DESCENDING", sm.phaseString());
}

void test_phase_string_dwell_bottom() {
    sm.toHoming();
    sm.toReady();
    sm.toRunning();
    sm.setPhase(RunPhase::DWELL_BOTTOM);
    TEST_ASSERT_EQUAL_STRING("DWELL_BOTTOM", sm.phaseString());
}

void test_phase_string_ascending() {
    sm.toHoming();
    sm.toReady();
    sm.toRunning();
    sm.setPhase(RunPhase::ASCENDING);
    TEST_ASSERT_EQUAL_STRING("ASCENDING", sm.phaseString());
}

void test_phase_string_dwell_top() {
    sm.toHoming();
    sm.toReady();
    sm.toRunning();
    sm.setPhase(RunPhase::DWELL_TOP);
    TEST_ASSERT_EQUAL_STRING("DWELL_TOP", sm.phaseString());
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char** argv) {
    UNITY_BEGIN();

    // Construction
    RUN_TEST(test_initial_state_is_idle);
    RUN_TEST(test_initial_phase_is_none);
    RUN_TEST(test_initial_error_is_none);
    RUN_TEST(test_initial_homed_false);

    // Transitions
    RUN_TEST(test_to_homing_sets_state);
    RUN_TEST(test_to_homing_clears_phase);
    RUN_TEST(test_to_homing_clears_error);
    RUN_TEST(test_to_ready_sets_state);
    RUN_TEST(test_to_ready_sets_homed);
    RUN_TEST(test_to_ready_clears_phase);
    RUN_TEST(test_to_running_sets_state);
    RUN_TEST(test_to_paused_sets_state);
    RUN_TEST(test_to_idle_sets_state);
    RUN_TEST(test_to_idle_clears_phase);
    RUN_TEST(test_to_error_sets_state);
    RUN_TEST(test_to_error_stores_code);
    RUN_TEST(test_to_error_clears_phase);
    RUN_TEST(test_error_recovery_via_homing);

    // Phase transitions
    RUN_TEST(test_set_phase_descending);
    RUN_TEST(test_set_phase_dwell_bottom);
    RUN_TEST(test_set_phase_ascending);
    RUN_TEST(test_set_phase_dwell_top);

    // Guard functions
    RUN_TEST(test_can_run_true_only_in_ready);
    RUN_TEST(test_can_pause_true_only_in_running);
    RUN_TEST(test_can_resume_true_only_in_paused);
    RUN_TEST(test_can_jog_true_only_in_ready);

    // String representations
    RUN_TEST(test_state_string_idle);
    RUN_TEST(test_state_string_homing);
    RUN_TEST(test_state_string_ready);
    RUN_TEST(test_state_string_running);
    RUN_TEST(test_state_string_paused);
    RUN_TEST(test_state_string_error);
    RUN_TEST(test_phase_string_none);
    RUN_TEST(test_phase_string_descending);
    RUN_TEST(test_phase_string_dwell_bottom);
    RUN_TEST(test_phase_string_ascending);
    RUN_TEST(test_phase_string_dwell_top);

    return UNITY_END();
}
