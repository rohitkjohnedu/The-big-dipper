// =============================================================================
// test_motion_controller.cpp
//
// Native unit tests for MotionController.
//
// All hardware calls are compiled out by #ifdef ARDUINO guards, so:
//   - isMoveComplete()    always returns false  (motor never "arrives")
//   - isDwellComplete()   always returns true   (millis() not available)
//   - getPositionMm()     always returns 0.0f
//   - startMoveToMm()     sets _commandedVelocityMms and _targetMm, no HW call
//
// Tests therefore focus on: state/phase transitions, segment buffer management,
// parameter capture, and guard logic.
//
// Run with:
//   PATH="/c/msys64/mingw64/bin:$PATH" ~/.platformio/penv/Scripts/pio.exe test -e native
// =============================================================================

#include <unity.h>
#include "state_machine.h"
#include "motion_controller.h"

static StateMachine*    sm;
static MotionController* mc;

void setUp() {
    sm = new StateMachine();
    mc = new MotionController(*sm);
}

void tearDown() {
    delete mc;
    delete sm;
    mc = nullptr;
    sm = nullptr;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Advance sm to READY (homed = true).
static void bringToReady() {
    sm->toHoming();
    sm->toReady();
}

// Advance sm to RUNNING (no profile loaded — just the state).
static void bringToRunning() {
    bringToReady();
    sm->toRunning();
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

void test_constructor_commanded_velocity_zero() {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mc->getCommandedVelocityMms());
}

void test_constructor_position_zero() {
    // Returns 0.0f on native (no hardware)
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mc->getPositionMm());
}

void test_constructor_accel_zero() {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mc->getActualAccelMms2());
}

void test_constructor_state_idle() {
    TEST_ASSERT_EQUAL(SystemState::IDLE, sm->getState());
}

// ---------------------------------------------------------------------------
// Segment buffer
// ---------------------------------------------------------------------------

void test_add_segment_returns_true_until_buffer_full() {
    mc->beginSegmentedMove(1, 100, 500);
    for (int i = 0; i < MOVE_SEG_BUFFER_SIZE; i++) {
        TEST_ASSERT_TRUE(mc->addSegment(10.0f, 5.0f));
    }
    TEST_ASSERT_FALSE(mc->addSegment(10.0f, 5.0f));  // one past the limit
}

void test_begin_segmented_move_resets_buffer() {
    mc->beginSegmentedMove(1, 100, 500);
    // Fill to capacity
    for (int i = 0; i < MOVE_SEG_BUFFER_SIZE; i++) {
        mc->addSegment(1.0f, 1.0f);
    }
    TEST_ASSERT_FALSE(mc->addSegment(1.0f, 1.0f));  // full

    // Reset with a new call
    mc->beginSegmentedMove(2, 200, 300);

    // Buffer should be empty again
    for (int i = 0; i < MOVE_SEG_BUFFER_SIZE; i++) {
        TEST_ASSERT_TRUE(mc->addSegment(1.0f, 1.0f));
    }
    TEST_ASSERT_FALSE(mc->addSegment(1.0f, 1.0f));
}

// ---------------------------------------------------------------------------
// runProfile()
// ---------------------------------------------------------------------------

void test_run_profile_transitions_to_running() {
    bringToReady();
    mc->runProfile(10.0f, 8.0f, 50.0f, 100.0f, 1000, 500, 3);
    TEST_ASSERT_EQUAL(SystemState::RUNNING, sm->getState());
}

void test_run_profile_sets_phase_descending() {
    bringToReady();
    mc->runProfile(10.0f, 8.0f, 50.0f, 100.0f, 1000, 500, 3);
    TEST_ASSERT_EQUAL(RunPhase::DESCENDING, sm->getPhase());
}

void test_run_profile_captures_dip_speed_as_commanded_velocity() {
    bringToReady();
    mc->runProfile(15.0f, 8.0f, 50.0f, 100.0f, 1000, 500, 1);
    TEST_ASSERT_EQUAL_FLOAT(15.0f, mc->getCommandedVelocityMms());
}

// ---------------------------------------------------------------------------
// runLoadedMove()
// ---------------------------------------------------------------------------

void test_run_loaded_move_transitions_to_running() {
    bringToReady();
    mc->beginSegmentedMove(1, 100, 500);
    mc->addSegment(50.0f, 10.0f);
    mc->addSegment(-50.0f, 8.0f);
    mc->runLoadedMove();
    TEST_ASSERT_EQUAL(SystemState::RUNNING, sm->getState());
}

void test_run_loaded_move_sets_phase_descending() {
    bringToReady();
    mc->beginSegmentedMove(1, 100, 500);
    mc->addSegment(50.0f, 10.0f);
    mc->runLoadedMove();
    TEST_ASSERT_EQUAL(RunPhase::DESCENDING, sm->getPhase());
}

void test_run_loaded_move_empty_buffer_does_nothing() {
    bringToReady();
    mc->beginSegmentedMove(1, 100, 500);
    // No segments added
    mc->runLoadedMove();
    // Should stay READY (no transition occurred)
    TEST_ASSERT_EQUAL(SystemState::READY, sm->getState());
}

void test_run_loaded_move_captures_first_segment_speed() {
    bringToReady();
    mc->beginSegmentedMove(1, 100, 500);
    mc->addSegment(50.0f, 12.5f);
    mc->runLoadedMove();
    TEST_ASSERT_EQUAL_FLOAT(12.5f, mc->getCommandedVelocityMms());
}

// ---------------------------------------------------------------------------
// stop()
// ---------------------------------------------------------------------------

void test_stop_transitions_to_ready() {
    bringToRunning();
    mc->stop();
    TEST_ASSERT_EQUAL(SystemState::READY, sm->getState());
}

void test_stop_clears_commanded_velocity() {
    bringToReady();
    mc->runProfile(10.0f, 8.0f, 50.0f, 100.0f, 1000, 500, 1);
    mc->stop();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mc->getCommandedVelocityMms());
}

// ---------------------------------------------------------------------------
// estop()
// ---------------------------------------------------------------------------

void test_estop_from_idle_raises_error() {
    mc->estop();
    TEST_ASSERT_EQUAL(SystemState::ERROR, sm->getState());
}

void test_estop_from_running_raises_error() {
    bringToRunning();
    mc->estop();
    TEST_ASSERT_EQUAL(SystemState::ERROR, sm->getState());
}

void test_estop_clears_commanded_velocity() {
    bringToReady();
    mc->runProfile(10.0f, 8.0f, 50.0f, 100.0f, 1000, 500, 1);
    mc->estop();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mc->getCommandedVelocityMms());
}

// ---------------------------------------------------------------------------
// pause()
// ---------------------------------------------------------------------------

void test_pause_transitions_to_paused() {
    bringToRunning();
    mc->pause();
    TEST_ASSERT_EQUAL(SystemState::PAUSED, sm->getState());
}

void test_pause_preserves_phase_in_snapshot() {
    bringToReady();
    mc->runProfile(10.0f, 8.0f, 50.0f, 100.0f, 1000, 500, 1);
    // phase is DESCENDING after runProfile
    TEST_ASSERT_EQUAL(RunPhase::DESCENDING, sm->getPhase());
    mc->pause();
    // phase is still DESCENDING (pause doesn't clear it)
    TEST_ASSERT_EQUAL(RunPhase::DESCENDING, sm->getPhase());
}

// ---------------------------------------------------------------------------
// onEndstopTriggered()
// ---------------------------------------------------------------------------

void test_endstop_bottom_not_homing_raises_error() {
    // isTop=false while not in HOMING mode → unexpected endstop → ERROR
    mc->onEndstopTriggered(false);
    TEST_ASSERT_EQUAL(SystemState::ERROR, sm->getState());
}

void test_endstop_top_not_homing_raises_error() {
    // isTop=true but mode is not HOMING (it's NONE after construction) → ERROR
    mc->onEndstopTriggered(true);
    TEST_ASSERT_EQUAL(SystemState::ERROR, sm->getState());
}

void test_endstop_bottom_while_running_raises_error() {
    bringToRunning();
    mc->onEndstopTriggered(false);
    TEST_ASSERT_EQUAL(SystemState::ERROR, sm->getState());
}

// ---------------------------------------------------------------------------
// update() in NONE mode
// ---------------------------------------------------------------------------

void test_update_none_mode_does_not_change_state() {
    mc->update();
    TEST_ASSERT_EQUAL(SystemState::IDLE, sm->getState());
}

// ---------------------------------------------------------------------------
// Misc — no crash guards
// ---------------------------------------------------------------------------

void test_set_soft_limits_does_not_crash() {
    mc->setSoftLimits(5.0f, 800.0f);
    TEST_PASS();
}

void test_set_debug_on_does_not_crash() {
    mc->setDebug(true);
    mc->setDebug(false);
    TEST_PASS();
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    UNITY_BEGIN();

    // Construction
    RUN_TEST(test_constructor_commanded_velocity_zero);
    RUN_TEST(test_constructor_position_zero);
    RUN_TEST(test_constructor_accel_zero);
    RUN_TEST(test_constructor_state_idle);

    // Segment buffer
    RUN_TEST(test_add_segment_returns_true_until_buffer_full);
    RUN_TEST(test_begin_segmented_move_resets_buffer);

    // runProfile
    RUN_TEST(test_run_profile_transitions_to_running);
    RUN_TEST(test_run_profile_sets_phase_descending);
    RUN_TEST(test_run_profile_captures_dip_speed_as_commanded_velocity);

    // runLoadedMove
    RUN_TEST(test_run_loaded_move_transitions_to_running);
    RUN_TEST(test_run_loaded_move_sets_phase_descending);
    RUN_TEST(test_run_loaded_move_empty_buffer_does_nothing);
    RUN_TEST(test_run_loaded_move_captures_first_segment_speed);

    // stop
    RUN_TEST(test_stop_transitions_to_ready);
    RUN_TEST(test_stop_clears_commanded_velocity);

    // estop
    RUN_TEST(test_estop_from_idle_raises_error);
    RUN_TEST(test_estop_from_running_raises_error);
    RUN_TEST(test_estop_clears_commanded_velocity);

    // pause
    RUN_TEST(test_pause_transitions_to_paused);
    RUN_TEST(test_pause_preserves_phase_in_snapshot);

    // onEndstopTriggered
    RUN_TEST(test_endstop_bottom_not_homing_raises_error);
    RUN_TEST(test_endstop_top_not_homing_raises_error);
    RUN_TEST(test_endstop_bottom_while_running_raises_error);

    // update
    RUN_TEST(test_update_none_mode_does_not_change_state);

    // Misc
    RUN_TEST(test_set_soft_limits_does_not_crash);
    RUN_TEST(test_set_debug_on_does_not_crash);

    return UNITY_END();
}
