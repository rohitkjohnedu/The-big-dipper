#pragma once
#include <Arduino.h>

/**
 * @file  state_machine.h
 * @brief System state tracking for the dip coater.
 *
 * The StateMachine holds the current operating state (IDLE, HOMING, READY,
 * RUNNING, PAUSED, ERROR), the active run phase (DESCENDING, DWELL_BOTTOM,
 * ASCENDING, DWELL_TOP), and the last error code.
 *
 * State diagram (simplified)
 * --------------------------
 *   IDLE ──HOME──► HOMING ──endstop+backoff──► READY
 *   READY ──RUN──► RUNNING ──complete──► READY
 *   RUNNING ──PAUSE──► PAUSED ──RESUME──► RUNNING
 *   any ──fault──► ERROR ──HOME──► HOMING
 */

// =============================================================================
// Enumerations
// =============================================================================

/** @brief Top-level operating states of the system. */
enum class SystemState : uint8_t {
    IDLE,       ///< Powered on, not homed.  Only HOME is accepted.
    HOMING,     ///< Driving toward the top endstop to zero the position encoder.
    READY,      ///< Homed and stationary.  All commands accepted.
    RUNNING,    ///< Executing a dip profile.
    PAUSED,     ///< Mid-profile pause.  Only RESUME or STOP accepted.
    ERROR,      ///< Fault condition.  Requires HOME or hardware reset to recover.
};

/** @brief Sub-phase within a RUNNING dip profile. */
enum class RunPhase : uint8_t {
    NONE,           ///< Not currently in a profile move.
    DESCENDING,     ///< Moving down toward the solution.
    DWELL_BOTTOM,   ///< Stationary in the solution for the programmed dwell time.
    ASCENDING,      ///< Withdrawing upward from the solution.
    DWELL_TOP,      ///< Stationary at the top position, waiting between dips.
};

/** @brief Fault codes stored when the system transitions to ERROR. */
enum class ErrorCode : uint8_t {
    NONE,
    ENDSTOP_TRIGGERED_UNEXPECTEDLY,
    SOFT_LIMIT_EXCEEDED,
    COMMAND_INVALID_STATE,
    COMMAND_PARSE_ERROR,
    PROFILE_INVALID,
    SEG_BUFFER_OVERFLOW,
};

// =============================================================================
// StateMachine class
// =============================================================================

/**
 * @brief Tracks system state, run phase, and error codes.
 *
 * All state changes go through explicit transition methods so the rest of
 * the firmware never writes _state directly.  This makes valid transitions
 * easy to audit and invalid ones easy to catch.
 */
class StateMachine {
public:

    StateMachine();

    // -------------------------------------------------------------------------
    // Getters
    // -------------------------------------------------------------------------

    /** @brief Returns the current top-level system state. */
    SystemState getState() const;

    /** @brief Returns the current run phase (meaningful only while RUNNING). */
    RunPhase    getPhase() const;

    /** @brief Returns the last recorded error code. */
    ErrorCode   getError() const;

    /** @brief Returns true if the system has been successfully homed at least once. */
    bool        isHomed()  const;

    // -------------------------------------------------------------------------
    // State transitions
    // -------------------------------------------------------------------------

    /** @brief Transition to HOMING — clears phase and error. */
    void toHoming();

    /** @brief Transition to READY — sets the homed flag. */
    void toReady();

    /** @brief Transition to RUNNING. */
    void toRunning();

    /** @brief Transition to PAUSED. */
    void toPaused();

    /** @brief Transition to IDLE. */
    void toIdle();

    /**
     * @brief Transition to ERROR and record the fault code.
     * @param code  The ErrorCode describing the fault.
     */
    void toError(ErrorCode code);

    /** @brief Update the run phase without changing the top-level state. */
    void setPhase(RunPhase phase);

    // -------------------------------------------------------------------------
    // Capability queries  (used by CommandParser for pre-condition checks)
    // -------------------------------------------------------------------------

    /** @brief Returns true if a dip profile can be started (state == READY). */
    bool canRun()    const;

    /** @brief Returns true if the profile can be paused (state == RUNNING). */
    bool canPause()  const;

    /** @brief Returns true if the profile can be resumed (state == PAUSED). */
    bool canResume() const;

    /** @brief Returns true if jogging is allowed (state == READY). */
    bool canJog()    const;

    // -------------------------------------------------------------------------
    // String helpers  (for CMD GET_STATE and serial diagnostics)
    // -------------------------------------------------------------------------

    /** @brief Returns a human-readable string for the current state. */
    const char* stateString() const;

    /** @brief Returns a human-readable string for the current run phase. */
    const char* phaseString() const;

private:

    SystemState _state;
    RunPhase    _phase;
    ErrorCode   _error;
    bool        _homed;
};
