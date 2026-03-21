#pragma once
#include <Arduino.h>
#include "config.h"
#include "state_machine.h"
#include "motion_controller.h"
#include "diagnostics.h"

class Telemetry;   // forward declaration — implemented in step 7

/**
 * @file  command_parser.h
 * @brief Serial command parser for the dip coater.
 *
 * CommandParser owns all serial I/O.  Call begin() once from setup() and
 * update() on every loop() iteration.  It is non-blocking — it never calls
 * delay().
 *
 * Command namespaces
 * ------------------
 *   CMD <verb> [args]        — production protocol used by the Python UI
 *   DIAG <verb> [args]       — hardware diagnostics for manual serial testing
 *   HELP  |  DIAG CMD        — print the full command list
 *
 * Every CMD command produces exactly one ACK or ERR response line.
 *
 * Segmented move handshake
 * ------------------------
 *   CMD BEGIN_SEGMENTED_MOVE <n_segs> <n_dips> <dwell_bot_ms> <dwell_top_ms>
 *   CMD MOVE_SEG <dist_mm> <speed_mm_s>    (repeat n_segs times)
 *   CMD RUN_LOADED_MOVE
 *
 * The parser enters segment-collection mode after BEGIN_SEGMENTED_MOVE and
 * sends ACK PROFILE_READY once all n_segs segments have been received.
 * Only MOVE_SEG and ESTOP are accepted while collecting segments.
 */
class CommandParser {
public:

    /**
     * @brief Construct with references to the shared subsystems.
     * @param sm   Shared state machine.
     * @param mc   Shared motion controller.
     * @param diag Shared diagnostics module.
     */
    CommandParser(StateMachine& sm, MotionController& mc, Diagnostics& diag);

    /** @brief Initialise the serial port.  Call once from setup(). */
    void begin();

    /** @brief Read and dispatch serial input.  Call every loop(). */
    void update();

    // -------------------------------------------------------------------------
    // Telemetry wiring (step 7)
    // -------------------------------------------------------------------------

    /** @brief Wire the Telemetry module once it is available (step 7). */
    void    setTelemetry(Telemetry* telem);

    /** @brief Returns the current telemetry rate (Hz, 0 = off). */
    uint8_t telemRateHz() const { return _telemRateHz; }

private:

    // -------------------------------------------------------------------------
    // Members
    // -------------------------------------------------------------------------

    StateMachine&     _sm;
    MotionController& _mc;
    Diagnostics&      _diag;
    Telemetry*        _telem;

    char    _buf[SERIAL_BUFFER_SIZE];
    uint8_t _len;
    bool    _bufOverflow;   ///< Set when a line exceeded SERIAL_BUFFER_SIZE; cleared on newline
    uint8_t _telemRateHz;

    // Segmented move collection state
    bool    _collectingSegs;
    uint8_t _segTotal;
    uint8_t _segReceived;

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /** @brief Route a complete line to the CMD or DIAG dispatcher. */
    void dispatch(char* line);

    /** @brief Dispatch a CMD verb+args string. */
    void dispatchCmd(char* args);

    /** @brief Dispatch a DIAG command line. */
    void dispatchDiag(char* line);

    /** @brief Print the full command reference to Serial. */
    void printHelp();

    // CMD handlers — each receives a pointer to the remainder of the argument string
    void cmdHome();
    void cmdGetState();
    void cmdStop();
    void cmdEstop();
    void cmdPause();
    void cmdResume();
    void cmdJog(char* p);
    void cmdRunProfile(char* p);
    void cmdBeginSegmentedMove(char* p);
    void cmdMoveSeg(char* p);
    void cmdDwellSeg(char* p);
    void cmdMove(char* p);
    void cmdRunLoadedMove();
    void cmdSetTelemRate(char* p);
    void cmdSetSoftLimits(char* p);

    // Response helpers
    void ack(const char* cmd);
    void err(const char* cmd, const char* reason);
};
