#pragma once
#include <Arduino.h>
#include "config.h"
#include "state_machine.h"
#include "motion_controller.h"
#include "diagnostics.h"

class Telemetry;   // forward declaration — implemented in step 7

// =============================================================================
// CommandParser
//
// Owns all serial I/O. Call begin() from setup(), update() from every loop().
// Non-blocking — never calls delay().
//
// Handles two command namespaces:
//   CMD <verb> [args]  — production protocol (Python UI)
//   DIAG <verb> [args] — hardware diagnostics (manual serial)
//   HELP               — print command list
//
// Every CMD command produces exactly one ACK or ERR response.
//
// Segmented move handshake:
//   CMD BEGIN_SEGMENTED_MOVE <n_segs> <n_dips> <dwell_bot_ms> <dwell_top_ms>
//   CMD MOVE_SEG <dist_mm> <speed_mm_s>    (repeat n_segs times)
//   CMD RUN_LOADED_MOVE
// Parser enters segment-collection mode after BEGIN_SEGMENTED_MOVE and sends
// ACK PROFILE_READY once all segments are received.
// =============================================================================

class CommandParser {
public:
    CommandParser(StateMachine& sm, MotionController& mc, Diagnostics& diag);

    void begin();    // call once from setup()
    void update();   // call every loop() — owns all serial I/O

    // Wire to Telemetry once that module exists (step 7)
    void    setTelemetry(Telemetry* telem);
    uint8_t telemRateHz() const { return _telemRateHz; }

private:
    StateMachine&     _sm;
    MotionController& _mc;
    Diagnostics&      _diag;
    Telemetry*        _telem;

    char    _buf[SERIAL_BUFFER_SIZE];
    uint8_t _len;
    uint8_t _telemRateHz;

    // Segmented move collection state
    bool    _collectingSegs;
    uint8_t _segTotal;
    uint8_t _segReceived;

    void dispatch(char* line);
    void dispatchCmd(char* args);
    void dispatchDiag(char* line);
    void printHelp();

    // CMD handlers — receive pointer to remainder of argument string
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
    void cmdRunLoadedMove();
    void cmdSetTelemRate(char* p);
    void cmdSetSoftLimits(char* p);
    void cmdDebug(char* p);

    // Response helpers
    void ack(const char* cmd);
    void err(const char* cmd, const char* reason);
};
