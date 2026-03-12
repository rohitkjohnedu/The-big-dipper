#pragma once

#include <Arduino.h>
#include "config.h"
#include "state_machine.h"

// Forward declarations — defined in their respective headers (steps 4 & 5)
class MotionController;
class Telemetry;

// =============================================================================
// CommandParser
//
// Reads the serial buffer one character at a time in update() (called every
// loop()). On '\n', tokenises the accumulated line and dispatches to the
// appropriate handler. Never blocks. Returns ACK or ERR on every command.
//
// Segment-collection mode: after BEGIN_SEGMENTED_MOVE, the parser collects
// exactly <n_segments> CMD MOVE_SEG lines, then emits ACK PROFILE_READY.
// Only MOVE_SEG is accepted during collection; any other command gets ERR.
// =============================================================================
class CommandParser {
public:
    CommandParser(StateMachine& sm, MotionController& mc, Telemetry& telem);

    // Non-blocking. Call every loop().
    void update();

private:
    StateMachine&     _sm;
    MotionController& _mc;
    Telemetry&        _telem;

    char    _buf[SERIAL_BUFFER_SIZE];
    uint8_t _bufLen;

    // Segmented-move collection state
    bool     _collectingSegs;
    uint8_t  _segsExpected;
    uint8_t  _segsReceived;
    uint8_t  _segNDips;
    uint16_t _segDwellBottomMs;
    uint16_t _segDwellTopMs;

    void processLine();
    void dispatch(char** tokens, uint8_t nTokens);

    // Command handlers
    void handleHome();
    void handleGetState();
    void handleSetTelemRate(char** tokens, uint8_t nTokens);
    void handleSetSoftLimits(char** tokens, uint8_t nTokens);
    void handleDebug(char** tokens, uint8_t nTokens);
    void handleStop();
    void handleEstop();
    void handlePause();
    void handleResume();
    void handleJog(char** tokens, uint8_t nTokens);
    void handleRunProfile(char** tokens, uint8_t nTokens);
    void handleBeginSegmentedMove(char** tokens, uint8_t nTokens);
    void handleMoveSeg(char** tokens, uint8_t nTokens);
    void handleRunLoadedMove();

    void sendAck(const char* cmd);
    void sendErr(const char* cmd, const char* reason);
};
