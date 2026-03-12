#include "command_parser.h"
#include "motion_controller.h"   // written in step 4
#include "telemetry.h"           // written in step 5

// =============================================================================
// Constructor
// =============================================================================

CommandParser::CommandParser(StateMachine& sm, MotionController& mc, Telemetry& telem)
    : _sm(sm)
    , _mc(mc)
    , _telem(telem)
    , _bufLen(0)
    , _collectingSegs(false)
    , _segsExpected(0)
    , _segsReceived(0)
    , _segNDips(0)
    , _segDwellBottomMs(0)
    , _segDwellTopMs(0)
{}

// =============================================================================
// update() — called every loop(). Non-blocking.
// =============================================================================

void CommandParser::update() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            _buf[_bufLen] = '\0';
            processLine();
            _bufLen = 0;
        } else if (_bufLen < SERIAL_BUFFER_SIZE - 1) {
            _buf[_bufLen++] = c;
        }
        // If buffer fills without a newline: silently drop the character.
        // The line will fail tokenisation and get ERR — no silent hang.
    }
}

// =============================================================================
// processLine() — tokenise and dispatch
// =============================================================================

void CommandParser::processLine() {
    // Strip trailing \r (Windows line endings)
    if (_bufLen > 0 && _buf[_bufLen - 1] == '\r') {
        _buf[--_bufLen] = '\0';
    }
    if (_bufLen == 0) return;

    // Tokenise into at most 16 fields (enough for any command)
    char*   tokens[16];
    uint8_t nTokens = 0;
    char*   tok = strtok(_buf, " ");
    while (tok != nullptr && nTokens < 16) {
        tokens[nTokens++] = tok;
        tok = strtok(nullptr, " ");
    }

    if (nTokens < 2) {
        sendErr("UNKNOWN", "too_few_tokens");
        return;
    }
    if (strcmp(tokens[0], "CMD") != 0) {
        sendErr("UNKNOWN", "missing_CMD_prefix");
        return;
    }

    dispatch(tokens, nTokens);
}

// =============================================================================
// dispatch() — route to the correct handler
// =============================================================================

void CommandParser::dispatch(char** tokens, uint8_t nTokens) {
    const char* cmd = tokens[1];

    // In segment-collection mode only MOVE_SEG is accepted.
    if (_collectingSegs) {
        if (strcmp(cmd, "MOVE_SEG") == 0) {
            handleMoveSeg(tokens, nTokens);
        } else {
            sendErr(cmd, "expected_MOVE_SEG");
        }
        return;
    }

    if      (strcmp(cmd, "HOME")                 == 0) handleHome();
    else if (strcmp(cmd, "GET_STATE")            == 0) handleGetState();
    else if (strcmp(cmd, "SET_TELEM_RATE")       == 0) handleSetTelemRate(tokens, nTokens);
    else if (strcmp(cmd, "SET_SOFT_LIMITS")      == 0) handleSetSoftLimits(tokens, nTokens);
    else if (strcmp(cmd, "DEBUG")                == 0) handleDebug(tokens, nTokens);
    else if (strcmp(cmd, "STOP")                 == 0) handleStop();
    else if (strcmp(cmd, "ESTOP")                == 0) handleEstop();
    else if (strcmp(cmd, "PAUSE")                == 0) handlePause();
    else if (strcmp(cmd, "RESUME")               == 0) handleResume();
    else if (strcmp(cmd, "JOG")                  == 0) handleJog(tokens, nTokens);
    else if (strcmp(cmd, "RUN_PROFILE")          == 0) handleRunProfile(tokens, nTokens);
    else if (strcmp(cmd, "BEGIN_SEGMENTED_MOVE") == 0) handleBeginSegmentedMove(tokens, nTokens);
    else if (strcmp(cmd, "RUN_LOADED_MOVE")      == 0) handleRunLoadedMove();
    else {
        sendErr(cmd, "unknown_command");
        _sm.toError(ErrorCode::COMMAND_PARSE_ERROR);
    }
}

// =============================================================================
// Command handlers
// =============================================================================

void CommandParser::handleHome() {
    SystemState s = _sm.getState();
    if (s != SystemState::IDLE && s != SystemState::READY && s != SystemState::ERROR) {
        sendErr("HOME", "invalid_state");
        _sm.toError(ErrorCode::COMMAND_INVALID_STATE);
        return;
    }
    _sm.toHoming();
    _mc.executeHome();
    sendAck("HOME");
}

void CommandParser::handleGetState() {
    Serial.print("STATE ");
    Serial.print(_sm.stateString());
    Serial.print(" ");
    Serial.println(_sm.phaseString());
}

void CommandParser::handleSetTelemRate(char** tokens, uint8_t nTokens) {
    if (nTokens < 3) { sendErr("SET_TELEM_RATE", "missing_hz"); return; }
    int hz = atoi(tokens[2]);
    if (hz < 0 || hz > MAX_TELEM_RATE_HZ) {
        sendErr("SET_TELEM_RATE", "hz_out_of_range");
        return;
    }
    _telem.setRate(hz);
    sendAck("SET_TELEM_RATE");
}

void CommandParser::handleSetSoftLimits(char** tokens, uint8_t nTokens) {
    if (nTokens < 4) { sendErr("SET_SOFT_LIMITS", "missing_args"); return; }
    float minMm = atof(tokens[2]);
    float maxMm = atof(tokens[3]);
    if (minMm < 0.0f || maxMm > TRAVEL_MAX_MM || minMm >= maxMm) {
        sendErr("SET_SOFT_LIMITS", "limits_out_of_range");
        return;
    }
    _mc.setSoftLimits(minMm, maxMm);
    sendAck("SET_SOFT_LIMITS");
}

void CommandParser::handleDebug(char** tokens, uint8_t nTokens) {
    if (nTokens < 3) { sendErr("DEBUG", "missing_arg"); return; }
    bool on = (strcmp(tokens[2], "ON") == 0);
    _mc.setDebug(on);
    sendAck("DEBUG");
}

void CommandParser::handleStop() {
    SystemState s = _sm.getState();
    if (s != SystemState::RUNNING && s != SystemState::PAUSED) {
        sendErr("STOP", "invalid_state");
        _sm.toError(ErrorCode::COMMAND_INVALID_STATE);
        return;
    }
    _mc.stop();
    sendAck("STOP");
}

void CommandParser::handleEstop() {
    // ESTOP is accepted in any state — no guard.
    _mc.estop();
    sendAck("ESTOP");
}

void CommandParser::handlePause() {
    if (!_sm.canPause()) {
        sendErr("PAUSE", "invalid_state");
        _sm.toError(ErrorCode::COMMAND_INVALID_STATE);
        return;
    }
    _mc.pause();
    sendAck("PAUSE");
}

void CommandParser::handleResume() {
    if (!_sm.canResume()) {
        sendErr("RESUME", "invalid_state");
        _sm.toError(ErrorCode::COMMAND_INVALID_STATE);
        return;
    }
    _mc.resume();
    sendAck("RESUME");
}

void CommandParser::handleJog(char** tokens, uint8_t nTokens) {
    if (!_sm.canJog()) {
        sendErr("JOG", "invalid_state");
        _sm.toError(ErrorCode::COMMAND_INVALID_STATE);
        return;
    }
    if (nTokens < 4) { sendErr("JOG", "missing_args"); return; }
    if (strcmp(tokens[2], "UP") != 0 && strcmp(tokens[2], "DOWN") != 0) {
        sendErr("JOG", "direction_must_be_UP_or_DOWN");
        return;
    }
    bool  up    = (strcmp(tokens[2], "UP") == 0);
    float speed = atof(tokens[3]);
    if (speed <= 0.0f) { sendErr("JOG", "speed_must_be_positive"); return; }
    _mc.jog(up, speed);
    sendAck("JOG");
}

void CommandParser::handleRunProfile(char** tokens, uint8_t nTokens) {
    if (!_sm.canRun()) {
        sendErr("RUN_PROFILE", "invalid_state");
        _sm.toError(ErrorCode::COMMAND_INVALID_STATE);
        return;
    }
    // CMD RUN_PROFILE dipSpeed withdrawSpeed accel depth dwellBottom dwellTop nDips
    //  [0]    [1]       [2]        [3]        [4]   [5]     [6]         [7]    [8]
    if (nTokens < 9) { sendErr("RUN_PROFILE", "missing_args"); return; }

    float dipSpeed      = atof(tokens[2]);
    float withdrawSpeed = atof(tokens[3]);
    float accel         = atof(tokens[4]);
    float depth         = atof(tokens[5]);
    int   dwellBottom   = atoi(tokens[6]);
    int   dwellTop      = atoi(tokens[7]);
    int   nDips         = atoi(tokens[8]);

    if (dipSpeed <= 0.0f || withdrawSpeed <= 0.0f || accel <= 0.0f ||
        depth <= 0.0f || nDips < 1) {
        sendErr("RUN_PROFILE", "invalid_params");
        _sm.toError(ErrorCode::PROFILE_INVALID);
        return;
    }
    _mc.runProfile(dipSpeed, withdrawSpeed, accel, depth, dwellBottom, dwellTop, nDips);
    sendAck("RUN_PROFILE");
}

void CommandParser::handleBeginSegmentedMove(char** tokens, uint8_t nTokens) {
    if (!_sm.canRun()) {
        sendErr("BEGIN_SEGMENTED_MOVE", "invalid_state");
        _sm.toError(ErrorCode::COMMAND_INVALID_STATE);
        return;
    }
    // CMD BEGIN_SEGMENTED_MOVE nSegs nDips dwellBottom dwellTop
    //  [0]        [1]           [2]   [3]     [4]        [5]
    if (nTokens < 6) { sendErr("BEGIN_SEGMENTED_MOVE", "missing_args"); return; }

    uint8_t nSegs = (uint8_t)atoi(tokens[2]);
    if (nSegs == 0 || nSegs > MOVE_SEG_BUFFER_SIZE) {
        sendErr("BEGIN_SEGMENTED_MOVE", "segment_count_out_of_range");
        _sm.toError(ErrorCode::SEG_BUFFER_OVERFLOW);
        return;
    }

    _segNDips         = (uint8_t)atoi(tokens[3]);
    _segDwellBottomMs = (uint16_t)atoi(tokens[4]);
    _segDwellTopMs    = (uint16_t)atoi(tokens[5]);
    _segsExpected     = nSegs;
    _segsReceived     = 0;
    _collectingSegs   = true;

    _mc.beginSegmentedMove(_segNDips, _segDwellBottomMs, _segDwellTopMs);
    sendAck("BEGIN_SEGMENTED_MOVE");
}

void CommandParser::handleMoveSeg(char** tokens, uint8_t nTokens) {
    // CMD MOVE_SEG distance_mm speed_mm_s
    //  [0]   [1]      [2]         [3]
    if (nTokens < 4) {
        sendErr("MOVE_SEG", "missing_args");
        _collectingSegs = false;
        return;
    }
    float distMm  = atof(tokens[2]);
    float speedMms = atof(tokens[3]);

    if (!_mc.addSegment(distMm, speedMms)) {
        // Protocol specifies this error as "ERR SEG_BUFFER_OVERFLOW" (no extra field)
        Serial.println("ERR SEG_BUFFER_OVERFLOW");
        _sm.toError(ErrorCode::SEG_BUFFER_OVERFLOW);
        _collectingSegs = false;
        return;
    }

    _segsReceived++;
    if (_segsReceived == _segsExpected) {
        // All segments received — signal Python that RUN_LOADED_MOVE may be sent
        _collectingSegs = false;
        Serial.println("ACK PROFILE_READY");
    }
    // Individual MOVE_SEG lines are not ACKed — only ACK PROFILE_READY at the end.
}

void CommandParser::handleRunLoadedMove() {
    if (!_sm.canRun()) {
        sendErr("RUN_LOADED_MOVE", "invalid_state");
        _sm.toError(ErrorCode::COMMAND_INVALID_STATE);
        return;
    }
    _mc.runLoadedMove();
    sendAck("RUN_LOADED_MOVE");
}

// =============================================================================
// Response helpers
// =============================================================================

void CommandParser::sendAck(const char* cmd) {
    Serial.print("ACK ");
    Serial.println(cmd);
}

void CommandParser::sendErr(const char* cmd, const char* reason) {
    Serial.print("ERR ");
    Serial.print(cmd);
    if (reason[0] != '\0') {
        Serial.print(" ");
        Serial.print(reason);
    }
    Serial.println();
}
