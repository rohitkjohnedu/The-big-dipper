#include "command_parser.h"

// =============================================================================
// Number parsing helpers — avoid sscanf (unreliable on STM32)
// =============================================================================

/**
 * @brief Advance past spaces in *p, then parse and return a float.
 *
 * Uses strtof() which advances the pointer to the first character after the
 * number, making it safe to call repeatedly on the same argument string to
 * extract consecutive values without manual pointer arithmetic.
 */
static float nextFloat(char*& p) {
    while (*p == ' ') p++;     // skip leading whitespace
    char* end;
    float v = strtof(p, &end);
    if (end == p) return NAN;  // nothing was parsed — no digits at current position
    p = end;                   // advance past the parsed token
    return v;
}

/** @brief Advance past spaces in *p, then parse and return a long integer. */
static long nextLong(char*& p) {
    while (*p == ' ') p++;
    char* end;
    long v = strtol(p, &end, 10);
    p = end;
    return v;
}

// =============================================================================
// Constructor / begin
// =============================================================================

CommandParser::CommandParser(StateMachine& sm, MotionController& mc, Diagnostics& diag)
    : _sm(sm)
    , _mc(mc)
    , _diag(diag)
    , _telem(nullptr)
    , _len(0)
    , _bufOverflow(false)
    , _telemRateHz(DEFAULT_TELEM_RATE_HZ)
    , _collectingSegs(false)
    , _segTotal(0)
    , _segReceived(0)
{}

void CommandParser::begin() {
    _len            = 0;
    _collectingSegs = false;
}

void CommandParser::setTelemetry(Telemetry* telem) {
    _telem = telem;
}

// =============================================================================
// update() — call every loop()
// =============================================================================

void CommandParser::update() {
    // Accumulate characters into _buf until a newline is received.
    // The buffer is null-terminated and dispatched as a complete line.
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\n') {
            // End of line — null-terminate and strip trailing CR if present
            // (handles both \n and \r\n line endings from different terminals).
            _buf[_len] = '\0';
            if (_len > 0 && _buf[_len - 1] == '\r') _buf[--_len] = '\0';

            if (_bufOverflow) {
                // The line was longer than SERIAL_BUFFER_SIZE — the buffer holds
                // only a truncated prefix, which would parse incorrectly.
                // Report the error and discard the whole line.
                Serial.println("ERR CMD line_too_long");
                _bufOverflow = false;
            } else if (_len > 0) {
                dispatch(_buf);
            }
            _len = 0;

        } else if (_len < (uint8_t)(sizeof(_buf) - 1)) {
            // Normal character — append to buffer.
            _buf[_len++] = c;
        } else {
            // Buffer full — mark overflow so the line is discarded on newline.
            _bufOverflow = true;
        }
    }
}

// =============================================================================
// Response helpers
// =============================================================================

void CommandParser::ack(const char* cmd) {
    // Every successful CMD command produces exactly one ACK response.
    Serial.print("ACK ");
    Serial.println(cmd);
}

void CommandParser::err(const char* cmd, const char* reason) {
    // Every rejected CMD command produces exactly one ERR response.
    // The format is: ERR <command> <reason_code>
    Serial.print("ERR ");
    Serial.print(cmd);
    Serial.print(" ");
    Serial.println(reason);
}

// =============================================================================
// Top-level dispatch
// =============================================================================

void CommandParser::dispatch(char* line) {
    // Strip trailing whitespace (spaces or stray CR characters).
    int len = (int)strlen(line);
    while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\r'))
        line[--len] = '\0';

    // Route to the appropriate namespace handler.
    if (strcmp(line, "HELP") == 0 || strcmp(line, "DIAG CMD") == 0) {
        printHelp();
        return;
    }
    if (strncmp(line, "DIAG", 4) == 0) { dispatchDiag(line);      return; }
    if (strncmp(line, "CMD ",  4) == 0) { dispatchCmd(line + 4);   return; }

    // Unknown prefix — report and discard.
    Serial.print("ERR unknown_command: ");
    Serial.println(line);
}

// =============================================================================
// CMD dispatch
// =============================================================================

void CommandParser::dispatchCmd(char* args) {
    // ---- Segment collection mode guard -------------------------------------
    // After CMD BEGIN_SEGMENTED_MOVE the parser enters a collection mode
    // where only MOVE_SEG and ESTOP are valid.  Any other command aborts
    // the collection and reports an error.
    if (_collectingSegs) {
        if (strncmp(args, "MOVE_SEG ",  9) == 0) { cmdMoveSeg(args + 9);  return; }
        if (strncmp(args, "DWELL_SEG ", 10)== 0) { cmdDwellSeg(args + 10);return; }
        if (strcmp(args,  "ESTOP")      == 0)    { cmdEstop();             return; }
        _collectingSegs = false;
        err("CMD", "segment_collection_aborted — only MOVE_SEG, DWELL_SEG or ESTOP accepted");
        return;
    }

    // ---- Zero-argument commands ---------------------------------------------
    if (strcmp(args, "HOME")           == 0) { cmdHome();          return; }
    if (strcmp(args, "GET_STATE")      == 0) { cmdGetState();      return; }
    if (strcmp(args, "STOP")           == 0) { cmdStop();          return; }
    if (strcmp(args, "ESTOP")          == 0) { cmdEstop();         return; }
    if (strcmp(args, "PAUSE")          == 0) { cmdPause();         return; }
    if (strcmp(args, "RESUME")         == 0) { cmdResume();        return; }
    if (strcmp(args, "RUN_LOADED_MOVE")== 0) { cmdRunLoadedMove(); return; }

    // ---- Commands with arguments --------------------------------------------
    // strncmp matches the keyword prefix; the handler receives the pointer
    // past the keyword+space so it can parse arguments with nextFloat/nextLong.
    if (strncmp(args, "MOVE ",                 5)  == 0) { cmdMove(args + 5);                return; }
    if (strncmp(args, "JOG ",                  4)  == 0) { cmdJog(args + 4);                 return; }
    if (strncmp(args, "RUN_PROFILE ",         12)  == 0) { cmdRunProfile(args + 12);         return; }
    if (strncmp(args, "BEGIN_SEGMENTED_MOVE ", 21) == 0) { cmdBeginSegmentedMove(args + 21); return; }
    if (strncmp(args, "MOVE_SEG ",             9)  == 0) { cmdMoveSeg(args + 9);             return; }
    if (strncmp(args, "DWELL_SEG ",           10)  == 0) { cmdDwellSeg(args + 10);           return; }
    if (strncmp(args, "SET_TELEM_RATE ",       15) == 0) { cmdSetTelemRate(args + 15);       return; }
    if (strncmp(args, "SET_SOFT_LIMITS ",      16) == 0) { cmdSetSoftLimits(args + 16);      return; }

    err(args, "unknown_cmd");
}

// =============================================================================
// CMD handlers
// =============================================================================

void CommandParser::cmdHome() {
    // HOME is accepted from IDLE (never homed), READY (re-home), or ERROR
    // (recovery).  It is not accepted while RUNNING or PAUSED.
    SystemState s = _sm.getState();
    if (s != SystemState::IDLE && s != SystemState::READY && s != SystemState::ERROR) {
        err("HOME", "invalid_state");
        return;
    }
    _sm.toHoming();
    _mc.executeHome();
    ack("HOME");
}

void CommandParser::cmdGetState() {
    // Returns the current state and phase on a single line so the Python UI
    // can parse it with a split.  Format: "STATE <state> <phase>"
    Serial.print("STATE ");
    Serial.print(_sm.stateString());
    Serial.print(" ");
    Serial.println(_sm.phaseString());
}

void CommandParser::cmdStop() {
    // STOP is a graceful (soft) stop.  It is valid while RUNNING, PAUSED, or
    // READY (to stop a jog).  ESTOP is available from any state.
    SystemState s = _sm.getState();
    if (s != SystemState::RUNNING && s != SystemState::PAUSED && s != SystemState::READY) {
        err("STOP", "invalid_state");
        return;
    }
    _mc.stop();
    ack("STOP");
}

void CommandParser::cmdEstop() {
    // Emergency stop — always accepted, no state check.
    // Also cancels any in-progress segment collection.
    _collectingSegs = false;
    _mc.estop();
    ack("ESTOP");
}

void CommandParser::cmdPause() {
    if (_sm.getState() != SystemState::RUNNING) {
        err("PAUSE", "invalid_state");
        return;
    }
    _mc.pause();
    ack("PAUSE");
}

void CommandParser::cmdResume() {
    if (_sm.getState() != SystemState::PAUSED) {
        err("RESUME", "invalid_state");
        return;
    }
    _mc.resume();
    ack("RESUME");
}

void CommandParser::cmdMove(char* p) {
    if (_sm.getState() != SystemState::READY) {
        err("MOVE", "invalid_state");
        return;
    }
    float dist  = nextFloat(p);
    float speed = nextFloat(p);
    float accel = nextFloat(p);

    // dist is required — reject if unparseable (e.g. "CMD MOVE abc").
    if (isnan(dist)) { err("MOVE", "bad_args"); return; }

    // Speed and accel default to the profile defaults if not supplied (or if
    // the supplied value is 0 / negative).  NaN from a failed parse also
    // falls through to the default via the <= 0.0f check (NaN is not > 0).
    if (!(speed > 0.0f)) speed = DEFAULT_DIP_SPEED_MM_S;
    if (!(accel > 0.0f)) accel = DEFAULT_ACCEL_MM_S2;

    // Transition to RUNNING before issuing the move so that updateMove()
    // in MotionController correctly identifies this as a CMD MOVE (not a
    // Diagnostics move).
    _sm.toRunning();
    _mc.moveByMm(dist, speed, accel);
    ack("MOVE");
}

void CommandParser::cmdJog(char* p) {
    if (_sm.getState() != SystemState::READY) {
        err("JOG", "invalid_state");
        return;
    }
    // Parse the direction keyword ("UP" or "DOWN") before the speed value.
    bool up;
    if      (strncmp(p, "UP ",   3) == 0) { up = true;  p += 3; }
    else if (strncmp(p, "DOWN ", 5) == 0) { up = false; p += 5; }
    else { err("JOG", "bad_args"); return; }

    float speed = nextFloat(p);
    if (speed <= 0.0f) { err("JOG", "bad_speed"); return; }

    // Transition to RUNNING so that GET_STATE reflects the motor is moving,
    // and so that PAUSE/STOP are accepted while the jog is active.
    _sm.toRunning();
    _mc.jog(up, speed);
    ack("JOG");
}

void CommandParser::cmdRunProfile(char* p) {
    if (_sm.getState() != SystemState::READY) {
        err("RUN_PROFILE", "invalid_state");
        return;
    }
    // Parse all 7 profile parameters in order.
    float dipSpd   = nextFloat(p);
    float wdrawSpd = nextFloat(p);
    float accel    = nextFloat(p);
    float depth    = nextFloat(p);
    long  dwellBot = nextLong(p);
    long  dwellTop = nextLong(p);
    long  nDips    = nextLong(p);

    // Validate: motion parameters must be positive; dwell times must be >= 0.
    // An invalid profile transitions directly to ERROR to prevent a silent
    // bad-parameter run.
    // Use !(> 0) instead of <= 0 so that NaN (from a failed nextFloat parse)
    // is correctly treated as invalid — NaN comparisons always return false,
    // so "NaN <= 0" would pass silently, but "!(NaN > 0)" correctly fails.
    if (!(dipSpd > 0) || !(wdrawSpd > 0) || !(accel > 0) || !(depth > 0)
            || nDips <= 0 || dwellBot < 0 || dwellTop < 0) {
        err("RUN_PROFILE", "invalid_params");
        _sm.toError(ErrorCode::PROFILE_INVALID);
        return;
    }
    _mc.runProfile(dipSpd, wdrawSpd, accel, depth,
                   (uint32_t)dwellBot, (uint32_t)dwellTop, (int)nDips);
    ack("RUN_PROFILE");
}

void CommandParser::cmdBeginSegmentedMove(char* p) {
    if (_sm.getState() != SystemState::READY) {
        err("BEGIN_SEGMENTED_MOVE", "invalid_state");
        return;
    }
    long nSegs = nextLong(p);

    // nSegs must be at least 1 and must fit in the segment buffer.
    if (nSegs <= 0 || nSegs > MOVE_SEG_BUFFER_SIZE) {
        err("BEGIN_SEGMENTED_MOVE", "invalid_params");
        _sm.toError(ErrorCode::PROFILE_INVALID);
        return;
    }
    // Clear the segment buffer in MotionController and enter collection mode.
    // The parser will now only accept MOVE_SEG until all nSegs segments arrive.
    _mc.beginSegmentedMove();
    _collectingSegs = true;
    _segTotal       = (uint8_t)nSegs;
    _segReceived    = 0;
    ack("BEGIN_SEGMENTED_MOVE");
}

void CommandParser::cmdMoveSeg(char* p) {
    if (!_collectingSegs) {
        err("MOVE_SEG", "not_in_segment_collection");
        return;
    }
    float dist  = nextFloat(p);
    float speed = nextFloat(p);
    float accel = nextFloat(p);

    // dist is required — reject if unparseable.
    if (isnan(dist)) {
        err("MOVE_SEG", "bad_args");
        _collectingSegs = false;
        return;
    }
    // speed must be a positive number; NaN is not > 0, so it is caught here.
    if (!(speed > 0.0f)) {
        err("MOVE_SEG", "bad_speed");
        _collectingSegs = false;
        return;
    }
    // accel defaults if not supplied or invalid (NaN is also not > 0).
    if (!(accel > 0.0f)) accel = DEFAULT_ACCEL_MM_S2;

    // addSegment() returns false if the buffer is full.
    if (!_mc.addSegment(dist, speed, accel)) {
        err("MOVE_SEG", "seg_buffer_overflow");
        _collectingSegs = false;
        _sm.toError(ErrorCode::SEG_BUFFER_OVERFLOW);
        return;
    }
    _segReceived++;

    // When all expected segments have arrived, exit collection mode and
    // send ACK PROFILE_READY to tell the Python UI to issue RUN_LOADED_MOVE.
    if (_segReceived >= _segTotal) {
        _collectingSegs = false;
        ack("PROFILE_READY");
    }
}

void CommandParser::cmdDwellSeg(char* p) {
    if (!_collectingSegs) {
        err("DWELL_SEG", "not_in_segment_collection");
        return;
    }
    long ms = nextLong(p);
    if (ms <= 0) {
        err("DWELL_SEG", "bad_dwell_ms");
        _collectingSegs = false;
        return;
    }
    if (!_mc.addDwellSegment((uint32_t)ms)) {
        err("DWELL_SEG", "seg_buffer_overflow");
        _collectingSegs = false;
        _sm.toError(ErrorCode::SEG_BUFFER_OVERFLOW);
        return;
    }
    _segReceived++;
    if (_segReceived >= _segTotal) {
        _collectingSegs = false;
        ack("PROFILE_READY");
    }
}

void CommandParser::cmdRunLoadedMove() {
    if (_sm.getState() != SystemState::READY) {
        err("RUN_LOADED_MOVE", "invalid_state");
        return;
    }
    // Guard against running if segment collection was started but never completed.
    if (_collectingSegs) {
        err("RUN_LOADED_MOVE", "segment_collection_incomplete");
        return;
    }
    _mc.runLoadedMove();
    ack("RUN_LOADED_MOVE");
}

void CommandParser::cmdSetTelemRate(char* p) {
    long hz = nextLong(p);
    if (hz < 0 || hz > MAX_TELEM_RATE_HZ) {
        err("SET_TELEM_RATE", "out_of_range");
        return;
    }
    _telemRateHz = (uint8_t)hz;
    // _telem->setRate(_telemRateHz) will be wired here in step 7
    ack("SET_TELEM_RATE");
}

void CommandParser::cmdSetSoftLimits(char* p) {
    // Allow changing limits while IDLE or READY.
    // Not permitted while RUNNING / PAUSED to avoid mid-move limit changes.
    SystemState s = _sm.getState();
    if (s != SystemState::READY && s != SystemState::IDLE) {
        err("SET_SOFT_LIMITS", "invalid_state");
        return;
    }
    float minMm = nextFloat(p);
    float maxMm = nextFloat(p);

    // Reject unparseable arguments.  Also catches NaN: "NaN < NaN" is false,
    // so !(minMm < maxMm) correctly rejects NaN inputs.
    if (isnan(minMm) || isnan(maxMm) || !(minMm < maxMm)) {
        err("SET_SOFT_LIMITS", "min_must_be_less_than_max");
        return;
    }
    _mc.setSoftLimits(minMm, maxMm);
    ack("SET_SOFT_LIMITS");
}

// =============================================================================
// DIAG dispatch
// =============================================================================

void CommandParser::dispatchDiag(char* line) {
    // Exact-match commands (no arguments)
    if      (strcmp(line, "DIAG MOTOR")       == 0) { _diag.startMotorTest();        return; }
    else if (strcmp(line, "DIAG MOTORENCODER")== 0) { _diag.startMotorEncoderTest(); return; }
    else if (strcmp(line, "DIAG ENDSTOP")     == 0) { _diag.startEndstopTest();      return; }
    else if (strcmp(line, "DIAG EXIT")        == 0) { _diag.exit();                  return; }
    else if (strcmp(line, "DIAG POS")         == 0) { _diag.printPosition();         return; }
    else if (strcmp(line, "DIAG JOG STOP")    == 0) { _diag.exit();                  return; }

    // Commands with an optional speed argument
    else if (strncmp(line, "DIAG JOG DOWN", 13) == 0) {
        char* p   = line + 13;
        float spd = nextFloat(p);
        // Default to 5 mm/s if no speed was supplied.
        _diag.startJog(false, spd > 0.0f ? spd : 5.0f);

    } else if (strncmp(line, "DIAG JOG UP", 11) == 0) {
        char* p   = line + 11;
        float spd = nextFloat(p);
        _diag.startJog(true, spd > 0.0f ? spd : 5.0f);

    } else if (strncmp(line, "DIAG MOVE", 9) == 0) {
        // Parse up to three optional arguments: distance, speed, accel.
        // Fall back to the DIAG defaults if any argument is absent or zero.
        char* p     = line + 9;
        float mm    = nextFloat(p);
        float speed = nextFloat(p);
        float accel = nextFloat(p);
        _diag.startMoveTest(mm    != 0.0f ? mm    : 10.0f,
                            speed >  0.0f ? speed : Diagnostics::DIAG_SPEED_MMS,
                            accel >  0.0f ? accel : Diagnostics::DIAG_ACCEL_MMS2);

    } else if (strncmp(line, "DIAG CAL RESULT", 15) == 0) {
        // DIAG CAL RESULT must be checked before DIAG CAL because the shorter
        // prefix "DIAG CAL" would match both — order matters here.
        char* p  = line + 15;
        float mm = nextFloat(p);
        _diag.computeCalResult(mm);

    } else if (strncmp(line, "DIAG CAL", 8) == 0) {
        char* p  = line + 8;
        float mm = nextFloat(p);
        // Default to 50 mm if no distance was given.
        _diag.startCalMove(mm != 0.0f ? mm : 50.0f);

    } else {
        Serial.print("ERR unknown_diag_command: ");
        Serial.println(line);
    }
}

// =============================================================================
// Help
// =============================================================================

void CommandParser::printHelp() {
    Serial.println("--- Production commands (CMD) ---");
    Serial.println("  CMD HOME");
    Serial.println("  CMD GET_STATE");
    Serial.println("  CMD STOP");
    Serial.println("  CMD ESTOP");
    Serial.println("  CMD PAUSE");
    Serial.println("  CMD RESUME");
    Serial.println("  CMD MOVE <dist_mm> [speed_mm_s] [accel_mm_s2]");
    Serial.println("  CMD JOG <UP|DOWN> <speed_mm_s>");
    Serial.println("  CMD RUN_PROFILE <dip_spd> <wdraw_spd> <accel> <depth_mm> <dwell_bot_ms> <dwell_top_ms> <n_dips>");
    Serial.println("  CMD BEGIN_SEGMENTED_MOVE <n_segs>");
    Serial.println("  CMD MOVE_SEG  <dist_mm> <speed_mm_s> [accel_mm_s2]  (counts toward n_segs)");
    Serial.println("  CMD DWELL_SEG <dwell_ms>                             (counts toward n_segs)");
    Serial.println("  CMD RUN_LOADED_MOVE");
    Serial.println("  CMD SET_TELEM_RATE <hz>               (0=off, max 50)");
    Serial.println("  CMD SET_SOFT_LIMITS <min_mm> <max_mm>");
    Serial.println("--- Diagnostics commands (DIAG) ---");
    Serial.println("  DIAG MOTOR              move down 5mm then up 5mm");
    Serial.println("  DIAG MOTORENCODER       same + encoder pass/fail check");
    Serial.println("  DIAG ENDSTOP            live endstop monitor (trigger manually)");
    Serial.println("  DIAG MOVE <mm> [spd] [acc]");
    Serial.println("  DIAG JOG DOWN [spd]     continuous jog down");
    Serial.println("  DIAG JOG UP   [spd]     continuous jog up");
    Serial.println("  DIAG JOG STOP");
    Serial.println("  DIAG POS                print current position mm");
    Serial.println("  DIAG CAL <mm>           calibration move");
    Serial.println("  DIAG CAL RESULT <mm>    enter measured distance");
    Serial.println("  DIAG EXIT               stop test, return to idle");
    Serial.println("  HELP                    show this list");
}
