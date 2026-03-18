#include "command_parser.h"

// =============================================================================
// Number parsing helpers — avoid sscanf (unreliable on STM32)
// =============================================================================

static float nextFloat(char*& p) {
    while (*p == ' ') p++;
    char* end;
    float v = strtof(p, &end);
    p = end;
    return v;
}

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
    : _sm(sm), _mc(mc), _diag(diag), _telem(nullptr)
    , _len(0)
    , _telemRateHz(DEFAULT_TELEM_RATE_HZ)
    , _collectingSegs(false), _segTotal(0), _segReceived(0)
{}

void CommandParser::begin() {
    _len             = 0;
    _collectingSegs  = false;
}

void CommandParser::setTelemetry(Telemetry* telem) {
    _telem = telem;
}

// =============================================================================
// update() — call every loop()
// =============================================================================

void CommandParser::update() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            _buf[_len] = '\0';
            if (_len > 0 && _buf[_len - 1] == '\r') _buf[--_len] = '\0';
            if (_len > 0) dispatch(_buf);
            _len = 0;
        } else if (_len < (uint8_t)(sizeof(_buf) - 1)) {
            _buf[_len++] = c;
        }
    }
}

// =============================================================================
// Response helpers
// =============================================================================

void CommandParser::ack(const char* cmd) {
    Serial.print("ACK ");
    Serial.println(cmd);
}

void CommandParser::err(const char* cmd, const char* reason) {
    Serial.print("ERR ");
    Serial.print(cmd);
    Serial.print(" ");
    Serial.println(reason);
}

// =============================================================================
// Top-level dispatch
// =============================================================================

void CommandParser::dispatch(char* line) {
    // Strip trailing whitespace
    int len = (int)strlen(line);
    while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\r'))
        line[--len] = '\0';

    if (strcmp(line, "HELP") == 0 || strcmp(line, "DIAG CMD") == 0) {
        printHelp();
        return;
    }

    if (strncmp(line, "DIAG", 4) == 0) {
        dispatchDiag(line);
        return;
    }

    if (strncmp(line, "CMD ", 4) == 0) {
        dispatchCmd(line + 4);
        return;
    }

    Serial.print("ERR unknown_command: ");
    Serial.println(line);
}

// =============================================================================
// CMD dispatch
// =============================================================================

void CommandParser::dispatchCmd(char* args) {
    // While collecting segments, only MOVE_SEG and ESTOP are accepted
    if (_collectingSegs) {
        if (strncmp(args, "MOVE_SEG ", 9) == 0) { cmdMoveSeg(args + 9); return; }
        if (strcmp(args, "ESTOP") == 0)          { cmdEstop(); return; }
        _collectingSegs = false;
        err("CMD", "segment_collection_aborted — only MOVE_SEG or ESTOP accepted");
        return;
    }

    if (strcmp(args, "HOME") == 0)                      { cmdHome();           return; }
    if (strcmp(args, "GET_STATE") == 0)                 { cmdGetState();       return; }
    if (strcmp(args, "STOP") == 0)                      { cmdStop();           return; }
    if (strcmp(args, "ESTOP") == 0)                     { cmdEstop();          return; }
    if (strcmp(args, "PAUSE") == 0)                     { cmdPause();          return; }
    if (strcmp(args, "RESUME") == 0)                    { cmdResume();         return; }
    if (strcmp(args, "RUN_LOADED_MOVE") == 0)           { cmdRunLoadedMove();  return; }

    if (strncmp(args, "MOVE ",                 5)  == 0) { cmdMove(args + 5);                 return; }
    if (strncmp(args, "JOG ",                  4)  == 0) { cmdJog(args + 4);                  return; }
    if (strncmp(args, "RUN_PROFILE ",         12)  == 0) { cmdRunProfile(args + 12);          return; }
    if (strncmp(args, "BEGIN_SEGMENTED_MOVE ", 21) == 0) { cmdBeginSegmentedMove(args + 21);  return; }
    if (strncmp(args, "MOVE_SEG ",             9)  == 0) { cmdMoveSeg(args + 9);              return; }
    if (strncmp(args, "SET_TELEM_RATE ",       15) == 0) { cmdSetTelemRate(args + 15);        return; }
    if (strncmp(args, "SET_SOFT_LIMITS ",      16) == 0) { cmdSetSoftLimits(args + 16);       return; }
    if (strncmp(args, "DEBUG ",                6)  == 0) { cmdDebug(args + 6);                return; }

    err(args, "unknown_cmd");
}

// =============================================================================
// CMD handlers
// =============================================================================

void CommandParser::cmdHome() {
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
    Serial.print("STATE ");
    Serial.print(_sm.stateString());
    Serial.print(" ");
    Serial.println(_sm.phaseString());
}

void CommandParser::cmdStop() {
    SystemState s = _sm.getState();
    if (s != SystemState::RUNNING && s != SystemState::PAUSED && s != SystemState::READY) {
        err("STOP", "invalid_state");
        return;
    }
    _mc.stop();
    ack("STOP");
}

void CommandParser::cmdEstop() {
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

    if (speed <= 0.0f) speed = DEFAULT_DIP_SPEED_MM_S;
    if (accel <= 0.0f) accel = DEFAULT_ACCEL_MM_S2;

    _sm.toRunning();
    _mc.moveByMm(dist, speed, accel);
    ack("MOVE");
}

void CommandParser::cmdJog(char* p) {
    if (_sm.getState() != SystemState::READY) {
        err("JOG", "invalid_state");
        return;
    }
    // p = "UP <speed>" or "DOWN <speed>"
    bool up;
    if (strncmp(p, "UP ", 3) == 0)        { up = true;  p += 3; }
    else if (strncmp(p, "DOWN ", 5) == 0) { up = false; p += 5; }
    else { err("JOG", "bad_args"); return; }

    float speed = nextFloat(p);
    if (speed <= 0.0f) { err("JOG", "bad_speed"); return; }

    _mc.jog(up, speed);
    ack("JOG");
}

void CommandParser::cmdRunProfile(char* p) {
    if (_sm.getState() != SystemState::READY) {
        err("RUN_PROFILE", "invalid_state");
        return;
    }
    float dipSpd    = nextFloat(p);
    float wdrawSpd  = nextFloat(p);
    float accel     = nextFloat(p);
    float depth     = nextFloat(p);
    long  dwellBot  = nextLong(p);
    long  dwellTop  = nextLong(p);
    long  nDips     = nextLong(p);

    if (dipSpd <= 0 || wdrawSpd <= 0 || accel <= 0 || depth <= 0 || nDips <= 0) {
        err("RUN_PROFILE", "invalid_params");
        _sm.toError(ErrorCode::PROFILE_INVALID);
        return;
    }
    _mc.runProfile(dipSpd, wdrawSpd, accel, depth,
                   (int)dwellBot, (int)dwellTop, (int)nDips);
    ack("RUN_PROFILE");
}

void CommandParser::cmdBeginSegmentedMove(char* p) {
    if (_sm.getState() != SystemState::READY) {
        err("BEGIN_SEGMENTED_MOVE", "invalid_state");
        return;
    }
    long nSegs    = nextLong(p);
    long nDips    = nextLong(p);
    long dwellBot = nextLong(p);
    long dwellTop = nextLong(p);

    if (nSegs <= 0 || nSegs > MOVE_SEG_BUFFER_SIZE || nDips <= 0) {
        err("BEGIN_SEGMENTED_MOVE", "invalid_params");
        _sm.toError(ErrorCode::PROFILE_INVALID);
        return;
    }
    _mc.beginSegmentedMove((uint8_t)nDips, (uint16_t)dwellBot, (uint16_t)dwellTop);
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

    if (speed <= 0.0f) {
        err("MOVE_SEG", "bad_speed");
        _collectingSegs = false;
        return;
    }
    if (!_mc.addSegment(dist, speed)) {   // positive mm = up, matches firmware convention
        err("MOVE_SEG", "seg_buffer_overflow");
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
    if (_sm.getState() != SystemState::READY && _sm.getState() != SystemState::IDLE) {
        err("SET_SOFT_LIMITS", "invalid_state");
        return;
    }
    float minMm = nextFloat(p);
    float maxMm = nextFloat(p);
    if (minMm >= maxMm) {
        err("SET_SOFT_LIMITS", "min_must_be_less_than_max");
        return;
    }
    _mc.setSoftLimits(minMm, maxMm);
    ack("SET_SOFT_LIMITS");
}

void CommandParser::cmdDebug(char* p) {
    while (*p == ' ') p++;
    if (strcmp(p, "ON") == 0)       { _mc.setDebug(true);  ack("DEBUG"); }
    else if (strcmp(p, "OFF") == 0) { _mc.setDebug(false); ack("DEBUG"); }
    else                            { err("DEBUG", "expected_ON_or_OFF"); }
}

// =============================================================================
// DIAG dispatch — mirrors the handling previously in .ino dispatchCommand()
// =============================================================================

void CommandParser::dispatchDiag(char* line) {
    if (strcmp(line, "DIAG MOTOR") == 0) {
        _diag.startMotorTest();
    } else if (strcmp(line, "DIAG MOTORENCODER") == 0) {
        _diag.startMotorEncoderTest();
    } else if (strcmp(line, "DIAG ENDSTOP") == 0) {
        _diag.startEndstopTest();
    } else if (strcmp(line, "DIAG EXIT") == 0) {
        _diag.exit();
    } else if (strcmp(line, "DIAG POS") == 0) {
        _diag.printPosition();
    } else if (strcmp(line, "DIAG JOG STOP") == 0) {
        _diag.exit();
    } else if (strncmp(line, "DIAG JOG DOWN", 13) == 0) {
        char* p = line + 13;
        float spd = *p ? atof(p) : 5.0f;
        _diag.startJog(false, spd);
    } else if (strncmp(line, "DIAG JOG UP", 11) == 0) {
        char* p = line + 11;
        float spd = *p ? atof(p) : 5.0f;
        _diag.startJog(true, spd);
    } else if (strncmp(line, "DIAG MOVE", 9) == 0) {
        char* p = line + 9;
        float mm    = *p ? atof(p) : 10.0f;
        while (*p == ' ') p++;
        while (*p && *p != ' ') p++;
        float speed = *p ? atof(p) : 0.0f;
        while (*p == ' ') p++;
        while (*p && *p != ' ') p++;
        float accel = *p ? atof(p) : 0.0f;
        _diag.startMoveTest(mm,
                            speed > 0.0f ? speed : Diagnostics::DIAG_SPEED_MMS,
                            accel > 0.0f ? accel : Diagnostics::DIAG_ACCEL_MMS2);
    } else if (strncmp(line, "DIAG CAL RESULT", 15) == 0) {
        float mm = *(line + 15) ? atof(line + 15) : 0.0f;
        _diag.computeCalResult(mm);
    } else if (strncmp(line, "DIAG CAL", 8) == 0) {
        float mm = *(line + 8) ? atof(line + 8) : 50.0f;
        _diag.startCalMove(mm);
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
    Serial.println("  CMD BEGIN_SEGMENTED_MOVE <n_segs> <n_dips> <dwell_bot_ms> <dwell_top_ms>");
    Serial.println("  CMD MOVE_SEG <dist_mm> <speed_mm_s>   (repeat n_segs times)");
    Serial.println("  CMD RUN_LOADED_MOVE");
    Serial.println("  CMD SET_TELEM_RATE <hz>               (0=off, max 50)");
    Serial.println("  CMD SET_SOFT_LIMITS <min_mm> <max_mm>");
    Serial.println("  CMD DEBUG <ON|OFF>");
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
