# Dip Coater Firmware — Test Commands

Commands are sent over serial at 115200 baud.
Each line is sent individually. Expected responses are shown after `→`.

---

## 1. Startup Check

```
HELP
```
→ Prints full command list. Confirms serial link is alive.

```
CMD GET_STATE
```
→ `STATE IDLE NONE` (before homing)

---

## 2. Homing

```
CMD HOME
```
→ `ACK HOME`
Motor drives upward until top endstop fires, then backs off 5 mm.

```
CMD GET_STATE
```
→ `STATE READY NONE`

---

## 3. Endstop Verification

Must be in READY state. Manually trigger each endstop switch by hand while monitoring.

```
DIAG ENDSTOP
```
→ `DIAG:ENDSTOP:START`

Trigger top endstop manually:
→ `DIAG:ENDSTOP:TOP:TRIGGERED`
→ `DIAG:ENDSTOP:TOP:RELEASED`

Trigger bottom endstop manually:
→ `DIAG:ENDSTOP:BOTTOM:TRIGGERED`
→ `DIAG:ENDSTOP:BOTTOM:RELEASED`

```
DIAG EXIT
```
→ `DIAG:EXIT`

---

## 4. Motor Connectivity

```
DIAG MOTOR
```
→ `DIAG:MOTOR:START down=5mm speed=2mm/s`
→ `DIAG:MOTOR:move down complete`
→ `DIAG:MOTOR:move up complete`
→ `DIAG:MOTOR:DONE`

With encoder check:
```
DIAG MOTORENCODER
```
→ `DIAG:MOTORENCODER:START down=5mm speed=2mm/s`
→ `DIAG:PASS:Move down`
→ `DIAG:PASS:Move up`
→ `DIAG:MOTORENCODER:DONE passed=2 failed=0`

---

## 5. Position Read

```
DIAG POS
```
→ `DIAG:POS:-5.00` (approximately, after homing backoff)

---

## 6. Jog

```
DIAG JOG DOWN 2
```
→ `DIAG:JOG:DOWN:2.0`
Motor moves continuously downward at 2 mm/s.

```
DIAG POS
```
→ Position increases negatively over time.

```
DIAG JOG STOP
```
→ `DIAG:EXIT`  Motor stops.

```
DIAG JOG UP 5
```
→ `DIAG:JOG:UP:5.0`
Motor moves continuously upward at 5 mm/s.

```
DIAG JOG STOP
```

---

## 7. DIAG MOVE — Speed and Accuracy

Home first, then run these in order. Each move returns to start before the next.

### Normal speed
```
DIAG MOVE -10 2 10
```
→ `DIAG:MOVE:START mm=-10.0 speed=2.000mm/s accel=10.0mm/s2`
→ `DIAG:MOVE:PASS commanded=-10.0mm actual=-10.0mm`

### Slow speed (key fix — previously broken below 0.167 mm/s)
```
DIAG MOVE -10 0.15 10
```
→ `DIAG:MOVE:PASS commanded=-10.0mm actual=-10.0mm`

```
DIAG MOVE -10 0.1 10
```
→ `DIAG:MOVE:PASS commanded=-10.0mm actual=-10.0mm`

### Short move (previously intermittent on 1 mm)
```
DIAG MOVE -1 1 20
```
→ `DIAG:MOVE:PASS commanded=-1.0mm actual=-1.0mm`

### Upward move
```
DIAG MOVE 5 2 10
```
→ `DIAG:MOVE:PASS commanded=5.0mm actual=5.0mm`

### Default arguments (no speed/accel given — uses 2 mm/s, 5 mm/s²)
```
DIAG MOVE -5
```
→ `DIAG:MOVE:START mm=-5.0 speed=2.000mm/s accel=5.0mm/s2`
→ `DIAG:MOVE:PASS commanded=-5.0mm actual=-5.0mm`

### Soft limit rejection (no START message should appear)
```
CMD SET_SOFT_LIMITS -20 10
DIAG MOVE -50 1 10
```
→ `ERR SOFT_LIMIT_EXCEEDED target=-55.00mm limits=[-20.00, 10.00]`
No `DIAG:MOVE:START` line should appear.

Restore limits:
```
CMD SET_SOFT_LIMITS -890 10
```
→ `ACK SET_SOFT_LIMITS`

---

## 8. CMD MOVE (production single move)

Must be in READY state.

```
CMD MOVE -10 2 10
```
→ `ACK MOVE`
→ `CMD:MOVE:DONE` (after motor reaches target)

```
CMD MOVE 10 2 10
```
→ `ACK MOVE`
→ `CMD:MOVE:DONE`

### Slow CMD MOVE (exercises isMoveComplete fix)
```
CMD MOVE -5 0.5 10
```
→ `ACK MOVE`
→ `CMD:MOVE:DONE`

---

## 9. Jog (production)

```
CMD JOG DOWN 3
```
→ `ACK JOG`  Motor jogs continuously downward.

```
CMD STOP
```
→ `ACK STOP`  Motor stops, state returns to READY.

```
CMD JOG UP 3
```
→ `ACK JOG`

```
CMD STOP
```
→ `ACK STOP`

---

## 10. Pause and Resume

Start a slow move, then pause mid-move.

```
CMD MOVE -30 1 5
```
→ `ACK MOVE`

(Wait ~5 seconds, then:)
```
CMD PAUSE
```
→ `ACK PAUSE`
→ `CMD GET_STATE` should return `STATE PAUSED ...`

```
CMD RESUME
```
→ `ACK RESUME`
→ Motor continues to target, then `CMD:MOVE:DONE`

---

## 11. ESTOP

```
CMD MOVE -30 1 5
```
→ `ACK MOVE`

(Immediately:)
```
CMD ESTOP
```
→ `ACK ESTOP`  Motor hard-stops. State → ERROR.

```
CMD GET_STATE
```
→ `STATE ERROR NONE`

Recover:
```
CMD HOME
```
→ `ACK HOME`  Re-homes and returns to READY.

---

## 12. Dip Profile

Home first. Adjust depth to suit your setup.

### Single dip
```
CMD RUN_PROFILE 2 2 10 20 2000 1000 1
```
→ `ACK RUN_PROFILE`
Sequence: descend 20 mm at 2 mm/s → dwell 2000 ms → ascend at 2 mm/s → READY.

### Multiple dips
```
CMD RUN_PROFILE 2 5 10 20 2000 1000 3
```
→ `ACK RUN_PROFILE`
3 dips: descend at 2 mm/s, withdraw at 5 mm/s, 2 s at bottom, 1 s at top between dips.

### Pause mid-profile
```
CMD RUN_PROFILE 2 2 10 20 5000 1000 3
```
→ `ACK RUN_PROFILE`

(During descent:)
```
CMD PAUSE
```
→ `ACK PAUSE`

```
CMD RESUME
```
→ `ACK RESUME`  Profile continues from where it paused.

---

## 13. Segmented Move

Two-segment dip: slow approach then fast retract.

```
CMD BEGIN_SEGMENTED_MOVE 2 1 3000 500
CMD MOVE_SEG -20 1
CMD MOVE_SEG 20 5
```
→ `ACK BEGIN_SEGMENTED_MOVE`
→ (no ack per segment)
→ `ACK PROFILE_READY`

```
CMD RUN_LOADED_MOVE
```
→ `ACK RUN_LOADED_MOVE`
Descends 20 mm at 1 mm/s, dwells 3 s, ascends 20 mm at 5 mm/s.

---

## 14. Soft Limits

```
CMD SET_SOFT_LIMITS -50 10
```
→ `ACK SET_SOFT_LIMITS`

Try to exceed the new limit:
```
CMD MOVE -60 2 10
```
→ `ERR SOFT_LIMIT_EXCEEDED target=-65.00mm limits=[-50.00, 10.00]`

Restore:
```
CMD SET_SOFT_LIMITS -890 10
```
→ `ACK SET_SOFT_LIMITS`

---

## 15. Telemetry Rate

```
CMD SET_TELEM_RATE 10
```
→ `ACK SET_TELEM_RATE`  (10 Hz streaming once telemetry module is wired up)

```
CMD SET_TELEM_RATE 0
```
→ `ACK SET_TELEM_RATE`  (off)

---

## 16. Calibration

Run a known distance move, measure with calipers, report back.

```
DIAG CAL -50
```
→ `DIAG:CAL:START commanded=-50.0mm at 10mm/s`
→ `DIAG:CAL:DONE encoder=-50.00mm`
→ Prompts you to measure with calipers and enter result.

Measure actual displacement, e.g. 49.7 mm:
```
DIAG CAL RESULT 49.7
```
→ `DIAG:CAL:RESULT commanded=-50.0mm encoder=50.00mm actual=49.70mm`
→ `DIAG:CAL:correction=0.9940`
→ If outside 1%: prints new `LEADSCREW_MM_PER_REV` value to update in config.h.

---

## 17. State Machine — Invalid Command Rejection

Commands sent in wrong states should be rejected cleanly.

Before homing (state = IDLE):
```
CMD MOVE -10 2 10
```
→ `ERR MOVE invalid_state`

```
CMD RUN_PROFILE 2 2 10 20 1000 500 1
```
→ `ERR RUN_PROFILE invalid_state`

While running (state = RUNNING):
```
CMD HOME
```
→ `ERR HOME invalid_state`
