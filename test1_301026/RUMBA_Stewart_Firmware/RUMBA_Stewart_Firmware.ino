/*
 * RUMBA ATmega2560 - 6-AXIS BARE-METAL STEPPER FIRMWARE
 * Smooth motion + gentle 3-stage homing + E-stop
 *
 * Native AVR C++ with avr-libc register headers only.
 * No Arduino.h, AccelStepper, motion-planner libraries, or Arduino core.
 *
 * Serial at 250000 baud, one command per line (LF):
 *   HELLO STEWART        MATLAB handshake, answered HELLO_OK (see MATLAB LINK)
 *   PING                 keep-alive from MATLAB (no reply)
 *   HOME                 home all six motors, then move to the Z height
 *   START                begin accepting SETF targets
 *   SETF seq q0..q5      six signed targets, microsteps * 256
 *   STATUS               "STATUS <state> p0 p1 p2 p3 p4 p5"
 *   NEUTRAL              "NEUTRAL <z_cm*100> n0 n1 n2 n3 n4 n5"
 *                        built-in Z height and its level-pose positions
 *                        (same signed microstep units as STATUS)
 *   HOLD                 smooth deceleration to a stop
 *   RESET                clear a fault / E-stop (motors must be re-homed)
 *   e   (at line start)  software E-stop
 *   !   (anywhere)       software E-stop, acted on inside the UART interrupt
 *
 * ----------------------------------------------------------------------
 * BUILT-IN Z HEIGHT
 * ----------------------------------------------------------------------
 * The level working height is stored here (Z_HEIGHT_CM_X100 and
 * NEUTRAL_AWAY_USTEPS). Generate both lines with StewartFirmwareZ.m in
 * MATLAB, paste them below and upload. HOME homes the motors and then
 * glides to that height. MATLAB reads the height back with NEUTRAL and
 * refuses to run if its IK disagrees.
 *
 * ----------------------------------------------------------------------
 * MATLAB LINK (motors only move while the MATLAB program is running)
 * ----------------------------------------------------------------------
 * After power-up or reset the board does NOTHING: motors stay disabled and
 * HOME / START / SETF / HOLD are refused with "ERR NO_MATLAB" until the
 * MATLAB program sends "HELLO STEWART". MATLAB then keeps the link alive
 * with any command (STATUS, SETF, PING...) at least every LINK_TIMEOUT_S.
 * If the lines stop (MATLAB stopped, crashed, cable pulled):
 *   - during homing   -> immediate stop, fault "LINK_LOST"
 *   - while streaming -> smooth HOLD, "WARN LINK_LOST_HOLD"
 *   - while idle      -> "WARN LINK_LOST"
 * and nothing moves again until MATLAB sends HELLO again. Motors stay
 * energized so the plate holds its position instead of dropping.
 * STATUS, NEUTRAL, RESET and the E-stop work without the handshake.
 *
 * ----------------------------------------------------------------------
 * HOMING (all six motors together, every move ramped, no jerks)
 * ----------------------------------------------------------------------
 *   1. RELEASE  any motor already on its switch moves off it
 *   2. FAST     slow approach (HOME_APPROACH_USTEPS_S) until each switch
 *               triggers; each motor stops the moment its own switch does
 *   3. BACKOFF  all motors move BACKOFF_STEPS out again (smooth trapezoid)
 *   4. SLOW     slower approach (HOME_FINAL_USTEPS_S); the switch trigger
 *               point of this pass is position 0 (the 21-degree angle)
 *   5. Z MOVE   all motors glide together (one synchronized trapezoid) to
 *               NEUTRAL_AWAY_USTEPS = the level platform at Z_HEIGHT
 * Messages: HOMING, FAST_HOME_ALL_FOUND, BACKOFF_ALL_DONE,
 *           SLOW_HOME_ALL_FOUND, MOVING_TO_Z, AT_Z <z_cm*100>, HOMED_ALL
 *           (or ERR STOPPED <reason>)
 * HOME while homing is already running just answers HOMING.
 *
 * ----------------------------------------------------------------------
 * E-STOP
 * ----------------------------------------------------------------------
 * Software: "!" or a line starting with "e" stops all motors inside the
 * UART receive interrupt (microseconds), even in the middle of homing.
 * Hardware (optional): a normally-closed button on a spare pin, checked
 * 10000 times per second; see ESTOP_INPUT_ENABLED below.
 * After any E-stop: drivers stay energized (plate holds position),
 * every command except STATUS and RESET is refused, then HOME is required.
 * This does NOT replace a hardwired E-stop that cuts motor power.
 *
 * ----------------------------------------------------------------------
 * MOTION
 * ----------------------------------------------------------------------
 *  Step generator (Timer3, 10 kHz): 16.16 fixed-point position + velocity
 *    per motor; evenly spaced step pulses at any speed.
 *  Controller (Timer4, 500 Hz, interruptible): feed-forward + P tracking
 *    with speed and acceleration limits, so velocity never jumps.
 *  SETF stream: targets go into a playback buffer stamped with their
 *    arrival time and are replayed a short, adaptive delay later
 *    (15-100 ms), interpolating between them. Irregular packet timing
 *    from MATLAB therefore becomes ONE continuous move, never
 *    move-stop-move. Repeated identical SETF mean "hold here".
 *    A single target (first one, after a pause, or a big jump) uses a
 *    synchronized trapezoid: all motors start and finish together.
 *
 * TEST UNLOADED WITH RODS DETACHED, PLATE SUPPORTED, HARDWIRED E-STOP.
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#if F_CPU != 16000000UL
#error "ATmega2560 firmware requires 16 MHz F_CPU"
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ===================== USER SETTINGS =====================
// Units: microsteps (1/16), seconds. 3200 microsteps = 1 rev = 8.89 per degree.

// ---- Homing ----
#define HOME_APPROACH_USTEPS_S  160.0f  // 1st approach, release, back-off (~18 deg/s)
#define HOME_FINAL_USTEPS_S      40.0f  // 2nd, slower approach   (~4.5 deg/s)
#define HOME_ACCEL_USTEPS_S2   1600.0f  // gentle ramp for all homing moves
#define BACKOFF_STEPS            80     // "out again" distance (~9 deg)
#define MAX_RELEASE_STEPS       240     // travel limits per homing stage
#define MAX_FAST_STEPS          800
#define MAX_SLOW_STEPS          120     // must exceed BACKOFF_STEPS
#define HOME_STAGE_TIMEOUT_S     20UL

// ---- Built-in Z height (level platform after homing) ----
// >>> PASTE THE TWO LINES PRINTED BY StewartFirmwareZ.m HERE <<<
// NEUTRAL_AWAY_USTEPS = microsteps AWAY from each switch (always positive),
// motor order X, Y, Z, E0, E1, E2.
#define Z_HEIGHT_CM_X100  1650   // e.g. 2045 = 20.45 cm; 0 = not set
#define NEUTRAL_AWAY_LIST   694, 694, 694, 694, 694, 694

#define ZMOVE_VMAX_USTEPS_S     300.0f  // glide to Z height (~34 deg/s)
#define ZMOVE_ACCEL_USTEPS_S2  1200.0f  // gentle ramp

// ---- MATLAB link ----
// No line from MATLAB for this long -> stop and lock until HELLO again.
#define LINK_TIMEOUT_S            2.0f

// ---- Streaming motion ----
#define VMAX_USTEPS_S        1600.0f  // top motor speed   (~180 deg/s)
#define ACCEL_USTEPS_S2      8000.0f  // planned accel     (~900 deg/s^2)
#define TRACK_ACCEL_FACTOR      1.5f  // controller may use 1.5x ACCEL to correct
#define KP_PER_S               30.0f  // position-error gain (1/s)
#define SETTLE_USTEPS           0.1f
// Stream playback buffer: targets are replayed at the times they ARRIVED,
// delayed by an adaptive amount, so irregular SETF timing from MATLAB
// becomes one continuous move instead of move-stop-move.
#define STREAM_DELAY_MIN_S      0.015f // shortest playback delay (latency)
#define STREAM_DELAY_MAX_S      0.100f // longest playback delay
#define STREAM_DELAY_MARGIN     1.3f   // delay = 1.3 x recent worst SETF gap + 4 ms
#define STREAM_MAX_PERIOD_S     0.100f // pause longer than this = new single move
#define STREAM_RING            32      // buffered targets (power of 2)
#define WATCHDOG_S              1.0f   // no SETF for this long -> smooth HOLD

// ---- Optional hardwired E-stop input to the RUMBA ----
// Wire a NORMALLY-CLOSED button between a free RUMBA pin and GND.
// Internal pull-up: closed (normal) = LOW, pressed or broken wire = HIGH.
// Set ESTOP_INPUT_ENABLED to 1 and fill in the AVR port/bit of the pin you
// used (look it up in the RUMBA pinout; the six endstop inputs PC0..PC5
// and all STEP/DIR/EN pins are already taken).
#define ESTOP_INPUT_ENABLED 0
#if ESTOP_INPUT_ENABLED
  #define ESTOP_PIN_REG   PINE      // example only - CHANGE to your pin
  #define ESTOP_DDR_REG   DDRE
  #define ESTOP_PORT_REG  PORTE
  #define ESTOP_BIT       PE4
#endif

// ---- Fixed ----
#define STEP_ISR_HZ          10000UL
#define CTRL_HZ                500UL
#define DEBOUNCE_TICKS       10000UL  // 5 ms (Timer1 = 2 MHz)
#define AXES 6
#define MAX_AWAY_STEPS    1000
#define SWITCH_TOLERANCE    80
#define POSITION_SCALE     256L

static constexpr int16_t NEUTRAL_AWAY_USTEPS[AXES] = { NEUTRAL_AWAY_LIST };
static constexpr bool neutralOk(uint8_t i) {
    return i >= AXES ? true
         : (NEUTRAL_AWAY_USTEPS[i] > 0 && NEUTRAL_AWAY_USTEPS[i] <= MAX_AWAY_STEPS && neutralOk(i + 1));
}

static_assert(VMAX_USTEPS_S <= STEP_ISR_HZ / 2, "VMAX must be <= half the step ISR rate");
static_assert(ZMOVE_VMAX_USTEPS_S <= VMAX_USTEPS_S, "Z move speed above VMAX");
static_assert(HOME_FINAL_USTEPS_S < HOME_APPROACH_USTEPS_S, "2nd approach must be slower");
static_assert(MAX_SLOW_STEPS > BACKOFF_STEPS + 20, "Slow search must reach past the back-off");
static_assert(BACKOFF_STEPS > 0, "Bad homing distances");
static_assert(neutralOk(0), "NEUTRAL_AWAY_LIST: each value must be 1..MAX_AWAY_STEPS");
static_assert(Z_HEIGHT_CM_X100 >= 0, "Z_HEIGHT_CM_X100 must not be negative");

static const float DT        = 1.0f / CTRL_HZ;
static const float VEL_TO_Q  = 65536.0f / STEP_ISR_HZ;  // usteps/s -> 16.16 per tick
static const float DV_MAX    = TRACK_ACCEL_FACTOR * ACCEL_USTEPS_S2 / CTRL_HZ;
static const float DV_HOME   = HOME_ACCEL_USTEPS_S2 / CTRL_HZ;
static const uint16_t WATCHDOG_CTRL_TICKS = (uint16_t)(WATCHDOG_S * CTRL_HZ);

// ===================== RUMBA DIRECT REGISTER PINS =====================
// STEP Arduino numbers: 17,54,57,23,26,29.
// DIR  Arduino numbers: 16,47,56,22,25,28.
// EN   Arduino numbers: 48,55,62,24,27,39. Active LOW.
// ENDSTOPS Arduino numbers: 37,36,35,34,33,32. Active LOW -> PC0..PC5.
static inline void stepsLow() {
    PORTH &= (uint8_t)~_BV(PH0);
    PORTF &= (uint8_t)~(_BV(PF0) | _BV(PF3));
    PORTA &= (uint8_t)~(_BV(PA1) | _BV(PA4) | _BV(PA7));
}

static inline void stepHigh(uint8_t m) {
    if (m &  1) PORTH |= _BV(PH0);
    if (m &  2) PORTF |= _BV(PF0);
    if (m &  4) PORTF |= _BV(PF3);
    if (m &  8) PORTA |= _BV(PA1);
    if (m & 16) PORTA |= _BV(PA4);
    if (m & 32) PORTA |= _BV(PA7);
}

// Current level of each DIR pin, bit i = axis i (HIGH = positive direction).
static volatile uint8_t dirState = 0;

static inline void setDir(uint8_t i, bool high) {
    switch (i) {
        case 0: if (high) PORTH |= _BV(PH1); else PORTH &= (uint8_t)~_BV(PH1); break;
        case 1: if (high) PORTL |= _BV(PL2); else PORTL &= (uint8_t)~_BV(PL2); break;
        case 2: if (high) PORTF |= _BV(PF2); else PORTF &= (uint8_t)~_BV(PF2); break;
        case 3: if (high) PORTA |= _BV(PA0); else PORTA &= (uint8_t)~_BV(PA0); break;
        case 4: if (high) PORTA |= _BV(PA3); else PORTA &= (uint8_t)~_BV(PA3); break;
        case 5: if (high) PORTA |= _BV(PA6); else PORTA &= (uint8_t)~_BV(PA6); break;
    }
    if (high) dirState |= (uint8_t)(1U << i);
    else      dirState &= (uint8_t)~(1U << i);
}

static inline void enableMotor(uint8_t i) {
    switch (i) {
        case 0: PORTL &= (uint8_t)~_BV(PL1); break;
        case 1: PORTF &= (uint8_t)~_BV(PF1); break;
        case 2: PORTK &= (uint8_t)~_BV(PK0); break;
        case 3: PORTA &= (uint8_t)~_BV(PA2); break;
        case 4: PORTA &= (uint8_t)~_BV(PA5); break;
        case 5: PORTG &= (uint8_t)~_BV(PG2); break;
    }
}

// HOME_DIR = DIR level that moves TOWARD the switch.
static const uint8_t HOME_DIR[AXES] = {1,0,1,0,1,0};
static const char* const MOTOR_NAME[AXES] = {"X","Y","Z","E0","E1","E2"};
static inline int8_t awaySign(uint8_t i) { return HOME_DIR[i] ? -1 : 1; }

#if ESTOP_INPUT_ENABLED
static inline bool estopInputActive() { return (ESTOP_PIN_REG & _BV(ESTOP_BIT)) != 0; }
#endif

// ===================== TIMER1: 2 MHz CLOCK =====================
static volatile uint32_t timerHigh = 0;
ISR(TIMER1_OVF_vect) { timerHigh += 65536UL; }

static uint32_t nowTicks() {
    uint32_t hi; uint16_t lo; uint8_t overflow;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        hi = timerHigh;
        lo = TCNT1;
        overflow = TIFR1 & _BV(TOV1);
        if (overflow && lo < 0x8000U) hi += 65536UL;
    }
    return hi + lo;
}

// ===================== SHARED MOTION STATE =====================
enum HomeState : uint8_t { IDLE, RELEASE, FAST, BACKOFF, SLOW, ZMOVE };
enum CtrlMode  : uint8_t { CTRL_OFF, CTRL_RUN, CTRL_STOP, CTRL_JOG };

// posQ   : commanded position, microsteps in 16.16 fixed point.
// velQ   : velocity, 16.16 microsteps per step tick (written by controller).
// stepPos: microsteps actually sent to the driver (= round(posQ)).
static volatile bool    motionOn = false;
static volatile uint8_t ctrlMode = CTRL_OFF;
static volatile bool    limitsOn = true;     // soft limits off while searching switches
static volatile int32_t posQ[AXES]  = {0};
static volatile int32_t velQ[AXES]  = {0};
static volatile int16_t stepPos[AXES] = {0};
static volatile bool    stepIsHigh = false;

static volatile bool    streaming = false;   // accepting SETF
static volatile uint8_t motorFault = 0;      // 1 watchdog, 2 travel, 3 unexpected switch
static volatile uint8_t motorFaultAxis = 0;
static volatile bool    estopLatched = false;
static volatile uint8_t estopSource = 0;     // 1 serial, 2 hardware input
static volatile uint8_t stableSwitch = 0;

static inline bool pressed(uint8_t i) { return (stableSwitch & (1U << i)) != 0; }

// Interrupt-safe instant stop of all motion (called from ISRs).
static inline void haltAllFromIsr() {
    motionOn = false;
    ctrlMode = CTRL_OFF;
    streaming = false;
    for (uint8_t k = 0; k < AXES; ++k) velQ[k] = 0;
}
static inline void estopNow(uint8_t source) {
    haltAllFromIsr();
    if (!estopLatched) estopSource = source;
    estopLatched = true;
}
static inline void motionFault(uint8_t code, uint8_t axis) {
    haltAllFromIsr();
    motorFaultAxis = axis;
    motorFault = code;
}

// ===================== UART0: 250000 BAUD =====================
static volatile uint8_t rx[256], rxHead = 0, rxTail = 0;
#define RXERR_FRAME   0x01U
#define RXERR_OVERRUN 0x02U
#define RXERR_PARITY  0x04U
#define RXERR_BUFFER  0x08U
static volatile uint8_t rxErrorFlags = 0;
static volatile bool rxLineStart = true;
ISR(USART0_RX_vect) {
    uint8_t flags = UCSR0A;   // must be read before UDR0
    uint8_t c = UDR0;
    uint8_t errors = 0;
    if (flags & _BV(FE0))  errors |= RXERR_FRAME;
    if (flags & _BV(DOR0)) errors |= RXERR_OVERRUN;
    if (flags & _BV(UPE0)) errors |= RXERR_PARITY;
    if (errors) { rxErrorFlags |= errors; return; }
    // E-stop is handled HERE, not in the main loop: it works even if the
    // main loop is busy, the RX buffer is full, or a line is half-received.
    if (c == '!' || (rxLineStart && (c == 'e' || c == 'E'))) {
        estopNow(1);
        rxLineStart = false;
        return;
    }
    rxLineStart = (c == '\n');
    uint8_t next = (uint8_t)(rxHead + 1);
    if (next == rxTail) { rxErrorFlags |= RXERR_BUFFER; return; }
    rx[rxHead] = c;
    rxHead = next;
}
static void putc0(char c) {
    while (!(UCSR0A & _BV(UDRE0))) {}
    UDR0 = (uint8_t)c;
}
static void print(const char* s) { while (*s) putc0(*s++); }
static void printNum(long v) { char b[16]; ltoa(v,b,10); print(b); }
static void println(const char* s) { print(s); print("\r\n"); }

// ===================== LIMIT SWITCHES =====================
static uint8_t rawSwitch = 0;
static uint32_t changedAt[AXES] = {0};

static void readSwitches() {
    const uint32_t now = nowTicks();
    const uint8_t current = (uint8_t)(~PINC) & 0x3F;
    uint8_t stable = stableSwitch;
    for (uint8_t i = 0; i < AXES; ++i) {
        const uint8_t b = (uint8_t)(1U << i);
        if ((current & b) != (rawSwitch & b)) {
            if (current & b) rawSwitch |= b; else rawSwitch &= (uint8_t)~b;
            changedAt[i] = now;
        }
        if ((uint32_t)(now - changedAt[i]) >= DEBOUNCE_TICKS) {
            if (rawSwitch & b) stable |= b;
            else stable &= (uint8_t)~b;
        }
    }
    stableSwitch = stable;
}

// ===================== STEP GENERATOR (Timer3, 10 kHz) =====================
ISR(TIMER3_COMPA_vect) {
    // End last tick's pulse: STEP stays high ~100 us, far above driver minimums.
    if (stepIsHigh) { stepsLow(); stepIsHigh = false; }
#if ESTOP_INPUT_ENABLED
    if (estopInputActive()) estopNow(2);
#endif
    if (!motionOn) return;

    const bool limits = limitsOn;
    uint8_t mask = 0;
    for (uint8_t i = 0; i < AXES; ++i) {
        const int32_t p = posQ[i] + velQ[i];
        posQ[i] = p;
        const int16_t want = (int16_t)((p + 0x8000L) >> 16);  // round to microstep
        const int16_t have = stepPos[i];
        if (want == have) continue;

        const bool up = want > have;
        const uint8_t b = (uint8_t)(1U << i);
        if (up != ((dirState & b) != 0)) {
            // Direction reversal: change DIR now, send the step next tick
            // (gives the driver 100 us of DIR setup time).
            setDir(i, up);
            continue;
        }
        const int16_t next = (int16_t)(have + (up ? 1 : -1));
        if (limits) {
            const int16_t away = (int16_t)(next * awaySign(i));
            if (away < 0 || away > MAX_AWAY_STEPS) { motionFault(2, i); return; }
            if (pressed(i) && away > SWITCH_TOLERANCE) { motionFault(3, i); return; }
        }
        stepPos[i] = next;
        mask |= b;
    }
    if (mask) {
        stepHigh(mask);
        stepIsHigh = true;
    }
}

// ===================== MOTION CONTROLLER (Timer4, 500 Hz) =====================
// Single-move mailbox (trapezoid), written by main, read by the controller.
static volatile bool  mbReady = false;
static volatile float mbVmax = VMAX_USTEPS_S;
static volatile float mbAccel = ACCEL_USTEPS_S2;
static volatile float mbTarget[AXES];
static volatile float jogVel[AXES];         // CTRL_JOG target speeds (homing)
static volatile uint16_t heartbeat = 0;
static volatile bool  moveDone = true;      // reference finished and all motors stopped

// Stream ring buffer: main pushes (arrival time, target); controller replays.
struct Sample { uint32_t t; float q[AXES]; };
static Sample ring[STREAM_RING];
static volatile uint8_t ringHead = 0, ringTail = 0;   // main owns head, controller tail
static volatile float streamDelayTarget = 0.03f;      // set by main from SETF gaps
static_assert((STREAM_RING & (STREAM_RING - 1)) == 0, "STREAM_RING must be power of 2");

enum RefMode : uint8_t { REF_HOLD, REF_TRAP, REF_PLAY };

// Controller-private state (touched by main only inside ATOMIC blocks).
static float ref[AXES], vel[AXES], tgt[AXES], segStart[AXES], segDelta[AXES];
static uint8_t refMode = REF_HOLD;
static uint16_t segK = 0, segN = 0;
static float trTa = 0, trT = 0, trAn = 0, trVn = 0;  // normalized trapezoid
static Sample playPrev;                               // sample at/behind playback time
static float playDelay = 0.03f;                       // current playback delay (s)
static volatile bool ctrlBusy = false;

static void writeVel(uint8_t i, float v) {
    vel[i] = v;
    const int32_t vq = (int32_t)(v * VEL_TO_Q);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { velQ[i] = vq; }
}

// Synchronized trapezoid from the current reference to mbTarget.
static void beginTrapezoid() {
    ringTail = ringHead;                    // a single move replaces any stream
    float dmax = 0.0f;
    for (uint8_t i = 0; i < AXES; ++i) {
        tgt[i] = mbTarget[i];
        segStart[i] = ref[i];
        segDelta[i] = tgt[i] - ref[i];
        const float d = fabsf(segDelta[i]);
        if (d > dmax) dmax = d;
    }
    if (dmax < 1e-3f) {
        for (uint8_t i = 0; i < AXES; ++i) ref[i] = tgt[i];
        refMode = REF_HOLD;
        return;
    }
    const float V = mbVmax, A = mbAccel;
    if (dmax * A > V * V) { trTa = V / A; trT = dmax / V + trTa; }
    else                  { trTa = sqrtf(dmax / A); trT = 2.0f * trTa; }
    trAn = A / dmax;
    trVn = trAn * trTa;
    float n = ceilf(trT * CTRL_HZ);
    if (n < 1.0f) n = 1.0f;
    if (n > 60000.0f) n = 60000.0f;
    segN = (uint16_t)n;
    segK = 0;
    refMode = REF_TRAP;
}

static void controlTick() {
    const uint8_t mode = ctrlMode;
    if (!motionOn || mode == CTRL_OFF) return;

    if (mode == CTRL_JOG) {                // homing: ramp each motor to its jog speed
        for (uint8_t i = 0; i < AXES; ++i) {
            float dv = jogVel[i] - vel[i];
            if (dv >  DV_HOME) dv =  DV_HOME;
            if (dv < -DV_HOME) dv = -DV_HOME;
            writeVel(i, vel[i] + dv);
        }
        return;
    }

    if (mode == CTRL_STOP) {               // smooth deceleration to zero
        bool stopped = true;
        for (uint8_t i = 0; i < AXES; ++i) {
            float v = vel[i];
            if (v > DV_MAX) v -= DV_MAX; else if (v < -DV_MAX) v += DV_MAX; else v = 0.0f;
            if (v != 0.0f) stopped = false;
            writeVel(i, v);
        }
        if (stopped) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                if (ctrlMode == CTRL_STOP) { motionOn = false; ctrlMode = CTRL_OFF; }
            }
            moveDone = true;
        }
        return;
    }

    // ---- CTRL_RUN ----
    if (streaming && ++heartbeat > WATCHDOG_CTRL_TICKS) {
        streaming = false;
        ctrlMode = CTRL_STOP;
        motorFault = 1;
        return;
    }
    if (mbReady) { mbReady = false; beginTrapezoid(); }

    // Adapt the playback delay smoothly (fast up, slow down). Changing it
    // only stretches time slightly; it never makes the reference jump.
    float dD = streamDelayTarget - playDelay;
    if (dD >  0.25f * DT) dD =  0.25f * DT;
    if (dD < -0.02f * DT) dD = -0.02f * DT;
    playDelay += dD;
    const float timeScale = 1.0f - dD * CTRL_HZ;   // d(playback time)/dt

    const uint32_t now = nowTicks();
    const uint32_t playT = now - (uint32_t)(playDelay * 2000000.0f);

    // New stream data while not streaming: start playback from where the
    // reference is now, so position and speed stay continuous.
    if (refMode != REF_PLAY && ringTail != ringHead) {
        playPrev.t = playT;
        for (uint8_t i = 0; i < AXES; ++i) playPrev.q[i] = ref[i];
        refMode = REF_PLAY;
    }

    // Reference for this tick.
    float vref[AXES] = {0};
    bool refMoving = false;
    if (refMode == REF_TRAP) {
        float s = 1.0f, sd = 0.0f;
        ++segK;
        if (segK >= segN) {
            refMode = REF_HOLD;
        } else {
            float t = segK * DT;
            if (t > trT) t = trT;
            if (t < trTa) {
                s = 0.5f * trAn * t * t;           sd = trAn * t;
            } else if (t < trT - trTa) {
                s = 0.5f * trAn * trTa * trTa + trVn * (t - trTa); sd = trVn;
            } else {
                const float tr = trT - t;
                s = 1.0f - 0.5f * trAn * tr * tr;  sd = trAn * tr;
            }
            refMoving = true;
        }
        for (uint8_t i = 0; i < AXES; ++i) {
            ref[i] = refMoving ? segStart[i] + segDelta[i] * s : tgt[i];
            vref[i] = segDelta[i] * sd;
        }
    } else if (refMode == REF_PLAY) {
        // Drop samples the playback time has passed.
        while (ringTail != ringHead && (int32_t)(playT - ring[ringTail].t) >= 0) {
            playPrev = ring[ringTail];
            ringTail = (uint8_t)((ringTail + 1) & (STREAM_RING - 1));
        }
        if (ringTail != ringHead) {
            // Interpolate between the samples around the playback time.
            const Sample& nx = ring[ringTail];
            const float span = (float)(int32_t)(nx.t - playPrev.t);
            float f = span > 0.0f ? (float)(int32_t)(playT - playPrev.t) / span : 1.0f;
            if (f < 0.0f) f = 0.0f;
            if (f > 1.0f) f = 1.0f;
            const float rate = span > 0.0f ? 2000000.0f / span * timeScale : 0.0f;
            for (uint8_t i = 0; i < AXES; ++i) {
                const float d = nx.q[i] - playPrev.q[i];
                ref[i] = playPrev.q[i] + f * d;
                vref[i] = d * rate;
                tgt[i] = nx.q[i];
            }
            refMoving = true;
        } else {
            // Buffer ran dry: hold the last received target.
            for (uint8_t i = 0; i < AXES; ++i) ref[i] = tgt[i] = playPrev.q[i];
        }
    } else {
        for (uint8_t i = 0; i < AXES; ++i) ref[i] = tgt[i];
    }

    const float dvMax = (refMode == REF_PLAY) ? DV_MAX
                      : (mbAccel * TRACK_ACCEL_FACTOR / CTRL_HZ);

    bool allStopped = true;
    for (uint8_t i = 0; i < AXES; ++i) {
        int32_t pq;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { pq = posQ[i]; }
        const float p = (float)pq * (1.0f / 65536.0f);

        float vc = vref[i] + KP_PER_S * (ref[i] - p);
        if (vc >  VMAX_USTEPS_S) vc =  VMAX_USTEPS_S;
        if (vc < -VMAX_USTEPS_S) vc = -VMAX_USTEPS_S;
        float dv = vc - vel[i];
        if (dv >  dvMax) dv =  dvMax;
        if (dv < -dvMax) dv = -dvMax;
        float v = vel[i] + dv;

        if (!refMoving && fabsf(tgt[i] - p) < SETTLE_USTEPS && fabsf(v) <= dvMax)
            v = 0.0f;
        if (v != 0.0f) allStopped = false;
        writeVel(i, v);
    }
    moveDone = !refMoving && allStopped && !mbReady && ringTail == ringHead;
}

ISR(TIMER4_COMPA_vect, ISR_NOBLOCK) {
    if (ctrlBusy) return;
    ctrlBusy = true;
    controlTick();
    ctrlBusy = false;
}

// Start (or re-start) the controller in the given mode (main context).
// If motors are still moving (e.g. START during a HOLD ramp-down), their
// current speed is kept, so there is no jerk.
static void controllerStart(uint8_t mode) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        const bool moving = motionOn && ctrlMode != CTRL_OFF;
        for (uint8_t i = 0; i < AXES; ++i) {
            if (!moving) {
                posQ[i] = (int32_t)stepPos[i] * 65536L;
                velQ[i] = 0;
                vel[i] = 0.0f;
            }
            ref[i] = tgt[i] = (float)posQ[i] * (1.0f / 65536.0f);
            jogVel[i] = 0.0f;
        }
        refMode = REF_HOLD;
        segN = 0;
        ringHead = ringTail = 0;
        playDelay = streamDelayTarget;
        mbReady = false;
        heartbeat = 0;
        moveDone = !moving;
        ctrlMode = mode;
        motionOn = true;
    }
}

// Post a single target, reached with a synchronized trapezoid (main context).
static void postTarget(const float* t, float vmax, float accel) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (uint8_t i = 0; i < AXES; ++i) mbTarget[i] = t[i];
        mbVmax = vmax;
        mbAccel = accel;
        mbReady = true;
        moveDone = false;
        heartbeat = 0;
    }
}

// Append a stream sample with its arrival time (main context).
static void pushSample(uint32_t t, const float* q) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        const uint8_t next = (uint8_t)((ringHead + 1) & (STREAM_RING - 1));
        if (next == ringTail) return;        // full (should not happen): skip
        ring[ringHead].t = t;
        for (uint8_t i = 0; i < AXES; ++i) ring[ringHead].q[i] = q[i];
        ringHead = next;
        moveDone = false;
        heartbeat = 0;
    }
}

// ===================== STOPS =====================
static bool fault = false;
static uint8_t homeState = IDLE;
static bool homed = false;
static bool linkActive = false;          // MATLAB said HELLO and is still talking
static uint32_t lastLineTicks = 0;       // time of the last complete command line
static const uint32_t LINK_TIMEOUT_TICKS = (uint32_t)(LINK_TIMEOUT_S * 2000000.0f);

static void hardStop() {                       // instant (faults only)
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        haltAllFromIsr();
        mbReady = false;
        moveDone = true;
        for (uint8_t i = 0; i < AXES; ++i) { vel[i] = 0.0f; jogVel[i] = 0.0f; }
        stepsLow();
        stepIsHigh = false;
    }
}
static void smoothStop() {                     // ramp down
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        streaming = false;
        mbReady = false;
        if (ctrlMode == CTRL_RUN) ctrlMode = CTRL_STOP;
    }
}
static void stopFault(const char* reason, const char* axis = nullptr) {
    hardStop();
    fault = true;
    homeState = IDLE;
    homed = false;
    limitsOn = true;
    print("ERR STOPPED "); print(reason);
    if (axis) { putc0(' '); print(axis); }
    print("\r\n");
    println("RESET_REQUIRED");
}

// ===================== SIX MOTOR HOMING =====================
static bool stageDone[AXES];
static int16_t stageStart[AXES];
static uint32_t stageStartTicks = 0;

// Stop one motor immediately (switch reached). Main context.
static void stopAxis(uint8_t i) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        jogVel[i] = 0.0f;
        vel[i] = 0.0f;
        velQ[i] = 0;
        posQ[i] = (int32_t)stepPos[i] * 65536L;  // drop any half-finished step
    }
}

static void beginJogStage(uint8_t s) {
    homeState = s;
    stageStartTicks = nowTicks();
    controllerStart(CTRL_JOG);
    const float speed = (s == SLOW) ? HOME_FINAL_USTEPS_S : HOME_APPROACH_USTEPS_S;
    for (uint8_t i = 0; i < AXES; ++i) {
        stageStart[i] = stepPos[i];
        stageDone[i] = (s == RELEASE && !pressed(i));
        if (stageDone[i]) continue;
        const float dirSign = (s == RELEASE) ? awaySign(i) : -awaySign(i);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { jogVel[i] = speed * dirSign; }
    }
}

// Synchronized smooth move. absolute=false: away[i] is added to the current
// position; absolute=true: away[i] is the target distance from the switch.
static void beginMoveStage(uint8_t s, const int16_t* away, bool absolute,
                           float vmax, float accel) {
    homeState = s;
    stageStartTicks = nowTicks();
    controllerStart(CTRL_RUN);
    float t[AXES];
    for (uint8_t i = 0; i < AXES; ++i) {
        const int16_t base = absolute ? 0 : stepPos[i];
        t[i] = (float)(base + awaySign(i) * away[i]);
    }
    postTarget(t, vmax, accel);
}

static void printZ() {                         // "20.45" from 2045
    printNum(Z_HEIGHT_CM_X100 / 100);
    putc0('.');
    const int frac = Z_HEIGHT_CM_X100 % 100;
    if (frac < 10) putc0('0');
    printNum(frac);
}

static void startHome() {
    if (!fault && homeState != IDLE) { println("HOMING"); return; }   // already running
    if (fault || streaming || homeState != IDLE || ctrlMode != CTRL_OFF) {
        println("ERR BUSY"); return;
    }
    homed = false;
    limitsOn = false;                         // true position unknown until SLOW stage
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (uint8_t i = 0; i < AXES; ++i) {
            enableMotor(i);
            stepPos[i] = 0;
            posQ[i] = 0;
        }
    }
    println("HOMING");
    beginJogStage(RELEASE);
}

static void homeTick() {
    if (homeState == IDLE || fault) return;
    if ((uint32_t)(nowTicks() - stageStartTicks) > HOME_STAGE_TIMEOUT_S * 2000000UL) {
        stopFault("HOME_TIMEOUT"); return;
    }

    // ---- Smooth moves: BACKOFF and ZMOVE ----
    if (homeState == BACKOFF || homeState == ZMOVE) {
        if (!moveDone || ctrlMode != CTRL_RUN) return;
        for (uint8_t i = 0; i < AXES; ++i)
            if (pressed(i)) {
                stopFault(homeState == BACKOFF ? "SWITCH_STUCK" : "ENDSTOP_STILL_ACTIVE",
                          MOTOR_NAME[i]);
                return;
            }
        if (homeState == BACKOFF) {
            println("BACKOFF_ALL_DONE");
            beginJogStage(SLOW);
        } else {
            hardStop();                       // already stationary; torque stays on
            homeState = IDLE;
            homed = true;
            print("AT_Z "); printZ(); print("\r\n");
            println("HOMED_ALL");
        }
        return;
    }

    // ---- Switch searches: RELEASE, FAST, SLOW ----
    const uint16_t limit = (homeState == RELEASE) ? MAX_RELEASE_STEPS :
                           (homeState == FAST) ? MAX_FAST_STEPS : MAX_SLOW_STEPS;
    bool allDone = true;
    for (uint8_t i = 0; i < AXES; ++i) {
        if (stageDone[i]) continue;
        const bool reached = (homeState == RELEASE) ? !pressed(i) : pressed(i);
        if (reached) {
            stopAxis(i);
            stageDone[i] = true;
            if (homeState == SLOW) {          // THIS is the 21-degree zero
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { stepPos[i] = 0; posQ[i] = 0; }
            }
            continue;
        }
        int16_t pos;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { pos = stepPos[i]; }
        if ((uint16_t)abs(pos - stageStart[i]) >= limit) {
            stopFault("HOME_TRAVEL_LIMIT", MOTOR_NAME[i]); return;
        }
        allDone = false;
    }
    if (!allDone) return;

    if (homeState == RELEASE) {
        beginJogStage(FAST);
    } else if (homeState == FAST) {
        println("FAST_HOME_ALL_FOUND");
        int16_t backoff[AXES];
        for (uint8_t i = 0; i < AXES; ++i) backoff[i] = BACKOFF_STEPS;
        beginMoveStage(BACKOFF, backoff, false, HOME_APPROACH_USTEPS_S, HOME_ACCEL_USTEPS_S2);
    } else {                                  // SLOW
        println("SLOW_HOME_ALL_FOUND");
        limitsOn = true;                      // positions now known
        println("MOVING_TO_Z");
        beginMoveStage(ZMOVE, NEUTRAL_AWAY_USTEPS, true, ZMOVE_VMAX_USTEPS_S, ZMOVE_ACCEL_USTEPS_S2);
    }
}

// ===================== MATLAB SERIAL COMMANDS =====================
static uint32_t lastSeq = 0;
static bool haveSeq = false;
static uint32_t lastChangeTicks = 0;   // last SETF whose target changed
static uint32_t lastPacketTicks = 0;   // last SETF of any kind (incl. heartbeats)
static float lastPosted[AXES];
static float worstGap = 0.02f;         // recent worst gap between stream samples (s)

static void status() {
    int16_t p[AXES];
    bool active;
    uint8_t mode;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (uint8_t i = 0; i < AXES; ++i) p[i] = stepPos[i];
        active = streaming;
        mode = ctrlMode;
    }
    print("STATUS ");
    print(fault ? "STOPPED" : homeState != IDLE ? "HOMING" :
          active ? "STREAMING" : mode == CTRL_STOP ? "STOPPING" :
          homed ? "HOMED" : "NOT_HOMED");
    for (uint8_t i = 0; i < AXES; ++i) { putc0(' '); printNum(p[i]); }
    print("\r\n");
}
// "NEUTRAL <z_cm*100> n0..n5" in the same signed units as STATUS.
static void neutralReport() {
    print("NEUTRAL ");
    printNum(Z_HEIGHT_CM_X100);
    for (uint8_t i = 0; i < AXES; ++i) {
        putc0(' ');
        printNum((long)awaySign(i) * NEUTRAL_AWAY_USTEPS[i]);
    }
    print("\r\n");
}
static void startStream() {
    if (fault || !homed || homeState != IDLE) { println("ERR HOME_REQUIRED"); return; }
    if (ctrlMode == CTRL_RUN || ctrlMode == CTRL_JOG) { println("ERR BUSY"); return; }
    controllerStart(CTRL_RUN);                // also smooth if a HOLD is still ramping down
    streaming = true;
    haveSeq = false;
    for (uint8_t i = 0; i < AXES; ++i) lastPosted[i] = tgt[i];
    println("STREAM_READY");
}
static void resetCommand() {
#if ESTOP_INPUT_ENABLED
    if (estopInputActive()) { println("ERR ESTOP_ACTIVE"); return; }
#endif
    hardStop();
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        estopLatched = false;
        motorFault = 0;
    }
    fault = false;
    homed = false;
    homeState = IDLE;
    limitsOn = true;
    println("RESET_OK");
}
static bool parseSetf(char* args) {
    char* token = strtok(args, " ,\t\r");
    if (!token) return false;
    char* end;
    uint32_t seq = strtoul(token, &end, 10);
    if (end == token || *end != '\0') return false;
    float next[AXES];
    for (uint8_t i = 0; i < AXES; ++i) {
        token = strtok(nullptr, " ,\t\r");
        if (!token) return false;
        long q = strtol(token, &end, 10);
        if (end == token || *end != '\0') return false;
        const int32_t awayQ = (int32_t)q * awaySign(i);
        if (awayQ < 0 || awayQ > (int32_t)MAX_AWAY_STEPS * POSITION_SCALE)
            return false;
        next[i] = (float)q * (1.0f / POSITION_SCALE);
    }
    if (strtok(nullptr, " ,\t\r") != nullptr) return false;
    if (haveSeq && seq < lastSeq) return true;          // stale packet: ignore
    lastSeq = seq;

    bool changed = !haveSeq;
    for (uint8_t i = 0; i < AXES; ++i)
        if (next[i] != lastPosted[i]) changed = true;
    const uint32_t now = nowTicks();
    const bool heldBefore = haveSeq && lastPacketTicks != lastChangeTicks;
    const float sinceChange = (float)(uint32_t)(now - lastChangeTicks) * 0.5e-6f;
    const float sincePacket = (float)(uint32_t)(now - lastPacketTicks) * 0.5e-6f;

    if (!changed) {                                     // heartbeat: target unchanged
        lastPacketTicks = now;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { heartbeat = 0; }
        return true;
    }

    // Would following this change as a stream need more than 2x VMAX?
    float dmax = 0.0f;
    for (uint8_t i = 0; i < AXES; ++i) {
        const float d = fabsf(next[i] - lastPosted[i]);
        if (d > dmax) dmax = d;
    }
    const float gap = heldBefore ? sincePacket : sinceChange;
    const bool bigJump = dmax > 2.0f * VMAX_USTEPS_S * (gap > 0.004f ? gap : 0.004f);

    if (!haveSeq || sincePacket > STREAM_MAX_PERIOD_S || bigJump) {
        // First target, or after a pause, or a large jump: one smooth
        // synchronized trapezoid move.
        postTarget(next, VMAX_USTEPS_S, ACCEL_USTEPS_S2);
    } else {
        // Continuous stream. If MATLAB had been repeating the same target
        // (holding still), record that it was still there at the last
        // repeat, so the move starts then instead of being stretched out.
        if (heldBefore) pushSample(lastPacketTicks, lastPosted);
        pushSample(now, next);
        // Playback delay follows the recent worst gap (decays ~30%/s).
        float decayed = worstGap * (1.0f - 0.35f * gap);
        if (decayed < 0.0f) decayed = 0.0f;
        worstGap = gap > decayed ? gap : decayed;
        float d = STREAM_DELAY_MARGIN * worstGap + 0.004f;
        if (d < STREAM_DELAY_MIN_S) d = STREAM_DELAY_MIN_S;
        if (d > STREAM_DELAY_MAX_S) d = STREAM_DELAY_MAX_S;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { streamDelayTarget = d; }
    }
    for (uint8_t i = 0; i < AXES; ++i) lastPosted[i] = next[i];
    lastChangeTicks = lastPacketTicks = now;
    haveSeq = true;
    return true;
}
// MATLAB handshake / keep-alive. Accepted in every state, even after a fault.
static bool linkCommand(const char* line) {
    if (strcmp(line,"HELLO STEWART") == 0) { linkActive = true; println("HELLO_OK"); return true; }
    if (strcmp(line,"PING") == 0) return true;
    return false;
}
static void command(char* line) {
    // Without the MATLAB handshake only harmless queries are allowed.
    if (!linkActive && strcmp(line,"STATUS") != 0 && strcmp(line,"NEUTRAL") != 0 &&
        strcmp(line,"RESET") != 0) {
        println("ERR NO_MATLAB"); return;
    }
    if (strcmp(line,"HOME") == 0 || strcmp(line,"Home") == 0) { startHome(); return; }
    if (strcmp(line,"START") == 0 || strcmp(line,"Start") == 0) { startStream(); return; }
    if (strcmp(line,"STATUS") == 0) { status(); return; }
    if (strcmp(line,"NEUTRAL") == 0) { neutralReport(); return; }
    if (strcmp(line,"RESET") == 0) { resetCommand(); return; }
    if (strcmp(line,"HOLD") == 0) {
        if (homeState != IDLE) { stopFault("HOMING_ABORTED"); return; }
        smoothStop(); println("HOLDING"); return;
    }
    if (strncmp(line,"SETF ",5) == 0) {
        if (!streaming || !homed) { println("ERR STREAM_NOT_READY"); return; }
        if (!parseSetf(line+5)) stopFault("INVALID_TARGET");
        return;
    }
    println("ERR UNKNOWN_COMMAND");
}
static char commandLine[120];
static uint8_t commandLength = 0;
static void readCommands() {
    uint8_t errors = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        errors = rxErrorFlags;
        if (errors) {
            rxErrorFlags = 0;
            rxTail = rxHead;
        }
    }
    if (errors) {
        commandLength = 0;
        const char* reason = (errors & RXERR_OVERRUN) ? "UART_DATA_OVERRUN" :
                             (errors & RXERR_FRAME)   ? "UART_FRAMING_ERROR" :
                             (errors & RXERR_PARITY)  ? "UART_PARITY_ERROR" :
                                                       "UART_RX_BUFFER_OVERFLOW";
        if (!fault && (streaming || homeState != IDLE || ctrlMode != CTRL_OFF))
            stopFault(reason);
        else { print("WARN "); println(reason); }
        return;
    }
    while (rxTail != rxHead) {
        const char c = (char)rx[rxTail++];
        if (c == '\n') {
            commandLine[commandLength] = '\0';
            if (commandLength) {
                lastLineTicks = nowTicks();
                if (linkCommand(commandLine)) { /* handled */ }
                else if (!fault) command(commandLine);
                else if (strcmp(commandLine, "RESET") == 0) resetCommand();
                else if (strcmp(commandLine, "STATUS") == 0) status();
                else println("ERR RESET_REQUIRED");
            }
            commandLength = 0;
        } else if (c != '\r') {
            if (commandLength < sizeof(commandLine)-1)
                commandLine[commandLength++] = c;
            else { commandLength = 0; if (!fault) stopFault("SERIAL_LINE_OVERFLOW"); return; }
        }
        if (estopLatched && !fault) return;   // report the E-stop first
    }
}

// ===================== HARDWARE SETUP =====================
static void setupHardware() {
    // Active-LOW drivers: disable before changing DDR.
    PORTL |= _BV(PL1);
    PORTF |= _BV(PF1);
    PORTK |= _BV(PK0);
    PORTA |= _BV(PA2) | _BV(PA5);
    PORTG |= _BV(PG2);

    DDRH |= _BV(PH0) | _BV(PH1);
    DDRF |= _BV(PF0) | _BV(PF1) | _BV(PF2) | _BV(PF3);
    DDRA |= _BV(PA0) | _BV(PA1) | _BV(PA2) | _BV(PA3) |
            _BV(PA4) | _BV(PA5) | _BV(PA6) | _BV(PA7);
    DDRL |= _BV(PL1) | _BV(PL2);
    DDRK |= _BV(PK0);
    DDRG |= _BV(PG2);
    stepsLow();
    for (uint8_t i = 0; i < AXES; ++i) setDir(i, false);

    DDRC &= (uint8_t)~0x3FU;
    PORTC |= 0x3FU; // six endstop input pullups
    rawSwitch = stableSwitch = (uint8_t)(~PINC) & 0x3FU;

#if ESTOP_INPUT_ENABLED
    ESTOP_DDR_REG  &= (uint8_t)~_BV(ESTOP_BIT);
    ESTOP_PORT_REG |= _BV(ESTOP_BIT);   // pull-up
#endif

    // Timer1 free-running at 2 MHz: time base.
    TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
    TIFR1 = _BV(TOV1);
    TIMSK1 = _BV(TOIE1);
    TCCR1B = _BV(CS11);

    // Timer3: step generator, CTC at STEP_ISR_HZ.
    TCCR3A = 0; TCCR3B = 0; TCNT3 = 0;
    OCR3A = (uint16_t)(2000000UL / STEP_ISR_HZ - 1UL);
    TIFR3 = _BV(OCF3A);
    TIMSK3 = _BV(OCIE3A);
    TCCR3B = _BV(WGM32) | _BV(CS31);

    // Timer4: motion controller, CTC at CTRL_HZ.
    TCCR4A = 0; TCCR4B = 0; TCNT4 = 0;
    OCR4A = (uint16_t)(2000000UL / CTRL_HZ - 1UL);
    TIFR4 = _BV(OCF4A);
    TIMSK4 = _BV(OCIE4A);
    TCCR4B = _BV(WGM42) | _BV(CS41);

    // UART0: 16 MHz, double speed, UBRR=7 -> 250000 baud exactly.
    UCSR0A = _BV(U2X0); UBRR0H = 0; UBRR0L = 7;
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
    UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
    sei();
    println("READY - HELLO, PING, HOME, START, SETF, STATUS, NEUTRAL, HOLD, RESET, e, !");
    println("WAITING_FOR_MATLAB");
}

// Stop and lock if MATLAB stops talking.
static void linkWatchdog() {
    if (!linkActive) return;
    if ((uint32_t)(nowTicks() - lastLineTicks) <= LINK_TIMEOUT_TICKS) return;
    linkActive = false;
    if (fault) return;
    if (homeState != IDLE) {
        stopFault("LINK_LOST");                // homing moves are slow: stop now
    } else if (streaming || ctrlMode == CTRL_RUN) {
        smoothStop();
        println("WARN LINK_LOST_HOLD");
    } else {
        println("WARN LINK_LOST");
    }
}

static void mainLoopOnce() {
    if (estopLatched && !fault)
        stopFault(estopSource == 2 ? "HARDWARE_ESTOP" : "SOFTWARE_ESTOP");
    if (motorFault) {
        uint8_t reason, axis;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            reason = motorFault; axis = motorFaultAxis; motorFault = 0;
        }
        if (reason == 1) println("WARN STREAM_WATCHDOG_HOLD");
        else if (!fault) stopFault(reason == 2 ? "AXIS_SOFT_LIMIT" : "UNEXPECTED_ENDSTOP",
                                   MOTOR_NAME[axis]);
    }
    readCommands();
    readSwitches();
    linkWatchdog();
    homeTick();
}

int main(void) {
    cli();
    setupHardware();
    for (;;) mainLoopOnce();
}