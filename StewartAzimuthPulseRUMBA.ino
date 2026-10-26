/*
  STEWART STREAM RUMBA / ATmega2560
  Six independent simultaneous homing state machines; 21 degrees DOWN = 0 steps.
  SETF seq QX QY QZ QE0 QE1 QE2: signed targets in 1/256 microsteps.
  Continuous target replacement: SET seq X Y Z E0 E1 E2
  V2: fractional targets, target-velocity feedforward, continuous phase
  accumulation and acceleration limits. No command queue.
  Configurable velocity and acceleration limits; no unverified speed increase.
  THESE ARE TEST SETTINGS; VALIDATE torque/current and travel unloaded.
  No stored IK trajectory, no waiting for DONE between target updates.
  Software E-stop: a single 'e' or 'E' at start of a serial command.
  TEST ONLY WITH CONNECTING RODS REMOVED and plate independently supported.
  Homing with all six rods attached has NOT been validated.
  Software stop retains driver enable; NOT a safety-rated E-stop.
*/
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

const byte AXES = 6;
const byte STEP_PIN[AXES]   = {17,54,57,23,26,29};
const byte DIR_PIN[AXES]    = {16,47,56,22,25,28};
const byte ENABLE_PIN[AXES] = {48,55,62,24,27,39};
const byte SWITCH_PIN[AXES] = {37,36,35,34,33,32};
const byte HOME_DIR[AXES]   = {HIGH,LOW,HIGH,LOW,HIGH,LOW};
const byte SWITCH_ACTIVE   = LOW; // verified; disconnected NO switch may appear inactive
const char* AXIS_NAME[AXES] = {"X","Y","Z","E0","E1","E2"};

// Homing: gentle rates used previously.
const unsigned long HOME_APPROACH_US = 15000UL;
const unsigned long HOME_FINAL_US = 24000UL;
const unsigned long DEBOUNCE_US = 5000UL;
const long MAX_SEARCH = 800;
const long MAX_RELEASE = 240;
const long BACKOFF = 80;
const long MAX_FINAL = BACKOFF + 40;

// Conservative per-axis position servo. Increase only after unloaded testing.
const unsigned long MOTION_TICK_US = 1000UL; // 1 kHz internal velocity update
// STEP pulses are scheduled separately at microsecond time resolution.
const float MAX_RATE_STEPS_S = 1200.0f; // UNVERIFIED ceiling, reduce after overheat
const float MAX_ACCEL_STEPS_S2 =2400.0f; // UNVERIFIED; measure before increasing
const float POSITION_GAIN_S = 12.0f; // lower gain reduces microstep hunting
const float VELOCITY_FF_BLEND = 0.60f; // smoother estimate of new target velocity
const float FF_MAX_AGE_US = 90000.0f; // stop extrapolating stale setpoints
const long POSITION_SCALE = 256L;   // 1/256 microstep target resolution
const unsigned long STREAM_TIMEOUT_US = 250000UL; // watchdog: 250 ms without a new target
const unsigned long MIN_STEP_INTERVAL_US = 800UL; // pulse pacing, similar to working 1 ms delay
const long MAX_AWAY_STEPS = 1000; // not a verified mechanical limit
const long SWITCH_TOLERANCE_STEPS = 80;

enum HomePhase : byte {H_IDLE,H_RELEASE,H_SEARCH,H_BACKOFF,H_FINAL,H_DONE};
HomePhase phase[AXES];
long homeCounter[AXES] = {0};
unsigned long lastHomePulseUs[AXES] = {0};
bool rawSwitch[AXES], stableSwitch[AXES];
unsigned long switchChangedUs[AXES];
long motorPosition[AXES] = {0};
long desiredSteps[AXES] = {0}; // legacy integer view for diagnostics/homing
float desiredPosition[AXES] = {0}; // microsteps, fractional
float prevReceivedPosition[AXES] = {0};
float feedRate[AXES] = {0};        // microsteps/s
float motorRate[AXES] = {0};       // acceleration-limited microsteps/s
float pulsePhase[AXES] = {0};      // fractional microstep integration
unsigned long lastPulseEvalUs = 0;
unsigned long lastStepUs[AXES] = {0};
unsigned long lastMotionUs = 0;
unsigned long lastTargetUs = 0;
unsigned long lastSeq = 0;
unsigned long packetsReceived = 0;
bool homing = false, allHomed = false, streaming = false, fault = false;
char lineBuffer[110];
byte lineLength = 0;

byte opposite(byte direction) { return direction == HIGH ? LOW : HIGH; }
int awaySign(byte i) { return HOME_DIR[i] == HIGH ? -1 : 1; }
bool hit(byte i) { return stableSwitch[i] == SWITCH_ACTIVE; }

void faultStop(const __FlashStringHelper *why) {
  if (fault) return;
  fault = true; homing = false; streaming = false; allHomed = false;
  for (byte i=0;i<AXES;i++) {
    desiredSteps[i] = motorPosition[i];
    desiredPosition[i] = (float)motorPosition[i];
    prevReceivedPosition[i] = desiredPosition[i];
    motorRate[i] = 0; feedRate[i] = 0; pulsePhase[i] = 0;
    digitalWrite(STEP_PIN[i],LOW);
  }
  // Driver enables remain ON to retain holding torque. Disconnect motor power
  // using the external hardwired stop if current/temperature is unsafe.
  Serial.print(F("ERR STOPPED ")); Serial.println(why);
  Serial.println(F("RESET_REQUIRED"));
}

void updateSwitches() {
  unsigned long now=micros();
  for(byte i=0;i<AXES;i++) {
    bool reading=digitalRead(SWITCH_PIN[i]);
    if(reading!=rawSwitch[i]) {rawSwitch[i]=reading;switchChangedUs[i]=now;}
    if((unsigned long)(now-switchChangedUs[i])>=DEBOUNCE_US)
      stableSwitch[i]=reading;
  }
}

void enterPhase(byte i, HomePhase p) {
  phase[i]=p;homeCounter[i]=0;lastHomePulseUs[i]=micros();
}

void beginHome() {
  if(fault||homing||streaming) {Serial.println(F("ERR BUSY"));return;}
  allHomed=false;
  for(byte i=0;i<AXES;i++) {
    digitalWrite(ENABLE_PIN[i],LOW); // motor holding enabled
    motorRate[i]=0; feedRate[i]=0; pulsePhase[i]=0;
    desiredPosition[i]=prevReceivedPosition[i]=(float)motorPosition[i];
    enterPhase(i,hit(i)?H_RELEASE:H_SEARCH);
  }
  homing=true;
  Serial.println(F("HOMING"));
}

void tickHome() {
  if(!homing||fault)return;
  bool done=true;
  for(byte i=0;i<AXES;i++) {
    if(phase[i]==H_DONE) continue;
    done=false;
    if(phase[i]==H_RELEASE&&!hit(i))enterPhase(i,H_SEARCH);
    if(phase[i]==H_SEARCH&&hit(i)) {
      enterPhase(i,H_BACKOFF);
      Serial.print(AXIS_NAME[i]);Serial.println(F(" SWITCH_FOUND"));
    }
    if(phase[i]==H_BACKOFF&&homeCounter[i]>=BACKOFF) {
      if(hit(i)){faultStop(F("SWITCH_STUCK"));return;}
      enterPhase(i,H_FINAL);
    }
    if(phase[i]==H_FINAL&&hit(i)) {
      // Each switch's SECOND, slow contact becomes exactly ZERO STEPS.
      motorPosition[i]=desiredSteps[i]=0;
      desiredPosition[i]=prevReceivedPosition[i]=0.0f;
      motorRate[i]=feedRate[i]=pulsePhase[i]=0.0f;
      phase[i]=H_DONE;
      Serial.print(AXIS_NAME[i]); Serial.println(F(" ZERO 0"));
      continue;
    }
    if((phase[i]==H_RELEASE&&homeCounter[i]>=MAX_RELEASE)||
       (phase[i]==H_SEARCH&&homeCounter[i]>=MAX_SEARCH)||
       (phase[i]==H_FINAL&&homeCounter[i]>=MAX_FINAL)) {
      faultStop(F("HOME_SWITCH_OR_TRAVEL_FAILURE"));return;
    }
    unsigned long now=micros();
    unsigned long interval=phase[i]==H_FINAL?HOME_FINAL_US:HOME_APPROACH_US;
    if((unsigned long)(now-lastHomePulseUs[i])<interval)continue;
    lastHomePulseUs[i]=now;
    byte direction=(phase[i]==H_RELEASE||phase[i]==H_BACKOFF)?
       opposite(HOME_DIR[i]):HOME_DIR[i];
    digitalWrite(DIR_PIN[i],direction);
    delayMicroseconds(5);digitalWrite(STEP_PIN[i],HIGH);
    delayMicroseconds(5);digitalWrite(STEP_PIN[i],LOW);
    homeCounter[i]++;
  }
  if(done) {homing=false;allHomed=true;Serial.println(F("HOMED_ALL"));}
}

bool parseTarget(char *args, unsigned long &seq, long outputQ[AXES], bool fractional) {
  char *token=strtok(args," \t,\r");
  if(!token)return false;
  char *end=nullptr;
  seq=strtoul(token,&end,10);
  if(end==token||*end!='\0')return false;
  for(byte i=0;i<AXES;i++) {
    token=strtok(nullptr," \t,\r");
    if(!token)return false;
    long v=strtol(token,&end,10);
    if(end==token||*end!='\0')return false;
    // Avoid 32-bit multiplication overflow when receiving malformed values.
    int64_t scaled=(int64_t)v*(fractional?1L:POSITION_SCALE);
    int64_t away=scaled*(int64_t)awaySign(i);
    if(away<0||away>(int64_t)MAX_AWAY_STEPS*POSITION_SCALE)return false;
    outputQ[i]=(long)scaled;
  }
  return strtok(nullptr," \t,\r")==nullptr;
}

void beginStreaming() {
  if(fault||homing||!allHomed){Serial.println(F("ERR HOME_REQUIRED"));return;}
  streaming=true;
  for(byte i=0;i<AXES;i++) {
    desiredSteps[i]=motorPosition[i];
    desiredPosition[i]=prevReceivedPosition[i]=(float)motorPosition[i];
    motorRate[i]=feedRate[i]=pulsePhase[i]=0.0f;
    lastStepUs[i]=micros();
  }
  lastTargetUs=micros();
  lastMotionUs=micros();lastPulseEvalUs=micros();
  packetsReceived=0; lastSeq=0;
  Serial.println(F("STREAM_READY"));
}

void holdMotion() {
  streaming=false;
  for(byte i=0;i<AXES;i++) {
    desiredSteps[i]=motorPosition[i];
    desiredPosition[i]=prevReceivedPosition[i]=(float)motorPosition[i];
    motorRate[i]=feedRate[i]=pulsePhase[i]=0.0f;
  }
  Serial.println(F("HOLDING"));
}

void status() {
  Serial.print(F("STATUS "));
  Serial.print(fault?F("STOPPED"):(homing?F("HOMING"):(streaming?F("STREAMING"):
    (allHomed?F("HOMED"):F("NOT_HOMED")))));
  for(byte i=0;i<AXES;i++){Serial.print(' ');Serial.print(motorPosition[i]);}
  Serial.println();
}

void command(char *cmd) {
  if(fault)return;
  while(*cmd==' '||*cmd=='\t')cmd++;
  size_t n=strlen(cmd);
  while(n>0&&(cmd[n-1]==' '||cmd[n-1]=='\t'))cmd[--n]='\0';
  if(strcmp(cmd,"HOME")==0||strcmp(cmd,"Home")==0){beginHome();return;}
  if(strcmp(cmd,"START")==0||strcmp(cmd,"Start")==0){beginStreaming();return;}
  if(strcmp(cmd,"HOLD")==0){holdMotion();return;}
  if(strcmp(cmd,"STATUS")==0){status();return;}
  if(strncmp(cmd,"SETF ",5)==0 || strncmp(cmd,"SET ",4)==0) {
    if(!streaming||!allHomed){Serial.println(F("ERR STREAM_NOT_READY"));return;}
    bool fractional=(cmd[3]=='F');
    unsigned long seq;
    long valuesQ[AXES];
    // Save the original command before parseTarget modifies it
char originalCommand[sizeof(lineBuffer)];
strncpy(originalCommand, cmd, sizeof(originalCommand) - 1);
originalCommand[sizeof(originalCommand) - 1] = '\0';

    if (!parseTarget(cmd + (fractional ? 5 : 4),
                    seq, valuesQ, fractional)) {

      Serial.print(F("REJECTED_COMMAND: "));
      Serial.println(originalCommand);

      faultStop(F("INVALID_TARGET"));
      return;
    }
    if(packetsReceived>0 && seq<lastSeq) return; // ignore older commands
    unsigned long now=micros();
    float dt=(float)(now-lastTargetUs)*1.0e-6f;
    bool canEstimate=(packetsReceived>0 && dt>=0.005f && dt<=0.15f);
    for(byte i=0;i<AXES;i++) {
      float next=(float)valuesQ[i]/(float)POSITION_SCALE;
      if(canEstimate) {
        float observed=(next-prevReceivedPosition[i])/dt;
        if(observed>MAX_RATE_STEPS_S)observed=MAX_RATE_STEPS_S;
        if(observed<-MAX_RATE_STEPS_S)observed=-MAX_RATE_STEPS_S;
        feedRate[i]+=(observed-feedRate[i])*VELOCITY_FF_BLEND;
      } else {
        feedRate[i]=0.0f;
      }
      prevReceivedPosition[i]=next;
      desiredPosition[i]=next;
      desiredSteps[i]=lroundf(next);
    }
    lastSeq=seq;lastTargetUs=now;packetsReceived++;
    if(packetsReceived%50UL==0UL) {
      Serial.print(F("RX "));Serial.println(lastSeq);
    }
    return;
  }
  Serial.println(F("ERR UNKNOWN_COMMAND"));
}

void readCommands() {
  while(Serial.available()>0) {
    char c=(char)Serial.read();
    // Standalone 'e' stops immediately at the beginning of a command.
    // An 'e' embedded in Home must not accidentally trigger E-stop.
    if((c=='e'||c=='E')&&lineLength==0) {
      faultStop(F("SOFTWARE_ESTOP"));return;
    }
    if(fault)return;
    if(c=='\n'){
      lineBuffer[lineLength]='\0';
      if(lineLength)command(lineBuffer);
      lineLength=0;
    } else if(c!='\r'){
      if(lineLength<sizeof(lineBuffer)-1)lineBuffer[lineLength++]=c;
      else {lineLength=0;faultStop(F("SERIAL_RX_OVERFLOW"));return;}
    }
  }
}

// Update acceleration-limited velocities on a fixed control interval.
// The position error includes the fractional step-phase, so it does not jump
// by one whole microstep whenever a pulse is generated.
void tickMotorControl() {
  if(!streaming||fault)return;
  unsigned long now=micros();
  if((unsigned long)(now-lastTargetUs)>STREAM_TIMEOUT_US){
    faultStop(F("STREAM_WATCHDOG_TIMEOUT"));return;
  }
  unsigned long elapsed=now-lastMotionUs;
  if(elapsed<MOTION_TICK_US)return;
  lastMotionUs=now;
  float dt=(float)elapsed*1.0e-6f;
  if(dt>0.01f)dt=0.01f;
  bool ffFresh=(unsigned long)(now-lastTargetUs)<(unsigned long)FF_MAX_AGE_US;

  for(byte i=0;i<AXES;i++) {
    // Preserve motorRate[] across packets. Servo corrects fractional position
    // error while the feedforward term maintains continuity between updates.
    const float virtualPos=(float)motorPosition[i]+pulsePhase[i];
    const float error=desiredPosition[i]-virtualPos;
    const float ff=ffFresh?feedRate[i]:0.0f;
    float targetRate=ff+POSITION_GAIN_S*error;

    // If setpoint stops moving, slow down early enough to avoid hunting.
    // A moving feedforward target still gets bounded by maximum motor speed.
    if(!ffFresh || fabsf(ff)<0.01f) {
      const float stopRate=sqrtf(2.0f*MAX_ACCEL_STEPS_S2*fabsf(error));
      if(targetRate>stopRate) targetRate=stopRate;
      if(targetRate<-stopRate) targetRate=-stopRate;
    }
    if(targetRate>MAX_RATE_STEPS_S)targetRate=MAX_RATE_STEPS_S;
    if(targetRate<-MAX_RATE_STEPS_S)targetRate=-MAX_RATE_STEPS_S;

    // Acceleration AND deceleration are limited each 1-ms controller tick.
    const float maxChange=MAX_ACCEL_STEPS_S2*dt;
    const float requestedChange=targetRate-motorRate[i];
    const float appliedChange=requestedChange>maxChange?maxChange:
                              (requestedChange<-maxChange?-maxChange:requestedChange);
    motorRate[i]+=appliedChange;
  }
}

// Integrate the continuous speed command. STEP pulses occur only when a
// phase accumulator crosses one full microstep, independently of 2ms ticks.
// Physical microsteps are still discrete. Enforce rounded endpoint bounds.
void tickStepPulses() {
  if(!streaming||fault)return;
  unsigned long now=micros();
  unsigned long elapsed=now-lastPulseEvalUs;
  if(elapsed==0)return;
  lastPulseEvalUs=now;
  float dt=(float)elapsed*1.0e-6f;
  if(dt>0.02f)dt=0.02f; // no burst after a host stall
  int8_t pulse[AXES]={0};
  for(byte i=0;i<AXES;i++) {
    pulsePhase[i]+=motorRate[i]*dt;
    if(pulsePhase[i]>1.5f)pulsePhase[i]=1.5f;
    if(pulsePhase[i]<-1.5f)pulsePhase[i]=-1.5f;
    int8_t dir=pulsePhase[i]>=1.0f?1:(pulsePhase[i]<=-1.0f?-1:0);
    // Close a near-complete pulse at a fixed endpoint: otherwise a
    // proportional rate can asymptote to zero with a 0.999-step residual.
    if(!dir && fabsf(pulsePhase[i])>=0.97f &&
       lroundf(desiredPosition[i])!=motorPosition[i] &&
       (unsigned long)(now-lastStepUs[i])>=8000UL) {
      dir=pulsePhase[i]>0.0f?1:-1;
    }
    if(!dir)continue;
    if ((unsigned long)(now-lastStepUs[i]) < MIN_STEP_INTERVAL_US) continue;
    long nextPos=motorPosition[i]+dir;
    long roundedGoal=lroundf(desiredPosition[i]);
    // Never move beyond the newest rounded positional target.
    if((dir>0 && nextPos>roundedGoal)||(dir<0 && nextPos<roundedGoal)) {
      pulsePhase[i]=dir>0?0.49f:-0.49f;
      continue;
    }
    long away=nextPos*awaySign(i);
    if(away<0||away>MAX_AWAY_STEPS){faultStop(F("AXIS_SOFT_LIMIT"));return;}
    if(hit(i)&&away>SWITCH_TOLERANCE_STEPS){
      faultStop(F("UNEXPECTED_ENDSTOP"));return;
    }
    pulse[i]=dir;
    digitalWrite(DIR_PIN[i],dir>0?HIGH:LOW);
  }
  bool any=false;
  for(byte i=0;i<AXES;i++)if(pulse[i])any=true;
  if(any){
    delayMicroseconds(5); // DIR setup
    for(byte i=0;i<AXES;i++)if(pulse[i])digitalWrite(STEP_PIN[i],HIGH);
    delayMicroseconds(5);
    for(byte i=0;i<AXES;i++)if(pulse[i]){
      digitalWrite(STEP_PIN[i],LOW);
      motorPosition[i]+=pulse[i];
      pulsePhase[i]-=pulse[i];
      lastStepUs[i]=now;
    }
  }
}

void setup() {
  for(byte i=0;i<AXES;i++) {
    digitalWrite(ENABLE_PIN[i],HIGH);pinMode(ENABLE_PIN[i],OUTPUT);
    digitalWrite(STEP_PIN[i],LOW);pinMode(STEP_PIN[i],OUTPUT);
    digitalWrite(DIR_PIN[i],LOW);pinMode(DIR_PIN[i],OUTPUT);
    pinMode(SWITCH_PIN[i],INPUT_PULLUP);
    rawSwitch[i]=stableSwitch[i]=digitalRead(SWITCH_PIN[i]);
    switchChangedUs[i]=micros();phase[i]=H_IDLE;
  }
  Serial.begin(250000); // AVR Mega 16 MHz supports this exact nominal rate
  Serial.println(F("READY - HOME THEN START; STREAM SET; e STOPS"));
}

void loop() {
  readCommands();
  if(fault)return;
  updateSwitches();
  tickHome();
  tickMotorControl();
  tickStepPulses();
}
