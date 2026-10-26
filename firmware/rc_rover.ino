// RC CAR (ESP32) - WiFi access point + servo control.
// Board: ESP32 Dev Module. Library needed: "ESP32Servo" (Library Manager, latest version).
//
// Power: servos run from +BATT, ESP32 from the buck converter. All grounds common.
// Start with the arm folded UPRIGHT and the steering centred before powering on.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// ---------- WiFi ----------
const char*    WIFI_SSID = "RCCar";
const char*    WIFI_PASS = "12345678";   // change it on BOTH sketches (min 8 chars)
const uint16_t PORT      = 4210;

// ---------- Pins (from car schematic) ----------
const int PIN_DRIVE    = 13;  // SIG_DRIVE    - continuous
const int PIN_STEER    = 14;  // SIG_STEER    - 180 deg positional
const int PIN_GRIPPER  = 32;  // SIG_GRIP     - continuous
const int PIN_BASE     = 25;  // SIG_BASE     - 180 deg positional
const int PIN_SHOULDER = 26;  // SIG_SHOULDER - 180 deg positional
const int PIN_ELBOW    = 27;  // SIG_ELBOW    - 180 deg positional

const int FAILSAFE_MS = 500;  // stop if no packet for this long
const int UPDATE_MS   = 20;

// ---------- Drive (continuous) ----------
const int   DRIVE_STOP_US = 1500;   // trim the servo's screw (or this) until it sits still
const int   DRIVE_DIR     = 1;      // -1 reverses
const int   DRIVE_MAX_US  = 500;    // 500 = full speed; lower to cap top speed
const float DRIVE_ACCEL   = 4.0f;   // throttle units/sec; smooths starts (less current spike)

// ---------- Steering (180 deg servo) ----------
const int   STEER_CENTER_DEG = 90;  // adjust so the wheels point straight
const int   STEER_DIR        = 1;   // -1 reverses left/right
const float STEER_MAX_DEG    = 30;  // each side of centre; reduce if the rack binds
const float STEER_MAX_DPS    = 400; // steering speed limit

// ---------- Gripper (continuous) ----------
const int   GRIP_STOP_US   = 1500;
const int   GRIP_DIR       = 1;     // -1 swaps close/open direction
const int   GRIP_SPEED_US  = 300;
const float GRIP_TRAVEL_MS = 1500;  // CALIBRATE: ms at that speed for one full 360 turn

// ---------- Arm (180 deg servos) ----------
// Pose is in degrees from upright. servo angle = UPRIGHT + DIR * pose
const int   BASE_CENTER_DEG = 90, BASE_MIN = 0, BASE_MAX = 180, BASE_DIR = 1;
const float BASE_RATE_DPS   = 90;   // base speed at full stick

const int   SH_UPRIGHT_DEG = 90, SH_DIR = 1;   // flip DIR if the shoulder tilts the wrong way
const int   EL_UPRIGHT_DEG = 90, EL_DIR = 1;   // flip DIR if the elbow bends the wrong way

// Lowest reach pose: upper arm 15 deg above horizontal = 75 deg tilt from vertical;
// forearm 10 deg below horizontal = 100 deg from vertical = 25 deg bend relative to upper arm.
const float SH_MAX_TILT = 75;
const float EL_MAX_BEND = 25;       // smaller range = elbow moves slower than shoulder
const float REACH_RATE  = 0.5f;     // reach units/sec at full stick (0.5 = 2 s for full travel)

const float TRIM_DEG    = 30;       // each pot offsets its joint by up to +-this many degrees
const float SH_TILT_MIN = -30, SH_TILT_MAX = 90;   // hard joint limits (degrees from upright)
const float EL_BEND_MIN = -30, EL_BEND_MAX = 90;
const float JOINT_MAX_DPS = 120;    // joint speed limit (smooths pot moves)

// Must match the controller sketch
struct __attribute__((packed)) Packet {
  uint8_t magic;
  int8_t  roverX, roverY, armX, armY, shoulderTrim, elbowTrim;
  uint8_t grip;
  uint8_t seq;
};

WiFiUDP udp;
Servo sDrive, sSteer, sBase, sShoulder, sElbow, sGripper;

Packet   pkt;
bool     haveLink = false, wasLinked = false;
uint32_t lastPktMs = 0, lastUpdate = 0;

float driveCmd   = 0;
float steerAngle = STEER_CENTER_DEG;
float gripPosMs  = 0;                  // estimated gripper rotation (0 = open)
float baseAngle  = BASE_CENTER_DEG;
float reach      = 0;                  // 0 = upright, 1 = fully forward/down
float shAngle    = SH_UPRIGHT_DEG;
float elAngle    = EL_UPRIGHT_DEG;

float slew(float cur, float target, float maxStep) {
  float d = target - cur;
  if (d > maxStep) d = maxStep;
  if (d < -maxStep) d = -maxStep;
  return cur + d;
}

void stopContinuous() {
  driveCmd = 0;
  sDrive.writeMicroseconds(DRIVE_STOP_US);
  sGripper.writeMicroseconds(GRIP_STOP_US);
}

void updateDrive(float y, float dt) {
  driveCmd = slew(driveCmd, constrain(y, -1.0f, 1.0f), DRIVE_ACCEL * dt);
  sDrive.writeMicroseconds(DRIVE_STOP_US + (int)(DRIVE_DIR * driveCmd * DRIVE_MAX_US));
}

void updateSteer(float x, float dt) {
  float target = STEER_CENTER_DEG + STEER_DIR * constrain(x, -1.0f, 1.0f) * STEER_MAX_DEG;
  steerAngle = slew(steerAngle, target, STEER_MAX_DPS * dt);
  sSteer.write((int)steerAngle);
}

// Press: turn one full revolution (timed) and stop. Release: turn back exactly as far as it went.
void updateGripper(bool pressed, float dtMs) {
  int dir = 0;
  if (pressed && gripPosMs < GRIP_TRAVEL_MS) dir = 1;
  else if (!pressed && gripPosMs > 0)        dir = -1;

  if (dir != 0) {
    gripPosMs = constrain(gripPosMs + dir * dtMs, 0.0f, GRIP_TRAVEL_MS);
    sGripper.writeMicroseconds(GRIP_STOP_US + GRIP_DIR * dir * GRIP_SPEED_US);
  } else {
    sGripper.writeMicroseconds(GRIP_STOP_US);
  }
}

void updateArm(const Packet& in, float dt) {
  // Base: stick sets rotation speed
  baseAngle += BASE_DIR * (in.armX / 100.0f) * BASE_RATE_DPS * dt;
  baseAngle = constrain(baseAngle, (float)BASE_MIN, (float)BASE_MAX);
  sBase.write((int)baseAngle);

  // Synchronized reach: one value drives both joints
  reach += (in.armY / 100.0f) * REACH_RATE * dt;
  reach = constrain(reach, 0.0f, 1.0f);

  // Pots add an independent fine offset on top of the synchronized pose
  float shTilt = reach * SH_MAX_TILT + (in.shoulderTrim / 100.0f) * TRIM_DEG;
  float elBend = reach * EL_MAX_BEND + (in.elbowTrim / 100.0f) * TRIM_DEG;
  shTilt = constrain(shTilt, SH_TILT_MIN, SH_TILT_MAX);
  elBend = constrain(elBend, EL_BEND_MIN, EL_BEND_MAX);

  float shTarget = constrain(SH_UPRIGHT_DEG + SH_DIR * shTilt, 0.0f, 180.0f);
  float elTarget = constrain(EL_UPRIGHT_DEG + EL_DIR * elBend, 0.0f, 180.0f);
  shAngle = slew(shAngle, shTarget, JOINT_MAX_DPS * dt);
  elAngle = slew(elAngle, elTarget, JOINT_MAX_DPS * dt);
  sShoulder.write((int)shAngle);
  sElbow.write((int)elAngle);
}

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  Servo* all[] = {&sDrive, &sSteer, &sBase, &sShoulder, &sElbow, &sGripper};
  for (Servo* s : all) s->setPeriodHertz(50);

  // Attach one at a time so six servos don't all jerk to position at once
  // (the sudden current draw can brown out the ESP32).
  sDrive.attach(PIN_DRIVE, 1000, 2000);
  sDrive.writeMicroseconds(DRIVE_STOP_US);
  sGripper.attach(PIN_GRIPPER, 1000, 2000);
  sGripper.writeMicroseconds(GRIP_STOP_US);
  delay(200);
  sSteer.attach(PIN_STEER, 500, 2400);
  sSteer.write(STEER_CENTER_DEG);
  delay(300);
  sBase.attach(PIN_BASE, 500, 2400);
  sBase.write((int)baseAngle);
  delay(300);
  sShoulder.attach(PIN_SHOULDER, 500, 2400);
  sShoulder.write((int)shAngle);
  delay(300);
  sElbow.attach(PIN_ELBOW, 500, 2400);
  sElbow.write((int)elAngle);
  delay(300);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS, 1, 0, 2);   // channel 1, visible, max 2 clients
  udp.begin(PORT);
  Serial.print("AP up, IP: ");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  // Receive; keep only the newest valid packet
  int sz = udp.parsePacket();
  while (sz > 0) {
    if (sz == (int)sizeof(Packet)) {
      Packet p;
      udp.read((uint8_t*)&p, sizeof(p));
      if (p.magic == 0xA5) { pkt = p; lastPktMs = millis(); haveLink = true; }
    } else {
      while (udp.available()) udp.read();   // discard unexpected data
    }
    sz = udp.parsePacket();
  }

  uint32_t now = millis();
  if (now - lastUpdate < UPDATE_MS) return;
  float dt = (now - lastUpdate) / 1000.0f;
  lastUpdate = now;
  if (dt > 0.1f) dt = 0.1f;   // avoid big jumps after a stall

  bool linked = haveLink && (now - lastPktMs < (uint32_t)FAILSAFE_MS);
  if (linked != wasLinked) {
    wasLinked = linked;
    Serial.println(linked ? "Controller linked" : "Link lost - failsafe");
  }
  if (!linked) {
    stopContinuous();         // arm and steering simply hold their last position
    return;
  }

  updateDrive(pkt.roverY / 100.0f, dt);
  updateSteer(pkt.roverX / 100.0f, dt);
  updateArm(pkt, dt);
  updateGripper(pkt.grip, dt * 1000.0f);
}
