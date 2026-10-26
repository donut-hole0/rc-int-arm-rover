// RC CONTROLLER (ESP32) - reads 2 joysticks + 2 pots, sends UDP packets to the car.
// Board: ESP32 Dev Module. No extra libraries needed.
// Sticks must be untouched (centered) at power-on: that is when centre is measured.

#include <WiFi.h>
#include <WiFiUdp.h>

// ---------- WiFi (must match the car) ----------
const char*     WIFI_SSID = "RCCar";
const char*     WIFI_PASS = "12345678";   // change it on BOTH sketches (min 8 chars)
const IPAddress CAR_IP(192, 168, 4, 1);
const uint16_t  CAR_PORT  = 4210;

// ---------- Pins (from controller schematic) ----------
const int PIN_ROVER_X      = 34;  // steering
const int PIN_ROVER_Y      = 35;  // drive
const int PIN_ARM_BASE     = 32;  // arm joystick X (base rotation)
const int PIN_ARM_FWD      = 33;  // arm joystick Y (reach)
const int PIN_ARM_SHOULDER = 36;  // pot RV2
const int PIN_ARM_ELBOW    = 39;  // pot RV1
const int PIN_GRIP_BTN     = 14;  // joystick press switch (to GND when pressed)
const int PIN_LED          = 2;   // onboard LED if your board has one (lit = linked to car)

// Flip any of these if a control moves the wrong way
const bool INV_ROVER_X  = false;
const bool INV_ROVER_Y  = false;   // pushing forward should drive forward
const bool INV_ARM_X    = false;
const bool INV_ARM_Y    = false;   // pushing forward should EXTEND the arm
const bool INV_SHOULDER = false;
const bool INV_ELBOW    = false;

// ---------- Tuning ----------
const bool  DEBUG_SERIAL   = true;    // prints values at 115200 baud; set false when done
const int   ADC_MIN = 150;            // usable ADC range at 11 dB (ends are nonlinear)
const int   ADC_MAX = 3900;           // if a stick never reaches +-100, tune using debug output
const float STICK_DEADZONE = 0.08f;
const float POT_DEADZONE   = 0.04f;
const int   SEND_INTERVAL_MS = 20;    // 50 Hz

// Must be identical in the car sketch
struct __attribute__((packed)) Packet {
  uint8_t magic;        // 0xA5
  int8_t  roverX;       // -100..100
  int8_t  roverY;
  int8_t  armX;
  int8_t  armY;
  int8_t  shoulderTrim; // -100..100 (pot)
  int8_t  elbowTrim;
  uint8_t grip;         // 1 = pressed
  uint8_t seq;
};

WiFiUDP udp;
int cRoverX, cRoverY, cArmX, cArmY;   // stick centres, measured at boot

int readAvg(int pin) {
  long s = 0;
  for (int i = 0; i < 16; i++) s += analogRead(pin);
  return s / 16;
}

float applyDeadzone(float v, float dz) {
  if (fabsf(v) < dz) return 0;
  return (v > 0) ? (v - dz) / (1 - dz) : (v + dz) / (1 - dz);
}

float stickNorm(int raw, int center, bool inv) {
  float v = (raw >= center) ? (float)(raw - center) / (ADC_MAX - center)
                            : -(float)(center - raw) / (center - ADC_MIN);
  v = constrain(v, -1.0f, 1.0f);
  v = applyDeadzone(v, STICK_DEADZONE);
  return inv ? -v : v;
}

float potNorm(int raw, bool inv) {
  float v = ((float)(raw - ADC_MIN) / (ADC_MAX - ADC_MIN)) * 2.0f - 1.0f;
  v = constrain(v, -1.0f, 1.0f);
  v = applyDeadzone(v, POT_DEADZONE);
  return inv ? -v : v;
}

int8_t pct(float v) { return (int8_t)lroundf(constrain(v, -1.0f, 1.0f) * 100.0f); }

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  const int analogPins[] = {PIN_ROVER_X, PIN_ROVER_Y, PIN_ARM_BASE, PIN_ARM_FWD, PIN_ARM_SHOULDER, PIN_ARM_ELBOW};
  for (int p : analogPins) analogSetPinAttenuation(p, ADC_11db);
  pinMode(PIN_GRIP_BTN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  delay(500);   // let the joystick supply settle, then record centres
  cRoverX = readAvg(PIN_ROVER_X);
  cRoverY = readAvg(PIN_ROVER_Y);
  cArmX   = readAvg(PIN_ARM_BASE);
  cArmY   = readAvg(PIN_ARM_FWD);
  Serial.printf("Stick centres: roverX=%d roverY=%d armX=%d armY=%d\n", cRoverX, cRoverY, cArmX, cArmY);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // lower latency
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void loop() {
  static uint32_t lastTry = 0, lastSend = 0, lastChange = 0, lastPrint = 0;
  static bool lastRaw = false, btn = false, udpUp = false;
  static uint8_t seq = 0;

  // Debounced button
  bool raw = digitalRead(PIN_GRIP_BTN) == LOW;
  if (raw != lastRaw) { lastRaw = raw; lastChange = millis(); }
  if (millis() - lastChange > 20) btn = raw;

  // Keep WiFi connected
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(PIN_LED, LOW);
    if (udpUp) { udp.stop(); udpUp = false; }
    if (millis() - lastTry > 3000) {
      lastTry = millis();
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      Serial.println("Connecting to car...");
    }
    return;
  }
  if (!udpUp) { udp.begin(CAR_PORT); udpUp = true; Serial.println("Connected."); }
  digitalWrite(PIN_LED, HIGH);

  if (millis() - lastSend < SEND_INTERVAL_MS) return;
  lastSend = millis();

  Packet p;
  p.magic        = 0xA5;
  p.roverX       = pct(stickNorm(readAvg(PIN_ROVER_X), cRoverX, INV_ROVER_X));
  p.roverY       = pct(stickNorm(readAvg(PIN_ROVER_Y), cRoverY, INV_ROVER_Y));
  p.armX         = pct(stickNorm(readAvg(PIN_ARM_BASE), cArmX, INV_ARM_X));
  p.armY         = pct(stickNorm(readAvg(PIN_ARM_FWD), cArmY, INV_ARM_Y));
  p.shoulderTrim = pct(potNorm(readAvg(PIN_ARM_SHOULDER), INV_SHOULDER));
  p.elbowTrim    = pct(potNorm(readAvg(PIN_ARM_ELBOW), INV_ELBOW));
  p.grip         = btn ? 1 : 0;
  p.seq          = seq++;

  udp.beginPacket(CAR_IP, CAR_PORT);
  udp.write((const uint8_t*)&p, sizeof(p));
  udp.endPacket();

  if (DEBUG_SERIAL && millis() - lastPrint > 250) {
    lastPrint = millis();
    Serial.printf("rover X%4d Y%4d | arm X%4d Y%4d | trim sh%4d el%4d | grip %d\n",
                  p.roverX, p.roverY, p.armX, p.armY, p.shoulderTrim, p.elbowTrim, p.grip);
  }
}
