// ======================================================================
// Window Evacuation Button-Activated Release System (Window E-BARS)
// MLE04641 Arduino Nano ESP32 | ABX00083
// DHT22 + Servo + Buzzer + RGB + 7-Seg + Button + EEPROM + Serial CLI
// ======================================================================

// --------------------------- Includes ---------------------------------
#include <WiFi.h>
struct WifiConfig {
  const char* ssid;
  const char* password;
};

//CHANGE WIFI NAMES AND PASSWORDS
const int WIFI_COUNT = 5;
WifiConfig wifiList[WIFI_COUNT] = {
  { "wifissid1", "password1" },
  { "wifissid2", "password2" },
  { "wifissid3", "password3" },
  { "wifissid4", "password4" },
  { "wifissid5", "password5" }
};

int wifiIndex = 0;
bool wifiConnecting = false;
unsigned long wifiConnectStart = 0;
const unsigned long WIFI_TIMEOUT_MS = 20000;

void startWifiConnect() {
  WiFi.disconnect();
  WiFi.begin(wifiList[wifiIndex].ssid, wifiList[wifiIndex].password);
  wifiConnectStart = millis();
  wifiConnecting = true;
}

void wifiTask() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnecting = false;
    return;
  }

  if (!wifiConnecting) {
    startWifiConnect();
    return;
  }

  if (millis() - wifiConnectStart >= WIFI_TIMEOUT_MS) {
    wifiIndex = (wifiIndex + 1) % WIFI_COUNT;
    startWifiConnect();
  }
}
#include <time.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <EEPROM.h>

// ------------------------- Pin Assignment ------------------------------

// Adjustable pin numbers
// Digital pins use D-style names for clarity on Nano ESP32.

#define PIN_DHT          D11
#define PIN_SERVO        A3
#define PIN_BUTTON       D2
#define PIN_BUZZER       D13

// Common-cathode RGB LED on three PWM-capable pins.
#define PIN_RGB_R        A0
#define PIN_RGB_G        A1
#define PIN_RGB_B        A2

// Seven segment display pins.
#define PIN_SEG_A        D9
#define PIN_SEG_B        D10
#define PIN_SEG_C        D4
#define PIN_SEG_D        D5
#define PIN_SEG_E        D6
#define PIN_SEG_F        D8
#define PIN_SEG_G        D7
#define PIN_SEG_DP       D3

// -------------------------- DHT Sensor --------------------------------

#define DHTTYPE          DHT22

DHT dht(PIN_DHT, DHTTYPE);

// ---------------------- Servo Configuration ---------------------------

// Adjustable servo positions (degrees)
static int cfg_SERVO_POS_CLOSED   = 52;   // default closed position
static int cfg_SERVO_POS_RELEASED = 152;  // default released position

// ---------------------- Button Configuration --------------------------

// Active low button with pull-up.
// Press = LOW, Release = HIGH.

static const unsigned long BUTTON_DEBOUNCE_MS = 50;

// Multi-click and long press timing.
// Values are defaults; user adjusts through ADJ system later.

static unsigned long cfg_CLICKGAP   = 450;  // ms between clicks in one sequence
static unsigned long cfg_LONGLONG   = 1200; // ms hold for long press

// ---------------------- Buzzer Configuration --------------------------

// Base timing values for buzzer patterns in milliseconds.
// These values are defaults; user can adjust through ADJ BUZZLO / BUZZMED / BUZZHI.

static unsigned long cfg_BUZZLO     = 400;
static unsigned long cfg_BUZZMED    = 250;
static unsigned long cfg_BUZZHI     = 120;

// ------------------------- RGB Configuration --------------------------

// Brightness level from 0.0 to 1.0, applied as a multiplier.
// Can be adjusted through ADJ RGBLEVEL.

static float cfg_RGBLEVEL           = 0.6f;

// ------------------------- DHT Timing ---------------------------------

// Time between DHT22 reads in milliseconds.
// Can be adjusted through ADJ DHTPERIOD.

static unsigned long cfg_DHTPERIOD  = 2000;

// ----------------------- Idle Sleep Timing ----------------------------

// Duration of inactivity before sleep in seconds.
// Can be adjusted through ADJ IDLESLEEP.

static unsigned long cfg_IDLESLEEP_SEC  = 30; // 30 seconds default

// Overlay display duration in milliseconds for short messages.
// Can be adjusted through ADJ SEGOVR.

static unsigned long cfg_SEGOVR     = 3000;

// Test duration for TEMP BAND tests in seconds.
// Can be adjusted through ADJ TESTDUR.

static unsigned long cfg_TESTDUR    = 3;   // default 3 seconds

// ---------------------- WiFi / Time Config ----------------------------

// WiFi connection is handled by wifiTask() using wifiList[] and wifiIndex.

// Timezone: UTC+8 | Adjustable only through code
static const long GMT_OFFSET_SEC      = 8L * 3600L;
static const int  DAYLIGHT_OFFSET_SEC = 0;

// Time since last successful time sync (epoch seconds)
static time_t lastTimeSyncEpoch       = 0;

// Resync after 30 days
static const time_t RESYNC_INTERVAL_SEC = 30L * 24L * 60L * 60L;

// Internal state for non-blocking NTP sync
static bool ntpWaiting                = false;
static unsigned long ntpStartMS       = 0;
static const unsigned long NTP_TIMEOUT_MS = 10000UL;

// ---------------------- Fire Thresholds -------------------------------

// Fire temperature default thresholds in deg. C.
// Can be adjusted through ADJ FIRESTART and ADJ FIRECLR.

static float cfg_FIRESTART          = 35.0f;
static float cfg_FIRECLR            = 35.0f;

// Rapid rise threshold default in deg. C.
// If last temp and current temp is more than ROTCDELTA, activate FireActive
// Can be adjusted through ADJ ROTCDELTA.

static float cfg_ROTCDELTA          = 1.0f;

// Automatic time sync threshold default in deg. C.
// Syncs time after crossing this temperature threshold
// Can be adjusted through ADJ TIMEAUTO.

static float cfg_TIMEAUTO           = 33.0f;

// Maximum allowed simulated temperature for safety. Adjustable through ADJ SIMMAX.

static float cfg_SIMMAX             = 1674.1f;

// ---------------------- Simulation Noise ------------------------------

static float cfg_NOISELEVEL         = 1.0f; // 0.0 to 20.0

// ------------------------ Sleep and Flags -----------------------------

// Sleep flag for display and RGB.

static bool displaySleep            = false;

// Any user / system activity must wake the device
static volatile bool gActivityPulse = false;

inline void recordActivity()
{
  gActivityPulse = true;
}

// Manual override flag.

static bool manualOverride          = false;

// Mute flag for buzzer.

static bool muteBuzzer              = false;

// Global flag when sensor is in error state.

static bool sensorError             = false;

// Fire state flags.

static bool fireActive              = false;
static bool inFireRange             = false;

// Rapid rise detection flags.

static bool rapidRiseAlarm          = false;
static bool prevRapidRiseAlarm      = false;

// WiFi time state flag (time sync logic stub in this sketch).

static bool timeSynced              = false;
static bool timeSyncInProgress      = false;

// Stopwatch for fire duration.

static bool swFireRunning           = false;
static unsigned long swFireStartMS  = 0;
static unsigned long swFireElapsedMS = 0;

// ----------------------- Temperature State ----------------------------

static float lastValidTemp          = NAN;
static float lastValidHum           = NAN;
static float prevTemp               = NAN;
static float currentTemp            = NAN;
static float currentHum             = NAN;

static int   currentBand            = 0;
static int   prevBand               = 0;

// ------------------------ Simulation State ----------------------------

static bool  simActive              = false;
bool simPreserveNow                 = false;

// BaseSimTemp is the setpoint the user types in with SIM.
// This will not be printed.
static float BaseSimTemp            = 25.0f;

// NoisySimTemp is what the system uses during SIM.
// This Simulated Tempearute is the one getting printed.
static float NoisySimTemp           = 25.0f;

// ------------------------ Button State --------------------------------

static bool           buttonStable       = HIGH;
static bool           buttonLastRaw     = HIGH;
static unsigned long  buttonLastDebounce = 0;

static unsigned long  buttonPressTime   = 0;
static unsigned long  buttonReleaseTime = 0;

static bool           longPressFired    = false;

static int            clickCount        = 0;
static unsigned long  lastClickTime     = 0;

// ------------------------ Servo State ---------------------------------

Servo fireServo;

static int targetServoPos            = cfg_SERVO_POS_CLOSED;
static int currentServoPos           = cfg_SERVO_POS_CLOSED;
static unsigned long lastServoStepMS = 0;

// ———————— Buzzer State ––––––––––––––––

static bool buzzerActive             = false;
static unsigned long buzzerLastMS    = 0;
static bool buzzerOnPhase            = false;

// These are be real definitions (not extern), because updateBuzzer() uses them.
bool buzzerBandTestActive            = false;
int  buzzerBandTestBand              = 0;
unsigned long buzzerBandTestEndMS    = 0;

// Also define the missing RGB + BUZZ test request flags referenced by the command handlers.
bool buzzerTestRequested             = false;

// BUZZ TEST quick beeps state
static bool buzzerQuickTestActive     = false;
static uint8_t buzzerQuickTestBeep    = 0;
static bool buzzerQuickTestOnPhase    = false;
static unsigned long buzzerQuickTestNextMS = 0;

bool rgbTestPulseRequested           = false;
bool rgbTestRampRequested            = false;
int  rgbTestBrightLevelRequested     = -1;  // use -1 as “none requested”

// -------------------------- RGB State ---------------------------------

enum RgbMode
{
  RGBMODE_SAFE = 0,
  RGBMODE_FIRE,
  RGBMODE_RAPID,
  RGBMODE_OVERRIDE,
  RGBMODE_ERROR,
  RGBMODE_SLEEP,
  RGBMODE_WIFI_SYNC,
  RGBMODE_TEST
};

static RgbMode rgbMode               = RGBMODE_SAFE;
static RgbMode rgbModeBeforeTest     = RGBMODE_SAFE;

static unsigned long rgbTestEndMS    = 0;

// ---------------------- Seven Segment State ---------------------------

// Display override symbol and timing.
static bool           displayOverrideActive = false;
static char           displayOverrideChar   = ' ';
static unsigned long  displayOverrideEndMS  = 0;

// Flag to request TESTSEG animation.
static bool           displayReqTestSeg     = false;

// ---------------------- Test Engine Flags + State ----------------------

// Requests (set by serial commands)
static bool displayCountUpRequested    = false;  // request for display count up
static bool displayCountDownRequested  = false;  // request for display count down

static bool servoTestRequested         = false;  // request for servo testing
static bool rgbTestFlashRequested      = false;  // request for RGB flash testing

// Actives (driven by updateTestEngines)
static bool displayCountUpActive       = false; 
static bool displayCountDownActive     = false;

static int  displayTestIndex           = 0;      // display test step index
static unsigned long displayTestNextMS = 0;      // next display step time (ms)

// Servo sweep test
static bool servoSweepTestActive       = false;
static int  servoSweepPhase            = 0;      // servo sweep phase id
static unsigned long servoSweepNextStep = 0;     // next servo move time (ms)
static unsigned long servoSweepEndMS   = 0;      // servo sweep end time (ms)
static unsigned long servoSweepStepMS  = 400;    // delay per move (ms), mutable

// RGB flash test
static bool rgbFlashTestActive         = false;
static unsigned long rgbSimpleTestEndMS = 0;     // RGB simple test end time (ms)

// Missing TEMP BAND + extra test flags referenced later
static bool servoBandTestActive        = false;
static int  servoBandTestBand          = 0;      // current band id
static unsigned long servoBandTestEndMS = 0;     // servo band test end time (ms)

static bool rgbBandTestActive          = false;
static unsigned long rgbBandTestEndMS  = 0;      // RGB band test end time (ms)

static bool rgbPulseTestActive         = false;
static bool rgbRampTestActive          = false;
static bool rgbBrightTestActive        = false;
static int  rgbBrightTestLevel         = 0;      // brightness test level

static bool displayAllTestActive       = false;
static unsigned long displayAllTestEndMS = 0;    // display all test end time (ms)
static bool displayTestAllRequested    = false;  // request for all display tests

// Stop-all helper (used by display tests)

// Prototype for a function
void clearDisplayOverride();

// ------------------------ Statistics ----------------------------------

struct StatsData
{
  uint32_t magic;
  uint32_t totalFireTimeMS;
  float    maxTempC;
  uint32_t singleClicks;
  uint32_t doubleClicks;
  uint32_t tripleClicks;
  uint32_t quadClicks;
  uint32_t longPresses;
  uint32_t simSessions;
  uint32_t sensorErrors;
  uint32_t lastFireDurationMS;
};

static StatsData stats;

// ---------------------- EEPROM Layout ---------------------------------

// Overall EEPROM size of MLE04641 Arduino Nano ESP32 | ABX00083

static const int EEPROM_SIZE            = 4096;

// Simple layout:
// 0            : 4 bytes statistics magic and header start
// 0..sizeof(StatsData)-1 : stats structure
// Next        : event log region

static const uint32_t STATS_MAGIC       = 0x57574431UL;

static int eepromStatsOffset            = 0;
static int eepromLogOffset              = sizeof(StatsData);
static int eepromLogSize                = EEPROM_SIZE - sizeof(StatsData);

// Log pointer index and limit.

static uint16_t cfg_LOGLIMIT            = 256; // limit for partial dumps

// ------------------------- Event System -------------------------------

// Event types (simple char codes to keep storage small).

enum EventType : uint8_t
{
  EVT_NONE   = 0,
  EVT_BND    = 1,  // band change
  EVT_THR    = 2,  // threshold fire in or out
  EVT_ROTC   = 3,  // rapid rise
  EVT_BTN    = 4,  // button sequences
  EVT_SYS    = 5,  // system events
  EVT_CMD    = 6,  // command events
  EVT_SIM    = 7   // simulation events
};

// Event reasons for THR / BTN / SYS and others.

enum ThrReason : uint8_t
{
  THR_TEMP35UP = 1,
  THR_TEMP35DN = 2
};

enum RotcRating : uint8_t
{
  ROTC_NONE       = 0,
  ROTC_MODERATE   = 1,
  ROTC_MOD_FAST   = 2,
  ROTC_FAST       = 3,
  ROTC_VFAST      = 4,
  ROTC_SUPER      = 5,
  ROTC_EXTREME    = 6
};

enum BtnReason : uint8_t
{
  BTN_1       = 1,
  BTN_2       = 2,
  BTN_3       = 3,
  BTN_4       = 4,
  BTN_LONG    = 5
};

enum SysReason : uint8_t
{
  SYS_SLEEP_ON      = 1,
  SYS_SLEEP_OFF     = 2,
  SYS_SERVO_DN      = 3,
  SYS_SERVO_UP      = 4,
  SYS_CLR_EEP_BTN   = 5,
  SYS_MANUAL_ON     = 6,
  SYS_MANUAL_OFF    = 7,
  SYS_MUTE_ON       = 8,
  SYS_MUTE_OFF      = 9,
  SYS_SENSOR_ERR    = 10
};

enum CmdReason : uint8_t
{
  CMD_SIM_TEMP      = 1,
  CMD_SIM_OFF       = 2,
  CMD_TEMPBAND_TEST = 3
};

enum SimReason : uint8_t
{
SIM_START         = 1,
SIM_STOP          = 2,
SIM_TEMP_SET      = 3
};

// Generic event record, fixed length for EEPROM.
// Size: 18 bytes

struct EventRecord
{
  uint8_t  type;        // EventType
  uint8_t  reason;      // reason per type
  uint8_t  band;        // band or band-like info
  uint8_t  flags;       // bit flags (manual override, rapid rise, fire, sim)
  uint32_t millisStamp; // millis at event
  uint32_t tempX10;     // temperature times ten
  uint32_t aux;         // duration or auxiliary value
  uint16_t humX10;      // humidity times ten
};

// Bit flags for EventRecord.flags

static const uint8_t FLAG_MANUAL    = 0x01;
static const uint8_t FLAG_FIRE      = 0x02;
static const uint8_t FLAG_ROTC      = 0x04;
static const uint8_t FLAG_SIM       = 0x08;

// Simple log pointer.

static uint16_t logWriteIndex        = 0;

// ---------------------- Serial Status Control -------------------------

// Time between periodic status prints.
// Default: 2 seconds.
static unsigned long cfg_SWPRINT     = 2000;
static unsigned long lastStatusPrint = 0;

// Error-silence logic for periodic status.
// Stop printing status lines if last two are error lines
static bool lastStatusWasError       = false;
static bool secondLastStatusError    = false;

// ---------------------- ADJ System Metadata ---------------------------

// Forward declarations for ADJ handlers.

void adj_TIMEAUTO(const String &arg);
void adj_FIRESTART(const String &arg);
void adj_FIRECLR(const String &arg);
void adj_ROTCDELTA(const String &arg);
void adj_CLICKGAP(const String &arg);
void adj_LONGLONG(const String &arg);
void adj_IDLESLEEP(const String &arg);
void adj_SEGOVR(const String &arg);
void adj_BUZZLO(const String &arg);
void adj_BUZZMED(const String &arg);
void adj_BUZZHI(const String &arg);
void adj_NOISELEVEL(const String &arg);
void adj_TESTDUR(const String &arg);
void adj_RGBLEVEL(const String &arg);
void adj_SIMMAX(const String &arg);
void adj_SWPRINT(const String &arg);
void adj_DHTPERIOD(const String &arg);
void adj_BANDLOG(const String &arg);
void adj_LOGLIMIT(const String &arg);

// BANDLOG flag.

static bool cfg_BANDLOG              = true;

// --------------------- Forward Declarations ---------------------------

// Core loops.

void updateDHT();
void updateBandsAndFire();
void updateRapidRise();
void updateServo();
void updateBuzzer();
void updateRGB();
void updateSevenSeg();
void updateButton();
void updateSleepLogic();
void updateSimulationNoise();
void updatePeriodicStatus();

// Command handling.

void handleSerialInput();
void processCommand(const String &cmd, const String &args);

// Helper printers.

void printStatusLine(bool fromSimulation);
void printStats();
void printLog(uint16_t limitCount);
void printHelpMain();
void printHelpStatsAndLog();

// EEPROM helpers.

void loadStatsFromEEPROM();
void saveStatsToEEPROM();
void clearStatsAndLog();
void appendEvent(const EventRecord &ev);
void loadLogWriteIndex();

// Event builders.

void logBandChange(int oldBand, int newBand);
void logFireThreshold(bool entering, float temp);
void logRapidRise(float delta, RotcRating rating);
void logButton(BtnReason reason);
void logSystem(SysReason reason, uint32_t aux);
void logCommand(CmdReason reason, uint32_t aux);
void logSim(SimReason reason, float temp);

// Display helpers.

void setDisplayOverrideChar(char c, unsigned long durationMS);
void clearDisplayOverride();
void setDisplayBlankForCritical();

// RGB helpers.

void setRgbMode(RgbMode mode);

static bool gRgbTestMode = false;
static int gRgbTestBand = -1;
static unsigned long gRgbTestUntilMs = 0;

void setRgbTestModeForBand(int band, unsigned long durationMs) {
  gRgbTestMode = true;
  gRgbTestBand = band;

  unsigned long now = millis();
  gRgbTestUntilMs = now + durationMs;
}

// Optional helpers. Use only if other code expects these.
bool isRgbTestModeActive() {
  if (!gRgbTestMode) return false;

  unsigned long now = millis();
  if ((long)(now - gRgbTestUntilMs) >= 0) {
    gRgbTestMode = false;
    gRgbTestBand = -1;
    return false;
  }
  return true;
}

int getRgbTestBand() {
  return gRgbTestBand;
}

// Servo helpers.

void setServoTargetClosed();
void setServoTargetReleased();
void servoSetImmediate(int pos);

// Simulation helpers.

void simSetTemperature(float t);
void simStop();

// Button pattern handlers.

void onSingleClick();
void onDoubleClick();
void onTripleClick();
void onQuadClick();
void onLongPress();

// Utility.

int computeBandForTemperature(float temp);
RotcRating computeRotcRating(float delta);
float readDhtTemperature();
float readDhtHumidity();

// Seven seg helpers.

void sevenSegShowDigit(int digit);
void sevenSegShowChar(char c);
void sevenSegAllOff();
void sevenSegTestSeg();
void sevenSegApplyCurrent();

// RGB low-level.

void rgbWriteRaw(uint8_t r, uint8_t g, uint8_t b);

// Buzzer low-level.

void buzzerSet(bool on);

// Time-related helpers.

void maybeStartTimeSync(float temp);
void updateTimeSync();
void requestTimeSync();

// -------------------------- Setup -------------------------------------

void setup()
{
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);

  pinMode(PIN_SEG_A, OUTPUT);
  pinMode(PIN_SEG_B, OUTPUT);
  pinMode(PIN_SEG_C, OUTPUT);
  pinMode(PIN_SEG_D, OUTPUT);
  pinMode(PIN_SEG_E, OUTPUT);
  pinMode(PIN_SEG_F, OUTPUT);
  pinMode(PIN_SEG_G, OUTPUT);
  pinMode(PIN_SEG_DP, OUTPUT);

  sevenSegAllOff();
  rgbWriteRaw(0, 0, 0);
  buzzerSet(false);

  fireServo.attach(PIN_SERVO);
  servoSetImmediate(cfg_SERVO_POS_CLOSED);

  dht.begin();

  EEPROM.begin(EEPROM_SIZE);
  loadStatsFromEEPROM();
  loadLogWriteIndex();

  lastStatusPrint        = millis();
  lastServoStepMS        = millis();
  buzzerLastMS           = millis();
  buttonLastDebounce     = millis();

  lastValidTemp          = NAN;
  lastValidHum           = NAN;
  prevTemp               = NAN;
  currentTemp            = NAN;
  currentHum             = NAN;
  sensorError            = false;

  manualOverride         = false;
  muteBuzzer             = false;
  inFireRange            = false;
  fireActive             = false;
  rapidRiseAlarm         = false;
  prevRapidRiseAlarm     = false;

  simActive              = false;

  displaySleep           = false;
  displayOverrideActive  = false;
  displayReqTestSeg      = false;

  rgbMode                = RGBMODE_SAFE;
  rgbModeBeforeTest      = RGBMODE_SAFE;

  lastStatusWasError     = false;
  secondLastStatusError  = false;

  Serial.println();
  Serial.println(F("[BOOT] Fire-Alert Window Guard Controller ready."));
  Serial.println(F("Type HELP for list of commands."));
    // Initial NTP time sync at boot
  requestTimeSync();
}

// --------------------------- Loop -------------------------------------

void updateTestEngines(unsigned long nowMS);
void startDisplayCountDown();
void startDisplayCountUp();
void updateDisplayCountUp(unsigned long nowMS);
void updateDisplayCountDown(unsigned long nowMS);

void loop()
{
  unsigned long now = millis();

  handleSerialInput();

  // Sensor Loop
  updateDHT();

  // Run SIM engine before any logic that consumes SIM temperature
  updateSimulationNoise();

  updateBandsAndFire();
  updateRapidRise();
  updateSleepLogic();
  updateTimeSync();

  // User interaction and outputs
  updateButton();
  updateServo();
  updateBuzzer();
  updateRGB();
  updateSevenSeg();

  // Run your test engines from loop (non-blocking)
  updateTestEngines(now);

  // Periodic status line on Serial
  updatePeriodicStatus();
}

void updateRapidRise()
{
  // Block rapid-rise only for real sensor error, not for SIM
  if (sensorError && !simActive)
  {
    rapidRiseAlarm = false;
    prevRapidRiseAlarm = false;
    return;
  }

  unsigned long now = millis();
  float tempToUse = simActive ? NoisySimTemp : currentTemp;

  // Reset SIM sampling state when SIM toggles
  static bool lastSimState = false;
  static float prevSimTemp = NAN;
  static unsigned long lastSimSampleMS = 0;

  if (simActive != lastSimState)
  {
    lastSimState = simActive;
    prevSimTemp = NAN;
    lastSimSampleMS = 0;
  }

  if (isnan(tempToUse))
  {
    rapidRiseAlarm = false;
    prevRapidRiseAlarm = false;
    return;
  }

  if (tempToUse >= cfg_FIRESTART)
  {
    rapidRiseAlarm = false;
    prevRapidRiseAlarm = false;
    return;
  }

  float delta = 0.0f;

  if (simActive)
  {
    // Sample once per second for SIM
    if (lastSimSampleMS != 0 && (now - lastSimSampleMS) < 1000UL)
    {
      return;
    }
    lastSimSampleMS = now;

    if (isnan(prevSimTemp))
    {
      prevSimTemp = tempToUse;
      rapidRiseAlarm = false;
      prevRapidRiseAlarm = false;
      return;
    }

    delta = tempToUse - prevSimTemp;
    prevSimTemp = tempToUse;
  }
  else
  {
    // Real sensor path uses prevTemp updated by updateDHT()
    if (isnan(prevTemp))
    {
      rapidRiseAlarm = false;
      prevRapidRiseAlarm = false;
      return;
    }
    delta = tempToUse - prevTemp;
  }

  RotcRating rating = (delta < cfg_ROTCDELTA) ? ROTC_NONE : computeRotcRating(delta);

  prevRapidRiseAlarm = rapidRiseAlarm;
  rapidRiseAlarm     = (rating != ROTC_NONE);

  if (!prevRapidRiseAlarm && rapidRiseAlarm)
  {
    recordActivity();
    logRapidRise(delta, rating);
  }
}

void startDisplayCountUp()
{
  stopAllTests();

  displayCountUpActive = true;
  displayTestIndex     = 0;
  displayTestNextMS    = millis();
}

void updateDisplayCountUp(unsigned long nowMS)
{
  if (!displayCountUpActive) return;
  if (nowMS < displayTestNextMS) return;
  displayTestNextMS = nowMS + 350;

  if (displayTestIndex > 9)
  {
    displayCountUpActive = false;
    clearDisplayOverride();

    if (!displaySleep)
    {
      sevenSegDisplayTestSeg();
    }
    return;
  }

  setDisplayOverrideChar((char)('0' + displayTestIndex), 400);
  displayTestIndex++;
}

// Countdown state
static int displayCountDownPhase = 0;  // 0 = start hold, 1 = countdown, 2 = end blink
static int displayEndBlinkIndex  = 0;  // 0..5 for 0,_ repeated 3 times

void stopAllTests()
{
  displayCountUpRequested    = false;
  displayCountDownRequested  = false;
  displayTestAllRequested    = false;

  displayCountUpActive       = false;
  displayCountDownActive     = false;
  displayAllTestActive       = false;

  displayTestIndex           = 0;
  displayTestNextMS          = 0;

  displayCountDownPhase      = 0;
  displayEndBlinkIndex       = 0;

  servoTestRequested         = false;
  servoSweepTestActive       = false;
  servoBandTestActive        = false;

  rgbTestFlashRequested      = false;
  rgbFlashTestActive         = false;
  rgbBandTestActive          = false;
  rgbPulseTestActive         = false;
  rgbRampTestActive          = false;
  rgbBrightTestActive        = false;
  rgbBrightTestLevel         = 0;

  buzzerBandTestActive       = false;

  clearDisplayOverride();
}

void startDisplayCountDown()
{
  stopAllTests();

  displayCountDownActive = true;
  displayCountDownPhase  = 0;
  displayEndBlinkIndex   = 0;

  displayTestIndex       = 9;
  displayTestNextMS      = millis();
}

void updateDisplayCountDown(unsigned long nowMS)
{
  if (!displayCountDownActive) return;
  if (nowMS < displayTestNextMS) return;

  if (displayCountDownPhase == 0)
  {
    if (displayTestIndex == 9)
    {
      setDisplayOverrideChar('9', 900);
      displayTestNextMS = nowMS + 900;
      displayTestIndex  = 8;
      return;
    }

    if (displayTestIndex >= 0)
    {
      setDisplayOverrideChar((char)('0' + displayTestIndex), 350);
      displayTestNextMS = nowMS + 350;
      displayTestIndex--;
      return;
    }

    displayCountDownPhase = 1;
    displayEndBlinkIndex  = 0;
    displayTestNextMS     = nowMS;
    return;
  }

  if (displayCountDownPhase == 1)
  {
    if (displayEndBlinkIndex < 6)
    {
      if ((displayEndBlinkIndex % 2) == 0) setDisplayOverrideChar('0', 180);
      else clearDisplayOverride();

      displayTestNextMS = nowMS + 180;
      displayEndBlinkIndex++;
      return;
    }

    displayCountDownActive = false;
    clearDisplayOverride();

    if (!displaySleep)
    {
      sevenSegDisplayTestSeg();
    }
    return;
  }
}
// --------------------- DHT Reading Logic ------------------------------

// Reads DHT data based on cfg_DHTPERIOD.
// Maintains sensorError flag and lastValid readings.

void updateDHT()
{
  static unsigned long lastDhtReadMS = 0;
  unsigned long now = millis();

  if (simActive)
  {
    sensorError = false;
    return;
  }

  if (now - lastDhtReadMS < cfg_DHTPERIOD)
  {
    return;
  }

  lastDhtReadMS = now;

  float t = readDhtTemperature();
  float h = readDhtHumidity();

  if (isnan(t) || isnan(h))
  {
     if (!sensorError)
    {
    sensorError = true;
    stats.sensorErrors++;
    saveStatsToEEPROM();

    // Log a Sensor Error event in EEPROM
    logSystem(SYS_SENSOR_ERR, 0);
    }
  currentTemp = NAN;
  currentHum  = NAN;
  }
  else
  {
    sensorError = false;
    prevTemp    = lastValidTemp;
    lastValidTemp = t;
    lastValidHum  = h;
    currentTemp   = t;
    currentHum    = h;
  }
}

// Hardware read wrappers

float readDhtTemperature()
{
  float t = dht.readTemperature();
  return t;
}

float readDhtHumidity()
{
  float h = dht.readHumidity();
  return h;
}

// ------------------ Temperature Bands and Fire ------------------------

// Computes band value 0..9 or higher internal band for logging.
// Display uses 0..9 only, not band 10 or more.

int computeBandForTemperature(float temp)
{
  if (isnan(temp)) return 0;

  static const float limits[] = {
    30.0f, 35.0f, 40.0f, 45.0f, 50.0f,
    60.0f, 70.0f, 80.0f, 90.0f, 100.0f
  };

  for (int i = 0; i < 10; i++)
    if (temp < limits[i]) return i;

  int b = (int)(temp / 10.0f);
  if (b < 10) b = 10;
  return b;
}

// Updates band, handles band logging, handles high temperature blanking.

void updateBandsAndFire()
{
  // Use simulation temperature when simulation is active
  float tempToUse = simActive ? NoisySimTemp : currentTemp;

  // Sensor error should only block real-sensor mode
  if (!simActive && sensorError)
  {
    currentBand = 0;
    return;
  }

  // Always guard against NaN
  if (isnan(tempToUse))
  {
    currentBand = 0;
    return;
  }

  prevBand    = currentBand;
  currentBand = computeBandForTemperature(tempToUse);

  if (currentBand != prevBand)
  {
    recordActivity();

    if (cfg_BANDLOG)
    {
     logBandChange(prevBand, currentBand);
    }

    if (displaySleep)
    {
      displaySleep = false;
      setRgbMode(RGBMODE_SAFE);
      logSystem(SYS_SLEEP_OFF, 0);
    }
  }

  bool wasInFireRange = inFireRange;

  if (tempToUse >= cfg_FIRESTART) inFireRange = true;
  else if (tempToUse < cfg_FIRECLR) inFireRange = false;

  if (!wasInFireRange && inFireRange)
  {
    recordActivity();
    fireActive    = true;
    swFireRunning = true;
    swFireStartMS = millis();

    // Deploy window guard on fire
    setServoTargetReleased();
    logSystem(SYS_SERVO_DN, 0);

    logFireThreshold(true, tempToUse);

    if (tempToUse > stats.maxTempC)
    {
      stats.maxTempC = tempToUse;
      saveStatsToEEPROM();
    }
  }

  if (wasInFireRange && !inFireRange)
  {
    fireActive = false;

    if (swFireRunning)
    {
      unsigned long now      = millis();
      unsigned long duration = now - swFireStartMS;

      swFireRunning       = false;
      swFireElapsedMS     = duration;
      stats.totalFireTimeMS += duration;
      stats.lastFireDurationMS = duration;
      saveStatsToEEPROM();
    }

    // Re-close window guard only if user is not in manual override
    if (!manualOverride)
    {
      setServoTargetClosed();
      logSystem(SYS_SERVO_UP, 0);
    }

    logFireThreshold(false, tempToUse);
  }

  maybeStartTimeSync(tempToUse);
}
// ------------------- Rapid Rise Detection -----------------------------

RotcRating computeRotcRating(float delta)
{
  if (delta < 0.0f) return ROTC_NONE;
  if (delta < 2.0f) return ROTC_MODERATE;
  if (delta < 3.0f) return ROTC_MOD_FAST;
  if (delta < 4.0f) return ROTC_FAST;
  if (delta < 5.0f) return ROTC_VFAST;
  if (delta < 6.0f) return ROTC_SUPER;
  return ROTC_EXTREME;
}

// -------------------- Simulation Noise Update -------------------------

// ------------------------ Simulation Phase Engine ----------------------
// Spec phases: Init → Rising → Hold → CoolFast → CoolSlow → End

enum SimPhase : uint8_t
{
  SIMPHASE_INIT = 0,
  SIMPHASE_RISING,
  SIMPHASE_HOLD,
  SIMPHASE_COOL_FAST,
  SIMPHASE_COOL_SLOW,
  SIMPHASE_END
};

static SimPhase simPhase = SIMPHASE_INIT;

static float targetSimTemp = 60.0f;

static unsigned long simStartMS        = 0;
static unsigned long simLastStepMS     = 0;

static unsigned long holdEndMS         = 0;

static bool slowDelayArmed             = false;
static unsigned long slowDelayEndMS    = 0;

static bool belowClrTimerRunning       = false;
static unsigned long belowClrStartMS   = 0;

static float noisyVel = 0.0f;   // noise memory velocity

static float frand(float a, float b)
{
  long r = random(0, 10001); // 0..10000
  float t = (float)r / 10000.0f;
  return a + (b - a) * t;
}

static void simStartSession(float targetC)
{
  targetSimTemp = targetC;

  BaseSimTemp  = frand(28.0f, 32.0f);
  NoisySimTemp = BaseSimTemp;
  noisyVel     = 0.0f;

  simPhase     = SIMPHASE_RISING;

  simStartMS   = millis();
  simLastStepMS = simStartMS;

  holdEndMS     = 0;

  slowDelayArmed  = false;
  slowDelayEndMS  = 0;

  belowClrTimerRunning = false;
  belowClrStartMS      = 0;

  simActive = true;

  stats.simSessions++;
  saveStatsToEEPROM();
  logSim(SIM_START, BaseSimTemp);
}

void updateSimulationNoise()
{
  // Reset happens automatically when simulation is not on.
  static bool lastNoisyValid = false;
  static int32_t lastNoisyX10 = 0;

  if (!simActive)
  {
    lastNoisyValid = false;
    return;
  }

  unsigned long now = millis();

  // --------- Step BaseSimTemp once per second (phase engine) ----------
  if (now - simLastStepMS >= 1000)
  {
    simLastStepMS += 1000;

    // total sim stop rules
    if ((now - simStartMS) >= (10UL * 60UL * 1000UL))
    {
      simPhase = SIMPHASE_END;
    }

    switch (simPhase)
    {
      case SIMPHASE_RISING:
      {
        float rate = frand(3.0f, 5.0f); // C per second
        BaseSimTemp += rate;

        if (BaseSimTemp >= targetSimTemp)
        {
          BaseSimTemp = targetSimTemp;

          unsigned long holdSec = (unsigned long)random(10, 31); // 10..30
          holdEndMS = now + holdSec * 1000UL;
          simPhase = SIMPHASE_HOLD;
        }
        break;
      }

      case SIMPHASE_HOLD:
      {
        if (holdEndMS != 0 && now >= holdEndMS)
        {
          simPhase = SIMPHASE_COOL_FAST;
        }
        break;
      }

      case SIMPHASE_COOL_FAST:
      {
        float rate = frand(0.8f, 1.5f);
        BaseSimTemp -= rate;

        if (BaseSimTemp <= cfg_FIRECLR)
        {
          // crossed below FIRECLR → slow sim cooling after 0..1s delay
          BaseSimTemp = cfg_FIRECLR;

          slowDelayArmed = true;
          slowDelayEndMS = now + (unsigned long)(frand(0.0f, 1.0f) * 1000.0f);

          simPhase = SIMPHASE_COOL_SLOW;

          belowClrTimerRunning = true;
          belowClrStartMS      = now;
        }
        break;
      }

      case SIMPHASE_COOL_SLOW:
      {
        if (slowDelayArmed && now < slowDelayEndMS)
        {
          // wait
        }
        else
        {
          slowDelayArmed = false;

          float rate = frand(0.1f, 0.5f);
          BaseSimTemp -= rate;
        }

        // stay below FIRECLR timer
        if (BaseSimTemp < cfg_FIRECLR)
        {
          if (!belowClrTimerRunning)
          {
            belowClrTimerRunning = true;
            belowClrStartMS = now;
          }
          if ((now - belowClrStartMS) >= 30000UL)
          {
            simPhase = SIMPHASE_END;
          }
        }
        else
        {
          belowClrTimerRunning = false;
        }

        break;
      }

      case SIMPHASE_END:
      default:
      {
        simActive = false;
        logSim(SIM_STOP, BaseSimTemp);
        Serial.println(F("[SIM] Simulation ended."));
        lastNoisyValid = false;
        return;
      }
    }

    if (BaseSimTemp < -40.0f) BaseSimTemp = -40.0f;
    if (BaseSimTemp > cfg_SIMMAX) BaseSimTemp = cfg_SIMMAX;
  }

  // --------- NoisySimTemp: wandering line with memory ----------
  if (cfg_NOISELEVEL <= 0.0f)
  {
    NoisySimTemp = BaseSimTemp;
  }
  else
  {
    float span = (float)cfg_NOISELEVEL;       // 0..20 from ADJ
    if (span < 0.0f) span = 0.0f;
    if (span > 20.0f) span = 20.0f;

    float pull = 0.15f;                       // pull back to BaseSimTemp
    float kick = frand(-0.08f, 0.08f) * span; // random acceleration

    noisyVel = (noisyVel * 0.85f) + kick;     // memory
    NoisySimTemp += noisyVel;

    // pull toward BaseSimTemp
    NoisySimTemp += (BaseSimTemp - NoisySimTemp) * pull;

    if (NoisySimTemp < -40.0f) NoisySimTemp = -40.0f;
    if (NoisySimTemp > cfg_SIMMAX) NoisySimTemp = cfg_SIMMAX;
  }

  // --------- Monotonic cooling clamp (printed SimTemp) ----------
  // During cooling phases, NoisySimTemp never increases unless the user commands a new higher simtemp
  // Rising becomes allowed again when simPhase changes to RISING (higher target logic).
  bool cooling = (simPhase == SIMPHASE_COOL_FAST) || (simPhase == SIMPHASE_COOL_SLOW);

  int32_t noisyX10 = (int32_t)lroundf(NoisySimTemp * 10.0f);

  if (!lastNoisyValid)
  {
    lastNoisyValid = true;
    lastNoisyX10 = noisyX10;
  }
  else if (cooling)
  {
    if (noisyX10 > lastNoisyX10)
    {
      noisyX10 = lastNoisyX10;
      NoisySimTemp = noisyX10 / 10.0f;
    }
    lastNoisyX10 = noisyX10;
  }
  else
  {
    lastNoisyX10 = noisyX10;
  }

  // --------- Sim Live Prints ----------
  static unsigned long lastSimPrintMS = 0;
  if (now - lastSimPrintMS >= 1000) // prints once per second
  {
    lastSimPrintMS = now;
    Serial.print(F("SimTemp="));
    Serial.print(NoisySimTemp, 1);

    if (NoisySimTemp >= cfg_FIRESTART)
    {
      Serial.print(F(" | Fire = TRUE"));
    }

    Serial.print(F(" | Sim = TRUE"));
    Serial.println();
  }
}
// ---------------------- Sleep Logic -----------------------------------

void updateSleepLogic()
{
  static unsigned long lastActivityMS = 0;

  static bool firstRun = true;
  if (firstRun)
  {
    firstRun = false;
    lastActivityMS = millis();
  }

  bool activity = false;

  // External activity pulse (commands, WiFi, buttons, system events)
  if (gActivityPulse)
  {
    activity = true;
    gActivityPulse = false;
  }

  if (currentBand != prevBand) activity = true;
  if (fireActive || rapidRiseAlarm || sensorError || simActive) activity = true; // device wakes up on an activity if any is true

  static bool lastSensorErrorFlag = false;
  if (sensorError != lastSensorErrorFlag)
  {
    activity = true;
    lastSensorErrorFlag = sensorError;
  }

  static bool lastManualOverrideFlag = false;
  if (manualOverride != lastManualOverrideFlag)
  {
    activity = true;
    lastManualOverrideFlag = manualOverride;
  }

  if (activity) lastActivityMS = millis();

  unsigned long now = millis();

  bool servoClosed = (targetServoPos == cfg_SERVO_POS_CLOSED);

  bool conditionsForSleep =
    (!sensorError)         &&
    (!fireActive)          &&
    (!rapidRiseAlarm)      &&
    servoClosed            &&
    ((now - lastActivityMS) >= (cfg_IDLESLEEP_SEC * 1000UL));

  // Pre-sleep TESTSEG gate
  static bool sleepPending = false;
  static unsigned long sleepPendingStartMS = 0;
  const unsigned long SLEEP_TESTSEG_TIMEOUT_MS = 1500UL;

  // Cancel pending sleep when conditions disappear
  if (!conditionsForSleep)
  {
    sleepPending = false;

    if (displaySleep)
    {
      displaySleep = false;
      setRgbMode(RGBMODE_SAFE);
      logSystem(SYS_SLEEP_OFF, 0);
    }
    return;
  }

  // Conditions satisfied, not yet sleeping
  if (!displaySleep)
  {
    // Start TESTSEG once
    if (!sleepPending)
    {
      sleepPending = true;
      sleepPendingStartMS = now;

      clearDisplayOverride();
      sevenSegDisplayTestSeg();
    }

    // Enter sleep only after TESTSEG ends, or after timeout
    bool testSegDone = !sevenSegIsOverrideActive();
    bool timeoutHit  = (now - sleepPendingStartMS) >= SLEEP_TESTSEG_TIMEOUT_MS;

    if (testSegDone || timeoutHit)
    {
      sleepPending = false;

      displaySleep = true;
      rgbModeBeforeTest = rgbMode;
      setRgbMode(RGBMODE_SLEEP);
      sevenSegAllOff();
      muteBuzzer = false;
      logSystem(SYS_SLEEP_ON, 0);
    }
  }
}

// ------------------------ Periodic Status -----------------------------

void updatePeriodicStatus()
{
  unsigned long now = millis();

  if (cfg_SWPRINT == 0)
  {
    return;
  }

  if (now - lastStatusPrint < cfg_SWPRINT)
  {
    return;
  }

  lastStatusPrint = now;

  bool fromSim = simActive;

  printStatusLine(fromSim);
}

// Status line with error-silence logic.

void printStatusLine(bool fromSimulation)
{
  bool isErrorLine = false;

  float t;
  float h;

  if (fromSimulation)
  {
    t = NoisySimTemp;
    h = lastValidHum;
  }
  else
  {
    t = currentTemp;
    h = currentHum;
  }

  if (sensorError || isnan(t) || isnan(h))
  {
    isErrorLine = true;
  }

  bool errorSilenceEligible = sensorError && !fireActive && !manualOverride;

  if (isErrorLine && errorSilenceEligible)
  {
    if (lastStatusWasError && secondLastStatusError)
    {
      return;
    }
  }

  if (errorSilenceEligible)
  {
    secondLastStatusError = lastStatusWasError;
    lastStatusWasError    = isErrorLine;
  }
  else
  {
    secondLastStatusError = false;
    lastStatusWasError    = false;
  }

  Serial.print(F("Temperature: "));
  if (isErrorLine) Serial.print(F("ERROR"));
  else
  {
    Serial.print(t, 1);
    Serial.print(F(" C"));
  }

  Serial.print(F(" | Humidity: "));
  if (isErrorLine) Serial.print(F("ERROR"));
  else
  {
    Serial.print(h, 1);
    Serial.print(F(" %"));
  }

  auto printTrueFlag = [&](const __FlashStringHelper* label, bool value)
  {
    if (!value) return;
    Serial.print(F(" | "));
    Serial.print(label);
    Serial.print(F(" = TRUE"));
  };

  printTrueFlag(F("MO"), manualOverride);
  printTrueFlag(F("Mute"), muteBuzzer);
  printTrueFlag(F("Fire"), fireActive);
  printTrueFlag(F("RoTC"), rapidRiseAlarm);
  printTrueFlag(F("Sim"), simActive);

  // Date-time field in format: MM/DD/YYYY H:MM:SS AM/PM
  Serial.print(F(" | "));
  if (!timeSynced)
  {
    Serial.print(F("Time is not synced."));
  }
  else
  {
    time_t nowEpoch = time(nullptr);
    struct tm timeinfo;
    if (!localtime_r(&nowEpoch, &timeinfo))
    {
      Serial.print(F("ERR"));
    }
    else
    {
      int month = timeinfo.tm_mon + 1;
      int day   = timeinfo.tm_mday;
      int year  = timeinfo.tm_year + 1900;

      int hour24 = timeinfo.tm_hour;
      bool isPM  = (hour24 >= 12);
      int hour12 = hour24 % 12;
      if (hour12 == 0) hour12 = 12;

      if (month < 10) Serial.print('0');
      Serial.print(month);
      Serial.print('/');
      if (day < 10) Serial.print('0');
      Serial.print(day);
      Serial.print('/');
      Serial.print(year);
      Serial.print(' ');

      // Removes leading zero on single digit hour (e.g 04:xx:xx → 4:xx:xx | 11:xx:xx → 11:xx:xx)
      Serial.print(hour12);
      Serial.print(':');
      if (timeinfo.tm_min < 10) Serial.print('0');
      Serial.print(timeinfo.tm_min);
      Serial.print(':');
      if (timeinfo.tm_sec < 10) Serial.print('0');
      Serial.print(timeinfo.tm_sec);
      Serial.print(' ');
      Serial.print(isPM ? F("PM") : F("AM"));
    }
  }

  if (!isErrorLine && !isnan(t))
  {
    int band = computeBandForTemperature(t);
    Serial.print(F(" | Band="));
    Serial.print(band);
  }

  Serial.println();
}

// ---------------------- Time Sync Stub Logic --------------------------

// Background time sync stub.
// Allows for integration with WiFi and NTP.

void maybeStartTimeSync(float temp)
{
  // auto-time-sync is disabled during simulation.
  // SIM is a test mode. It should not steal RGB priority or start WiFi.
  if (simActive)
  {
    return;
  }

  if (timeSynced || timeSyncInProgress)
  {
    return;
  }

  if (temp >= cfg_TIMEAUTO)
  {
    Serial.println(F("[TIME] TIMEAUTO threshold reached, requesting sync."));
    requestTimeSync();
  }
}

void updateTimeSync()
{
  // If we are in the middle of a sync, run the WiFi + NTP state machine.
  if (timeSyncInProgress)
  {
    // Drive WiFi multi-AP connection attempts in the background.
    wifiTask();

    // Still not connected to any AP yet → keep trying.
    if (WiFi.status() != WL_CONNECTED)
    {
      return;
    }

    // Once WiFi is connected, start NTP if not yet started.
    if (!ntpWaiting)
    {
      recordActivity();
      Serial.println(F("[TIME] WiFi connected, requesting NTP."));
      configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC,
                 "pool.ntp.org", "time.nist.gov");
      ntpWaiting = true;
      ntpStartMS = millis();
      return;
    }

    // NTP already requested; poll getLocalTime without blocking.
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 0))  // 0 ms timeout → non-blocking
    {
      recordActivity();
      timeSynced        = true;
      lastTimeSyncEpoch = time(nullptr);
      Serial.println(F("[TIME] NTP sync successful."));

      // Once connected, turn WiFi back off to save power.
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);

      timeSyncInProgress = false;
      ntpWaiting         = false;
      setRgbMode(RGBMODE_SAFE);
      return;
    }

    // If NTP is taking too long, give up this attempt.
    if (millis() - ntpStartMS >= NTP_TIMEOUT_MS)
    {
      Serial.println(F("[TIME] timeout on acquisition of NTP time"));

      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);

      timeSyncInProgress = false;
      ntpWaiting         = false;
      setRgbMode(RGBMODE_SAFE);
    }

    return;
  }

  // Not currently syncing: check if we need a periodic resync.
  if (!timeSynced)
  {
    return;
  }

  time_t nowEpoch = time(nullptr);
  if (lastTimeSyncEpoch == 0)
  {
    lastTimeSyncEpoch = nowEpoch;
    return;
  }

  if (nowEpoch - lastTimeSyncEpoch >= RESYNC_INTERVAL_SEC)
  {
    Serial.println(F("[TIME] 30-day interval reached, requesting resync."));
    requestTimeSync();
  }
}

void requestTimeSync()
{
  if (timeSyncInProgress)
  {
    return;
  }

  timeSyncInProgress = true;
  timeSynced         = false;
  ntpWaiting         = false;
  ntpStartMS         = 0;

  setRgbMode(RGBMODE_WIFI_SYNC);

  // Prepare WiFi STA mode; actual connection attempts will be done by wifiTask() using wifiList[].
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

  // Reset WiFi round-robin state so wifiTask() starts from the first AP.
  wifiIndex      = 0;
  wifiConnecting = false;

  Serial.println(F("[TIME] Starting WiFi time sync."));
}

// Shared timestamp writer: prints "MM/DD/YYYY H:MM:SS AM/PM" without prefix or newline.

static bool writeLocalTimeStampToSerial(time_t epoch)
{
  struct tm timeinfo;
  if (!localtime_r(&epoch, &timeinfo))
  {
    return false;
  }

  int month = timeinfo.tm_mon + 1;
  int day   = timeinfo.tm_mday;
  int year  = timeinfo.tm_year + 1900;

  int hour24 = timeinfo.tm_hour;
  bool isPM  = (hour24 >= 12);
  int hour12 = hour24 % 12;
  if (hour12 == 0) hour12 = 12;

  if (month < 10) Serial.print('0');
  Serial.print(month);
  Serial.print('/');
  if (day < 10) Serial.print('0');
  Serial.print(day);
  Serial.print('/');
  Serial.print(year);
  Serial.print(' ');

  Serial.print(hour12);
  Serial.print(':');
  if (timeinfo.tm_min < 10) Serial.print('0');
  Serial.print(timeinfo.tm_min);
  Serial.print(':');
  if (timeinfo.tm_sec < 10) Serial.print('0');
  Serial.print(timeinfo.tm_sec);
  Serial.print(' ');
  Serial.print(isPM ? F("PM") : F("AM"));

  return true;
}

void printCurrentTime()
{
  if (!timeSynced)
  {
    Serial.println(F("[TIME] Not synced yet."));
    return;
  }

  time_t nowEpoch = time(nullptr);

  Serial.print(F("[TIME] "));
  if (!writeLocalTimeStampToSerial(nowEpoch))
  {
    Serial.println(F("Time sync failed."));
    return;
  }
  Serial.println();
}
// ======================================================================
// Part 2: events, EEPROM, servo, buzzer, RGB, seven segment, button
// ======================================================================

// ------------------------ EEPROM Helpers -------------------------------

void saveStatsToEEPROM()
{
  stats.magic = STATS_MAGIC;
  uint8_t *p = reinterpret_cast<uint8_t *>(&stats);
  for (int i = 0; i < (int)sizeof(StatsData); ++i)
  {
    EEPROM.write(eepromStatsOffset + i, p[i]);
  }
  EEPROM.commit();
}

void loadStatsFromEEPROM()
{
  StatsData temp;
  uint8_t *p = reinterpret_cast<uint8_t *>(&temp);
  for (int i = 0; i < (int)sizeof(StatsData); ++i)
  {
    p[i] = EEPROM.read(eepromStatsOffset + i);
  }

  if (temp.magic != STATS_MAGIC)
  {
    memset(&stats, 0, sizeof(StatsData));
    stats.magic = STATS_MAGIC;
    stats.maxTempC = -1000.0f;
    saveStatsToEEPROM();
    for (int i = eepromLogOffset; i < EEPROM_SIZE; ++i)
    {
      EEPROM.write(i, 0xFF);
    }
    EEPROM.commit();
    logWriteIndex = 0;
  }
  else
  {
    stats = temp;
  }
}

void clearStatsAndLog()
{
  memset(&stats, 0, sizeof(StatsData));
  stats.magic = STATS_MAGIC;
  stats.maxTempC = -1000.0f;
  saveStatsToEEPROM();

  for (int i = eepromLogOffset; i < EEPROM_SIZE; ++i)
  {
    EEPROM.write(i, 0xFF);
  }
  EEPROM.commit();
  logWriteIndex = 0;
}

void appendEvent(const EventRecord &ev)
{
  int recordSize = (int)sizeof(EventRecord);
  int maxRecords = eepromLogSize / recordSize;
  if (maxRecords <= 0)
  {
    return;
  }

  if (logWriteIndex >= (uint16_t)maxRecords)
  {
    logWriteIndex = 0;
  }

  int base = eepromLogOffset + logWriteIndex * recordSize;

  const uint8_t *p = reinterpret_cast<const uint8_t *>(&ev);
  for (int i = 0; i < recordSize; ++i)
  {
    EEPROM.write(base + i, p[i]);
  }
  EEPROM.commit();

  logWriteIndex++;
}

void loadLogWriteIndex()
{
  int recordSize = (int)sizeof(EventRecord);
  int maxRecords = eepromLogSize / recordSize;
  if (maxRecords <= 0)
  {
    logWriteIndex = 0;
    return;
  }

  logWriteIndex = 0;

  for (int idx = 0; idx < maxRecords; ++idx)
  {
    int base = eepromLogOffset + idx * recordSize;
    uint8_t firstByte = EEPROM.read(base);
    if (firstByte == 0xFF)
    {
      logWriteIndex = idx;
      return;
    }
  }

  logWriteIndex = maxRecords;
}

// ------------------------- Event Builders ------------------------------

static uint8_t buildFlags()
{
  uint8_t f = 0;
  if (manualOverride) f |= FLAG_MANUAL;
  if (fireActive)     f |= FLAG_FIRE;
  if (rapidRiseAlarm) f |= FLAG_ROTC;
  if (simActive)      f |= FLAG_SIM;
  return f;
}

void logBandChange(int oldBand, int newBand)
{
  EventRecord ev;
  memset(&ev, 0, sizeof(ev));
  ev.type        = EVT_BND;
  ev.reason      = 0;
  ev.band        = (uint8_t)newBand;
  ev.flags       = buildFlags();
  ev.millisStamp = millis();

  float t = simActive ? NoisySimTemp : currentTemp;
  float h = lastValidHum;

  if (isnan(t))
  {
    ev.tempX10 = 0;
  }
  else
  {
    ev.tempX10 = (uint32_t)lroundf(t * 10.0f);
  }

  if (isnan(h))
  {
    ev.humX10 = 0;
  }
  else
  {
    ev.humX10 = (uint16_t)lroundf(h * 10.0f);
  }

  ev.aux = (uint32_t)oldBand;

  appendEvent(ev);
}

void logFireThreshold(bool entering, float temp)
{
  EventRecord ev;
  memset(&ev, 0, sizeof(ev));
  ev.type        = EVT_THR;
  ev.reason      = entering ? (uint8_t)THR_TEMP35UP : (uint8_t)THR_TEMP35DN;
  ev.band        = (uint8_t)computeBandForTemperature(temp);
  ev.flags       = buildFlags();
  ev.millisStamp = millis();

  float h = lastValidHum;

  if (isnan(temp))
  {
    ev.tempX10 = 0;
  }
  else
  {
    ev.tempX10 = (uint32_t)lroundf(temp * 10.0f);
  }

  if (isnan(h))
  {
    ev.humX10 = 0;
  }
  else
  {
    ev.humX10 = (uint16_t)lroundf(h * 10.0f);
  }

  ev.aux = 0;

  appendEvent(ev);
}

void logRapidRise(float delta, RotcRating rating)
{
  EventRecord ev;
  memset(&ev, 0, sizeof(ev));
  ev.type        = EVT_ROTC;
  ev.reason      = (uint8_t)rating;
  ev.band        = (uint8_t)computeBandForTemperature(simActive ? NoisySimTemp : currentTemp);
  ev.flags       = buildFlags();
  ev.millisStamp = millis();

  float t = simActive ? NoisySimTemp : currentTemp;
  float h = lastValidHum;

  if (isnan(t))
  {
    ev.tempX10 = 0;
  }
  else
  {
    ev.tempX10 = (uint32_t)lroundf(t * 10.0f);
  }

  if (isnan(h))
  {
    ev.humX10 = 0;
  }
  else
  {
    ev.humX10 = (uint16_t)lroundf(h * 10.0f);
  }

  ev.aux = (uint32_t)lroundf(delta * 10.0f);

  appendEvent(ev);
}

void logButton(BtnReason reason)
{
  EventRecord ev;
  memset(&ev, 0, sizeof(ev));
  ev.type        = EVT_BTN;
  ev.reason      = (uint8_t)reason;
  ev.band        = (uint8_t)currentBand;
  ev.flags       = buildFlags();
  ev.millisStamp = millis();

  float t = simActive ? NoisySimTemp : currentTemp;
  float h = lastValidHum;

  if (isnan(t))
  {
    ev.tempX10 = 0;
  }
  else
  {
    ev.tempX10 = (uint32_t)lroundf(t * 10.0f);
  }

  if (isnan(h))
  {
    ev.humX10 = 0;
  }
  else
  {
    ev.humX10 = (uint16_t)lroundf(h * 10.0f);
  }

  ev.aux = 0;

  appendEvent(ev);
}

void logSystem(SysReason reason, uint32_t aux)
{
  EventRecord ev;
  memset(&ev, 0, sizeof(ev));
  ev.type        = EVT_SYS;
  ev.reason      = (uint8_t)reason;
  ev.band        = (uint8_t)currentBand;
  ev.flags       = buildFlags();
  ev.millisStamp = millis();

  float t = simActive ? NoisySimTemp : currentTemp;
  float h = lastValidHum;

  if (isnan(t))
  {
    ev.tempX10 = 0;
  }
  else
  {
    ev.tempX10 = (uint32_t)lroundf(t * 10.0f);
  }

  if (isnan(h))
  {
    ev.humX10 = 0;
  }
  else
  {
    ev.humX10 = (uint16_t)lroundf(h * 10.0f);
  }

  ev.aux = aux;

  appendEvent(ev);
}

void logCommand(CmdReason reason, uint32_t aux)
{
  EventRecord ev;
  memset(&ev, 0, sizeof(ev));
  ev.type        = EVT_CMD;
  ev.reason      = (uint8_t)reason;
  ev.band        = (uint8_t)currentBand;
  ev.flags       = buildFlags();
  ev.millisStamp = millis();

  float t = simActive ? NoisySimTemp : currentTemp;
  float h = lastValidHum;

  if (isnan(t))
  {
    ev.tempX10 = 0;
  }
  else
  {
    ev.tempX10 = (uint32_t)lroundf(t * 10.0f);
  }

  if (isnan(h))
  {
    ev.humX10 = 0;
  }
  else
  {
    ev.humX10 = (uint16_t)lroundf(h * 10.0f);
  }

  ev.aux = aux;

  appendEvent(ev);
}

void logSim(SimReason reason, float temp)
{
  EventRecord ev;
  memset(&ev, 0, sizeof(ev));
  ev.type        = EVT_SIM;
  ev.reason      = (uint8_t)reason;
  ev.band        = (uint8_t)computeBandForTemperature(temp);
  ev.flags       = buildFlags();
  ev.millisStamp = millis();

  float h = lastValidHum;

  if (isnan(temp))
  {
    ev.tempX10 = 0;
  }
  else
  {
    ev.tempX10 = (uint32_t)lroundf(temp * 10.0f);
  }

  if (isnan(h))
  {
    ev.humX10 = 0;
  }
  else
  {
    ev.humX10 = (uint16_t)lroundf(h * 10.0f);
  }

  ev.aux = 0;

  appendEvent(ev);
}

// -------------------------- STATS Printing ----------------------------

void printStats()
{
  Serial.println(F("=========== STATS ==========="));
  Serial.print(F("Total fire time (s): "));
  Serial.println(stats.totalFireTimeMS / 1000.0f, 1);

  Serial.print(F("Max Temp (C): "));
  if (stats.maxTempC < -999.0f)
  {
    Serial.println(F("none"));
  }
  else
  {
    Serial.println(stats.maxTempC, 1);
  }

  Serial.print(F("Single clicks: "));
  Serial.println(stats.singleClicks);

  Serial.print(F("Double clicks: "));
  Serial.println(stats.doubleClicks);

  Serial.print(F("Triple clicks: "));
  Serial.println(stats.tripleClicks);

  Serial.print(F("Quad clicks: "));
  Serial.println(stats.quadClicks);

  Serial.print(F("Long presses: "));
  Serial.println(stats.longPresses);

  Serial.print(F("Simulation sessions: "));
  Serial.println(stats.simSessions);

  Serial.print(F("Sensor error count: "));
  Serial.println(stats.sensorErrors);

  Serial.print(F("Last fire duration (s): "));
  Serial.println(stats.lastFireDurationMS / 1000.0f, 1);

  Serial.println(F("============================"));
}

// --------------------------- LOG Printing -----------------------------

void printLog(uint16_t limitCount)
{
  int recordSize = (int)sizeof(EventRecord);
  int maxRecords = eepromLogSize / recordSize;
  if (maxRecords <= 0)
  {
    Serial.println(F("[LOG] No log space configured."));
    return;
  }

  uint16_t totalRecords = 0;
  for (int idx = 0; idx < maxRecords; ++idx)
  {
    int base = eepromLogOffset + idx * recordSize;
    uint8_t firstByte = EEPROM.read(base);
    if (firstByte == 0xFF)
    {
      break;
    }
    totalRecords++;
  }

  if (totalRecords == 0)
  {
    Serial.println(F("[LOG] No events recorded."));
    return;
  }

  uint16_t toPrint = totalRecords;
  if (limitCount > 0 && limitCount < totalRecords)
  {
    toPrint = limitCount;
  }

  Serial.print(F("[LOG] Total records: "));
  Serial.println(totalRecords);
  Serial.print(F("[LOG] Printing last "));
  Serial.print(toPrint);
  Serial.println(F(" records."));

  int startIndex;
  if (toPrint == totalRecords)
  {
    startIndex = 0;
  }
  else
  {
    startIndex = totalRecords - toPrint;
  }

  for (int i = 0; i < (int)toPrint; ++i)
  {
    int idx = startIndex + i;
    int base = eepromLogOffset + idx * recordSize;

    EventRecord ev;
    uint8_t *p = reinterpret_cast<uint8_t *>(&ev);
    for (int j = 0; j < recordSize; ++j)
    {
      p[j] = EEPROM.read(base + j);
    }

    Serial.print(F("#"));
    Serial.print(idx);
    Serial.print(F(" t="));
    Serial.print(ev.millisStamp);

    Serial.print(F(" type="));
    Serial.print(ev.type);

    Serial.print(F(" reason="));
    Serial.print(ev.reason);

    Serial.print(F(" band="));
    Serial.print(ev.band);

    Serial.print(F(" flags="));
    Serial.print(ev.flags, HEX);

    Serial.print(F(" tempC="));
    Serial.print(ev.tempX10 / 10.0f, 1);

    Serial.print(F(" hum="));
    Serial.print(ev.humX10 / 10.0f, 1);

    Serial.print(F(" aux="));
    Serial.print(ev.aux);

    Serial.println();
  }
}

// ======================================================================
// Servo Control
// ======================================================================

void servoSetImmediate(int pos)
{
  currentServoPos = pos;
  targetServoPos  = pos;
  fireServo.write(pos);
}

void setServoTargetClosed()
{
  targetServoPos = cfg_SERVO_POS_CLOSED;
}

void setServoTargetReleased()
{
  targetServoPos = cfg_SERVO_POS_RELEASED;
}
void updateServo()
{
  unsigned long now = millis();

  if (currentServoPos == targetServoPos)
  {
    return;
  }

  // Smooth stepping with a time-based slew rate.
  // Servo moves toward target, non-blocking.

  const int SERVO_MAX_STEP_DEG = 20;                // cap per update
  const unsigned long SERVO_MIN_STEP_INTERVAL_MS = 10;

  // Normal speed vs fire speed (deg per second)
  const float SPEED_NORMAL_DPS = 2000.0f;
  const float SPEED_FIRE_DPS   = 4000.0f;

  if (now - lastServoStepMS < SERVO_MIN_STEP_INTERVAL_MS)
  {
    return;
  }

  unsigned long dt = now - lastServoStepMS;
  lastServoStepMS = now;

  int diff = targetServoPos - currentServoPos;
  int dir  = (diff > 0) ? 1 : -1;
  int adiff = (diff >= 0) ? diff : -diff;

  bool firePriorityMove = inFireRange || fireActive;
  float dps = firePriorityMove ? SPEED_FIRE_DPS : SPEED_NORMAL_DPS;

  // step = dps * dt seconds, clamped
  int step = (int)lroundf(dps * ((float)dt / 1000.0f));
  if (step < 1) step = 1;
  if (step > SERVO_MAX_STEP_DEG) step = SERVO_MAX_STEP_DEG;
  if (step > adiff) step = adiff;

  currentServoPos += dir * step;

  if ((dir > 0 && currentServoPos > targetServoPos) ||
      (dir < 0 && currentServoPos < targetServoPos))
  {
    currentServoPos = targetServoPos;
  }

  fireServo.write(currentServoPos);
}

// ======================================================================
// Buzzer Control
// ======================================================================

void buzzerSet(bool on)
{
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
}

void updateBuzzer()
{
  unsigned long now = millis();

  // Band buzzer timings. Editable only through code.
  const unsigned long B2_ON_MS  = 100;  // band 2 on-time
  const unsigned long B2_OFF_MS = 900;  // band 2 off-time

  const unsigned long B3_ON_MS  = 50;   // band 3 on-time
  const unsigned long B3_OFF_MS = 600;  // band 3 off-time

  const unsigned long B4_ON_MS  = 100;  // band 4 on-time
  const unsigned long B4_OFF_MS = 100;  // band 4 off-time

  const unsigned long OV_ON_MS  = 120;  // manual override on-time
  const unsigned long OV_OFF_MS = 180;  // manual override off-time

  // 0=OFF, 2=B2, 3=B3, 4=B4, 5=OVERRIDE, 6=TEST, 7=B5PLUS_CONST
  static int buzzerMode = 0;

  auto stopBuzzer = [&]()
  {
    buzzerSet(false);
    buzzerActive  = false;
    buzzerOnPhase = false;
  };

  auto bandFromTemp = [&](float t) -> int
  {
    if (isnan(t)) return -1;

    if (t < 30.0f)  return 0;
    if (t < 35.0f)  return 1;
    if (t < 40.0f)  return 2;
    if (t < 45.0f)  return 3;
    if (t < 50.0f)  return 4;
    if (t < 60.0f)  return 5;
    if (t < 70.0f)  return 6;
    if (t < 80.0f)  return 7;
    if (t < 90.0f)  return 8;
    if (t < 100.0f) return 9;
    return 10;
  };

  // Highest priority: mute always wins
  if (muteBuzzer)
  {
    buzzerMode = 0;
    stopBuzzer();
    return;
  }

  int desiredMode = 0;
  unsigned long onTime  = 0;
  unsigned long offTime = 0;

  // Priority 2: Manual override must work even during sensorError or NAN temp
  if (manualOverride)
  {
    desiredMode = 5;
    onTime  = OV_ON_MS;
    offTime = OV_OFF_MS;
  }
  // Priority 3: band test preview should also work even during sensorError
  else if (buzzerBandTestActive)
  {
    if (now >= buzzerBandTestEndMS)
    {
      buzzerBandTestActive = false;
      desiredMode = 0;
    }
    else
    {
      desiredMode = 6;

      int band = buzzerBandTestBand;

      if (band <= 1)
      {
        desiredMode = 0;
      }
      else if (band == 2)
      {
        onTime  = B2_ON_MS;
        offTime = B2_OFF_MS;
      }
      else if (band == 3)
      {
        onTime  = B3_ON_MS;
        offTime = B3_OFF_MS;
      }
      else if (band == 4)
      {
        onTime  = B4_ON_MS;
        offTime = B4_OFF_MS;
      }
      else
      {
        desiredMode = 7; // 5+
      }
    }
  }
  else
  {
    // Normal mode uses SIM temp when simActive, even if sensorError is true
    bool blockForSensorError = sensorError && !simActive;

    if (blockForSensorError)
    {
      desiredMode = 0;
    }
    else
    {
      float tempToUse = simActive ? NoisySimTemp : currentTemp;

      if (isnan(tempToUse))
      {
        desiredMode = 0;
      }
      else
      {
        int band = bandFromTemp(tempToUse);

        if (band <= 1)
        {
          desiredMode = 0;
        }
        else if (band == 2)
        {
          desiredMode = 2;
          onTime  = B2_ON_MS;
          offTime = B2_OFF_MS;
        }
        else if (band == 3)
        {
          desiredMode = 3;
          onTime  = B3_ON_MS;
          offTime = B3_OFF_MS;
        }
        else if (band == 4)
        {
          desiredMode = 4;
          onTime  = B4_ON_MS;
          offTime = B4_OFF_MS;
        }
        else
        {
          desiredMode = 7; // 5+
        }
      }
    }
  }

  // Apply mode change and start ON for blink modes
  if (desiredMode != buzzerMode)
  {
    buzzerMode   = desiredMode;
    buzzerLastMS = now;

    if (buzzerMode == 0)
    {
      stopBuzzer();
      return;
    }

    if (buzzerMode == 7)
    {
      buzzerSet(true);
      buzzerActive  = true;
      buzzerOnPhase = true;
      return;
    }

    buzzerOnPhase = true;
    buzzerSet(true);
    buzzerActive = true;
    return;
  }

  if (buzzerMode == 0)
  {
    stopBuzzer();
    return;
  }

  if (buzzerMode == 7)
  {
    buzzerSet(true);
    buzzerActive  = true;
    buzzerOnPhase = true;
    return;
  }

  if (onTime < 1) onTime = 1;
  if (offTime < 1) offTime = 1;

  if (buzzerOnPhase)
  {
    if (now - buzzerLastMS >= onTime)
    {
      buzzerOnPhase = false;
      buzzerLastMS  = now;
      buzzerSet(false);
    }
  }
  else
  {
    if (now - buzzerLastMS >= offTime)
    {
      buzzerOnPhase = true;
      buzzerLastMS  = now;
      buzzerSet(true);
    }
  }

  buzzerActive = buzzerOnPhase;
}

// ======================================================================
// RGB LED Control
// ======================================================================

void rgbWriteRaw(uint8_t r, uint8_t g, uint8_t b)
{
  float level = cfg_RGBLEVEL;
  if (level < 0.0f) level = 0.0f;
  if (level > 1.0f) level = 1.0f;

  uint8_t rr = (uint8_t)(r * level);
  uint8_t gg = (uint8_t)(g * level);
  uint8_t bb = (uint8_t)(b * level);

  analogWrite(PIN_RGB_R, rr);
  analogWrite(PIN_RGB_G, gg);
  analogWrite(PIN_RGB_B, bb);
}

void setRgbMode(RgbMode mode)
{
  rgbMode = mode;
}

void updateRGB()
{
  unsigned long now = millis();

  // Sleep override
  if (displaySleep)
  {
    rgbWriteRaw(0, 0, 0);
    return;
  }

  // Color wheel helper for RAMP
  auto wheel = [&](uint8_t p, uint8_t &r, uint8_t &g, uint8_t &b)
  {
    if (p < 85)
    {
      r = (uint8_t)(255 - p * 3);
      g = (uint8_t)(p * 3);
      b = 0;
      return;
    }
    if (p < 170)
    {
      p -= 85;
      r = 0;
      g = (uint8_t)(255 - p * 3);
      b = (uint8_t)(p * 3);
      return;
    }
    p -= 170;
    r = (uint8_t)(p * 3);
    g = 0;
    b = (uint8_t)(255 - p * 3);
  };

  // Band preview test path
  if (isRgbTestModeActive())
  {
    int band = getRgbTestBand();
    if (band >= 2) rgbWriteRaw(255, 0, 0);
    else           rgbWriteRaw(0, 255, 0);
    return;
  }

  // Internal state for FLASH / PULSE / RAMP / BRIGHT tests
  static uint8_t testMode = 0;                 // 0 none, 1 flash, 2 pulse, 3 ramp, 4 bright
  static unsigned long testEndMS = 0;
  static unsigned long testStartMS = 0;        // start time for FLASH sequencing
  static int brightLevel = 10;                 // 0..10

  auto startTest = [&](uint8_t mode, unsigned long durMs)
  {
    testMode = mode;
    testStartMS = now;
    testEndMS = now + durMs;

    rgbModeBeforeTest = rgbMode;
    rgbMode = RGBMODE_TEST;
  };

  // Start tests from request flags
  if (rgbTestFlashRequested)
  {
    rgbTestFlashRequested = false;
    startTest(1, 12000UL); // 3 s per color x 4 colors
  }
  else if (rgbTestPulseRequested)
  {
    rgbTestPulseRequested = false;
    startTest(2, cfg_TESTDUR * 1000UL);
  }
  else if (rgbTestRampRequested)
  {
    rgbTestRampRequested = false;
    startTest(3, cfg_TESTDUR * 1000UL);
  }
  else if (rgbTestBrightLevelRequested >= 0)
  {
    long lvl = rgbTestBrightLevelRequested;
    rgbTestBrightLevelRequested = -1;

    if (lvl < 0) lvl = 0;
    if (lvl > 10) lvl = 10;
    brightLevel = (int)lvl;

    startTest(4, cfg_TESTDUR * 1000UL);
  }

  // Render active test
  if (testMode != 0)
  {
    if ((long)(now - testEndMS) >= 0)
    {
      testMode = 0;
      rgbMode = rgbModeBeforeTest;
      // fall through to normal rendering
    }
    else
    {
      if (testMode == 1)
      {
        // FLASH: 3 s slices, blink 50 ms on, 50 ms off
        const unsigned long SLICE_MS = 3000UL;

        const unsigned long BLINK_ON_MS  = 50UL;
        const unsigned long BLINK_OFF_MS = 50UL;
        const unsigned long BLINK_PERIOD_MS = BLINK_ON_MS + BLINK_OFF_MS;

        unsigned long elapsed = now - testStartMS;

        uint8_t baseR = 0, baseG = 0, baseB = 0;

        unsigned long slice = elapsed / SLICE_MS; // 0..3
        if (slice == 0)      { baseR = 255; baseG = 255; baseB = 255; } // white
        else if (slice == 1) { baseR = 255; baseG = 0;   baseB = 0;   } // red
        else if (slice == 2) { baseR = 0;   baseG = 255; baseB = 0;   } // green
        else                 { baseR = 0;   baseG = 0;   baseB = 255; } // blue

        bool on = ((elapsed % BLINK_PERIOD_MS) < BLINK_ON_MS);
        rgbWriteRaw(on ? baseR : 0, on ? baseG : 0, on ? baseB : 0);
        return;
      }
      else if (testMode == 2)
      {
        // PULSE: breathe blue
        const unsigned long period = 1800;
        unsigned long ph = now % period;
        float x = (ph < (period / 2)) ? (float)ph / (float)(period / 2)
                                      : (float)(period - ph) / (float)(period / 2);
        uint8_t v = (uint8_t)(x * 255.0f);
        rgbWriteRaw(0, 0, v);
        return;
      }
      else if (testMode == 3)
      {
        // RAMP: color cycle
        uint8_t r, g, b;
        uint8_t hue = (uint8_t)((now / 8) & 0xFF);
        wheel(hue, r, g, b);
        rgbWriteRaw(r, g, b);
        return;
      }
      else if (testMode == 4)
      {
        // BRIGHT: white at level N
        uint8_t v = (uint8_t)((brightLevel * 255) / 10);
        rgbWriteRaw(v, v, v);
        return;
      }
    }
  }

  // ----------------------- NORMAL LAYER -----------------------

  bool ledSensorError = sensorError && !simActive;

  const unsigned long SAFE_GREEN_PERIOD_MS = 2000;
  const unsigned long SAFE_GREEN_ON_MS     = 1000;

  if (ledSensorError)
  {
    unsigned long period = 300;
    unsigned long phase  = now % period;
    bool on = (phase < 40) || (phase >= 80 && phase < 120);
    rgbWriteRaw(on ? 255 : 0, on ? 255 : 0, on ? 255 : 0);
    return;
  }

  if (manualOverride)
  {
    unsigned long period = 500;
    unsigned long onTime = 250;
    unsigned long phase  = now % period;
    bool on = (phase < onTime);
    rgbWriteRaw(on ? 255 : 0, on ? 255 : 0, 0);
    return;
  }

  // If there is no sensor error and FireActive is false, show SAFE green.
  // Time sync must not steal SAFE indication color.
  if (timeSyncInProgress && !simActive)
  {
    unsigned long phase = now % SAFE_GREEN_PERIOD_MS;
    bool on = (phase < SAFE_GREEN_ON_MS);
    rgbWriteRaw(0, on ? 255 : 0, 0);
    return;
  }

  float tempToUse = simActive ? NoisySimTemp : currentTemp;
  if (isnan(tempToUse))
  {
    rgbWriteRaw(0, 0, 0);
    return;
  }

  int band = computeBandForTemperature(tempToUse);

  if (band >= 2)
  {
    rgbWriteRaw(255, 0, 0);
    return;
  }

  unsigned long phase = now % SAFE_GREEN_PERIOD_MS;
  bool on = (phase < SAFE_GREEN_ON_MS);
  rgbWriteRaw(0, on ? 255 : 0, 0);
}

// ======================================================================
// Seven Segment Display Control
// ======================================================================

static const uint8_t SEG_PATTERN_DIGIT[10] =
{
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111  // 9
};

uint8_t patternForChar(char c)
{
  // Normalize to uppercase so 'a' and 'A' look the same, same for 'l' and 'L' etc.
  c = toupper((unsigned char)c);

  switch (c)
  {
    // Letters that look ok on 7-segment
    case 'A': return 0b01110111; // A, no bottom segment
    case 'B': return 0b01111100; // b
    case 'C': return 0b00111001; // C
    case 'D': return 0b01011110; // d
    case 'E': return 0b01111001; // E
    case 'F': return 0b01110001; // F
    case 'G': return 0b01101111; // g
    case 'H': return 0b01110110; // H
    case 'I': return 0b00000110; // I looks like 1
    case 'J': return 0b00011110; // J
    case 'L': return 0b00111000; // L
    case 'N': return 0b01010100; // n 
    case 'O': return 0b00111111; // O = 0
    case 'P': return 0b01110011; // P
    case 'Q': return 0b01100111; // q 
    case 'R': return 0b01010000; // r 
    case 'S': return 0b01101101; // S = 5
    case 'T': return 0b01111000; // t
    case 'U': return 0b00111110; // U
    case 'Y': return 0b01101110; // Y

    // Symbols
    case '-': return 0b01000000; // dash
    case '_': return 0b00001000; // underscore
    case ' ': return 0b00000000; // blank

    default:
      // Characters you cannot draw (K, M, V, X, Z, etc) fall back to blank
      return 0b00000000;
  }
}

void sevenSegWritePattern(uint8_t p)
{
  digitalWrite(PIN_SEG_A, (p & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_SEG_B, (p & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_SEG_C, (p & 0x04) ? HIGH : LOW);
  digitalWrite(PIN_SEG_D, (p & 0x08) ? HIGH : LOW);
  digitalWrite(PIN_SEG_E, (p & 0x10) ? HIGH : LOW);
  digitalWrite(PIN_SEG_F, (p & 0x20) ? HIGH : LOW);
  digitalWrite(PIN_SEG_G, (p & 0x40) ? HIGH : LOW);
  digitalWrite(PIN_SEG_DP, LOW);
}

void sevenSegAllOff()
{
  sevenSegWritePattern(0);
}

void sevenSegShowDigit(int digit)
{
  if (digit < 0 || digit > 9)
  {
    sevenSegAllOff();
    return;
  }
  uint8_t p = SEG_PATTERN_DIGIT[digit];
  sevenSegWritePattern(p);
}

void sevenSegShowChar(char c)
{
  // If the char is a digit, reuse the digit patterns
  if (c >= '0' && c <= '9')
  {
    int d = c - '0';
    sevenSegShowDigit(d);
    return;
  }

  // Otherwise use the custom patterns (E, L, S, -)
  uint8_t p = patternForChar(c);
  sevenSegWritePattern(p);
}

// Make these match your sevenSegWritePattern bit order.
static const uint8_t SEG_A = 0b00000001;
static const uint8_t SEG_B = 0b00000010;
static const uint8_t SEG_C = 0b00000100;
static const uint8_t SEG_D = 0b00001000;
static const uint8_t SEG_E = 0b00010000;
static const uint8_t SEG_F = 0b00100000;
static const uint8_t SEG_G = 0b01000000;

// Set to 1 if segments are active-low (0 = ON, 1 = OFF).
static const uint8_t SEVENSEG_ACTIVE_LOW = 0;

static inline uint8_t segDrive(uint8_t mask)
{
  return SEVENSEG_ACTIVE_LOW ? (uint8_t)~mask : mask;
}

// Single-cycle, non-blocking TESTSEG (no layers, no delay, no double cycle)
static const uint16_t TESTSEG_STEP_MS = 90;

static const uint8_t TESTSEG_ORDER[] = { SEG_B, SEG_A, SEG_F, SEG_E, SEG_D, SEG_C, SEG_G };
static const uint8_t TESTSEG_LEN = (uint8_t)(sizeof(TESTSEG_ORDER) / sizeof(TESTSEG_ORDER[0]));

static bool g_testseg_active = false;
static uint8_t g_testseg_step = 0;
static uint32_t g_testseg_next_ms = 0;

static void testSegStop()
{
  g_testseg_active = false;
  g_testseg_step = 0;
  g_testseg_next_ms = 0;
}

bool sevenSegIsOverrideActive()
{
  return g_testseg_active;
}

// Call from command handler for TESTSEG
void sevenSegDisplayTestSeg()
{
  uint32_t now = millis();
  g_testseg_active = true;
  g_testseg_step = 0;
  g_testseg_next_ms = now;
}

// Call once per loop from updateSevenSeg
void sevenSegTestSegService()
{
  if (!g_testseg_active) return;

  uint32_t now = millis();
  if ((int32_t)(now - g_testseg_next_ms) < 0) return;

  sevenSegWritePattern(segDrive(TESTSEG_ORDER[g_testseg_step]));
  g_testseg_step++;
  g_testseg_next_ms = now + TESTSEG_STEP_MS;

  if (g_testseg_step >= TESTSEG_LEN)
  {
    sevenSegAllOff();
    testSegStop();
  }
}

void sevenSegTestSeg()
{
  sevenSegDisplayTestSeg();
}

void setDisplayOverrideChar(char c, unsigned long durationMS)
{
  displayOverrideActive = true;
  displayOverrideChar   = c;
  displayOverrideEndMS  = millis() + durationMS;
}

void clearDisplayOverride()
{
  displayOverrideActive = false;
  displayOverrideChar   = ' ';
}

void setDisplayBlankForCritical()
{
  sevenSegAllOff();
}

void updateSevenSeg()
{
  if (displaySleep)
  {
    sevenSegAllOff();
    return;
  }

  // TESTSEG override has top priority while active
  sevenSegTestSegService();
  if (sevenSegIsOverrideActive())
  {
    return;
  }

  unsigned long now = millis();

  if (displayOverrideActive && now >= displayOverrideEndMS)
  {
    clearDisplayOverride();
  }

  if (sensorError)
  {
    if (displayOverrideActive) sevenSegShowChar(displayOverrideChar);
    else sevenSegShowChar('E');
    return;
  }

  float tempToUse = simActive ? NoisySimTemp : currentTemp;

  if (!isnan(tempToUse) && tempToUse >= 100.0f)
  {
    setDisplayBlankForCritical();
    return;
  }

  if (displayOverrideActive)
  {
    sevenSegShowChar(displayOverrideChar);
    return;
  }

  static bool init = false;
  static int stableBand = 0;
  static int candidateBand = 0;
  static unsigned long candidateSinceMS = 0;

  int rawBand = computeBandForTemperature(tempToUse);
  if (rawBand < 0) rawBand = 0;
  if (rawBand > 9) rawBand = 9;

  if (!init)
  {
    init = true;
    stableBand = rawBand;
    candidateBand = rawBand;
    candidateSinceMS = now;
  }

  if (rawBand != candidateBand)
  {
    candidateBand = rawBand;
    candidateSinceMS = now;
  }

  if (candidateBand != stableBand && (now - candidateSinceMS) >= 250UL)
  {
    stableBand = candidateBand;
  }

  sevenSegShowDigit(stableBand);
}

// ======================================================================
// Button Handling
// ======================================================================

void onSingleClick()
{
  recordActivity();
  manualOverride = !manualOverride;

  if (manualOverride)
  {
    // Manual override = force deploy
    setServoTargetReleased();
    logSystem(SYS_SERVO_DN, 0);     // existing servo event
    logSystem(SYS_MANUAL_ON, 0);    // Manual Override event
  }
  else
  {
    // Turning override off: close only if fire is not active
    if (!fireActive && !inFireRange)
    {
      setServoTargetClosed();
      logSystem(SYS_SERVO_UP, 0);   // existing servo event
    }
    logSystem(SYS_MANUAL_OFF, 0);   // Manual Override Off event
  }

  stats.singleClicks++;
  saveStatsToEEPROM();
  setDisplayOverrideChar('1', cfg_SEGOVR);
  logButton(BTN_1);
}

void onDoubleClick()
{
  muteBuzzer = !muteBuzzer;

  if (muteBuzzer)
  {
    logSystem(SYS_MUTE_ON, 0);   // "Mute = On"
  }
  else
  {
    logSystem(SYS_MUTE_OFF, 0);  // "Mute Off"
  }

  stats.doubleClicks++;
  saveStatsToEEPROM();
  setDisplayOverrideChar('2', cfg_SEGOVR);
  logButton(BTN_2);
}

void onTripleClick()
{
  stats.tripleClicks++;
  saveStatsToEEPROM();
  setDisplayOverrideChar('3', cfg_SEGOVR);
  logButton(BTN_3);

  printStats();
  printLog(cfg_LOGLIMIT);
}

void onQuadClick()
{
  stats.quadClicks++;
  saveStatsToEEPROM();
  setDisplayOverrideChar('4', cfg_SEGOVR);
  logButton(BTN_4);

  clearStatsAndLog();
  logSystem(SYS_CLR_EEP_BTN, 0);
}

void onLongPress()
{
  muteBuzzer = true;
  manualOverride = false;
  setServoTargetClosed();

  stats.longPresses++;
  saveStatsToEEPROM();

  setDisplayOverrideChar('L', cfg_SEGOVR);
  logButton(BTN_LONG);
}

void updateButton()
{
  unsigned long now = millis();

  // Tracks an early deploy that happened before CLICKGAP finalization.
  // Used to revert to CLOSED if the sequence becomes 2/3/4 clicks.
  static bool earlyReleaseFromSeq = false;

  bool raw = digitalRead(PIN_BUTTON);

  if (raw != buttonLastRaw)
  {
    buttonLastDebounce = now;
    buttonLastRaw      = raw;
  }

  if ((now - buttonLastDebounce) > BUTTON_DEBOUNCE_MS)
  {
    if (raw != buttonStable)
    {
      recordActivity();
      buttonStable = raw;

      if (buttonStable == LOW)
      {
        buttonPressTime = now;
        longPressFired  = false;
      }
      else
      {
        buttonReleaseTime = now;

        if (!longPressFired)
        {
          clickCount++;
          lastClickTime = now;

          // Immediate servo priority:
          // First click release triggers RELEASED immediately, only from fully CLOSED.
          // No twitch behavior when already at RELEASED.
          bool servoFullyClosed =
            (currentServoPos == cfg_SERVO_POS_CLOSED) &&
            (targetServoPos  == cfg_SERVO_POS_CLOSED);

          bool fireLockRelease = (fireActive || inFireRange);

          if (clickCount == 1 && servoFullyClosed && !fireLockRelease)
          {
            setServoTargetReleased();
            earlyReleaseFromSeq = true;
          }
        }
      }
    }
  }

  if (buttonStable == LOW && !longPressFired)
  {
    if ((now - buttonPressTime) >= cfg_LONGLONG && clickCount == 0)
    {
      longPressFired = true;
      onLongPress();
      clickCount = 0;
      earlyReleaseFromSeq = false;
    }
  }

  if (buttonStable == HIGH && clickCount > 0)
  {
    if ((now - lastClickTime) >= cfg_CLICKGAP)
    {
      int pattern = clickCount;
      if (pattern > 4) pattern = 4;

      bool fireLockRelease = (fireActive || inFireRange);

      // If the sequence became 2/3/4 clicks and an early deploy occurred, revert to CLOSED immediately. Twitch is allowed here.
      if (pattern > 1 && earlyReleaseFromSeq && !fireLockRelease)
      {
        servoSetImmediate(cfg_SERVO_POS_CLOSED);
        setServoTargetClosed();
      }

      // Single click keeps RELEASED (manual override toggle runs inside onSingleClick()).
      // No twitch path when servo already sits at RELEASED, since earlyReleaseFromSeq stays false.
      switch (pattern)
      {
        case 1: onSingleClick(); break;
        case 2: onDoubleClick(); break;
        case 3: onTripleClick(); break;
        case 4: onQuadClick();   break;
      }

      clickCount = 0;
      earlyReleaseFromSeq = false;
    }
  }
}

// ======================================================================
// Part 3: serial input, HELP system, ADJ system, SIM, TEMP BAND tests
// ======================================================================

// -------------------------- Serial Helpers -----------------------------

String serialLine;

String trimSpaces(const String &s)
{
  int start = 0;
  int end   = s.length() - 1;

  while (start <= end && isspace((unsigned char)s[start]))
  {
    start++;
  }
  while (end >= start && isspace((unsigned char)s[end]))
  {
    end--;
  }
  if (start > end)
  {
    return String("");
  }
  return s.substring(start, end + 1);
}

String toUpperCopy(const String &s)
{
  String out = s;
  out.toUpperCase();
  return out;
}

bool startsWithIgnoreCase(const String &s, const String &prefix)
{
  if (s.length() < prefix.length())
  {
    return false;
  }
  String left = s.substring(0, prefix.length());
  left.toUpperCase();
  String upPre = prefix;
  upPre.toUpperCase();
  return left == upPre;
}

void splitFirstWord(const String &line, String &first, String &rest)
{
  String t = trimSpaces(line);
  int sp = t.indexOf(' ');
  if (sp < 0)
  {
    first = t;
    rest  = String("");
    return;
  }
  first = t.substring(0, sp);
  rest  = trimSpaces(t.substring(sp + 1));
}

bool parseFloatSafe(const String &s, float &valueOut)
{
  String t = trimSpaces(s);
  if (t.length() == 0)
  {
    return false;
  }

  bool hasDigit = false;
  int dotCount  = 0;
  int start     = 0;

  if (t[0] == '+' || t[0] == '-')
  {
    start = 1;
  }

  for (int i = start; i < t.length(); ++i)
  {
    char c = t[i];
    if (c >= '0' && c <= '9')
    {
      hasDigit = true;
      continue;
    }
    if (c == '.')
    {
      dotCount++;
      continue;
    }
    return false;
  }

  if (!hasDigit || dotCount > 1)
  {
    return false;
  }

  valueOut = t.toFloat();
  return true;
}

bool parseIntSafe(const String &s, long &valueOut)
{
  String t = trimSpaces(s);
  if (t.length() == 0)
  {
    return false;
  }

  bool hasDigit = false;
  int start     = 0;

  if (t[0] == '+' || t[0] == '-')
  {
    start = 1;
  }

  for (int i = start; i < t.length(); ++i)
  {
    char c = t[i];
    if (c >= '0' && c <= '9')
    {
      hasDigit = true;
      continue;
    }
    return false;
  }

  if (!hasDigit)
  {
    return false;
  }

  valueOut = t.toInt();
  return true;
}

// ----------------------- Forward Declarations --------------------------

// Tests that will be detailed in Part 4
void startServoBandTest(int band, unsigned long durationMS);
void startBuzzerBandTest(int band, unsigned long durationMS);

// SIM helpers from Part 1 (assumed)
void setSimActive(bool active);
void setSimTemperature(float tempC);

// time sync helper from Part 1 (assumed)
void requestTimeSync();

// status printing helper from Part 1 (assumed)
void printStatusLinePeriodic();

// --------------------------- HELP System -------------------------------

void showHelpMain()
{
  Serial.println(F("========== HELP MAIN =========="));
  Serial.println(F("HELP WIFI      - time sync help"));
  Serial.println(F("HELP SSID      - WiFi SSID help"));
  Serial.println(F("HELP STATS     - stats and log help"));
  Serial.println(F("HELP LOG       - stats and log help"));
  Serial.println(F("HELP CTRL      - control commands"));
  Serial.println(F("HELP BUZZ      - buzzer commands"));
  Serial.println(F("HELP SERVO     - servo commands"));
  Serial.println(F("HELP TIME      - time commands"));
  Serial.println(F("HELP EEPROM    - EEPROM commands"));
  Serial.println(F("HELP RGB       - RGB commands"));
  Serial.println(F("HELP DISPLAY   - display commands"));
  Serial.println(F("HELP ADJ       - adjustment system"));
  Serial.println(F("Shortcuts: STATS, LOG, ADJ, SIM, BUZZ, SERVO, RGB, DISPLAY"));
  Serial.println(F("================================"));
}

void showHelpStatsLog()
{
  Serial.println(F("====== HELP STATS / LOG ======"));
  Serial.println(F("STATS          - print statistics summary"));
  Serial.println(F("LOG            - print full event log"));
  Serial.println(F("LOG N          - print last N events"));
  Serial.println(F("Triple click   - prints STATS and log with limit"));
  Serial.println(F("Quad click     - clears stats and event log"));
  Serial.println(F("==============================="));
}

void showHelpCtrl()
{
  Serial.println(F("=========== HELP CTRL ========="));
  Serial.println(F("Single click   - toggle manual override"));
  Serial.println(F("Double click   - toggle mute state"));
  Serial.println(F("Triple click   - print STATS and LOG"));
  Serial.println(F("Quad click     - clear stats and log"));
  Serial.println(F("Long press     - mute buzzer and cancel override"));
  Serial.println(F("Manual override moves window guard to Released."));
  Serial.println(F("Manual override stays active until next single click."));
  Serial.println(F("================================"));
}

void showHelpBuzz()
{
  Serial.println(F("=========== HELP BUZZ ========="));
  Serial.println(F("BUZZ TEST          - test beep buzzer"));
  Serial.println(F("BUZZ STOP          - mute buzzer and stop alarm"));
  Serial.println(F("BUZZ TEMP BAND X   - preview buzzer pattern for band X"));
  Serial.println(F("Temperature bands: 0 to 9"));
  Serial.println(F("Test length is TESTDUR seconds (ADJ TESTDUR)."));
  Serial.println(F("LGD keyword inside command line forces logging."));
  Serial.println(F("================================"));
}

void showHelpServo()
{
  Serial.println(F("=========== HELP SERVO ========"));
  Serial.println(F("SERVO TEMP BAND X  - preview servo behavior for band X"));
  Serial.println(F("Bands at or above fire band move servo to Released."));
  Serial.println(F("Test length is TESTDUR seconds (ADJ TESTDUR)."));
  Serial.println(F("SERVO TEST         - sweep servo between Closed and Released."));
  Serial.println(F("================================"));
}

void showHelpRgb()
{
  Serial.println(F("============ HELP RGB ========="));
  Serial.println(F("RGB TEMP BAND X    - preview RGB status color for band X"));
  Serial.println(F("RGB TEST FLASH     - fast blink test"));
  Serial.println(F("RGB TEST PULSE     - slow breathing test"));
  Serial.println(F("RGB TEST RAMP      - color cycling test"));
  Serial.println(F("RGB TEST BRIGHT N  - brightness test level N"));
  Serial.println(F("RGB OFF            - returns RGB to normal state"));
  Serial.println(F("Test length is TESTDUR seconds (ADJ TESTDUR)."));
  Serial.println(F("================================"));
}

void showHelpDisplay()
{
  Serial.println(F("========== HELP DISPLAY ======="));
  Serial.println(F("DISPLAY OFF            - turn display off"));
  Serial.println(F("DISPLAY ON             - restore band display"));
  Serial.println(F("DISPLAY DIGIT N [s]    - show digit N for optional seconds"));
  Serial.println(F("DISPLAY CHAR C [s]     - show character C for optional seconds"));
  Serial.println(F("DISPLAY BAND [s]       - show current band symbol"));
  Serial.println(F("DISPLAY TEMPBAND [s]   - same as DISPLAY BAND"));
  Serial.println(F("DISPLAY CLEAROVR       - clear overlay immediately"));
  Serial.println(F("DISPLAY STATE          - print display state"));
  Serial.println(F("TESTSEG                - segment test animation"));
  Serial.println(F("COUNTUP                - count from 0 to 9"));
  Serial.println(F("COUNTDOWN              - count from 9 to 0"));
  Serial.println(F("TESTALL                - full display test sequence"));
  Serial.println(F("STOP TEST, STOP DEMO   - stop running tests"));
  Serial.println(F("================================"));
}

void showHelpAdj()
{
  Serial.println(F("============ HELP ADJ ========="));
  Serial.println(F("ADJ NAME          - describe setting"));
  Serial.println(F("ADJ NAME X        - set setting to X"));
  Serial.println(F("Key entries:"));
  Serial.println(F("TIMEAUTO          - temp threshold for auto time sync"));
  Serial.println(F("FIRESTART         - fire entry temperature"));
  Serial.println(F("FIRECLR           - fire exit temperature"));
  Serial.println(F("ROTCDELTA         - rapid rise required jump"));
  Serial.println(F("CLICKGAP          - gap for grouping button clicks"));
  Serial.println(F("LONGLONG          - hold time for long press"));
  Serial.println(F("IDLESLEEP         - seconds without activity before sleep"));
  Serial.println(F("SEGOVR            - overlay duration in milliseconds"));
  Serial.println(F("BUZZLO            - base period low fire"));
  Serial.println(F("BUZZMED           - base period medium fire"));
  Serial.println(F("BUZZHI            - base period high fire"));
  Serial.println(F("NOISELEVEL        - simulation noise strength"));
  Serial.println(F("ADD NOISE EFFECT  - raise noise level"));
  Serial.println(F("LOWER NOISE EFFECT- lower noise level"));
  Serial.println(F("TESTDUR           - seconds for TEMP BAND tests"));
  Serial.println(F("RGBLEVEL          - RGB brightness scale"));
  Serial.println(F("SIMMAX            - maximum simulation temperature"));
  Serial.println(F("SWPRINT           - extra stopwatch prints on or off"));
  Serial.println(F("DHTPERIOD         - milliseconds between sensor reads"));
  Serial.println(F("BANDLOG           - on or off for band events"));
  Serial.println(F("LOGLIMIT          - default limit for log printing"));
  Serial.println(F("================================"));
}

void showHelpWifi()
{
  Serial.println(F("============ HELP WIFI ========"));
  Serial.println(F("TIMEAUTO threshold starts auto time sync"));
  Serial.println(F("Device uses stored SSID and password from Part 1 setup."));
  Serial.println(F("HELP SSID shows instructions for entering WiFi credentials."));
  Serial.println(F("================================"));
}

void showHelpSsid()
{
  Serial.println(F("============ HELP SSID ========"));
  Serial.println(F("WiFi credentials are stored in flash."));
  Serial.println(F("Use dedicated setup sketch or code section to change them."));
  Serial.println(F("Once stored, device uses them for time sync and logging."));
  Serial.println(F("================================"));
}

void showHelpTime()
{
  Serial.println(F("============ HELP TIME ========"));
  Serial.println(F("Device keeps a real time clock starting from WiFi sync."));
  Serial.println(F("TIME SYNC       - request WiFi time sync now"));
  Serial.println(F("TIME STATUS     - show whether time is synced"));
  Serial.println(F("TIME PRINT      - show current date and time"));
  Serial.println(F("TIME AUTOINFO   - describe TIMEAUTO behavior"));
  Serial.println(F("================================"));
}

void showHelpEeprom()
{
  Serial.println(F("=========== HELP EEPROM ======="));
  Serial.println(F("Quad click clears stats and event log."));
  Serial.println(F("Extra EEPROM tools to be added."));
  Serial.println(F("Stats and events live in dedicated sections inside EEPROM."));
  Serial.println(F("================================"));
}

// ------------------------- ADJ Implementation --------------------------

// Configuration variables from Part 1:
// float cfg_TIMEAUTO, cfg_FIRESTART, cfg_FIRECLR, cfg_ROTCDELTA;
// unsigned long cfg_CLICKGAP, cfg_LONGLONG, cfg_IDLESLEEP, cfg_SEGOVR;
// unsigned long cfg_BUZZLO, cfg_BUZZMED, cfg_BUZZHI;
// float cfg_NOISELEVEL, cfg_RGBLEVEL, cfg_SIMMAX;
// unsigned long cfg_TESTDUR, cfg_DHTPERIOD;
// bool cfg_SWPRINT, cfg_BANDLOG;
// uint16_t cfg_LOGLIMIT;

void describeAdjHeader(const char *name)
{
  Serial.print(F("[ADJ "));
  Serial.print(name);
  Serial.println(F("]"));
}

void printAdjFloat(const char *label, float defv, float minv, float maxv, float curv, const char *store)
{
  Serial.print(F("Effect: "));
  Serial.println(label);

  Serial.print(F("Default: "));
  Serial.println(defv, 1);

  Serial.print(F("Range: "));
  Serial.print(minv, 1);
  Serial.print(F(" to "));
  Serial.println(maxv, 1);

  Serial.print(F("Stored in: "));
  Serial.println(store);

  Serial.print(F("Current: "));
  Serial.println(curv, 1);
}

void printAdjULong(const char *label, unsigned long defv, unsigned long minv, unsigned long maxv, unsigned long curv, const char *store)
{
  Serial.print(F("Effect: "));
  Serial.println(label);

  Serial.print(F("Default: "));
  Serial.println(defv);

  Serial.print(F("Range: "));
  Serial.print(minv);
  Serial.print(F(" to "));
  Serial.println(maxv);

  Serial.print(F("Stored in: "));
  Serial.println(store);

  Serial.print(F("Current: "));
  Serial.println(curv);
}

void printAdjBool(const char *label, bool defv, const char *store, bool curv)
{
  Serial.print(F("Effect: "));
  Serial.println(label);

  Serial.print(F("Default: "));
  Serial.println(defv ? F("true") : F("false"));

  Serial.print(F("Stored in: "));
  Serial.println(store);

  Serial.print(F("Current: "));
  Serial.println(curv ? F("true") : F("false"));
}

void printAdjUInt16(const char *label, uint16_t defv, uint16_t minv, uint16_t maxv, uint16_t curv, const char *store)
{
  Serial.print(F("Effect: "));
  Serial.println(label);

  Serial.print(F("Default: "));
  Serial.println(defv);

  Serial.print(F("Range: "));
  Serial.print(minv);
  Serial.print(F(" to "));
  Serial.println(maxv);

  Serial.print(F("Stored in: "));
  Serial.println(store);

  Serial.print(F("Current: "));
  Serial.println(curv);
}

void handleAdjTimeAuto(const String &value)
{
  describeAdjHeader("TIMEAUTO");
  if (value.length() == 0)
  {
    printAdjFloat(
      "Temperature threshold for automatic WiFi time sync when time is unsynced.",
      33.0f, 20.0f, 60.0f, cfg_TIMEAUTO, "flash"
    );
    return;
  }

  float v;
  if (!parseFloatSafe(value, v))
  {
    Serial.println(F("[ADJ TIMEAUTO] Invalid number."));
    return;
  }

  if (v < 20.0f || v > 60.0f)
  {
    Serial.println(F("[ADJ TIMEAUTO] Out of range 20.0 to 60.0."));
    return;
  }

  float old = cfg_TIMEAUTO;
  cfg_TIMEAUTO = v;

  Serial.print(F("[ADJ TIMEAUTO] Set to "));
  Serial.print(cfg_TIMEAUTO, 1);
  Serial.print(F(" C (was "));
  Serial.print(old, 1);
  Serial.println(F(" C)."));
}

void handleAdjFireStart(const String &value)
{
  describeAdjHeader("FIRESTART");
  if (value.length() == 0)
  {
    printAdjFloat(
      "Temperature to enter fire range.",
      35.0f, 20.0f, 80.0f, cfg_FIRESTART, "flash"
    );
    return;
  }

  float v;
  if (!parseFloatSafe(value, v))
  {
    Serial.println(F("[ADJ FIRESTART] Invalid number."));
    return;
  }

  if (v < 20.0f || v > 80.0f)
  {
    Serial.println(F("[ADJ FIRESTART] Out of range 20.0 to 80.0."));
    return;
  }

  float old = cfg_FIRESTART;
  cfg_FIRESTART = v;

  Serial.print(F("[ADJ FIRESTART] Set to "));
  Serial.print(cfg_FIRESTART, 1);
  Serial.print(F(" C (was "));
  Serial.print(old, 1);
  Serial.println(F(" C)."));
}

void handleAdjFireClr(const String &value)
{
  describeAdjHeader("FIRECLR");
  if (value.length() == 0)
  {
    printAdjFloat(
      "Temperature to exit fire range.",
      34.9f, 20.0f, 80.0f, cfg_FIRECLR, "flash"
    );
    return;
  }

  float v;
  if (!parseFloatSafe(value, v))
  {
    Serial.println(F("[ADJ FIRECLR] Invalid number."));
    return;
  }

  if (v < 20.0f || v > 80.0f)
  {
    Serial.println(F("[ADJ FIRECLR] Out of range 20.0 to 80.0."));
    return;
  }

  float old = cfg_FIRECLR;
  cfg_FIRECLR = v;

  Serial.print(F("[ADJ FIRECLR] Set to "));
  Serial.print(cfg_FIRECLR, 1);
  Serial.print(F(" C (was "));
  Serial.print(old, 1);
  Serial.println(F(" C)."));
}

void handleAdjRotcDelta(const String &value)
{
  describeAdjHeader("ROTCDELTA");
  if (value.length() == 0)
  {
    printAdjFloat(
      "Required jump in temperature for rapid rise detection.",
      1.0f, 0.2f, 20.0f, cfg_ROTCDELTA, "flash"
    );
    return;
  }

  float v;
  if (!parseFloatSafe(value, v))
  {
    Serial.println(F("[ADJ ROTCDELTA] Invalid number."));
    return;
  }

  if (v < 0.2f || v > 20.0f)
  {
    Serial.println(F("[ADJ ROTCDELTA] Out of range 0.2 to 20.0."));
    return;
  }

  float old = cfg_ROTCDELTA;
  cfg_ROTCDELTA = v;

  Serial.print(F("[ADJ ROTCDELTA] Set to "));
  Serial.print(cfg_ROTCDELTA, 1);
  Serial.print(F(" C (was "));
  Serial.print(old, 1);
  Serial.println(F(" C)."));
}

void handleAdjClickGap(const String &value)
{
  describeAdjHeader("CLICKGAP");
  if (value.length() == 0)
  {
    printAdjULong(
      "Time gap in ms for grouping button clicks into one pattern.",
      500, 100, 2000, cfg_CLICKGAP, "flash"
    );
    return;
  }

  long v;
  if (!parseIntSafe(value, v))
  {
    Serial.println(F("[ADJ CLICKGAP] Invalid number."));
    return;
  }

  if (v < 100 || v > 2000)
  {
    Serial.println(F("[ADJ CLICKGAP] Out of range 100 to 2000."));
    return;
  }

  unsigned long old = cfg_CLICKGAP;
  cfg_CLICKGAP = (unsigned long)v;

  Serial.print(F("[ADJ CLICKGAP] Set to "));
  Serial.print(cfg_CLICKGAP);
  Serial.print(F(" ms (was "));
  Serial.print(old);
  Serial.println(F(" ms)."));
}

void handleAdjLongLong(const String &value)
{
  describeAdjHeader("LONGLONG");
  if (value.length() == 0)
  {
    printAdjULong(
      "Hold time in ms required for a long press.",
      2000, 500, 10000, cfg_LONGLONG, "flash"
    );
    return;
  }

  long v;
  if (!parseIntSafe(value, v))
  {
    Serial.println(F("[ADJ LONGLONG] Invalid number."));
    return;
  }

  if (v < 500 || v > 10000)
  {
    Serial.println(F("[ADJ LONGLONG] Out of range 500 to 10000."));
    return;
  }

  unsigned long old = cfg_LONGLONG;
  cfg_LONGLONG = (unsigned long)v;

  Serial.print(F("[ADJ LONGLONG] Set to "));
  Serial.print(cfg_LONGLONG);
  Serial.print(F(" ms (was "));
  Serial.print(old);
  Serial.println(F(" ms)."));
}

void handleAdjIdleSleep(const String &value)
{
  describeAdjHeader("IDLESLEEP");
  if (value.length() == 0)
  {
    printAdjULong(
    "Seconds of inactivity required before display and RGB sleep.", 600, 10, 3600, cfg_IDLESLEEP_SEC, "flash"
    );
    return;
  }

  long v;
  if (!parseIntSafe(value, v))
  {
    Serial.println(F("[ADJ IDLESLEEP] Invalid number."));
    return;
  }

  if (v < 5 || v > 3600.9)
  {
    Serial.println(F("[ADJ IDLESLEEP] Out of range 5 to 3600."));
    return;
  }

  unsigned long old = cfg_IDLESLEEP_SEC;
  cfg_IDLESLEEP_SEC = (unsigned long)v;

  Serial.print(F("[ADJ IDLESLEEP] Set to "));
  Serial.print(cfg_IDLESLEEP_SEC);
  Serial.print(F("s (was "));
  Serial.print(old);
  Serial.println(F("s)."));
}

void handleAdjSegOvr(const String &value)
{
  describeAdjHeader("SEGOVR");
  if (value.length() == 0)
  {
    printAdjULong(
      "Duration in ms for short overlay messages on 7-seg display.",
      3000, 500, 20000, cfg_SEGOVR, "flash"
    );
    return;
  }

  long v;
  if (!parseIntSafe(value, v))
  {
    Serial.println(F("[ADJ SEGOVR] Invalid number."));
    return;
  }

  if (v < 500 || v > 20000)
  {
    Serial.println(F("[ADJ SEGOVR] Out of range. (500ms to 20000ms)"));
    return;
  }

  unsigned long old = cfg_SEGOVR;
  cfg_SEGOVR = (unsigned long)v;

  Serial.print(F("[ADJ SEGOVR] Set to "));
  Serial.print(cfg_SEGOVR);
  Serial.print(F(" ms (was "));
  Serial.print(old);
  Serial.println(F(" ms)."));
}

void handleAdjBuzzLow(const String &value)
{
  describeAdjHeader("BUZZLO");
  if (value.length() == 0)
  {
    printAdjULong(
      "Base period in ms for low fire and rapid rise buzzer pattern.",
      800, 100, 5000, cfg_BUZZLO, "flash"
    );
    return;
  }

  long v;
  if (!parseIntSafe(value, v))
  {
    Serial.println(F("[ADJ BUZZLO] Invalid number."));
    return;
  }

  if (v < 100 || v > 5000)
  {
    Serial.println(F("[ADJ BUZZLO] Out of range 100 to 5000."));
    return;
  }

  unsigned long old = cfg_BUZZLO;
  cfg_BUZZLO = (unsigned long)v;

  Serial.print(F("[ADJ BUZZLO] Set to "));
  Serial.print(cfg_BUZZLO);
  Serial.print(F(" ms (was "));
  Serial.print(old);
  Serial.println(F(" ms)."));
}

void handleAdjBuzzMed(const String &value)
{
  describeAdjHeader("BUZZMED");
  if (value.length() == 0)
  {
    printAdjULong(
      "Base period in ms for medium fire buzzer pattern.",
      500, 100, 5000, cfg_BUZZMED, "flash"
    );
    return;
  }

  long v;
  if (!parseIntSafe(value, v))
  {
    Serial.println(F("[ADJ BUZZMED] Invalid number."));
    return;
  }

  if (v < 100 || v > 5000)
  {
    Serial.println(F("[ADJ BUZZMED] Out of range 100 to 5000."));
    return;
  }

  unsigned long old = cfg_BUZZMED;
  cfg_BUZZMED = (unsigned long)v;

  Serial.print(F("[ADJ BUZZMED] Set to "));
  Serial.print(cfg_BUZZMED);
  Serial.print(F(" ms (was "));
  Serial.print(old);
  Serial.println(F(" ms)."));
}

void handleAdjBuzzHi(const String &value)
{
  describeAdjHeader("BUZZHI");
  if (value.length() == 0)
  {
    printAdjULong(
      "Base period in ms for high fire buzzer pattern.",
      250, 80, 5000, cfg_BUZZHI, "flash"
    );
    return;
  }

  long v;
  if (!parseIntSafe(value, v))
  {
    Serial.println(F("[ADJ BUZZHI] Invalid number."));
    return;
  }

  if (v < 80 || v > 5000)
  {
    Serial.println(F("[ADJ BUZZHI] Out of range 80 to 5000."));
    return;
  }

  unsigned long old = cfg_BUZZHI;
  cfg_BUZZHI = (unsigned long)v;

  Serial.print(F("[ADJ BUZZHI] Set to "));
  Serial.print(cfg_BUZZHI);
  Serial.print(F(" ms (was "));
  Serial.print(old);
  Serial.println(F(" ms)."));
}

void handleAdjNoiseLevel(const String &value)
{
  describeAdjHeader("NOISELEVEL");
  if (value.length() == 0)
  {
    printAdjFloat(
      "Strength of simulation noise around simulated temperature.",
      0.5f, 0.0f, 20.0f, cfg_NOISELEVEL, "flash"
    );
    return;
  }

  float v;
  if (!parseFloatSafe(value, v))
  {
    Serial.println(F("[ADJ NOISELEVEL] Invalid number."));
    return;
  }

  if (v < 0.0f || v > 20.0f)
  {
    Serial.println(F("[ADJ NOISELEVEL] Out of range 0.0 to 20.0."));
    return;
  }

  float old = cfg_NOISELEVEL;
  cfg_NOISELEVEL = v;

  Serial.print(F("[ADJ NOISELEVEL] Set to "));
  Serial.print(cfg_NOISELEVEL, 2);
  Serial.print(F(" (was "));
  Serial.print(old, 2);
  Serial.println(F(")."));
}

void handleAdjAddNoiseEffect()
{
  describeAdjHeader("ADD NOISE EFFECT");
  float step = 0.5f;
  float old  = cfg_NOISELEVEL;
  cfg_NOISELEVEL += step;
  if (cfg_NOISELEVEL > 20.0f)
  {
    cfg_NOISELEVEL = 20.0f;
  }

  Serial.print(F("[ADJ ADD NOISE EFFECT] NOISELEVEL changed from "));
  Serial.print(old, 2);
  Serial.print(F(" to "));
  Serial.println(cfg_NOISELEVEL, 2);
}

void handleAdjLowerNoiseEffect()
{
  describeAdjHeader("LOWER NOISE EFFECT");
  float step = 0.5f;
  float old  = cfg_NOISELEVEL;
  if (cfg_NOISELEVEL >= step)
  {
    cfg_NOISELEVEL -= step;
  }
  else
  {
    cfg_NOISELEVEL = 0.0f;
  }

  Serial.print(F("[ADJ LOWER NOISE EFFECT] NOISELEVEL changed from "));
  Serial.print(old, 2);
  Serial.print(F(" to "));
  Serial.println(cfg_NOISELEVEL, 2);
}

void handleAdjTestDur(const String &value)
{
  describeAdjHeader("TESTDUR");
  if (value.length() == 0)
  {
    printAdjULong(
      "Default duration in seconds for TEMP BAND tests.",
      3, 1, 300, cfg_TESTDUR, "flash"
    );
    return;
  }

  long v;
  if (!parseIntSafe(value, v))
  {
    Serial.println(F("[ADJ TESTDUR] Invalid number."));
    return;
  }

  if (v < 1 || v > 300)
  {
    Serial.println(F("[ADJ TESTDUR] Out of range 1 to 300."));
    return;
  }

  unsigned long old = cfg_TESTDUR;
  cfg_TESTDUR = (unsigned long)v;

  Serial.print(F("[ADJ TESTDUR] Set to "));
  Serial.print(cfg_TESTDUR);
  Serial.print(F(" s (was "));
  Serial.print(old);
  Serial.println(F(" s)."));
}

void handleAdjRgbLevel(const String &value)
{
  describeAdjHeader("RGBLEVEL");
  if (value.length() == 0)
  {
    printAdjFloat(
      "Brightness scale for RGB LED from 0.0 to 1.0.",
      0.4f, 0.0f, 1.0f, cfg_RGBLEVEL, "flash"
    );
    return;
  }

  float v;
  if (!parseFloatSafe(value, v))
  {
    Serial.println(F("[ADJ RGBLEVEL] Invalid number."));
    return;
  }

  if (v < 0.0f || v > 1.0f)
  {
    Serial.println(F("[ADJ RGBLEVEL] Out of range 0.0 to 1.0."));
    return;
  }

  float old = cfg_RGBLEVEL;
  cfg_RGBLEVEL = v;

  Serial.print(F("[ADJ RGBLEVEL] Set to "));
  Serial.print(cfg_RGBLEVEL, 2);
  Serial.print(F(" (was "));
  Serial.print(old, 2);
  Serial.println(F(")."));
}

void handleAdjSimMax(const String &value)
{
  describeAdjHeader("SIMMAX");
  if (value.length() == 0)
  {
    printAdjFloat(
      "Maximum temperature allowed for simulation.",
      120.0f, 20.0f, 300.0f, cfg_SIMMAX, "flash"
    );
    return;
  }

  float v;
  if (!parseFloatSafe(value, v))
  {
    Serial.println(F("[ADJ SIMMAX] Invalid number."));
    return;
  }

  if (v < 20.0f || v > 300.0f)
  {
    Serial.println(F("[ADJ SIMMAX] Out of range 20.0 to 300.0."));
    return;
  }

  float old = cfg_SIMMAX;
  cfg_SIMMAX = v;

  Serial.print(F("[ADJ SIMMAX] Set to "));
  Serial.print(cfg_SIMMAX, 1);
  Serial.print(F(" C (was "));
  Serial.print(old, 1);
  Serial.println(F(" C)."));
}

void handleAdjSwPrint(const String &value)
{
  describeAdjHeader("SWPRINT");

  if (value.length() == 0)
  {
    printAdjBool(
      "Enable periodic status line prints (every 2s when ON).",
      true, "flash", (cfg_SWPRINT != 0)
    );
    return;
  }

  String up = toUpperCopy(value);
  if (up != "ON" && up != "OFF" && up != "1" && up != "0" && up != "TRUE" && up != "FALSE")
  {
    Serial.println(F("[ADJ SWPRINT] Use ON or OFF."));
    return;
  }

  bool newVal = (up == "ON" || up == "1" || up == "TRUE");
  bool oldVal = (cfg_SWPRINT != 0);
  cfg_SWPRINT = newVal ? 2000UL : 0UL;

  Serial.print(F("[ADJ SWPRINT] Set to "));
  Serial.print(newVal ? F("ON") : F("OFF"));
  Serial.print(F(" (was "));
  Serial.print(oldVal ? F("ON") : F("OFF"));
  Serial.println(F(")."));
}

void handleAdjDhtPeriod(const String &value)
{
  describeAdjHeader("DHTPERIOD");
  if (value.length() == 0)
  {
    printAdjULong(
      "Milliseconds between DHT sensor readings.",
      2000, 500, 20000, cfg_DHTPERIOD, "flash"
    );
    return;
  }

  long v;
  if (!parseIntSafe(value, v))
  {
    Serial.println(F("[ADJ DHTPERIOD] Invalid number."));
    return;
  }

  if (v < 500 || v > 20000)
  {
    Serial.println(F("[ADJ DHTPERIOD] Out of range 500 to 20000."));
    return;
  }

  unsigned long old = cfg_DHTPERIOD;
  cfg_DHTPERIOD = (unsigned long)v;

  Serial.print(F("[ADJ DHTPERIOD] Set to "));
  Serial.print(cfg_DHTPERIOD);
  Serial.print(F(" ms (was "));
  Serial.print(old);
  Serial.println(F(" ms)."));
}

void handleAdjBandLog(const String &value)
{
  describeAdjHeader("BANDLOG");
  if (value.length() == 0)
  {
    printAdjBool(
      "Enable or disable logging of temperature band change events.",
      true, "flash", cfg_BANDLOG
    );
    return;
  }

  String up = toUpperCopy(value);

  bool newVal;
  if (up == "ON" || up == "1" || up == "TRUE")
  {
    newVal = true;
  }
  else if (up == "OFF" || up == "0" || up == "FALSE")
  {
    newVal = false;
  }
  else
  {
    Serial.println(F("[ADJ BANDLOG] Use ON or OFF."));
    return;
  }

  bool old = cfg_BANDLOG;
  cfg_BANDLOG = newVal;

  Serial.print(F("[ADJ BANDLOG] Set to "));
  Serial.print(cfg_BANDLOG ? F("ON") : F("OFF"));
  Serial.print(F(" (was "));
  Serial.print(old ? F("ON") : F("OFF"));
  Serial.println(F(")."));
}

void handleAdjLogLimit(const String &value)
{
  describeAdjHeader("LOGLIMIT");
  if (value.length() == 0)
  {
    printAdjUInt16(
      "Default limit for partial log dumps.",
      100, 0, 1000, cfg_LOGLIMIT, "flash"
    );
    return;
  }

  long v;
  if (!parseIntSafe(value, v))
  {
    Serial.println(F("[ADJ LOGLIMIT] Invalid number."));
    return;
  }

  if (v < 0 || v > 1000)
  {
    Serial.println(F("[ADJ LOGLIMIT] Out of range 0 to 1000."));
    return;
  }

  uint16_t old = cfg_LOGLIMIT;
  cfg_LOGLIMIT = (uint16_t)v;

  Serial.print(F("[ADJ LOGLIMIT] Set to "));
  Serial.print(cfg_LOGLIMIT);
  Serial.print(F(" (was "));
  Serial.print(old);
  Serial.println(F(")."));
}

// Add this prototype near the other ADJ prototypes (top of ADJ section)
void handleAdjServoPos(const String &rest);

void handleAdjServoPos(const String &rest)
{
  // Format:
  // ADJ SERVO POS CLOSED 180
  // ADJ SERVO POS RELEASED 90
  // ADJ SERVO POS CLOSED
  // ADJ SERVO POS RELEASED

  String w1, r1;
  splitFirstWord(rest, w1, r1);
  String up1 = toUpperCopy(w1);

  if (up1 != "POS")
  {
    Serial.println(F("[ADJ SERVO] Use: ADJ SERVO POS CLOSED <deg> or ADJ SERVO POS RELEASED <deg>"));
    return;
  }

  String w2, r2;
  splitFirstWord(r1, w2, r2);
  String up2 = toUpperCopy(w2);

  if (up2 != "CLOSED" && up2 != "RELEASED")
  {
    Serial.println(F("[ADJ SERVO] Use CLOSED or RELEASED."));
    return;
  }

  // Describe mode
  if (r2.length() == 0)
  {
    if (up2 == "CLOSED")
    {
      Serial.println(F("[ADJ SERVO POS CLOSED]"));
      Serial.println(F("Effect: Servo target angle used for the closed state."));
      Serial.println(F("Default: 52"));
      Serial.println(F("Range: 0 to 180"));
      Serial.println(F("Stored in: flash"));
      Serial.print(F("Current: "));
      Serial.println(cfg_SERVO_POS_CLOSED);
      return;
    }
    else
    {
      Serial.println(F("[ADJ SERVO POS RELEASED]"));
      Serial.println(F("Effect: Servo target angle used for the released state."));
      Serial.println(F("Default: 152"));
      Serial.println(F("Range: 0 to 180"));
      Serial.println(F("Stored in: flash"));
      Serial.print(F("Current: "));
      Serial.println(cfg_SERVO_POS_RELEASED);
      return;
    }
  }

  long degL;
  if (!parseIntSafe(r2, degL))
  {
    Serial.println(F("[ADJ SERVO] Invalid number."));
    return;
  }

  if (degL < 0 || degL > 180)
  {
    Serial.println(F("[ADJ SERVO] Out of range 0 to 180."));
    return;
  }

  int deg = (int)degL;

  // Reject equal endpoints
  if (up2 == "CLOSED" && deg == cfg_SERVO_POS_RELEASED)
  {
    Serial.println(F("[ADJ SERVO POS CLOSED] Refused. CLOSED equals RELEASED."));
    return;
  }
  if (up2 == "RELEASED" && deg == cfg_SERVO_POS_CLOSED)
  {
    Serial.println(F("[ADJ SERVO POS RELEASED] Refused. The servo will not deploy if RELEASED = CLOSED."));
    return;
  }

  recordActivity();

  if (up2 == "CLOSED")
  {
    int old = cfg_SERVO_POS_CLOSED;
    cfg_SERVO_POS_CLOSED = deg;

    Serial.print(F("[ADJ SERVO POS CLOSED] Set to "));
    Serial.print(cfg_SERVO_POS_CLOSED);
    Serial.print(F(" deg (was "));
    Serial.print(old);
    Serial.println(F(" deg)."));

    // Keep target consistent if target was using the old endpoint
    if (targetServoPos == old) targetServoPos = cfg_SERVO_POS_CLOSED;
    return;
  }

  // RELEASED
  int old = cfg_SERVO_POS_RELEASED;
  cfg_SERVO_POS_RELEASED = deg;

  Serial.print(F("[ADJ SERVO POS RELEASED] Set to "));
  Serial.print(cfg_SERVO_POS_RELEASED);
  Serial.print(F(" deg (was "));
  Serial.print(old);
  Serial.println(F(" deg)."));

  if (targetServoPos == old) targetServoPos = cfg_SERVO_POS_RELEASED;
}

void handleAdjCommand(const String &args)
{
  String name;
  String rest;
  splitFirstWord(args, name, rest);
  String upName = toUpperCopy(name);

  if (upName.length() == 0)
  {
    showHelpAdj();
    return;
  }

  if (upName == "SERVO")
  {
    handleAdjServoPos(rest);
  }
  else if (upName == "TIMEAUTO")
  {
    handleAdjTimeAuto(rest);
  }
  else if (upName == "FIRESTART")
  {
    handleAdjFireStart(rest);
  }
  else if (upName == "FIRECLR")
  {
    handleAdjFireClr(rest);
  }
  else if (upName == "ROTCDELTA")
  {
    handleAdjRotcDelta(rest);
  }
  else if (upName == "CLICKGAP")
  {
    handleAdjClickGap(rest);
  }
  else if (upName == "LONGLONG")
  {
    handleAdjLongLong(rest);
  }
  else if (upName == "IDLESLEEP")
  {
    handleAdjIdleSleep(rest);
  }
  else if (upName == "SEGOVR")
  {
    handleAdjSegOvr(rest);
  }
  else if (upName == "BUZZLO")
  {
    handleAdjBuzzLow(rest);
  }
  else if (upName == "BUZZMED")
  {
    handleAdjBuzzMed(rest);
  }
  else if (upName == "BUZZHI")
  {
    handleAdjBuzzHi(rest);
  }
  else if (upName == "NOISELEVEL")
  {
    handleAdjNoiseLevel(rest);
  }
  else if (upName == "ADD")
  {
    String second;
    String remaining;
    splitFirstWord(rest, second, remaining);
    String upSecond = toUpperCopy(second);
    if (upSecond == "NOISE")
    {
      String third;
      String last;
      splitFirstWord(remaining, third, last);
      String upThird = toUpperCopy(third);
      if (upThird == "EFFECT") handleAdjAddNoiseEffect();
      else Serial.println(F("[ADJ ADD] Unknown subcommand. Use ADJ ADD NOISE EFFECT."));
    }
    else Serial.println(F("[ADJ ADD] Unknown subcommand."));
  }
  else if (upName == "LOWER")
  {
    String second;
    String remaining;
    splitFirstWord(rest, second, remaining);
    String upSecond = toUpperCopy(second);
    if (upSecond == "NOISE")
    {
      String third;
      String last;
      splitFirstWord(remaining, third, last);
      String upThird = toUpperCopy(third);
      if (upThird == "EFFECT") handleAdjLowerNoiseEffect();
      else Serial.println(F("[ADJ LOWER] Unknown subcommand. Use ADJ LOWER NOISE EFFECT."));
    }
    else Serial.println(F("[ADJ LOWER] Unknown subcommand."));
  }
  else if (upName == "TESTDUR")
  {
    handleAdjTestDur(rest);
  }
  else if (upName == "RGBLEVEL")
  {
    handleAdjRgbLevel(rest);
  }
  else if (upName == "SIMMAX")
  {
    handleAdjSimMax(rest);
  }
  else if (upName == "SWPRINT")
  {
    handleAdjSwPrint(rest);
  }
  else if (upName == "DHTPERIOD")
  {
    handleAdjDhtPeriod(rest);
  }
  else if (upName == "BANDLOG")
  {
    handleAdjBandLog(rest);
  }
  else if (upName == "LOGLIMIT")
  {
    handleAdjLogLimit(rest);
  }
  else
  {
    Serial.print(F("[ADJ] Unknown setting: "));
    Serial.println(upName);
  }
}

// ------------------------- SIM and TEST Commands -----------------------

static float urand01()
{
  long r = random(0, 10001); // 0..10000
  return (float)r / 10000.0f;
}

// Triangular distribution: min=a, peak=mode, max=b
static float frandTriangular(float a, float mode, float b)
{
  float u = urand01();
  float c = (mode - a) / (b - a);

  if (u < c)
  {
    return a + sqrtf(u * (b - a) * (mode - a));
  }
  return b - sqrtf((1.0f - u) * (b - a) * (b - mode));
}

// roomStart picker | Range 20.0..33.0, biased toward 30.5, one decimal place.
static float pickRoomStart()
{
  const float MINV  = 20.0f;
  const float MODEV = 30.5f;
  const float MAXV  = 33.0f;

  float x = frandTriangular(MINV, MODEV, MAXV);

  // Force 1 decimal place
  int32_t x10 = (int32_t)lroundf(x * 10.0f);
  if (x10 < (int32_t)lroundf(MINV * 10.0f)) x10 = (int32_t)lroundf(MINV * 10.0f);
  if (x10 > (int32_t)lroundf(MAXV * 10.0f)) x10 = (int32_t)lroundf(MAXV * 10.0f);

  return x10 / 10.0f;
}

void handleSimCommand(const String &args, bool lgd)
{
  String a  = trimSpaces(args);
  String up = toUpperCopy(a);

  // -------------------- OFF / STOP --------------------
  if (up == "OFF" || up == "STOP")
  {
    if (simActive)
    {
      simActive = false;
      simPhase  = SIMPHASE_INIT;

      logSim(SIM_STOP, NoisySimTemp);
      Serial.println(F("[SIM] Simulation stopped."));
    }
    else
    {
      Serial.println(F("[SIM] Already off dumbass could you not use your eyes??"));
    }

    // Reset state for next session (keep a sane idle baseline)
    targetSimTemp = 30.5f;
    BaseSimTemp   = 30.5f;
    NoisySimTemp  = 30.5f;
    noisyVel      = 0.0f;

    simStartMS    = 0;
    simLastStepMS = 0;

    holdEndMS            = 0;
    slowDelayArmed       = false;
    slowDelayEndMS       = 0;
    belowClrTimerRunning = false;
    belowClrStartMS      = 0;

    return;
  }

  // -------------------- Parse temperature --------------------
  String w1, rest;
  splitFirstWord(a, w1, rest);
  String w2, rest2;
  splitFirstWord(rest, w2, rest2);

  float tempCandidate = NAN;
  float v1, v2;

  bool w1Numeric = parseFloatSafe(w1, v1);
  bool w2Numeric = parseFloatSafe(w2, v2);

  if (w1Numeric && !w2Numeric)      tempCandidate = v1;
  else if (!w1Numeric && w2Numeric) tempCandidate = v2;
  else if (w1Numeric && w2Numeric)  tempCandidate = v1;
  else
  {
    Serial.println(F("[SIM] Use SIM N TEMP or SIM TEMP N or SIM OFF."));
    return;
  }

  if (isnan(tempCandidate))
  {
    Serial.println(F("[SIM] Invalid temperature value."));
    return;
  }

  if (tempCandidate < -40.0f || tempCandidate > cfg_SIMMAX)
  {
    Serial.print(F("[SIM] Temperature "));
    Serial.print(tempCandidate, 1);
    Serial.print(F(" C exceeds safe simulation range (-40.0 to "));
    Serial.print(cfg_SIMMAX, 1);
    Serial.println(F(" C)."));
    return;
  }

  // -------------------- Starting a new session --------------------
  if (!simActive)
  {
    unsigned long nowMS = millis();

    simActive    = true;
    simPhase     = SIMPHASE_INIT;

    // biased random roomStart simtemp each fresh session
    float roomStart = pickRoomStart();

    BaseSimTemp  = roomStart;
    NoisySimTemp = roomStart;
    noisyVel     = 0.0f;

    targetSimTemp = tempCandidate;

    simStartMS    = nowMS;
    simLastStepMS = nowMS;

    holdEndMS            = 0;
    slowDelayArmed       = false;
    slowDelayEndMS       = 0;
    belowClrTimerRunning = false;
    belowClrStartMS      = 0;

    if (BaseSimTemp < targetSimTemp) simPhase = SIMPHASE_RISING;
    else if (BaseSimTemp > targetSimTemp)
      simPhase = (BaseSimTemp <= cfg_FIRECLR) ? SIMPHASE_COOL_SLOW : SIMPHASE_COOL_FAST;
    else
    {
      simPhase  = SIMPHASE_HOLD;
      holdEndMS = nowMS + 3000UL;
    }

    stats.simSessions++;
    saveStatsToEEPROM();

    logSim(SIM_START, BaseSimTemp);
    logSim(SIM_TEMP_SET, targetSimTemp);

    Serial.print(F("[SIM] Simulation started. Start="));
    Serial.print(BaseSimTemp, 1);
    Serial.print(F(" C, Target="));
    Serial.print(targetSimTemp, 1);
    Serial.println(F(" C"));

    return;
  }

  // -------------------- Retarget active session --------------------
  targetSimTemp = tempCandidate;

  holdEndMS            = 0;
  slowDelayArmed       = false;
  slowDelayEndMS       = 0;
  belowClrTimerRunning = false;
  belowClrStartMS      = 0;

  if (BaseSimTemp < targetSimTemp)
  {
    simPhase = SIMPHASE_RISING;
  }
  else if (BaseSimTemp > targetSimTemp)
  {
    simPhase = (BaseSimTemp <= cfg_FIRECLR) ? SIMPHASE_COOL_SLOW : SIMPHASE_COOL_FAST;
  }
  else
  {
    simPhase  = SIMPHASE_HOLD;
    holdEndMS = millis() + 3000UL;
  }

  logSim(SIM_TEMP_SET, targetSimTemp);

  Serial.print(F("[SIM] Target updated. Start="));
  Serial.print(NoisySimTemp, 1);
  Serial.print(F(" C, Target="));
  Serial.print(targetSimTemp, 1);
  Serial.println(F(" C"));
}

void handleBuzzCommand(const String &args, bool lgd)
{
  String name;
  String rest;
  splitFirstWord(args, name, rest);
  String upName = toUpperCopy(name);

  if (upName.length() == 0)
  {
    showHelpBuzz();
    return;
  }

  if (upName == "TEST")
  {
    buzzerTestRequested = true;
    Serial.println(F("[BUZZ] Test requested."));
    return;
  }

  if (upName == "STOP")
  {
    muteBuzzer = true;
    buzzerSet(false);
    Serial.println(F("[BUZZ] Stop requested, mute enabled."));
    return;
  }

  if (upName == "TEMP")
  {
    String w2;
    String rest2;
    splitFirstWord(rest, w2, rest2);
    String up2 = toUpperCopy(w2);

    if (up2 != "BAND")
    {
      Serial.println(F("[BUZZ] Use BUZZ TEMP BAND X."));
      return;
    }

    String w3;
    String rest3;
    splitFirstWord(rest2, w3, rest3);

    long band;
    if (!parseIntSafe(w3, band))
    {
      Serial.println(F("[BUZZ] Band must be integer 0 to 9."));
      return;
    }

    if (band < 0 || band > 9)
    {
      Serial.println(F("[BUZZ] Band must be between 0 and 9."));
      return;
    }

    unsigned long durationMS = cfg_TESTDUR * 1000UL;
    startBuzzerBandTest((int)band, durationMS);
    Serial.print(F("[BUZZ] Temp band test started for band "));
    Serial.print(band);
    Serial.print(F(" for "));
    Serial.print(cfg_TESTDUR);
    Serial.println(F(" seconds."));
    return;
  }

  Serial.println(F("[BUZZ] Unknown subcommand. Use BUZZ TEST, BUZZ STOP, or BUZZ TEMP BAND X."));
}

void handleServoCommand(const String &args, bool lgd)
{
  String name;
  String rest;
  splitFirstWord(args, name, rest);
  String upName = toUpperCopy(name);

  if (upName.length() == 0)
  {
    showHelpServo();
    return;
  }

if (upName == "TEMP")
{
  String w2;
  String rest2;
  splitFirstWord(rest, w2, rest2);
  String up2 = toUpperCopy(w2);

  if (up2 != "BAND")
  {
    Serial.println(F("[SERVO] Use SERVO TEMP BAND X [DUR]."));
    return;
  }

  String w3;
  String rest3;
  splitFirstWord(rest2, w3, rest3);

  long band;
  if (!parseIntSafe(w3, band))
  {
    Serial.println(F("[SERVO] Band must be integer 0 to 9."));
    return;
  }

  if (band < 0 || band > 9)
  {
    Serial.println(F("[SERVO] Band must be between 0 and 9."));
    return;
  }

  // Optional duration in seconds after band
  unsigned long durationSec;
  if (rest3.length() == 0)
  {
    // no duration given → default from TESTDUR
    durationSec = cfg_TESTDUR;
  }
  else
  {
    long dur;
    if (!parseIntSafe(rest3, dur) || dur <= 0)
    {
      Serial.println(F("[SERVO] Duration must be a positive integer number of seconds."));
      return;
    }
    durationSec = (unsigned long)dur;
  }

  unsigned long durationMS = durationSec * 1000UL;
  startServoBandTest((int)band, durationMS);

  Serial.print(F("[SERVO] Temp band test started for band "));
  Serial.print(band);
  Serial.print(F(" for "));
  Serial.print(durationSec);
  Serial.println(F(" seconds."));
  return;
}

  if (upName == "TEST")
  {
    servoTestRequested = true; // assumed flag in Part 4
    Serial.println(F("[SERVO] Test requested."));
    return;
  }

  Serial.println(F("[SERVO] Unknown subcommand. Use SERVO TEMP BAND X or SERVO TEST."));
}

void handleRgbCommand(const String &args, bool lgd)
{
  String name;
  String rest;
  splitFirstWord(args, name, rest);
  String upName = toUpperCopy(name);

  if (upName.length() == 0)
  {
    showHelpRgb();
    return;
  }

  if (upName == "TEMP")
  {
    String w2;
    String rest2;
    splitFirstWord(rest, w2, rest2);
    String up2 = toUpperCopy(w2);

    if (up2 != "BAND")
    {
      Serial.println(F("[RGB] Use RGB TEMP BAND X."));
      return;
    }

    String w3;
    String rest3;
    splitFirstWord(rest2, w3, rest3);

    long band;
    if (!parseIntSafe(w3, band))
    {
      Serial.println(F("[RGB] Band must be integer 0 to 9."));
      return;
    }

    if (band < 0 || band > 9)
    {
      Serial.println(F("[RGB] Band must be between 0 and 9."));
      return;
    }

    unsigned long durationMS = cfg_TESTDUR * 1000UL;
    setRgbTestModeForBand((int)band, durationMS);
    Serial.print(F("[RGB] Temp band test started for band "));
    Serial.print(band);
    Serial.print(F(" for "));
    Serial.print(cfg_TESTDUR);
    Serial.println(F(" seconds."));
    return;
  }

  if (upName == "TEST")
  {
    String modeName;
    String rest2;
    splitFirstWord(rest, modeName, rest2);
    String upMode = toUpperCopy(modeName);

    if (upMode == "FLASH")
    {
      rgbTestFlashRequested = true; // Part 4 flag
      Serial.println(F("[RGB] FLASH test requested."));
    }
    else if (upMode == "PULSE")
    {
      rgbTestPulseRequested = true;
      Serial.println(F("[RGB] PULSE test requested."));
    }
    else if (upMode == "RAMP")
    {
      rgbTestRampRequested = true;
      Serial.println(F("[RGB] RAMP test requested."));
    }
    else if (upMode == "BRIGHT")
    {
      String levelStr;
      String rest3;
      splitFirstWord(rest2, levelStr, rest3);
      long level;
      if (!parseIntSafe(levelStr, level))
      {
        Serial.println(F("[RGB] BRIGHT level must be integer 0 to 100."));
        return;
      }
      if (level < 0) level = 0;
      if (level > 100) level = 100;
      rgbTestBrightLevelRequested = (int)level;
      Serial.print(F("[RGB] BRIGHT test requested level "));
      Serial.println(level);
    }
    else
    {
      Serial.println(F("[RGB] Use RGB TEST FLASH, PULSE, RAMP, or BRIGHT N."));
    }
    return;
  }

  if (upName == "OFF")
  {
    rgbMode = RGBMODE_SAFE;
    Serial.println(F("[RGB] Returned to normal state."));
    return;
  }

  Serial.println(F("[RGB] Unknown subcommand. Use RGB TEMP BAND X or RGB TEST <mode>."));
}

void handleDisplayCommand(const String &args)
{
  String name;
  String rest;
  splitFirstWord(args, name, rest);
  String upName = toUpperCopy(name);

  if (upName.length() == 0)
  {
    showHelpDisplay();
    return;
  }

  if (upName == "OFF" || upName == "ALLOFF")
  {
    displaySleep = true;
    sevenSegAllOff();
    Serial.println(F("[DISPLAY] Turned off."));
    return;
  }

  if (upName == "ON")
  {
    displaySleep = false;
    Serial.println(F("[DISPLAY] Turned on."));
    return;
  }

  if (upName == "DIGIT")
  {
    String digitStr;
    String rest2;
    splitFirstWord(rest, digitStr, rest2);

    long d;
    if (!parseIntSafe(digitStr, d))
    {
      Serial.println(F("[DISPLAY] DIGIT requires a number 0 to 9."));
      return;
    }

    if (d < 0 || d > 9)
    {
      Serial.println(F("[DISPLAY] DIGIT must be between 0 and 9."));
      return;
    }

    String durStr;
    String rest3;
    splitFirstWord(rest2, durStr, rest3);

    unsigned long durationMS;
    if (durStr.length() == 0)
    {
      durationMS = cfg_SEGOVR;
    }
    else
    {
      long valSec;
      if (!parseIntSafe(durStr, valSec))
      {
        Serial.println(F("[DISPLAY] Duration must be seconds as integer."));
        return;
      }
      if (valSec <= 0) valSec = 1;
      durationMS = (unsigned long)valSec * 1000UL;
    }

    setDisplayOverrideChar((char)('0' + (int)d), durationMS);
    Serial.print(F("[DISPLAY] DIGIT "));
    Serial.print(d);
    Serial.print(F(" for "));
    Serial.print(durationMS / 1000UL);
    Serial.println(F(" seconds."));
    return;
  }

  if (upName == "CHAR")
  {
    String charStr;
    String rest2;
    splitFirstWord(rest, charStr, rest2);

    if (charStr.length() == 0)
    {
      Serial.println(F("[DISPLAY] CHAR requires one character."));
      return;
    }

    char c = charStr[0];

    String durStr;
    String rest3;
    splitFirstWord(rest2, durStr, rest3);

    unsigned long durationMS;
    if (durStr.length() == 0)
    {
      durationMS = cfg_SEGOVR;
    }
    else
    {
      long valSec;
      if (!parseIntSafe(durStr, valSec))
      {
        Serial.println(F("[DISPLAY] Duration must be seconds as integer."));
        return;
      }
      if (valSec <= 0) valSec = 1;
      durationMS = (unsigned long)valSec * 1000UL;
    }

    setDisplayOverrideChar(c, durationMS);
    Serial.print(F("[DISPLAY] CHAR "));
    Serial.print(c);
    Serial.print(F(" for "));
    Serial.print(durationMS / 1000UL);
    Serial.println(F(" seconds."));
    return;
  }

  if (upName == "BAND" || upName == "TEMPBAND")
  {
    String durStr;
    String rest2;
    splitFirstWord(rest, durStr, rest2);

    unsigned long durationMS;
    if (durStr.length() == 0)
    {
      durationMS = cfg_SEGOVR;
    }
    else
    {
      long valSec;
      if (!parseIntSafe(durStr, valSec))
      {
        Serial.println(F("[DISPLAY] Duration must be seconds as integer."));
        return;
      }
      if (valSec <= 0) valSec = 1;
      durationMS = (unsigned long)valSec * 1000UL;
    }

    float t = simActive ? NoisySimTemp : currentTemp;
    int band = computeBandForTemperature(t);
    if (band < 0) band = 0;
    if (band > 9) band = 9;

    setDisplayOverrideChar((char)('0' + band), durationMS);
    Serial.print(F("[DISPLAY] BAND "));
    Serial.print(band);
    Serial.print(F(" for "));
    Serial.print(durationMS / 1000UL);
    Serial.println(F(" seconds."));
    return;
  }

  if (upName == "CLEAROVR")
  {
    clearDisplayOverride();
    Serial.println(F("[DISPLAY] Overlay cleared."));
    return;
  }

  if (upName == "STATE")
  {
    Serial.print(F("[DISPLAY STATE] Sleep="));
    Serial.print(displaySleep ? F("true") : F("false"));
    Serial.print(F(" overrideActive="));
    Serial.print(displayOverrideActive ? F("true") : F("false"));
    Serial.print(F(" overrideChar="));
    Serial.println(displayOverrideChar);
    return;
  }

  if (upName == "TESTSEG")
  {
    sevenSegTestSeg();
    Serial.println(F("[DISPLAY] TESTSEG done."));
    return;
  }

  if (upName == "COUNTUP")
  {
    displayCountUpRequested = true; // Part 4 flag
    Serial.println(F("[DISPLAY] COUNTUP requested."));
    return;
  }

  if (upName == "COUNTDOWN")
{
  startDisplayCountDown(); // phased countdown: 9..0, then 0 blank 0 blank 0 blank
  Serial.println(F("[DISPLAY] COUNTDOWN started."));
  return;
}

  if (upName == "TESTALL")
  {
    displayTestAllRequested = true; // Part 4 flag
    Serial.println(F("[DISPLAY] TESTALL requested."));
    return;
  }

  Serial.println(F("[DISPLAY] Unknown subcommand. Use DISPLAY OFF, ON, DIGIT, CHAR, BAND, CLEAROVR, STATE, or tests."));
}

// ------------------------ STATS and LOG Commands -----------------------

void handleStatsCommand(bool lgd)
{
  printStats();
}

void handleLogCommand(const String &args, bool lgd)
{
  String s = trimSpaces(args);
  if (s.length() == 0)
  {
    printLog(cfg_LOGLIMIT);
    return;
  }

  long limit;
  if (!parseIntSafe(s, limit))
  {
    Serial.println(F("[LOG] Limit argument must be integer."));
    return;
  }

  if (limit < 0)
  {
    limit = 0;
  }

  printLog((uint16_t)limit);
}

// ------------------------ TIME and WIFI commands -----------------------

void handleTimeCommand(const String &args)
{
  String name;
  String rest;
  splitFirstWord(args, name, rest);
  String upName = toUpperCopy(name);

  if (upName.length() == 0)
  {
    showHelpTime();
    return;
  }

  if (upName == "SYNC")
  {
    requestTimeSync();
    Serial.println(F("[TIME] Sync requested."));
    return;
  }

  if (upName == "STATUS")
  {
    Serial.print(F("[TIME] Synced="));
    Serial.println(timeSynced ? F("true") : F("false"));
    return;
  }

  if (upName == "PRINT")
  {
    printCurrentTime();
    return;
  }

  if (upName == "AUTOINFO")
  {
    handleAdjTimeAuto(String(""));
    return;
  }

  Serial.println(F("[TIME] Unknown subcommand. Use TIME SYNC, STATUS, PRINT, AUTOINFO."));
}

// -------------------------- STOP TEST / DEMO ---------------------------

void handleStopTests()
{
  stopAllTests();
  Serial.println(F("[CTRL] All tests and demos terminated."));
}

// ----------------------------- Help Router -----------------------------

void handleHelpCommand(const String &args)
{
  String topic = trimSpaces(args);
  String up = toUpperCopy(topic);

  if (up.length() == 0)
  {
    showHelpMain();
    return;
  }

  if (up == "WIFI")
  {
    showHelpWifi();
  }
  else if (up == "SSID")
    {
      showHelpSsid();
    }
    else if (up == "STATS" || up == "LOG")
      {
        showHelpStatsLog();
      }
      else if (up == "CTRL" || up == "CONTROL")
        {
          showHelpCtrl();
        }
        else if (up == "BUZZ")
          {
            showHelpBuzz();
          }
          else if (up == "SERVO")
            {
              showHelpServo();
            }
            else if (up == "TIME")
              {
                showHelpTime();
              }
              else if (up == "EEPROM")
                {
                  showHelpEeprom();
                }
                else if (up == "RGB")
                  {
                    showHelpRgb();
                  }
                  else if (up == "DISPLAY" || up == "DSPLY")
                    {
                      showHelpDisplay();
                    }
                      else if (up == "ADJ")
                        {
                          showHelpAdj();
                        }
  else
  {
    Serial.print(F("[HELP] Unknown topic: "));
    Serial.println(up);
    showHelpMain();
  }
}

// ------------------------------ LGD parser ------------------------------

String removeLgdToken(const String &line, bool &foundLgd)
{
  foundLgd = false;
  String result;

  String remaining = line;
  bool firstOut    = true;

  while (true)
  {
    String word;
    String rest;
    splitFirstWord(remaining, word, rest);

    if (word.length() == 0)
    {
      break;
    }

    String upWord = toUpperCopy(word);
    if (upWord == "LGD")
    {
      foundLgd = true;
    }
    else
    {
      if (!firstOut)
      {
        result += " ";
      }
      result += word;
      firstOut = false;
    }

    remaining = rest;
  }

  return trimSpaces(result);
}

// -------------------------- Command Table Dispatcher -------------------------

typedef void (*CmdFn)(const String& args, bool lgdPresent);

struct CmdEntry
{
  const char* name;
  CmdFn fn;
};

// Wrappers keep one common signature.
static void cmdHelp(const String& args, bool lgdPresent)    { (void)lgdPresent; handleHelpCommand(args); }
static void cmdStats(const String& args, bool lgdPresent)   { (void)args; handleStatsCommand(lgdPresent); }
static void cmdLog(const String& args, bool lgdPresent)     { handleLogCommand(args, lgdPresent); }
static void cmdAdj(const String& args, bool lgdPresent)     { (void)lgdPresent; handleAdjCommand(args); }
static void cmdSim(const String& args, bool lgdPresent)     { handleSimCommand(args, lgdPresent); }
static void cmdBuzz(const String& args, bool lgdPresent)    { handleBuzzCommand(args, lgdPresent); }
static void cmdServo(const String& args, bool lgdPresent)   { handleServoCommand(args, lgdPresent); }
static void cmdRgb(const String& args, bool lgdPresent)     { handleRgbCommand(args, lgdPresent); }
static void cmdDisplay(const String& args, bool lgdPresent) { (void)lgdPresent; handleDisplayCommand(args); }
static void cmdTime(const String& args, bool lgdPresent)    { (void)lgdPresent; handleTimeCommand(args); }

// Add new commands by adding rows here.
// Example:
// static void cmdWifi(const String& args, bool lgdPresent) { handleWifiCommand(args, lgdPresent); }
// {"WIFI", cmdWifi},

static const CmdEntry kCmds[] =
{
  {"HELP",    cmdHelp},
  {"H",       cmdHelp},
  {"?",       cmdHelp},
  {"STATS",   cmdStats},
  {"LOG",     cmdLog},
  {"ADJ",     cmdAdj},
  {"SIM",     cmdSim},
  {"BUZZ",    cmdBuzz},
  {"SERVO",   cmdServo},
  {"RGB",     cmdRgb},
  {"DISPLAY", cmdDisplay},
  {"TIME",    cmdTime},
};

static const size_t kCmdCount = sizeof(kCmds) / sizeof(kCmds[0]);

void processCommandLine(String line)
{
  String trimmed = trimSpaces(line);
  if (trimmed.length() == 0)
  {
    return;
  }

  bool lgdPresent = false;
  String noLgd = removeLgdToken(trimmed, lgdPresent);

  String cmd;
  String args;
  splitFirstWord(noLgd, cmd, args);
  String upCmd = toUpperCopy(cmd);

  // Special case: STOP TEST / STOP DEMO
  if (upCmd == "STOP")
  {
    String upArgs = toUpperCopy(trimSpaces(args));
    if (upArgs == "TEST" || upArgs == "DEMO")
    {
      handleStopTests();
      return;
    }
  }

  for (size_t i = 0; i < kCmdCount; i++)
  {
    if (upCmd == kCmds[i].name)
    {
      kCmds[i].fn(args, lgdPresent);
      return;
    }
  }

  Serial.println(F("[CMD] Unknown command."));
  Serial.print(F("You typed: "));
  Serial.println(line);
  Serial.println(F("Type HELP for commands."));
}

// ------------------------ Serial Input Handler -------------------------

void handleSerialInput()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    // Wake on any serial activity (single byte receive).
    recordActivity();

    if (c == '\r')
    {
      c = '\n';
    }

    if (c == '\n')
    {
      String line = serialLine;
      serialLine  = "";

      // Wake on full command submit (Enter).
      recordActivity();

      processCommandLine(line);
    }
    else
    {
      serialLine += c;
      if (serialLine.length() > 200)
      {
        serialLine = serialLine.substring(0, 200);
      }
    }
  }
}

// ======================================================================
// Part 4: test engines, RGB band tests, servo and buzzer TEMP BAND tests,
//         STOP TEST / DEMO handling, and auxiliary helpers.
// ======================================================================

// ----------------------------------------------------------------------
// Simple wrappers around core variables (assumed from Part 1 and Part 2)
// ----------------------------------------------------------------------

// These functions are small helpers that touch global state which is already defined earlier. 
// They do not change core logic, only provide a clear place for tests to interact with it.

void setSimActive(bool active)
{
  if (simActive == active)
  {
    return;
  }

  simActive = active;

  if (!simActive)
  {
    // When simulation ends, rapid rise and fire logic will be
    // naturally recalculated by the existing state machine.
  }
}

// Optional helper in case code wants explicit temperature setter for sim.
void setSimTemperature(float tempC)
{
  BaseSimTemp  = tempC;
  NoisySimTemp = tempC;
}

// ----------------------------------------------------------------------
// TEMP BAND test starters for buzzer, servo, and RGB
// ----------------------------------------------------------------------

void startBuzzerBandTest(int band, unsigned long durationMS)
{
  if (band < 0)
  {
    band = 0;
  }
  if (band > 9)
  {
    band = 9;
  }

  buzzerBandTestActive         = true;
  buzzerBandTestBand           = band;
  buzzerBandTestEndMS          = millis() + durationMS;

  // Core buzzer code in Part 2 should check buzzerBandTestActive and
  // buzzerBandTestBand to use the matching pattern instead of real band.
}

void startServoBandTest(int band, unsigned long durationMS)
{
  if (band < 0)
  {
    band = 0;
  }
  if (band > 9)
  {
    band = 9;
  }

  servoBandTestActive = true;
  servoBandTestBand   = band;
  servoBandTestEndMS  = millis() + durationMS;

  // Decide target position based on band vs fire band
  int fireBand = computeBandForTemperature(cfg_FIRESTART);

  if (band >= fireBand)
  {
    // Above or equal to fire band → release window guard
    setServoTargetReleased();
  }
  else
  {
    // Below fire band → force closed
    setServoTargetClosed();
  }
}

// ----------------------------------------------------------------------
// Internal helpers: small test engines
// ----------------------------------------------------------------------

// Each test engine is driven by updateTestEngines, uses flags and time limits and not blocking delays, so the main system can multitask logging and detecting fires.

// ------------------- Buzzer test (BUZZ TEST) --------------------

void updateBuzzerQuickTest(unsigned long nowMS)
{
  // Ignore test if fire is active
  if (fireActive || inFireRange)
  {
    return;
  }

  // Force buzzer ON for 250 ms, ignoring mute
  buzzerSet(true);
  delay(250);
  buzzerSet(false);
}

// -------------------------- Servo test ---------------------------

void startServoSweepTest(unsigned long durationMS)
{
  servoSweepTestActive  = true;
  servoSweepPhase       = 0;
  servoSweepStepMS      = 400;
  servoSweepNextStep    = millis();
  servoSweepEndMS       = millis() + durationMS;

  // First step triggers a move toward Released.
  setServoTargetReleased();
}

void updateServoSweepTest(unsigned long nowMS)
{
  if (!servoSweepTestActive)
  {
    return;
  }

  if (nowMS >= servoSweepEndMS)
  {
    // End of sweep test.
    servoSweepTestActive = false;
    setServoTargetClosed();
    return;
  }

  if (nowMS < servoSweepNextStep)
  {
    return;
  }

  servoSweepNextStep = nowMS + servoSweepStepMS;

  // Alternate between Closed and Released during the test.
  servoSweepPhase ^= 1;
  if (servoSweepPhase == 0)
  {
    setServoTargetClosed();
  }
  else
  {
    setServoTargetReleased();
  }
}

// -------------------------- RGB tests ---------------------------

void startRgbFlashTest(unsigned long durationMS)
{
  rgbFlashTestActive   = true;
  rgbPulseTestActive   = false;
  rgbRampTestActive    = false;
  rgbBrightTestActive  = false;
  rgbSimpleTestEndMS   = millis() + durationMS;

  rgbMode              = RGBMODE_TEST;
}

void startRgbPulseTest(unsigned long durationMS)
{
  rgbFlashTestActive   = false;
  rgbPulseTestActive   = true;
  rgbRampTestActive    = false;
  rgbBrightTestActive  = false;
  rgbSimpleTestEndMS   = millis() + durationMS;

  rgbMode              = RGBMODE_TEST;
}

void startRgbRampTest(unsigned long durationMS)
{
  rgbFlashTestActive   = false;
  rgbPulseTestActive   = false;
  rgbRampTestActive    = true;
  rgbBrightTestActive  = false;
  rgbSimpleTestEndMS   = millis() + durationMS;

  rgbMode              = RGBMODE_TEST;
}

void startRgbBrightTest(int levelPercent, unsigned long durationMS)
{
  if (levelPercent < 0)
  {
    levelPercent = 0;
  }
  if (levelPercent > 100)
  {
    levelPercent = 100;
  }

  rgbFlashTestActive   = false;
  rgbPulseTestActive   = false;
  rgbRampTestActive    = false;
  rgbBrightTestActive  = true;
  rgbBrightTestLevel   = levelPercent;
  rgbSimpleTestEndMS   = millis() + durationMS;

rgbMode              = RGBMODE_TEST;
}

void updateRgbSimpleTests(unsigned long nowMS)
{
  if (!rgbFlashTestActive &&
      !rgbPulseTestActive &&
      !rgbRampTestActive &&
      !rgbBrightTestActive)
  {
    return;
  }

  if (nowMS >= rgbSimpleTestEndMS)
  {
    // Stop the simple RGB test and return to safe mode.
    rgbFlashTestActive  = false;
    rgbPulseTestActive  = false;
    rgbRampTestActive   = false;
    rgbBrightTestActive = false;
    rgbMode             = RGBMODE_SAFE;
    return;
  }

  // The actual color updates come from the existing rgb state machine that checks rgbMode and rgbBrightTestLevel.
}

// -------------------------- Display tests ------------------------------

// Count up from 0 to 9 with simple overlay.
void startDisplayCountUpTest(unsigned long totalDurationMS)
{
  (void)totalDurationMS;

  displayCountUpActive    = true;
  displayCountDownActive  = false;
  displayAllTestActive    = false;
  displayTestIndex        = 0;

  const unsigned long perStep = 1000UL;

  displayTestNextMS   = millis();
  displayAllTestEndMS = 0;

  setDisplayOverrideChar('0', perStep);
}

void updateDisplayCountUpTest(unsigned long nowMS)
{
  if (!displayCountUpActive) return;
  if (nowMS < displayTestNextMS) return;

  // Compensation for loop + display refresh latency
  const unsigned long HOLD_MS = 1100UL;   // visible ≈ 1000 ms

  displayTestIndex++;

  if (displayTestIndex > 9)
  {
    displayCountUpActive = false;
    clearDisplayOverride();
    return;
  }

  char c = (char)('0' + displayTestIndex);
  setDisplayOverrideChar(c, HOLD_MS);
  displayTestNextMS = nowMS + HOLD_MS;
}

// Count down from 9 to 0 with simple overlay.
void startDisplayCountDownTest(unsigned long totalDurationMS)
{
  displayCountUpActive    = false;
  displayCountDownActive  = true;
  displayAllTestActive    = false;

  displayTestIndex        = 9;

  unsigned long perDigitMS = (totalDurationMS > 0UL) ? (totalDurationMS / 10UL) : 1000UL;
  if (perDigitMS < 50UL) perDigitMS = 50UL;

  displayTestNextMS = millis();

  setDisplayOverrideChar('9', perDigitMS);
}

// COUNTDOWN: 9..1, then end blink: O _ O _ O _

void updateDisplayCountDownTest(unsigned long nowMS)
{
  if (!displayCountDownActive) return;
  if (nowMS < displayTestNextMS) return;

  // Compensation for loop + display refresh latency
  const unsigned long HOLD_MS = 1100UL;
  const unsigned long BLINK_MS = 200UL;

  // Phase 0: 9..1, HOLD_MS each
  if (displayCountDownPhase == 0)
  {
    displayTestIndex--;

    if (displayTestIndex >= 1)
    {
      setDisplayOverrideChar((char)('0' + displayTestIndex), HOLD_MS);
      displayTestNextMS = nowMS + HOLD_MS;
      return;
    }

    displayCountDownPhase = 1;
    displayEndBlinkIndex  = 0;
    displayTestNextMS     = nowMS;
    return;
  }

  // Phase 1: 0 blank 0 blank 0 blank, BLINK_MS each
  if (displayCountDownPhase == 1)
  {
    bool showZero = (displayEndBlinkIndex % 2 == 0);
    char c = showZero ? '0' : ' ';

    setDisplayOverrideChar(c, BLINK_MS);
    displayEndBlinkIndex++;
    displayTestNextMS = nowMS + BLINK_MS;

    if (displayEndBlinkIndex >= 6)
    {
      displayCountDownActive = false;
      clearDisplayOverride();
    }
    return;
  }
}

// Display full test sequence: E, band, rotating digit, and a simple heartbeat.
void startDisplayAllTest(unsigned long durationMS)
{
  displayCountUpActive    = false;
  displayCountDownActive  = false;
  displayAllTestActive    = true;
  displayTestIndex        = 0;
  displayAllTestEndMS     = millis() + durationMS;
  displayTestNextMS       = millis();

  setDisplayOverrideChar('E', cfg_SEGOVR);
}

void updateDisplayAllTest(unsigned long nowMS)
{
  if (!displayAllTestActive)
  {
    return;
  }

  if (nowMS >= displayAllTestEndMS)
  {
    displayAllTestActive = false;
    clearDisplayOverride();
    return;
  }

  if (nowMS < displayTestNextMS)
  {
    return;
  }

  // Simple rotation: E, current band digit, dash, heartbeat pattern.
  displayTestIndex++;
  int phase = displayTestIndex % 4;

  if (phase == 0)
  {
    setDisplayOverrideChar('E', cfg_SEGOVR);
  }
  else if (phase == 1)
  {
    float t   = simActive ? NoisySimTemp : currentTemp;
    int band  = computeBandForTemperature(t);
    if (band < 0)
    {
      band = 0;
    }
    if (band > 9)
    {
      band = 9;
    }
    setDisplayOverrideChar((char)('0' + band), cfg_SEGOVR);
  }
  else if (phase == 2)
  {
    setDisplayOverrideChar('-', cfg_SEGOVR);
  }
  else
  {
    setDisplayOverrideChar('H', cfg_SEGOVR);
  }

  unsigned long stepMS = cfg_SEGOVR;
  if (stepMS < 200)
  {
    stepMS = 200;
  }
  displayTestNextMS = nowMS + stepMS;
}

// ----------------------------------------------------------------------
// Main test engine dispatcher
// ----------------------------------------------------------------------

// This function is intended to be called once each loop iteration from the existing loop. 
// It handles queued requests and keeps the test state machines running without blocking.

void updateTestEngines(unsigned long nowMS)
{
  // Handle one-shot BUZZ TEST request.
  if (buzzerTestRequested)
  {
    buzzerTestRequested = false;
    updateBuzzerQuickTest(nowMS);
  }

  // Handle one-shot SERVO TEST request.
  if (servoTestRequested)
  {
    servoTestRequested = false;
    startServoSweepTest(5000UL);
  }

  // Handle one-shot RGB tests from serial commands.
  if (rgbTestFlashRequested)
  {
    rgbTestFlashRequested = false;
    startRgbFlashTest(cfg_TESTDUR * 1000UL);
  }

  if (rgbTestPulseRequested)
  {
    rgbTestPulseRequested = false;
    startRgbPulseTest(cfg_TESTDUR * 1000UL);
  }

  if (rgbTestRampRequested)
  {
    rgbTestRampRequested = false;
    startRgbRampTest(cfg_TESTDUR * 1000UL);
  }

  if (rgbTestBrightLevelRequested >= 0)
  {
    int level = rgbTestBrightLevelRequested;
    rgbTestBrightLevelRequested = -1;
    startRgbBrightTest(level, cfg_TESTDUR * 1000UL);
  }

  // Handle display tests.
  if (displayCountUpRequested)
  {
    displayCountUpRequested = false;
    startDisplayCountUpTest(cfg_TESTDUR * 1000UL);
  }

  if (displayCountDownRequested)
  {
    displayCountDownRequested = false;

    // Use the phased countdown: 9..1 at ~1s each, then 0/_ blink at 200ms.
    startDisplayCountDown();
  }

  if (displayTestAllRequested)
  {
    displayTestAllRequested = false;
    startDisplayAllTest(cfg_TESTDUR * 1000UL);
  }

  // Maintain active tests over time.
  updateServoSweepTest(nowMS);
  updateRgbSimpleTests(nowMS);
  updateDisplayCountUpTest(nowMS);
  updateDisplayCountDownTest(nowMS);
  updateDisplayAllTest(nowMS);

  // Expire TEMP BAND overrides.
  if (buzzerBandTestActive && nowMS >= buzzerBandTestEndMS)
  {
    buzzerBandTestActive = false;
  }

  if (servoBandTestActive && nowMS >= servoBandTestEndMS)
  {
    servoBandTestActive = false;

    if (!fireActive && !manualOverride)
    {
      setServoTargetClosed();
    }
  }

  if (rgbBandTestActive && nowMS >= rgbBandTestEndMS)
  {
    rgbBandTestActive = false;
  }
}

//     This is a research project of Eric Satorre, Allen Ramos, and Joseph Mendoza
//               coded, assembled, optimized by Eric Angelo Satorre

//   Research Title: Window Evacuation Button-Activated Release System (Window E-BARS)
