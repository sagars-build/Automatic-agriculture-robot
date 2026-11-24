/*
  =====================================================================
   AGRI - Smart Agricultural Spraying Robot  (BUZZER VERSION)
   Developed by Sagar Kushwaha
   Controller : NodeMCU ESP8266
  =====================================================================
  CHANGES IN THIS VERSION
   - Servo motor REMOVED (no servo library, no sweep, nothing on website)
   - 5V BUZZER added on D4 (GPIO2)  [same pin the servo used]
   - Non-blocking buzzer sounds (millis based, nothing freezes):
       * Startup melody when the system powers on
       * Spray-start chirp when the pump turns ON
       * Spray-stop chirp when the pump turns OFF
       * Reverse "beep-beep" while the robot moves BACKWARD
       * Obstacle warning (double beep) when robot is moving
       * Emergency-stop alarm
       * "Field complete" fanfare in automatic mode
       * HORN button on the website (hold = horn sounds)

  BUZZER WIRING (recommended for a 5V buzzer)
   Buzzer (+)  -> 5V (VIN)
   Buzzer (-)  -> NPN transistor collector (2N2222 / BC547)
   Emitter     -> GND
   Base        -> D4 (GPIO2) through a 1k resistor
   (Common GND with NodeMCU. A tiny click at power-on is normal because
    GPIO2 is HIGH during boot.)
   Works best with a PASSIVE buzzer (plays real melodies). With an ACTIVE
   buzzer you get the same rhythm, but every note has the same pitch.

  PIN MAP
   OLED SCL -> D1 (GPIO5)      OLED SDA -> D2 (GPIO4)
   DHT22    -> D5 (GPIO14)     Soil AO  -> A0
   Ultrasonic Trig -> TX (GPIO1), Echo -> RX (GPIO3)
   Buzzer   -> D4 (GPIO2)
   Relay    -> D0 (GPIO16)  LOW = Pump ON, HIGH = Pump OFF
   Left Motor  RPWM -> D3 (GPIO0), LPWM -> D8 (GPIO15)
   Right Motor RPWM -> D6 (GPIO12), LPWM -> D7 (GPIO13)

  LIBRARIES: Adafruit DHT, Adafruit Unified Sensor, Adafruit GFX,
             Adafruit SSD1306 (ESP8266 core libs are built in)
  Board: NodeMCU 1.0 (ESP8266-12E Module)
  =====================================================================
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// ============================== PIN DEFINITIONS ==============================
#define OLED_SCL        5   // D1
#define OLED_SDA        4   // D2
#define DHT_PIN         14  // D5
#define SOIL_PIN        A0
#define TRIG_PIN        1   // TX
#define ECHO_PIN        3   // RX
#define BUZZER_PIN      2   // D4
#define RELAY_PIN       16  // D0
#define L_RPWM          0   // D3  (boot-strap pin, handled safely)
#define L_LPWM          15  // D8  (boot-strap pin, handled safely)
#define R_RPWM          12  // D6
#define R_LPWM          13  // D7

// ============================== CONSTANTS ==============================
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define DHT_TYPE         DHT22
#define PUMP_ON          LOW
#define PUMP_OFF         HIGH
#define AVG_SPEED_KMPH   10.0

const char* AP_SSID     = "AGRI Robot";
const char* AP_PASSWORD = "12345678";

// ============================== OBJECTS ==============================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DHT dht(DHT_PIN, DHT_TYPE);
ESP8266WebServer server(80);

// ============================== STATE VARIABLES ==============================
float temperature = 0, humidity = 0;
int   soilRaw = 0, soilPercent = 0;
float distanceCM = 0;
bool  obstacleDetected = false;
bool  prevObstacle = false;
bool  dhtError = false;

enum RobotMode { MODE_IDLE, MODE_MANUAL, MODE_AUTO };
RobotMode currentMode = MODE_IDLE;

enum MotionState { STOP, FORWARD, BACKWARD, LEFT, RIGHT };
MotionState currentMotion = STOP;

int    motorSpeedPercent = 60;
bool   pumpState          = false;
bool   emergencyStopFlag  = false;

unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 1000;

unsigned long lastOledSwitch = 0;
const unsigned long OLED_PAGE_INTERVAL = 2500;
int oledPage = 0;
const int OLED_TOTAL_PAGES = 18;

unsigned long lastUltrasonicRead = 0;
const unsigned long ULTRASONIC_INTERVAL = 200;

// ---------------- Automatic mode fields ----------------
float fieldLength = 0, fieldWidth = 0, rowSpacing = 0;
int   totalRows = 0;
float totalDistanceM = 0;
float estimatedTimeMin = 0;
float coveragePercent = 0;
float distanceTraveledM = 0;

enum AutoState { AUTO_IDLE, AUTO_ROW_FORWARD, AUTO_TURN1, AUTO_SHIFT, AUTO_TURN2, AUTO_OBSTACLE_WAIT, AUTO_DONE };
AutoState autoState = AUTO_IDLE;
int   autoCurrentRow = 0;
unsigned long autoStateStart = 0;
unsigned long autoRowRunTimeMs = 0;
unsigned long autoTurnTimeMs   = 700;
unsigned long autoShiftTimeMs  = 0;

String recommendationText = "System Ready";

// =====================================================================
//                       BUZZER SYSTEM (non-blocking)
// =====================================================================
struct Note { uint16_t freq; uint16_t dur; };   // freq 0 = silence (rest)

// Startup: rising arpeggio + little flourish
const Note MEL_STARTUP[] = {
  {523,110},{659,110},{784,110},{1047,180},{0,70},{784,90},{1047,90},{1319,380}
};
// Spray start: bright rising "water" trill
const Note MEL_SPRAY_ON[] = {
  {880,70},{1175,70},{1568,70},{1319,60},{1568,60},{1319,60},{1760,220}
};
// Spray stop: falling chirp
const Note MEL_SPRAY_OFF[] = {
  {1568,90},{1175,90},{880,90},{659,220}
};
// Obstacle warning: two sharp beeps
const Note MEL_OBSTACLE[] = {
  {2200,90},{0,60},{2200,90}
};
// Emergency stop alarm
const Note MEL_ESTOP[] = {
  {1000,160},{700,160},{1000,160},{700,160},{1000,160},{700,260}
};
// Field complete fanfare
const Note MEL_DONE[] = {
  {784,120},{988,120},{1175,120},{1568,260},{0,60},{1175,100},{1568,450}
};

const Note* melody = nullptr;
uint8_t melodyLen = 0, melodyIdx = 0;
bool melodyActive = false, noteSounding = false;
unsigned long noteStart = 0;

bool hornActive = false;
unsigned long hornStart = 0;
const unsigned long HORN_MAX_MS = 5000;   // safety: horn auto-stops after 5s

bool reverseBeepOn = false;
unsigned long lastReverseBeep = 0;
const unsigned long REVERSE_BEEP_MS = 300;

#define PLAY(m) playMelody(m, sizeof(m) / sizeof(Note))

void startNote() {
  Note n = melody[melodyIdx];
  noteStart = millis();
  if (n.freq > 0) { tone(BUZZER_PIN, n.freq); noteSounding = true; }
  else            { noTone(BUZZER_PIN);       noteSounding = false; }
}

void playMelody(const Note* m, uint8_t len) {
  if (hornActive) return;           // horn has priority
  melody = m; melodyLen = len; melodyIdx = 0;
  melodyActive = true;
  reverseBeepOn = false;
  startNote();
}

void hornOn() {
  if (!hornActive) {
    hornActive = true;
    melodyActive = false;
    reverseBeepOn = false;
    tone(BUZZER_PIN, 450);
  }
  hornStart = millis();
}

void hornOff() {
  if (hornActive) {
    hornActive = false;
    noTone(BUZZER_PIN);
  }
}

void updateBuzzer() {
  unsigned long now = millis();

  // 1) Horn (highest priority)
  if (hornActive) {
    if (now - hornStart > HORN_MAX_MS) hornOff();
    return;
  }

  // 2) Melodies
  if (melodyActive) {
    Note n = melody[melodyIdx];
    unsigned long elapsed = now - noteStart;
    if (noteSounding && elapsed >= (unsigned long)n.dur * 85UL / 100UL) {
      noTone(BUZZER_PIN);          // small gap so notes sound separate & clean
      noteSounding = false;
    }
    if (elapsed >= n.dur) {
      melodyIdx++;
      if (melodyIdx >= melodyLen) { melodyActive = false; noTone(BUZZER_PIN); }
      else startNote();
    }
    return;
  }

  // 3) Reverse beeping while moving backward
  if (currentMotion == BACKWARD && !emergencyStopFlag) {
    if (now - lastReverseBeep >= REVERSE_BEEP_MS) {
      lastReverseBeep = now;
      reverseBeepOn = !reverseBeepOn;
      if (reverseBeepOn) tone(BUZZER_PIN, 1000);
      else               noTone(BUZZER_PIN);
    }
  } else if (reverseBeepOn) {
    reverseBeepOn = false;
    noTone(BUZZER_PIN);
  }
}

// ============================== FUNCTION PROTOTYPES ==============================
void setupWiFi();
void setupWebServer();
void setupOLED();
void handleRoot();
void handleData();
void handleControl();
void handleAutoStart();
void handleAutoStop();
void readDHTSensor();
void readSoilSensor();
void readUltrasonicSensor();
void motorControl(MotionState dir, int speedPercent);
void safeMotorWrite(uint8_t pin, uint8_t value);
void pumpControl(bool on);
void updateOLEDDisplay();
void drawOledPage(int page);
void generateRecommendations();
void runAutomaticNavigation();
void emergencyStop();
void sensorDiagnostics();
String buildDataJSON();
String htmlPage();

// =====================================================================
//                                SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println(F("\n[AGRI] Booting Smart Agricultural Spraying Robot..."));
  Serial.println(F("[AGRI] Developed by Sagar Kushwaha"));
  Serial.flush();
  Serial.end(); // release TX/RX (GPIO1/GPIO3) for ultrasonic use

  // ---- Safe boot-pin handling for BTS7960 (GPIO0 / GPIO15) ----
  pinMode(L_RPWM, INPUT);
  pinMode(L_LPWM, INPUT);
  delay(5);
  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  digitalWrite(L_RPWM, LOW);
  digitalWrite(L_LPWM, LOW);

  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  digitalWrite(R_RPWM, LOW);
  digitalWrite(R_LPWM, LOW);

  // ---- Relay (Pump) - default OFF ----
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, PUMP_OFF);

  // ---- Ultrasonic ----
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // ---- Buzzer ----
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  setupOLED();
  dht.begin();
  setupWiFi();
  setupWebServer();

  recommendationText = "System Ready";
  PLAY(MEL_STARTUP);   // startup melody (plays in background)
}

// =====================================================================
//                                 LOOP
// =====================================================================
void loop() {
  server.handleClient();
  updateBuzzer();

  unsigned long now = millis();

  if (now - lastUltrasonicRead >= ULTRASONIC_INTERVAL) {
    lastUltrasonicRead = now;
    readUltrasonicSensor();
  }

  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readDHTSensor();
    readSoilSensor();
    sensorDiagnostics();
    generateRecommendations();
  }

  if (now - lastOledSwitch >= OLED_PAGE_INTERVAL) {
    lastOledSwitch = now;
    oledPage = (oledPage + 1) % OLED_TOTAL_PAGES;
    updateOLEDDisplay();
  }

  if (currentMode == MODE_AUTO && !emergencyStopFlag) {
    runAutomaticNavigation();
  }

  // Global obstacle safety net (manual mode)
  if (obstacleDetected && currentMotion == FORWARD && !emergencyStopFlag) {
    if (currentMode == MODE_MANUAL) motorControl(STOP, 0);
  }
}

// =====================================================================
//                          WI-FI SETUP (Access Point)
// =====================================================================
void setupWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  delay(200);
}

// =====================================================================
//                           OLED SETUP
// =====================================================================
void setupOLED() {
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(20, 15);
  display.println("AGRI");
  display.setTextSize(1);
  display.setCursor(10, 40);
  display.println("Sagar Kushwaha");
  display.display();
}

// =====================================================================
//                          WEB SERVER SETUP
// =====================================================================
void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/control", HTTP_GET, handleControl);
  server.on("/autostart", HTTP_GET, handleAutoStart);
  server.on("/autostop", HTTP_GET, handleAutoStop);
  server.onNotFound([]() {
    server.send(404, "text/plain", "404 - Not Found");
  });
  server.begin();
}

// =====================================================================
//                        HTTP ROUTE HANDLERS
// =====================================================================
void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleData() {
  server.send(200, "application/json", buildDataJSON());
}

// /control?cmd=forward|backward|left|right|stop|estop|estopclear|pumpon|pumpoff|hornon|hornoff|speed&val=NN&mode=manual|auto|home
void handleControl() {
  if (server.hasArg("mode")) {
    String m = server.arg("mode");
    if (m == "manual") { currentMode = MODE_MANUAL; emergencyStopFlag = false; }
    else if (m == "auto") { currentMode = MODE_AUTO; emergencyStopFlag = false; }
    else if (m == "home") { currentMode = MODE_IDLE; motorControl(STOP, 0); }
  }

  if (server.hasArg("cmd")) {
    String cmd = server.arg("cmd");

    // Horn always works, even during emergency stop
    if (cmd == "hornon")  { hornOn();  server.send(200, "text/plain", "OK"); return; }
    if (cmd == "hornoff") { hornOff(); server.send(200, "text/plain", "OK"); return; }

    if (emergencyStopFlag && cmd != "estopclear") {
      server.send(200, "text/plain", "BLOCKED_ESTOP");
      return;
    }

    if (cmd == "forward")       motorControl(FORWARD, motorSpeedPercent);
    else if (cmd == "backward") motorControl(BACKWARD, motorSpeedPercent);
    else if (cmd == "left")     motorControl(LEFT, motorSpeedPercent);
    else if (cmd == "right")    motorControl(RIGHT, motorSpeedPercent);
    else if (cmd == "stop")     motorControl(STOP, 0);
    else if (cmd == "estop")    emergencyStop();
    else if (cmd == "estopclear") { emergencyStopFlag = false; motorControl(STOP, 0); }
    else if (cmd == "pumpon")   pumpControl(true);
    else if (cmd == "pumpoff")  pumpControl(false);
    else if (cmd == "speed" && server.hasArg("val")) {
      int v = server.arg("val").toInt();
      motorSpeedPercent = constrain(v, 0, 100);
    }
  }

  server.send(200, "text/plain", "OK");
}

// /autostart?length=..&width=..&spacing=..
void handleAutoStart() {
  if (!server.hasArg("length") || !server.hasArg("width") || !server.hasArg("spacing")) {
    server.send(400, "text/plain", "MISSING_PARAMS");
    return;
  }
  float L = server.arg("length").toFloat();
  float W = server.arg("width").toFloat();
  float S = server.arg("spacing").toFloat();

  if (L <= 0 || W <= 0 || S <= 0 || S > W) {
    server.send(400, "text/plain", "INVALID_INPUT");
    return;
  }

  fieldLength = L;
  fieldWidth  = W;
  rowSpacing  = S;

  totalRows = max(1, (int)ceil(fieldWidth / rowSpacing));
  totalDistanceM = totalRows * fieldLength;

  float speedMps = (AVG_SPEED_KMPH * 1000.0) / 3600.0;
  estimatedTimeMin = (totalDistanceM / speedMps) / 60.0;

  autoRowRunTimeMs = (unsigned long)((fieldLength / speedMps) * 1000.0);
  autoShiftTimeMs  = (unsigned long)((rowSpacing  / speedMps) * 1000.0);

  autoCurrentRow = 0;
  distanceTraveledM = 0;
  coveragePercent = 0;
  autoState = AUTO_ROW_FORWARD;
  autoStateStart = millis();
  currentMode = MODE_AUTO;
  emergencyStopFlag = false;

  pumpControl(true);                 // plays spray-start sound
  motorControl(FORWARD, motorSpeedPercent);

  server.send(200, "text/plain", "AUTO_STARTED");
}

void handleAutoStop() {
  autoState = AUTO_IDLE;
  motorControl(STOP, 0);
  pumpControl(false);                // plays spray-stop sound
  currentMode = MODE_IDLE;
  server.send(200, "text/plain", "AUTO_STOPPED");
}

// =====================================================================
//                          SENSOR FUNCTIONS
// =====================================================================
void readDHTSensor() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    dhtError = true;
  } else {
    dhtError = false;
    temperature = t;
    humidity = h;
  }
}

void readSoilSensor() {
  soilRaw = analogRead(SOIL_PIN);
  soilPercent = constrain(map(soilRaw, 1023, 0, 0, 100), 0, 100);
}

void readUltrasonicSensor() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) {
    distanceCM = 400;
  } else {
    distanceCM = duration * 0.0343 / 2.0;
  }
  obstacleDetected = (distanceCM > 0 && distanceCM < 25.0);

  // Warning double-beep the moment an obstacle appears while robot is moving
  if (obstacleDetected && !prevObstacle && currentMotion != STOP && !emergencyStopFlag) {
    PLAY(MEL_OBSTACLE);
  }
  prevObstacle = obstacleDetected;
}

void sensorDiagnostics() {
  if (soilRaw <= 0 && soilRaw >= 1023) dhtError = true; // placeholder combined check
}

// =====================================================================
//                          MOTOR CONTROL (BTS7960 x2)
// =====================================================================
void safeMotorWrite(uint8_t pin, uint8_t value) {
  digitalWrite(pin, value);
}

void motorControl(MotionState dir, int speedPercent) {
  currentMotion = dir;
  int pwmVal = map(constrain(speedPercent, 0, 100), 0, 100, 0, 255);

  safeMotorWrite(L_RPWM, LOW);
  safeMotorWrite(L_LPWM, LOW);
  safeMotorWrite(R_RPWM, LOW);
  safeMotorWrite(R_LPWM, LOW);

  switch (dir) {
    case FORWARD:
      analogWrite(L_RPWM, pwmVal);
      analogWrite(R_RPWM, pwmVal);
      break;
    case BACKWARD:
      analogWrite(L_LPWM, pwmVal);
      analogWrite(R_LPWM, pwmVal);
      break;
    case LEFT:
      analogWrite(L_LPWM, pwmVal);
      analogWrite(R_RPWM, pwmVal);
      break;
    case RIGHT:
      analogWrite(L_RPWM, pwmVal);
      analogWrite(R_LPWM, pwmVal);
      break;
    case STOP:
    default:
      break;
  }
}

void emergencyStop() {
  emergencyStopFlag = true;
  motorControl(STOP, 0);
  pumpControl(false);
  recommendationText = "EMERGENCY STOP ACTIVE";
  PLAY(MEL_ESTOP);                   // alarm replaces the spray-stop chirp
}

// =====================================================================
//                          PUMP CONTROL (Relay)
// =====================================================================
void pumpControl(bool on) {
  bool changed = (pumpState != on);
  pumpState = on;
  digitalWrite(RELAY_PIN, on ? PUMP_ON : PUMP_OFF);
  if (changed) {
    if (on) PLAY(MEL_SPRAY_ON);
    else    PLAY(MEL_SPRAY_OFF);
  }
}

// =====================================================================
//                     INTELLIGENT RECOMMENDATION ENGINE
// =====================================================================
void generateRecommendations() {
  String r = "";

  if (dhtError) r += "Sensor error (DHT22). ";

  if (soilPercent < 30) r += "Soil needs irrigation. ";
  else r += "Soil moisture is sufficient. ";

  if (!dhtError) {
    if (temperature > 35) r += "High temperature. ";
    else r += "Normal temperature. ";

    if (humidity > 80) r += "High humidity. ";
    else if (humidity < 30) r += "Low humidity. ";
  }

  if (obstacleDetected) r += "Obstacle ahead. ";
  else r += "Safe to continue. ";

  if (currentMotion != STOP) r += "Robot moving. ";
  else r += "Robot stopped. ";

  if (pumpState) r += "Spraying in progress. Pump ON. ";
  else r += "Pump OFF. ";

  if (!dhtError) r += "System healthy.";

  recommendationText = r;
}

// =====================================================================
//                     AUTOMATIC NAVIGATION STATE MACHINE
// =====================================================================
void runAutomaticNavigation() {
  unsigned long now = millis();
  unsigned long elapsed = now - autoStateStart;

  if (autoState == AUTO_ROW_FORWARD && obstacleDetected) {
    motorControl(STOP, 0);
    pumpControl(false);
    autoState = AUTO_OBSTACLE_WAIT;
    recommendationText = "Obstacle Detected - Waiting";
    return;
  }

  switch (autoState) {
    case AUTO_ROW_FORWARD:
      distanceTraveledM = (autoCurrentRow * fieldLength) +
                           (fieldLength * min(1.0f, (float)elapsed / (float)max((unsigned long)1, autoRowRunTimeMs)));
      coveragePercent = constrain((distanceTraveledM / totalDistanceM) * 100.0, 0, 100);

      if (elapsed >= autoRowRunTimeMs) {
        motorControl(STOP, 0);
        autoCurrentRow++;
        if (autoCurrentRow >= totalRows) {
          autoState = AUTO_DONE;
          autoStateStart = now;
        } else {
          autoState = AUTO_TURN1;
          autoStateStart = now;
          motorControl((autoCurrentRow % 2 == 1) ? RIGHT : LEFT, motorSpeedPercent);
        }
      }
      break;

    case AUTO_TURN1:
      if (elapsed >= autoTurnTimeMs) {
        motorControl(FORWARD, motorSpeedPercent);
        autoState = AUTO_SHIFT;
        autoStateStart = now;
      }
      break;

    case AUTO_SHIFT:
      if (elapsed >= autoShiftTimeMs) {
        motorControl((autoCurrentRow % 2 == 1) ? RIGHT : LEFT, motorSpeedPercent);
        autoState = AUTO_TURN2;
        autoStateStart = now;
      }
      break;

    case AUTO_TURN2:
      if (elapsed >= autoTurnTimeMs) {
        pumpControl(true);
        motorControl(FORWARD, motorSpeedPercent);
        autoState = AUTO_ROW_FORWARD;
        autoStateStart = now;
      }
      break;

    case AUTO_OBSTACLE_WAIT:
      if (!obstacleDetected) {
        pumpControl(true);
        motorControl(FORWARD, motorSpeedPercent);
        autoState = AUTO_ROW_FORWARD;
        autoStateStart = now;
      }
      break;

    case AUTO_DONE:
      motorControl(STOP, 0);
      pumpControl(false);
      coveragePercent = 100;
      recommendationText = "Field Completed Successfully";
      currentMode = MODE_IDLE;
      autoState = AUTO_IDLE;
      PLAY(MEL_DONE);                // victory fanfare
      break;

    case AUTO_IDLE:
    default:
      break;
  }
}

// =====================================================================
//                          OLED DISPLAY PAGES
// =====================================================================
void updateOLEDDisplay() {
  drawOledPage(oledPage);
}

void drawOledPage(int page) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.setTextSize(1);

  switch (page) {
    case 0:
      display.setTextSize(2);
      display.setCursor(20, 10);
      display.println("AGRI");
      display.setTextSize(1);
      display.setCursor(10, 40);
      display.println("By Sagar Kushwaha");
      break;
    case 1:
      display.println("Temp & Humidity");
      display.setCursor(0, 20);
      display.print("Temp: "); display.print(dhtError ? -1 : temperature); display.println(" C");
      display.setCursor(0, 35);
      display.print("Humidity: "); display.print(dhtError ? -1 : humidity); display.println(" %");
      break;
    case 2:
      display.println("Soil Moisture");
      display.setCursor(0, 20);
      display.print(soilPercent); display.println(" %");
      break;
    case 3:
      display.println("Ultrasonic Distance");
      display.setCursor(0, 20);
      display.print(distanceCM); display.println(" cm");
      break;
    case 4:
      display.println("Obstacle Status");
      display.setCursor(0, 20);
      display.println(obstacleDetected ? "Obstacle Ahead" : "Path Clear");
      break;
    case 5:
      display.println("Pump Status");
      display.setCursor(0, 20);
      display.println(pumpState ? "Pump: ON" : "Pump: OFF");
      break;
    case 6:
      display.println("Horn / Buzzer");
      display.setCursor(0, 20);
      display.println(hornActive ? "HORN ON" : "Ready");
      break;
    case 7:
      display.println("Robot Mode");
      display.setCursor(0, 20);
      display.println(currentMode == MODE_MANUAL ? "Manual" : currentMode == MODE_AUTO ? "Automatic" : "Idle");
      break;
    case 8:
      display.println("Robot Motion");
      display.setCursor(0, 20);
      switch (currentMotion) {
        case FORWARD:  display.println("Forward"); break;
        case BACKWARD: display.println("Backward"); break;
        case LEFT:     display.println("Turning Left"); break;
        case RIGHT:    display.println("Turning Right"); break;
        default:       display.println("Stopped"); break;
      }
      break;
    case 9:
      display.println("Motor Speed");
      display.setCursor(0, 20);
      display.print(motorSpeedPercent); display.println(" %");
      break;
    case 10:
      display.println("WiFi Status");
      display.setCursor(0, 20);
      display.print("Clients: "); display.println(WiFi.softAPgetStationNum());
      break;
    case 11:
      display.println("IP Address");
      display.setCursor(0, 20);
      display.println(WiFi.softAPIP());
      break;
    case 12:
      display.println("Manual Mode");
      display.setCursor(0, 20);
      display.println(currentMode == MODE_MANUAL ? "ACTIVE" : "Inactive");
      break;
    case 13:
      display.println("Automatic Mode");
      display.setCursor(0, 20);
      display.println(currentMode == MODE_AUTO ? "ACTIVE" : "Inactive");
      break;
    case 14:
      display.println("Sensor Health");
      display.setCursor(0, 20);
      display.println(dhtError ? "DHT22 ERROR" : "All Sensors OK");
      break;
    case 15:
      display.println("System Ready");
      display.setCursor(0, 20);
      display.println(emergencyStopFlag ? "E-STOP ACTIVE" : "Ready");
      break;
    case 16:
      display.println("Spraying Status");
      display.setCursor(0, 20);
      display.println(pumpState ? "Spraying..." : "Not Spraying");
      break;
    case 17:
      display.println("Recommendation");
      display.setCursor(0, 15);
      display.setTextWrap(true);
      display.println(recommendationText.substring(0, 80));
      break;
  }
  display.display();
}

// =====================================================================
//                          JSON DATA BUILDER
// =====================================================================
String buildDataJSON() {
  String modeStr = (currentMode == MODE_MANUAL) ? "manual" : (currentMode == MODE_AUTO) ? "auto" : "idle";
  String motionStr;
  switch (currentMotion) {
    case FORWARD:  motionStr = "forward"; break;
    case BACKWARD: motionStr = "backward"; break;
    case LEFT:     motionStr = "left"; break;
    case RIGHT:    motionStr = "right"; break;
    default:       motionStr = "stop"; break;
  }

  String json = "{";
  json += "\"temperature\":" + String(dhtError ? -1 : temperature, 1) + ",";
  json += "\"humidity\":" + String(dhtError ? -1 : humidity, 1) + ",";
  json += "\"soil\":" + String(soilPercent) + ",";
  json += "\"distance\":" + String(distanceCM, 1) + ",";
  json += "\"obstacle\":" + String(obstacleDetected ? "true" : "false") + ",";
  json += "\"mode\":\"" + modeStr + "\",";
  json += "\"motion\":\"" + motionStr + "\",";
  json += "\"speed\":" + String(motorSpeedPercent) + ",";
  json += "\"pump\":" + String(pumpState ? "true" : "false") + ",";
  json += "\"horn\":" + String(hornActive ? "true" : "false") + ",";
  json += "\"estop\":" + String(emergencyStopFlag ? "true" : "false") + ",";
  json += "\"dhtError\":" + String(dhtError ? "true" : "false") + ",";
  json += "\"clients\":" + String(WiFi.softAPgetStationNum()) + ",";
  json += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
  json += "\"rec\":\"" + recommendationText + "\",";
  json += "\"totalRows\":" + String(totalRows) + ",";
  json += "\"totalDistance\":" + String(totalDistanceM, 1) + ",";
  json += "\"estTime\":" + String(estimatedTimeMin, 1) + ",";
  json += "\"coverage\":" + String(coveragePercent, 1) + ",";
  json += "\"currentRow\":" + String(autoCurrentRow);
  json += "}";
  return json;
}

// =====================================================================
//                         EMBEDDED WEB PAGE (HTML+CSS+JS)
// =====================================================================
String htmlPage() {
  String html = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>AGRI - Smart Agricultural Robot</title>
<style>
:root{
  --green:#22c55e; --green-dark:#16a34a; --bg:#0b1410; --card:rgba(255,255,255,0.06);
  --card-border:rgba(255,255,255,0.12); --text:#e6f4ea; --muted:#9db3a4; --danger:#ef4444; --amber:#f59e0b;
}
*{box-sizing:border-box; margin:0; padding:0; -webkit-user-select:none; user-select:none; -webkit-touch-callout:none;}
body{
  background:radial-gradient(circle at top, #10221a 0%, #060b08 70%);
  color:var(--text); font-family:'Segoe UI',Roboto,Arial,sans-serif; min-height:100vh; padding-bottom:30px;
}
.header{ text-align:center; padding:22px 10px 10px; }
.header h1{ font-size:2.2em; letter-spacing:3px; color:var(--green); text-shadow:0 0 18px rgba(34,197,94,.5); }
.header p{ color:var(--muted); font-size:.85em; margin-top:4px;}
.container{ max-width:520px; margin:0 auto; padding:10px 14px; }
.card{
  background:var(--card); border:1px solid var(--card-border); border-radius:18px;
  padding:16px; margin-bottom:14px; backdrop-filter:blur(10px);
  box-shadow:0 4px 20px rgba(0,0,0,.35); animation:fadeIn .5s ease;
}
@keyframes fadeIn{ from{opacity:0; transform:translateY(8px);} to{opacity:1; transform:none;} }
.status-grid{ display:grid; grid-template-columns:1fr 1fr; gap:8px; }
.status-item{ background:rgba(255,255,255,.04); border-radius:10px; padding:8px 10px; font-size:.8em; }
.status-item span{ display:block; color:var(--green); font-weight:600; font-size:1.05em; margin-top:2px;}
.nav-buttons{ display:flex; gap:12px; margin-top:6px; }
.nav-btn{
  flex:1; padding:22px 10px; border:none; border-radius:16px; font-size:1.1em; font-weight:700;
  color:#fff; background:linear-gradient(135deg,var(--green),var(--green-dark)); cursor:pointer;
  box-shadow:0 6px 18px rgba(34,197,94,.35); transition:transform .15s ease;
}
.nav-btn:active{ transform:scale(.95); }
.section-title{ font-size:1.05em; color:var(--green); margin-bottom:10px; font-weight:700; letter-spacing:1px;}
.dpad{ display:grid; grid-template-columns:1fr 1fr 1fr; grid-template-rows:auto auto auto; gap:8px; max-width:280px; margin:10px auto;}
.dpad button{
  padding:18px 0; font-size:1.4em; border:none; border-radius:14px; color:#fff;
  background:rgba(34,197,94,.18); border:1px solid var(--green); cursor:pointer;
}
.dpad button:active{ background:var(--green); }
.dpad .empty{ background:transparent; border:none; }
.btn-row{ display:flex; gap:10px; flex-wrap:wrap; margin-top:10px;}
.btn{
  flex:1; min-width:120px; padding:14px; border:none; border-radius:12px; font-weight:600;
  color:#fff; cursor:pointer; font-size:.9em; background:rgba(34,197,94,.2); border:1px solid var(--green);
}
.btn.danger{ background:rgba(239,68,68,.25); border-color:var(--danger); color:#fff; }
.btn.off{ background:rgba(255,255,255,.06); border-color:var(--card-border); color:var(--muted); }
.horn-btn{
  width:100%; padding:20px; margin-top:10px; border-radius:16px; font-size:1.2em; font-weight:800;
  letter-spacing:2px; color:#fff; cursor:pointer; border:1px solid var(--amber);
  background:linear-gradient(135deg,#f59e0b,#d97706); box-shadow:0 6px 18px rgba(245,158,11,.35);
  transition:transform .1s ease;
}
.horn-btn.pressed{ transform:scale(.96); box-shadow:0 0 28px rgba(245,158,11,.9); }
.hint{ text-align:center; color:var(--muted); font-size:.75em; margin-top:6px; }
.slider-wrap{ margin-top:14px; }
.slider-wrap label{ font-size:.85em; color:var(--muted); }
input[type=range]{ width:100%; accent-color:var(--green); margin-top:6px; }
input[type=number]{
  width:100%; padding:12px; border-radius:10px; border:1px solid var(--card-border);
  background:rgba(255,255,255,.05); color:var(--text); margin-top:6px; font-size:1em;
  -webkit-user-select:text; user-select:text;
}
.rec-box{ background:rgba(34,197,94,.08); border-left:3px solid var(--green); padding:10px 12px; border-radius:8px; font-size:.85em; line-height:1.5;}
.hidden{ display:none !important; }
.back-btn{ background:transparent; border:1px solid var(--card-border); color:var(--muted); padding:8px 14px; border-radius:10px; font-size:.8em; cursor:pointer; margin-bottom:12px;}
.progress-bar{ width:100%; height:10px; background:rgba(255,255,255,.08); border-radius:6px; overflow:hidden; margin-top:8px;}
.progress-fill{ height:100%; background:linear-gradient(90deg,var(--green),var(--green-dark)); width:0%; transition:width .5s ease;}
.pill{ display:inline-block; padding:3px 10px; border-radius:20px; font-size:.75em; background:rgba(34,197,94,.2); color:var(--green); margin-right:6px;}
.pill.bad{ background:rgba(239,68,68,.2); color:var(--danger); }
footer{ text-align:center; color:var(--muted); font-size:.75em; margin-top:20px; }
</style>
</head>
<body>

<div class="header">
  <h1>AGRI</h1>
  <p>Smart Agricultural Spraying Robot &middot; Developed by Sagar Kushwaha</p>
</div>

<div class="container">

  <!-- HOME PAGE -->
  <div id="page-home">
    <div class="card">
      <div class="status-grid">
        <div class="status-item">Robot<span id="h-connected">Connected</span></div>
        <div class="status-item">WiFi Status<span id="h-wifi">-</span></div>
        <div class="status-item">System<span id="h-system">Ready</span></div>
        <div class="status-item">IP Address<span id="h-ip">-</span></div>
      </div>
    </div>
    <div class="card">
      <div class="nav-buttons">
        <button class="nav-btn" onclick="goPage('manual')">Manual Mode</button>
        <button class="nav-btn" onclick="goPage('auto')">Automatic Mode</button>
      </div>
    </div>
    <div class="card">
      <div class="section-title">Live Dashboard</div>
      <div class="status-grid">
        <div class="status-item">Temperature<span id="d-temp">-- C</span></div>
        <div class="status-item">Humidity<span id="d-hum">-- %</span></div>
        <div class="status-item">Soil Moisture<span id="d-soil">-- %</span></div>
        <div class="status-item">Distance<span id="d-dist">-- cm</span></div>
        <div class="status-item">Obstacle<span id="d-obs">--</span></div>
        <div class="status-item">Robot Status<span id="d-motion">--</span></div>
        <div class="status-item">Pump<span id="d-pump">--</span></div>
        <div class="status-item">Horn<span id="d-horn">--</span></div>
      </div>
    </div>
    <div class="card">
      <div class="section-title">Recommendations</div>
      <div class="rec-box" id="d-rec">System Ready</div>
    </div>
  </div>

  <!-- MANUAL MODE PAGE -->
  <div id="page-manual" class="hidden">
    <button class="back-btn" onclick="goPage('home')">&larr; Back to Home</button>
    <div class="card">
      <div class="section-title">Manual Control</div>
      <div class="dpad">
        <div class="empty"></div><button ontouchstart="cmd('forward')" onmousedown="cmd('forward')" ontouchend="cmd('stop')" onmouseup="cmd('stop')">&#8593;</button><div class="empty"></div>
        <button ontouchstart="cmd('left')" onmousedown="cmd('left')" ontouchend="cmd('stop')" onmouseup="cmd('stop')">&#8592;</button>
        <button style="background:rgba(239,68,68,.3); border-color:#ef4444;" onclick="cmd('estop')">STOP</button>
        <button ontouchstart="cmd('right')" onmousedown="cmd('right')" ontouchend="cmd('stop')" onmouseup="cmd('stop')">&#8594;</button>
        <div class="empty"></div><button ontouchstart="cmd('backward')" onmousedown="cmd('backward')" ontouchend="cmd('stop')" onmouseup="cmd('stop')">&#8595;</button><div class="empty"></div>
      </div>
      <div class="btn-row">
        <button class="btn danger" onclick="cmd('estop')">Emergency Stop</button>
        <button class="btn off" onclick="cmd('estopclear')">Clear E-Stop</button>
      </div>
      <div class="slider-wrap">
        <label>Motor Speed: <span id="speedVal">60</span>%</label>
        <input type="range" min="0" max="100" value="60" id="speedSlider" oninput="setSpeed(this.value)">
      </div>
    </div>
    <div class="card">
      <div class="section-title">Pump &amp; Horn</div>
      <div class="btn-row">
        <button class="btn" onclick="cmd('pumpon')">Pump ON</button>
        <button class="btn off" onclick="cmd('pumpoff')">Pump OFF</button>
      </div>
      <button class="horn-btn" id="hornBtn"
        ontouchstart="hornDown(event)" ontouchend="hornUp(event)" ontouchcancel="hornUp(event)"
        onmousedown="hornDown(event)" onmouseup="hornUp(event)" onmouseleave="hornUp(event)">&#128226; HORN</button>
      <div class="hint">Press and hold to sound the horn</div>
    </div>
    <div class="card">
      <div class="section-title">Live Dashboard</div>
      <div class="status-grid">
        <div class="status-item">Temperature<span id="m-temp">--</span></div>
        <div class="status-item">Humidity<span id="m-hum">--</span></div>
        <div class="status-item">Soil<span id="m-soil">--</span></div>
        <div class="status-item">Distance<span id="m-dist">--</span></div>
      </div>
      <div class="rec-box" id="m-rec" style="margin-top:10px;">--</div>
    </div>
  </div>

  <!-- AUTOMATIC MODE PAGE -->
  <div id="page-auto" class="hidden">
    <button class="back-btn" onclick="goPage('home')">&larr; Back to Home</button>
    <div class="card">
      <div class="section-title">Field Configuration</div>
      <label>Field Length (m)</label>
      <input type="number" id="fLength" placeholder="e.g. 50">
      <label>Field Width (m)</label>
      <input type="number" id="fWidth" placeholder="e.g. 20">
      <label>Row Spacing (m)</label>
      <input type="number" id="fSpacing" placeholder="e.g. 1">
      <div class="btn-row">
        <button class="btn" onclick="startAuto()">START</button>
        <button class="btn danger" onclick="stopAuto()">STOP</button>
      </div>
    </div>
    <div class="card">
      <div class="section-title">Calculated Plan</div>
      <div class="status-grid">
        <div class="status-item">Rows<span id="a-rows">--</span></div>
        <div class="status-item">Total Distance<span id="a-dist">--</span></div>
        <div class="status-item">Est. Time<span id="a-time">--</span></div>
        <div class="status-item">Current Row<span id="a-row">--</span></div>
      </div>
      <div class="progress-bar"><div class="progress-fill" id="a-progress"></div></div>
      <div style="text-align:center; margin-top:6px; font-size:.85em;" id="a-coverage">0% Complete</div>
    </div>
    <div class="card">
      <div class="section-title">Status</div>
      <span class="pill" id="a-obs-pill">Path Clear</span>
      <span class="pill" id="a-pump-pill">Pump OFF</span>
      <div class="rec-box" id="a-rec" style="margin-top:10px;">--</div>
    </div>
  </div>

</div>
<footer>AGRI Robot &copy; Sagar Kushwaha</footer>

<script>
function goPage(p){
  document.getElementById('page-home').classList.add('hidden');
  document.getElementById('page-manual').classList.add('hidden');
  document.getElementById('page-auto').classList.add('hidden');
  document.getElementById('page-' + p).classList.remove('hidden');
  fetch('/control?mode=' + (p === 'manual' ? 'manual' : p === 'auto' ? 'auto' : 'home'));
}

function cmd(c){ fetch('/control?cmd=' + c); }

/* ---- Horn: hold to sound ---- */
let hornHeld = false;
function hornDown(e){
  e.preventDefault();
  if(hornHeld) return;
  hornHeld = true;
  document.getElementById('hornBtn').classList.add('pressed');
  cmd('hornon');
}
function hornUp(e){
  e.preventDefault();
  if(!hornHeld) return;
  hornHeld = false;
  document.getElementById('hornBtn').classList.remove('pressed');
  cmd('hornoff');
}

function setSpeed(v){
  document.getElementById('speedVal').innerText = v;
  fetch('/control?cmd=speed&val=' + v);
}

function startAuto(){
  const L = document.getElementById('fLength').value;
  const W = document.getElementById('fWidth').value;
  const S = document.getElementById('fSpacing').value;
  if(!L || !W || !S || parseFloat(S) > parseFloat(W)){ alert('Please enter valid field dimensions (spacing must be <= width).'); return; }
  fetch('/autostart?length=' + L + '&width=' + W + '&spacing=' + S);
}

function stopAuto(){ fetch('/autostop'); }

function refreshData(){
  fetch('/data').then(r => r.json()).then(d => {
    document.getElementById('h-wifi').innerText = 'AP Active';
    document.getElementById('h-ip').innerText = d.ip;
    document.getElementById('h-system').innerText = d.estop ? 'E-STOP' : 'Ready';

    document.getElementById('d-temp').innerText = (d.dhtError ? '--' : d.temperature) + ' C';
    document.getElementById('d-hum').innerText  = (d.dhtError ? '--' : d.humidity) + ' %';
    document.getElementById('d-soil').innerText = d.soil + ' %';
    document.getElementById('d-dist').innerText = d.distance + ' cm';
    document.getElementById('d-obs').innerText  = d.obstacle ? 'Obstacle!' : 'Path Clear';
    document.getElementById('d-motion').innerText = d.motion;
    document.getElementById('d-pump').innerText = d.pump ? 'ON' : 'OFF';
    document.getElementById('d-horn').innerText = d.horn ? 'Sounding' : 'Ready';
    document.getElementById('d-rec').innerText = d.rec;

    document.getElementById('m-temp').innerText = (d.dhtError ? '--' : d.temperature) + ' C';
    document.getElementById('m-hum').innerText  = (d.dhtError ? '--' : d.humidity) + ' %';
    document.getElementById('m-soil').innerText = d.soil + ' %';
    document.getElementById('m-dist').innerText = d.distance + ' cm';
    document.getElementById('m-rec').innerText = d.rec;

    document.getElementById('a-rows').innerText = d.totalRows;
    document.getElementById('a-dist').innerText = d.totalDistance + ' m';
    document.getElementById('a-time').innerText = d.estTime + ' min';
    document.getElementById('a-row').innerText = d.currentRow + ' / ' + d.totalRows;
    document.getElementById('a-progress').style.width = d.coverage + '%';
    document.getElementById('a-coverage').innerText = d.coverage + '% Complete';
    document.getElementById('a-obs-pill').innerText = d.obstacle ? 'Obstacle Detected' : 'Path Clear';
    document.getElementById('a-obs-pill').className = d.obstacle ? 'pill bad' : 'pill';
    document.getElementById('a-pump-pill').innerText = d.pump ? 'Pump ON' : 'Pump OFF';
    document.getElementById('a-rec').innerText = d.rec;
  }).catch(()=>{});
}
setInterval(refreshData, 1000);
refreshData();
</script>
</body>
</html>
)HTML";
  return html;
}
