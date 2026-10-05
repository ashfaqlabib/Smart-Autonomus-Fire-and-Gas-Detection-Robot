/*
  =========================================================
  SMART OBSTACLE-AVOIDING ROBOT WITH FIRE & GAS DETECTION
  ESP32 based | WiFi Web Dashboard | SH1106 OLED (U8g2)
  =========================================================

  FIXES IN THIS VERSION:
  - Obstacle stop distance increased (40cm) + active braking
    (not just cutting power) so the robot stops fast instead
    of coasting into whatever it detected.
  - Forward speed ramps up/down smoothly (soft-start) instead
    of jumping straight to full power.
  - Turning now uses a smooth pivot turn (one side forward, the
    other backward, ramped up/down) - needed because gear motors
    don't free-wheel, so a single-side push just drags instead
    of rotating the chassis.
  - If both left and right are blocked, the robot backs up first
    to make room, then re-scans and tries turning again.
  - Gas sensor auto-calibrates its own "normal" baseline at
    startup (with warm-up time) instead of using a fixed
    number that doesn't match your specific sensor.
  - Fire and Gas alerts are now checked independently, so
    both can be shown/blinking on the dashboard and OLED
    at the same time if both are detected.
  - Turn direction alternates when left/right distances are
    roughly equal, instead of always favoring one side.

  Required Libraries (install via Library Manager):
  - U8g2 (by oliver)
  - ESP32Servo
  - DHT sensor library (Adafruit) + Adafruit Unified Sensor
  - WiFi.h and WebServer.h (built-in with ESP32 board package)

  =========================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <ESP32Servo.h>

// ---------------- WIFI CREDENTIALS ----------------
const char* ssid     = "IoT Lab";
const char* password = "bubt1234";

// ---------------- PIN DEFINITIONS -----------------
#define GAS_PIN        34   // MQ-2 analog pin
#define FLAME_PIN      35   // Flame sensor digital pin

#define DHT_PIN        27
#define DHT_TYPE       DHT22

#define TRIG_PIN       5
#define ECHO_PIN       18

#define SERVO_PIN      13

#define ENA            14
#define IN1            26
#define IN2            25
#define ENB            33
#define IN3            32
#define IN4            15

#define BUZZER_PIN     12
#define RED_LED_PIN    4     // Fire alert
#define YELLOW_LED_PIN 2     // Gas alert

// ---------------- TUNABLE SETTINGS -----------------
#define OBSTACLE_DISTANCE_CM   40    // stop this far away (increased for braking distance)
#define EMERGENCY_STOP_CM      15    // absolute last-resort instant stop
#define TURN_TIE_MARGIN_CM     15    // if left/right differ by less than this, alternate direction
#define BLINK_INTERVAL_MS      300   // LED + buzzer blink speed during alerts
#define MOVE_SPEED             130   // top forward speed (reduced further to prevent hard hits)
#define TURN_SPEED             210   // pivot turns need more torque than straight driving
#define BACKUP_SPEED            140   // speed while reversing to find turning room
#define BACKUP_DURATION_MS      500   // how long to reverse when boxed in on both sides
#define TURN_DURATION_MS        750   // how long to hold the pivot turn
#define RAMP_STEP              12    // how fast PWM ramps up/down per loop (smaller = smoother)
#define BRAKE_HOLD_MS          120   // how long the active brake pulse holds

#define GAS_WARMUP_MS          20000 // MQ-2 needs time to stabilize after power-on
#define GAS_CALIB_SAMPLES      50
#define GAS_MARGIN             350   // how far above baseline counts as "gas detected"

// ---------------- GLOBAL OBJECTS -----------------
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
DHT dht(DHT_PIN, DHT_TYPE);
Servo scanServo;
WebServer server(80);

// ---------------- SENSOR STATE (shared with web dashboard) -----------------
struct RobotStatus {
  float distanceCm   = 0;
  int   gasValue     = 0;
  int   gasThreshold = 0;
  bool  fireDetected = false;
  bool  gasDetected  = false;
  float temperature  = 0;
  float humidity     = 0;
  String state       = "Initializing";
} status;

// ---------------- ALARM BLINK STATE (single shared clock) -----------------
unsigned long lastBlinkTime = 0;
bool blinkOn = false;

// ---------------- TURN MEMORY (avoids always turning the same way) -------
bool lastTurnWasLeft = false;

// ---------------- SPEED RAMP STATE (for smooth accel/brake) --------------
int currentSpeed = 0; // ramped forward speed, shared by both sides while driving straight

// =========================================================
//  SETUP
// =========================================================
void setup() {
  Serial.begin(115200);

  pinMode(FLAME_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);

  scanServo.setPeriodHertz(50);
  scanServo.attach(SERVO_PIN, 500, 2400);
  scanServo.write(90);

  dht.begin();

  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  showMessage("Booting...");

  connectWiFi();

  calibrateGasSensor();  // must happen AFTER wifi so user can see IP first, before main loop

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("Web server started.");

  status.state = "Running";
}

// =========================================================
//  MAIN LOOP  (fully non-blocking, checks fire+gas independently)
// =========================================================
void loop() {
  server.handleClient();

  readEnvironmentSensors();

  bool fire = status.fireDetected;
  bool gas  = status.gasDetected;

  if (fire || gas) {
    stopMotors();

    if (fire && gas)      status.state = "FIRE & GAS ALERT!";
    else if (fire)        status.state = "FIRE ALERT!";
    else                  status.state = "GAS ALERT!";

    updateAlarmBlink(fire, gas);
    updateOLED();
    return;
  } else {
    // no alert -> make sure everything is off and blink clock is reset
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    blinkOn = false;
  }

  // ---- Check distance FIRST, before committing to full speed ----
  status.distanceCm = getDistance(90);

  // Emergency stop: something is dangerously close - brake hard, don't even ramp
  if (status.distanceCm > 0 && status.distanceCm < EMERGENCY_STOP_CM) {
    brakeMotors();
    status.state = "Emergency Stop!";
    updateOLED();
    delay(80);
    return;
  }

  if (status.distanceCm > 0 && status.distanceCm < OBSTACLE_DISTANCE_CM) {
    brakeMotors();               // active brake - stops fast, no coasting into the obstacle
    status.state = "Obstacle! Scanning...";
    updateOLED();

    float leftDist  = getDistance(150);
    float rightDist = getDistance(30);
    scanServo.write(90);

    // Treat "no echo" (-1) as a clear/open path
    if (leftDist < 0)  leftDist  = 400;
    if (rightDist < 0) rightDist = 400;

    bool leftBlocked  = leftDist  < OBSTACLE_DISTANCE_CM;
    bool rightBlocked = rightDist < OBSTACLE_DISTANCE_CM;

    if (leftBlocked && rightBlocked) {
      // Boxed in on both sides -> back up first to create room, then
      // let the next loop iteration re-scan and try turning again.
      status.state = "No space - backing up";
      updateOLED();
      moveBackward(BACKUP_DURATION_MS);
      currentSpeed = 0;
    } else {
      bool turnLeftNow;
      if (fabs(leftDist - rightDist) < TURN_TIE_MARGIN_CM) {
        // Both sides roughly equally open -> alternate so the robot
        // doesn't always favor the same direction when it has a choice.
        turnLeftNow = !lastTurnWasLeft;
      } else {
        turnLeftNow = (leftDist > rightDist);
      }
      lastTurnWasLeft = turnLeftNow;

      // Pivot turn: one side forward, other side backward. Gear motors
      // don't free-wheel, so a real pivot (not a single-side push) is
      // what actually turns the chassis reliably.
      status.state = turnLeftNow ? "Turning Left (more space)" : "Turning Right (more space)";
      pivotTurn(turnLeftNow, TURN_DURATION_MS);
      currentSpeed = 0; // reset ramp so forward motion restarts gently
    }
  } else {
    // Path is clear -> smoothly ramp toward full forward speed
    status.state = "Moving Forward";
    rampForward(MOVE_SPEED);
  }

  updateOLED();
  delay(80);
}

// Drives both alert LEDs and the buzzer off ONE shared blink clock,
// so fire-only, gas-only, and fire+gas-together all blink in sync
// without any conflicting timers.
void updateAlarmBlink(bool fire, bool gas) {
  unsigned long now = millis();
  if (now - lastBlinkTime >= BLINK_INTERVAL_MS) {
    lastBlinkTime = now;
    blinkOn = !blinkOn;
  }
  digitalWrite(RED_LED_PIN,    (fire && blinkOn) ? HIGH : LOW);
  digitalWrite(YELLOW_LED_PIN, (gas  && blinkOn) ? HIGH : LOW);
  digitalWrite(BUZZER_PIN,     ((fire || gas) && blinkOn) ? HIGH : LOW);
}

// =========================================================
//  WIFI CONNECTION
// =========================================================
void connectWiFi() {
  showMessage("Connecting WiFi...");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    showMessage("IP: " + WiFi.localIP().toString());
    delay(2000);
  } else {
    Serial.println("\nWiFi connection failed. Continuing offline.");
    showMessage("WiFi Failed");
    delay(1500);
  }
}

// =========================================================
//  GAS SENSOR AUTO-CALIBRATION
//  MQ-2 reads high/unstable right after power-on. This waits
//  for warm-up, then measures YOUR sensor's actual "clean air"
//  baseline and sets the detection threshold relative to it.
// =========================================================
void calibrateGasSensor() {
  showMessage("Gas sensor warming up...");
  Serial.println("Warming up MQ-2 (20s)... keep it away from gas/smoke.");

  unsigned long start = millis();
  while (millis() - start < GAS_WARMUP_MS) {
    server.handleClient(); // keep responsive even during warm-up
    int secondsLeft = (GAS_WARMUP_MS - (millis() - start)) / 1000;
    showMessage("Warming up: " + String(secondsLeft) + "s");
    delay(500);
  }

  long sum = 0;
  for (int i = 0; i < GAS_CALIB_SAMPLES; i++) {
    sum += analogRead(GAS_PIN);
    delay(20);
  }
  int baseline = sum / GAS_CALIB_SAMPLES;
  status.gasThreshold = baseline + GAS_MARGIN;

  Serial.print("Gas baseline: "); Serial.println(baseline);
  Serial.print("Gas threshold set to: "); Serial.println(status.gasThreshold);

  showMessage("Gas calibrated: " + String(baseline));
  delay(1500);
}

// =========================================================
//  SENSOR READING
// =========================================================
void readEnvironmentSensors() {
  status.gasValue = analogRead(GAS_PIN);
  status.gasDetected = status.gasValue > status.gasThreshold;

  status.fireDetected = (digitalRead(FLAME_PIN) == LOW); // most flame sensors: LOW = fire detected

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h)) status.humidity = h;
  if (!isnan(t)) status.temperature = t;

  // Uncomment to watch raw gas readings in Serial Monitor for fine-tuning:
  // Serial.print("Gas raw: "); Serial.print(status.gasValue);
  // Serial.print(" | Threshold: "); Serial.println(status.gasThreshold);
}

// Measure distance with servo pointed at a given angle
float getDistance(int angle) {
  scanServo.write(angle);
  delay(300); // allow servo to settle

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;

  float distance = duration * 0.0343 / 2.0;
  return distance;
}

// =========================================================
//  MOTOR CONTROL (L298N)
// =========================================================

// Smoothly ramps forward speed toward `target` instead of jumping straight
// to full power. Called every loop iteration while the path is clear, so
// the robot gently accelerates rather than lurching forward.
void rampForward(int target) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);

  if (currentSpeed < target) currentSpeed = min(target, currentSpeed + RAMP_STEP);
  else if (currentSpeed > target) currentSpeed = max(target, currentSpeed - RAMP_STEP);

  analogWrite(ENA, currentSpeed);
  analogWrite(ENB, currentSpeed);
}

// Pivot turn: one side drives forward, the other drives backward at the
// same time. Gear motors (TT motors) have too much internal friction to
// free-wheel, so pushing with only one side just drags the idle wheels
// instead of turning the chassis. Driving both sides in opposite
// directions gives enough torque differential to actually rotate.
// Speed ramps up, holds, then ramps down for a smooth (not jerky) turn.
void pivotTurn(bool turnLeft, int durationMs) {
  unsigned long start = millis();
  int speed = 0;

  while (speed < TURN_SPEED) {
    speed = min(TURN_SPEED, speed + RAMP_STEP);
    setPivot(turnLeft, speed);
    delay(15);
  }

  unsigned long holdUntil = start + durationMs;
  while (millis() < holdUntil) {
    server.handleClient(); // keep dashboard responsive during the turn
    setPivot(turnLeft, TURN_SPEED);
    delay(15);
  }

  while (speed > 0) {
    speed = max(0, speed - RAMP_STEP);
    setPivot(turnLeft, speed);
    delay(15);
  }

  brakeMotors();
}

// Sets motor direction pins for a pivot turn at the given speed.
void setPivot(bool turnLeft, int speed) {
  if (turnLeft) {
    // Left side backward, right side forward -> rotates left
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    // Left side forward, right side backward -> rotates right
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  }
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// Reverses straight back for `durationMs` (used when boxed in on both
// sides), with a smooth ramp up/down, then brakes.
void moveBackward(int durationMs) {
  unsigned long start = millis();
  int speed = 0;

  while (speed < BACKUP_SPEED) {
    speed = min(BACKUP_SPEED, speed + RAMP_STEP);
    setBackward(speed);
    delay(15);
  }

  unsigned long holdUntil = start + durationMs;
  while (millis() < holdUntil) {
    server.handleClient();
    setBackward(BACKUP_SPEED);
    delay(15);
  }

  while (speed > 0) {
    speed = max(0, speed - RAMP_STEP);
    setBackward(speed);
    delay(15);
  }

  brakeMotors();
}

void setBackward(int speed) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// Active brake: shorts each motor through the H-bridge (both inputs low
// while enable stays high briefly) so the motor's own back-EMF slows it
// down fast, instead of just cutting power and letting it coast forward
// on momentum into the obstacle.
void brakeMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  delay(BRAKE_HOLD_MS);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  currentSpeed = 0;
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// =========================================================
//  OLED DISPLAY (U8g2 / SH1106)
// =========================================================
void showMessage(String msg) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 12, msg.c_str());
  u8g2.sendBuffer();
}

void updateOLED() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  u8g2.drawStr(0, 10, "Robot Status:");
  u8g2.drawStr(0, 22, status.state.c_str());

  char line[32];

  snprintf(line, sizeof(line), "Dist: %.1f cm", status.distanceCm);
  u8g2.drawStr(0, 34, line);

  snprintf(line, sizeof(line), "Gas: %s", status.gasDetected ? "DETECTED" : "Normal");
  u8g2.drawStr(0, 44, line);

  snprintf(line, sizeof(line), "Fire: %s", status.fireDetected ? "DETECTED" : "Normal");
  u8g2.drawStr(0, 54, line);

  snprintf(line, sizeof(line), "T:%.1fC H:%.0f%%", status.temperature, status.humidity);
  u8g2.drawStr(0, 64, line);

  u8g2.sendBuffer();
}

// =========================================================
//  WEB SERVER HANDLERS
// =========================================================

void handleRoot() {
  String html = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <title>Smart Autonomus Robot Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: 'Segoe UI', Arial, sans-serif;
      background: radial-gradient(circle at top, #1b2735 0%, #090a0f 100%);
      color: #e8eef5;
      min-height: 100vh;
      padding: 24px 16px 60px;
    }
    h1 { text-align: center; font-size: 22px; letter-spacing: 1px; color: #7fd8ff; margin-bottom: 4px; }
    .subtitle { text-align: center; color: #7a8699; font-size: 12px; margin-bottom: 24px; }
    .grid { max-width: 480px; margin: 0 auto; display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
    .card { grid-column: span 2; background: linear-gradient(145deg, #182030, #10151f); border: 1px solid #263042; border-radius: 16px; padding: 18px; box-shadow: 0 4px 18px rgba(0,0,0,0.35); text-align: center; }
    .card.half { grid-column: span 1; }
    .label { color: #8ea0b8; font-size: 12px; text-transform: uppercase; letter-spacing: 1px; margin-bottom: 6px; }
    .value { font-size: 26px; font-weight: 700; }
    .state-badge { display: inline-block; padding: 8px 20px; border-radius: 999px; font-size: 16px; font-weight: 600; background: #1e3a2f; color: #6ee7a8; transition: all .3s ease; }
    .state-badge.alert { background: #3a1e1e; color: #ff7a7a; animation: pulse 0.9s infinite; }
    @keyframes pulse { 0% { box-shadow: 0 0 0 0 rgba(255,90,90,0.5); } 70% { box-shadow: 0 0 0 12px rgba(255,90,90,0); } 100% { box-shadow: 0 0 0 0 rgba(255,90,90,0); } }
    .ok { color: #6ee7a8; } .bad { color: #ff7a7a; }
    .bar-track { width: 100%; height: 8px; background: #263042; border-radius: 6px; margin-top: 10px; overflow: hidden; }
    .bar-fill { height: 100%; border-radius: 6px; background: linear-gradient(90deg, #4fd1c5, #7fd8ff); transition: width .4s ease; }
    .dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%; margin-right: 6px; background: #6ee7a8; }
    .dot.bad { background: #ff5252; box-shadow: 0 0 8px #ff5252; }
    .badges { display: flex; gap: 8px; justify-content: center; margin-top: 10px; flex-wrap: wrap; }
    .mini-badge { font-size: 12px; padding: 4px 10px; border-radius: 999px; background: #263042; color: #9fb0c5; }
    .mini-badge.on-fire { background: #3a1e1e; color: #ff7a7a; }
    .mini-badge.on-gas { background: #3a3620; color: #ffd54f; }
    footer { text-align: center; color: #55617a; font-size: 11px; margin-top: 26px; }
  </style>
</head>
<body>
  <h1>SMART AUTONOMUS ROBOT DASHBOARD</h1>
  <div class="subtitle" id="conn"><span class="dot" id="connDot"></span>Live monitoring</div>

  <div class="grid">
    <div class="card">
      <div class="label">Current State</div>
      <div id="stateBadge" class="state-badge">Loading...</div>
      <div class="badges">
        <span class="mini-badge" id="fireBadge">Fire: Normal</span>
        <span class="mini-badge" id="gasBadge">Gas: Normal</span>
      </div>
    </div>

    <div class="card half">
      <div class="label">Distance</div>
      <div class="value" id="distance">-- cm</div>
      <div class="bar-track"><div class="bar-fill" id="distBar" style="width:0%"></div></div>
    </div>

    <div class="card half">
      <div class="label">Gas Level</div>
      <div class="value" id="gasVal">--</div>
      <div class="bar-track"><div class="bar-fill" id="gasBar" style="width:0%"></div></div>
    </div>

    <div class="card half">
      <div class="label">Temperature</div>
      <div class="value" id="temp">-- °C</div>
    </div>

    <div class="card half">
      <div class="label">Humidity</div>
      <div class="value" id="hum">-- %</div>
    </div>
  </div>

  <footer>Four Caders Team &middot; Auto-refresh every 1s</footer>

  <script>
    async function refresh() {
      try {
        const res = await fetch('/data');
        const d = await res.json();

        document.getElementById('connDot').className = 'dot';
        document.getElementById('conn').lastChild.textContent = 'Live monitoring';

        const badge = document.getElementById('stateBadge');
        badge.innerText = d.state;
        badge.className = 'state-badge' + ((d.fireDetected || d.gasDetected) ? ' alert' : '');

        const fireBadge = document.getElementById('fireBadge');
        fireBadge.innerText = 'Fire: ' + (d.fireDetected ? 'DETECTED' : 'Normal');
        fireBadge.className = 'mini-badge' + (d.fireDetected ? ' on-fire' : '');

        const gasBadge = document.getElementById('gasBadge');
        gasBadge.innerText = 'Gas: ' + (d.gasDetected ? 'DETECTED' : 'Normal');
        gasBadge.className = 'mini-badge' + (d.gasDetected ? ' on-gas' : '');

        document.getElementById('distance').innerText = d.distance.toFixed(1) + ' cm';
        const distPct = Math.max(0, Math.min(100, (d.distance / 100) * 100));
        document.getElementById('distBar').style.width = distPct + '%';

        document.getElementById('gasVal').innerText = d.gasValue;
        const gasPct = Math.max(0, Math.min(100, (d.gasValue / 4095) * 100));
        document.getElementById('gasBar').style.width = gasPct + '%';

        document.getElementById('temp').innerText = d.temperature.toFixed(1) + ' °C';
        document.getElementById('hum').innerText = d.humidity.toFixed(1) + ' %';
      } catch (e) {
        document.getElementById('connDot').className = 'dot bad';
        document.getElementById('conn').lastChild.textContent = 'Connection lost...';
      }
    }
    setInterval(refresh, 1000);
    refresh();
  </script>
</body>
</html>
)HTML";
  server.send(200, "text/html", html);
}

void handleData() {
  String json = "{";
  json += "\"state\":\"" + status.state + "\",";
  json += "\"distance\":" + String(status.distanceCm) + ",";
  json += "\"gasValue\":" + String(status.gasValue) + ",";
  json += "\"gasDetected\":" + String(status.gasDetected ? "true" : "false") + ",";
  json += "\"fireDetected\":" + String(status.fireDetected ? "true" : "false") + ",";
  json += "\"temperature\":" + String(status.temperature) + ",";
  json += "\"humidity\":" + String(status.humidity);
  json += "}";
  server.send(200, "application/json", json);
}
