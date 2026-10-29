#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ===================== CONFIGURATION =====================

// WiFi network created by the ESP32 (password must be at least 8 characters)
const char* AP_SSID = "RBot";
const char* AP_PASS = "abcd1234";

// Fixed IP of the Access Point (use any address you like).
IPAddress AP_IP(192, 168, 10, 1);       // ESP32 address 
IPAddress AP_GATEWAY(192, 168, 10, 1);  // usually the same as the AP address
IPAddress AP_SUBNET(255, 255, 255, 0);

// Pins of the four 180° servos
const int SERVO_PINS[4] = {13, 14, 27, 26};

// Pins of the two continuous rotation servos
const int MOTOR_LEFT_PIN  = 25;
const int MOTOR_RIGHT_PIN = 33;

// Pulse width of the 180° servos in microseconds (typical SG90 / MG90S: 500-2400)
const int PULSE_MIN_US = 500;
const int PULSE_MAX_US = 2400;

// --- Continuous rotation servos ---
// These servos are driven by pulse width, NOT by angle:
//   ~1500 us = stopped, shorter = one direction, longer = the other direction.
// STOP_*_US is the pulse at which each servo stands still. If a servo keeps
// creeping when stopped, tweak it (e.g. 1480, 1490, 1510, 1520) or turn the
// small trim potentiometer on the servo with a screwdriver until it stops.
const int STOP_LEFT_US  = 1500;
const int STOP_RIGHT_US = 1500;

// How far the pulse moves away from the stop value at 100 % speed
// (500 -> 1000-2000 us, the standard range for continuous servos).
const int SPEED_RANGE_US = 500;

// Pulse limits used when attaching the continuous servos
// (wide enough to leave room for trimming)
const int MOTOR_PULSE_MIN_US = 500;
const int MOTOR_PULSE_MAX_US = 2500;

// Since the motors are mounted mirrored, one of them must spin the opposite
// way for both to move forward. If the robot spins in place when pressing
// "Forward", flip one of these values. If it drives backward, flip both.
const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = true;

// Maximum speed (0-100 %)
const int SPEED = 100;

// When turning, the motor on the side you turn towards keeps this % of the
// speed. 0 = it stops completely, 50 = half speed.
const int TURN_FACTOR = 30;

// Safety: if no drive command is received within this time (e.g. the
// connection drops while a button is held), the motors stop.
const unsigned long COMMAND_TIMEOUT_MS = 400;

// ===================== GLOBALS =====================

WebServer server(80);

Servo servos[4];
Servo motorLeft;
Servo motorRight;

unsigned long lastCommandTime = 0;
bool isMoving = false;

// ===================== WEB PAGE =====================

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>ESP32 Control</title>
<style>
  :root{
    --bg:#d9dee3; --panel:#f4f6f8; --ink:#1b2733; --muted:#566573;
    --accent:#ff6a00; --line:#b4bec8;
  }
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
  html,body{margin:0;background:var(--bg);color:var(--ink);
    font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
  main{max-width:440px;margin:0 auto;padding:16px 16px 24px;
    display:flex;flex-direction:column;gap:16px}
  section{background:var(--panel);border:1px solid var(--line);
    border-radius:14px;padding:14px 16px 16px}
  h2{margin:0 0 6px;font-size:16px;font-weight:650}

  /* Sliders */
  .row{display:grid;grid-template-columns:62px 1fr 46px;align-items:center;gap:10px}
  .row label{font-size:15px}
  .row output{text-align:right;font-weight:650;font-variant-numeric:tabular-nums}
  input[type=range]{-webkit-appearance:none;appearance:none;width:100%;
    height:44px;margin:0;background:transparent}
  input[type=range]::-webkit-slider-runnable-track{height:6px;border-radius:3px;background:var(--ink)}
  input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:30px;height:30px;
    margin-top:-12px;border-radius:50%;background:var(--accent);
    border:3px solid var(--panel);box-shadow:0 0 0 1px var(--ink)}
  input[type=range]::-moz-range-track{height:6px;border-radius:3px;background:var(--ink)}
  input[type=range]::-moz-range-thumb{box-sizing:border-box;width:30px;height:30px;
    border-radius:50%;background:var(--accent);
    border:3px solid var(--panel);box-shadow:0 0 0 1px var(--ink)}
  input[type=range]:focus-visible{outline:2px solid var(--accent);outline-offset:2px;border-radius:6px}

  /* Drive buttons */
  .pad{display:grid;grid-template-columns:repeat(3,1fr);grid-template-rows:repeat(3,88px);
    gap:10px;max-width:330px;margin:12px auto 0}
  .pad button{display:flex;flex-direction:column;align-items:center;justify-content:center;gap:2px;
    border:0;border-radius:16px;background:var(--ink);color:#fff;
    font:inherit;cursor:pointer;
    touch-action:none;user-select:none;-webkit-user-select:none;-webkit-touch-callout:none;
    transition:background .08s,transform .08s}
  .pad button .g{font-size:28px;line-height:1}
  .pad button .t{font-size:13px}
  .pad button.on{background:var(--accent);transform:scale(.96)}
  .pad button:focus-visible{outline:3px solid var(--accent);outline-offset:2px}
  .up{grid-area:1/2} .left{grid-area:2/1} .right{grid-area:2/3} .down{grid-area:3/2}

  #status{min-height:20px;margin:0;text-align:center;font-size:14px;color:#b3261e}
</style>
</head>
<body>
<main>
  <section>
    <h2>180° servos</h2>
    <div id="sliders"></div>
  </section>

  <section>
    <h2>Drive</h2>
    <div class="pad">
      <button class="up"    data-dir="forward"  aria-label="Forward"><span class="g">&#9650;</span><span class="t">Forward</span></button>
      <button class="left"  data-dir="left"     aria-label="Left"><span class="g">&#9664;</span><span class="t">Left</span></button>
      <button class="right" data-dir="right"    aria-label="Right"><span class="g">&#9654;</span><span class="t">Right</span></button>
      <button class="down"  data-dir="backward" aria-label="Backward"><span class="g">&#9660;</span><span class="t">Backward</span></button>
    </div>
  </section>

  <p id="status" role="status"></p>
</main>

<script>
  const statusEl = document.getElementById('status');
  let connected = true;

  // Sends a request to the ESP32 and shows a warning if it does not answer
  function sendRequest(url){
    fetch(url, {cache:'no-store'})
      .then(() => { if(!connected){ connected = true; statusEl.textContent = ''; } })
      .catch(() => { if(connected){ connected = false; statusEl.textContent = 'No connection to the ESP32'; } });
  }

  // ---------- Sliders (180° servos) ----------
  const slidersContainer = document.getElementById('sliders');
  const lastSent = [0, 0, 0, 0];

  for(let i = 0; i < 4; i++){
    const row = document.createElement('div');
    row.className = 'row';
    row.innerHTML =
      `<label for="s${i}">Servo ${i+1}</label>` +
      `<input type="range" id="s${i}" min="0" max="180" value="90">` +
      `<output for="s${i}">90°</output>`;
    slidersContainer.appendChild(row);

    const slider = row.querySelector('input');
    const output = row.querySelector('output');

    slider.addEventListener('input', () => {
      output.textContent = slider.value + '°';
      const now = Date.now();
      if(now - lastSent[i] > 50){               // throttle to ~20 requests per second
        lastSent[i] = now;
        sendRequest(`/servo?id=${i}&a=${slider.value}`);
      }
    });
    // When released, always send the exact final value
    slider.addEventListener('change', () => sendRequest(`/servo?id=${i}&a=${slider.value}`));
  }

  // ---------- Buttons (continuous rotation servos) ----------
  let activeDir = null, activeButton = null, timer = null;

  function stopDrive(){
    if(!activeDir) return;
    clearInterval(timer);
    timer = null;
    activeButton.classList.remove('on');
    activeDir = null;
    activeButton = null;
    sendRequest('/drive?dir=stop');
  }

  document.querySelectorAll('.pad button').forEach(btn => {
    const dir = btn.dataset.dir;

    btn.addEventListener('pointerdown', e => {
      e.preventDefault();
      stopDrive();
      btn.setPointerCapture(e.pointerId);
      activeDir = dir;
      activeButton = btn;
      btn.classList.add('on');
      sendRequest('/drive?dir=' + dir);
      // While the button is held, the command is repeated
      timer = setInterval(() => sendRequest('/drive?dir=' + dir), 150);
    });

    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev =>
      btn.addEventListener(ev, () => { if(activeDir === dir) stopDrive(); })
    );
    btn.addEventListener('contextmenu', e => e.preventDefault());
  });

  // If the page loses focus, stop the motors
  window.addEventListener('blur', stopDrive);
  document.addEventListener('visibilitychange', () => { if(document.hidden) stopDrive(); });
</script>
</body>
</html>
)rawliteral";

// ===================== MOTOR CONTROL =====================

// speed: -100 (backward) ... 0 (stopped) ... +100 (forward)
// Continuous servos are controlled with the pulse width in microseconds.
void setMotor(Servo &motor, int speed, int stopUs, bool invert) {
  if (invert) speed = -speed;
  int pulse = stopUs + (speed * SPEED_RANGE_US) / 100;
  motor.writeMicroseconds(constrain(pulse, MOTOR_PULSE_MIN_US, MOTOR_PULSE_MAX_US));
}

void applyMotors(int left, int right) {
  setMotor(motorLeft,  left,  STOP_LEFT_US,  INVERT_LEFT);
  setMotor(motorRight, right, STOP_RIGHT_US, INVERT_RIGHT);
}

void stopMotors() {
  applyMotors(0, 0);
  isMoving = false;
}

// ===================== HTTP HANDLERS =====================

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

// /servo?id=0..3&a=0..180
void handleServo() {
  if (!server.hasArg("id") || !server.hasArg("a")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }
  int id = server.arg("id").toInt();
  int angle = constrain(server.arg("a").toInt(), 0, 180);

  if (id < 0 || id > 3) {
    server.send(400, "text/plain", "Invalid servo id");
    return;
  }
  servos[id].write(angle);
  server.send(200, "text/plain", "OK");
}

// /drive?dir=forward|backward|left|right|stop
void handleDrive() {
  String dir = server.arg("dir");
  int turnSpeed = SPEED * TURN_FACTOR / 100;

  if (dir == "forward") {
    applyMotors(SPEED, SPEED);
    isMoving = true;
  } else if (dir == "backward") {
    applyMotors(-SPEED, -SPEED);
    isMoving = true;
  } else if (dir == "left") {            // slows down the left motor
    applyMotors(turnSpeed, SPEED);
    isMoving = true;
  } else if (dir == "right") {           // slows down the right motor
    applyMotors(SPEED, turnSpeed);
    isMoving = true;
  } else if (dir == "stop") {
    stopMotors();
  } else {
    server.send(400, "text/plain", "Invalid direction");
    return;
  }

  lastCommandTime = millis();
  server.send(200, "text/plain", "OK");
}

// ===================== SETUP / LOOP =====================

void setup() {
  Serial.begin(115200);

  // PWM timers for the servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // 180° servos -> center position
  for (int i = 0; i < 4; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], PULSE_MIN_US, PULSE_MAX_US);
    servos[i].write(90);
  }

  // Continuous rotation servos -> stopped (1500 us pulse)
  motorLeft.setPeriodHertz(50);
  motorRight.setPeriodHertz(50);
  motorLeft.attach(MOTOR_LEFT_PIN, MOTOR_PULSE_MIN_US, MOTOR_PULSE_MAX_US);
  motorRight.attach(MOTOR_RIGHT_PIN, MOTOR_PULSE_MIN_US, MOTOR_PULSE_MAX_US);
  stopMotors();

  // Access Point with fixed IP
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET)) {
    Serial.println("Failed to configure the AP fixed IP");
  }
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("WiFi network: ");
  Serial.println(AP_SSID);
  Serial.print("Open in your browser: http://");
  Serial.println(WiFi.softAPIP());   // the fixed IP configured above

  // Server routes
  server.on("/", handleRoot);
  server.on("/servo", handleServo);
  server.on("/drive", handleDrive);
  server.begin();
}

void loop() {
  server.handleClient();

  // Safety stop if drive commands stop arriving
  if (isMoving && (millis() - lastCommandTime > COMMAND_TIMEOUT_MS)) {
    stopMotors();
  }
}
