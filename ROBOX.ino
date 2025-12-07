/* NovaX_Final.ino
   Single-file firmware for NOVA X (ESP8266)
   - AP mode web UI
   - Radar UI integrated (waves + scan line)
   - Ultrasonic servo scan (non-blocking)
   - Obstacle-avoid auto behavior (reverse, scan, choose direction)
   - Motor control (MX1508 style pin usage)
   - Pin mapping: set at top
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>
#include <pgmspace.h>

// =========================
// === USER CONFIGURE ====
// =========================
// Pins (change here if your wiring differs)
#define SERVO_PIN  D1    // scan servo
#define IN1        D7   // left motor forward (or A+)
#define IN2        D8    // left motor backward (or A-)
#define IN3        D5    // right motor forward (or B+)
#define IN4        D6    // right motor backward (or B-)
#define TRIG_PIN   D0
#define ECHO_PIN   D2
#define LED_LEFT   D3
#define LED_RIGHT  D4

// AP credentials
const char* AP_SSID = "NovaX-AP";
const char* AP_PASS = "novax1234";

// Behavior tuning
const long AUTO_CHECK_MS = 200;
const int OBSTACLE_THRESHOLD_CM = 90; // detect obstacle if below this
const int REVERSE_TARGET_CM = 40;     // reverse until >= this
const int SERVO_STEP_MS = 120;        // servo step time in scan
const int SERVO_STEP_DEG = 30;        // sweep stepping (e.g. -90, -60, -30, 0, 30, 60, 90)
const int TURN_PWM = 1023;             // pwm used during turning decision
const int FORWARD_PWM = 700;          // forward cruise pwm (0..1023)
const unsigned long TURN_DURATION_MS = 5000;
const unsigned long FORWARD_AFTER_TURN_MS = 1000;
const unsigned long REVERSE_TIMEOUT_MS = 7000;

// =========================
// === PROGMEM HTML UI ====
// =========================
// This is the "A" radar UI (semicircle waves, scanline). Kept compact but complete.
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Nova X — RoboCar</title>
<style>
:root{--bg:#071418;--panel:#0d2628;--accent:#1798ff;--accent2:#14b8a6;--danger:#ef4444;--muted:#8da3a6}
html,body{height:100%;margin:0;background:linear-gradient(180deg,#041017,#02131d);font-family:Arial;color:#e6f6f8}
.container{max-width:1100px;margin:10px auto;padding:10px;display:grid;grid-template-columns:1fr 1fr 360px;gap:10px}
.card{background:var(--panel);padding:12px;border-radius:12px;box-shadow:0 6px 18px rgba(0,0,0,0.35)}
.title{font-size:16px;margin-bottom:8px;color:#dff}
.centerTitle{text-align:center}

/* joystick / arrows */
.arrowBtn{width:74px;height:56px;border-radius:10px;border:0;background:var(--accent2);color:#fff;font-size:24px;cursor:pointer}
.arrowRow{display:flex;gap:8px;justify-content:center;margin:8px 0}

/* draw canvas */
#drawCanvas{width:300px;height:220px;background:#081722;border-radius:10px;display:none}

/* radar semicircle */
#radar{width:200px;height:100px;background:radial-gradient(circle at bottom,#0d2f3a,#051b22);border:3px solid rgba(23,152,255,0.35);border-bottom:none;border-radius:200px 200px 0 0;position:relative;overflow:hidden;margin:0 auto}
.wave{position:absolute;width:20px;height:20px;border:2px solid rgba(23,152,255,0.35);border-radius:50%;bottom:0;left:50%;transform:translateX(-50%);animation:waveAnim 2.2s ease-out infinite}
.wave:nth-child(2){animation-delay:0.6s} .wave:nth-child(3){animation-delay:1.2s}
@keyframes waveAnim{0%{width:0;height:0;opacity:1}100%{width:500px;height:500px;opacity:0}}
#scanLine{position:absolute;left:50%;bottom:0;width:4px;height:200px;background:rgba(23,152,255,0.9);border-radius:3px;transform-origin:bottom center;display:none}
.dot{width:10px;height:10px;border-radius:50%;background:#ff5252;position:absolute;display:none}

/* distance text inside radar */
#radDist{position:absolute;bottom:6px;left:50%;transform:translateX(-50%);font-size:13px;color:#bfefff}

/* buttons */
.btn{padding:8px 12px;border-radius:8px;border:0;background:var(--accent);color:#fff;cursor:pointer}
.btn.ghost{background:rgba(255,255,255,0.06)}
.row{display:flex;gap:8px;justify-content:center;margin-top:10px}

/* small responsive */
@media (max-width:900px){.container{grid-template-columns:1fr;}}
</style>
</head>
<body>
<div class="container">
  <div class="card">
    <div class="title">Controls</div>
    <div style="text-align:center">
      <button id="up" class="arrowBtn">&#9650;</button>
      <div class="arrowRow">
        <button id="left" class="arrowBtn">&#9664;</button>
        <button id="stop" class="arrowBtn" style="background:var(--danger);font-size:20px">&#9632;</button>
        <button id="right" class="arrowBtn">&#9654;</button>
      </div>
      <button id="down" class="arrowBtn">&#9660;</button>
    </div>

    <hr style="border-color:rgba(255,255,255,0.04);margin:10px 0">

    <canvas id="drawCanvas"></canvas>
    <div class="row" style="margin-top:8px">
      <button id="toggleDraw" class="btn ghost">Draw Mode</button>
      <button id="clearDraw" class="btn ghost">Clear</button>
      <button id="sendPath" class="btn">Send Path</button>
    </div>
  </div>

  <div class="card">
    <div class="centerTitle"><h2 style="margin:6px 0">NOVA X</h2></div>
    <div style="display:flex;justify-content:space-between;align-items:center">
      <div>Battery: <span id="battery">--%</span></div>
      <div>Status: <span id="conn">--</span></div>
    </div>

    <div class="row" style="margin-top:8px">
      <button id="modeToggle" class="btn">MANUAL</button>
      <button id="stopBtn" class="btn" style="background:var(--danger)">STOP</button>
    </div>

    <div style="margin-top:12px">
      <div class="title">Rotation</div>
      <div class="row">
        <button id="rotL" class="btn ghost">Rotate L</button>
        <button id="rotR" class="btn ghost">Rotate R</button>
        <button id="rot360" class="btn">360°</button>
      </div>
    </div>

    <div style="margin-top:12px">
      <div class="title">LEDs</div>
      <div class="row">
        <button class="ledBtn btn ghost" data-effect="off">Off</button>
        <button class="ledBtn btn ghost" data-effect="blink">Blink</button>
        <button class="ledBtn btn ghost" data-effect="warn">Warn</button>
        <button class="ledBtn btn ghost" data-effect="pulse">Pulse</button>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="title">Radar</div>
    <div id="radar">
      <div class="wave"></div><div class="wave"></div><div class="wave"></div>
      <div id="scanLine"></div>
      <div id="dotL" class="dot"></div>
      <div id="dotF" class="dot"></div>
      <div id="dotR" class="dot"></div>
      <div id="radDist">-- cm</div>
    </div>
    <div class="row" style="margin-top:8px">
      <button id="scan" class="btn">Scan</button>
      <button id="sonarToggle" class="btn ghost">Sonar ON</button>
    </div>
  </div>
</div>

<script>
/* helpers */
const api = (u, opts) => fetch(u, opts).catch(()=>{});
function hold(id, cmd){
  const el=document.getElementById(id);
  let iv=null;
  el.addEventListener('pointerdown',e=>{
    el.setPointerCapture(e.pointerId);
    api('/move?d='+cmd);
    iv=setInterval(()=>api('/move?d='+cmd),130);
  });
  el.addEventListener('pointerup',()=>{ clearInterval(iv); api('/move?d=S'); });
  el.addEventListener('pointerleave',()=>{ clearInterval(iv); api('/move?d=S'); });
}
hold('up','F'); hold('down','B'); hold('left','L'); hold('right','R');
document.getElementById('stop').onclick = ()=>api('/move?d=S');
document.getElementById('stopBtn').onclick = ()=>api('/stop');

/* mode toggle */
document.getElementById('modeToggle').onclick = async ()=>{
  const cur = document.getElementById('modeToggle').innerText;
  const set = (cur==='MANUAL') ? 'auto' : 'manual';
  await api('/mode?set='+set);
  document.getElementById('modeToggle').innerText = (set==='manual') ? 'MANUAL' : 'AUTO';
};

/* rotation */
document.getElementById('rotL').onclick = ()=>api('/rotate?dir=left');
document.getElementById('rotR').onclick = ()=>api('/rotate?dir=right');
document.getElementById('rot360').onclick = ()=>api('/rotate?dir=360');

/* LEDs */
document.querySelectorAll('.ledBtn').forEach(b=>{
  b.onclick=()=>{
    document.querySelectorAll('.ledBtn').forEach(x=>x.style.opacity=1);
    b.style.opacity=0.6;
    api('/led?effect='+b.dataset.effect);
  };
});

/* draw canvas */
const dc = document.getElementById('drawCanvas');
const toggleDraw = document.getElementById('toggleDraw');
let drawPoints = [];
function initDraw(){
  dc.width = dc.clientWidth; dc.height = dc.clientHeight;
  const ctx = dc.getContext('2d');
  ctx.fillStyle='#081722'; ctx.fillRect(0,0,dc.width,dc.height);
  ctx.strokeStyle='#14b8a6'; ctx.lineWidth=3; ctx.lineCap='round';
  drawPoints=[]; let drawing=false;
  function pos(e){ const r=dc.getBoundingClientRect(); const t=(e.touches?e.touches[0]:e); return {x:t.clientX-r.left,y:t.clientY-r.top}; }
  dc.onpointerdown = e=>{ drawing=true; const p=pos(e); if (drawPoints.length < 5000) {
    drawPoints.push([p.x/dc.width, p.y/dc.height]);
} ctx.beginPath(); ctx.moveTo(p.x,p.y); };
  dc.onpointermove = e=>{ if(!drawing) return; const p=pos(e); if (drawPoints.length < 5000) {
    drawPoints.push([p.x/dc.width, p.y/dc.height]);
} ctx.lineTo(p.x,p.y); ctx.stroke(); };
  dc.onpointerup = ()=>{ drawing=false; ctx.beginPath(); };
  dc.onpointerleave = (e)=>{
  drawing = false;
  ctx.beginPath();
  e.preventDefault();
};
dc.onpointercancel = ()=>{
  drawing = false;
  ctx.beginPath();
};
}
toggleDraw.onclick = ()=>{
  if(dc.style.display==='block'){
    dc.style.display='none';
    toggleDraw.innerText='Draw Mode';
  } else {
    dc.style.display='block';
    setTimeout(initDraw, 50);   // small delay prevents draw break
    toggleDraw.innerText='Joystick Mode';
  }
};
document.getElementById('clearDraw').onclick = ()=>{ if(dc.getContext){ const ctx=dc.getContext('2d'); ctx.clearRect(0,0,dc.width,dc.height); drawPoints=[]; } };
document.getElementById('sendPath').onclick = async ()=>{
  if(!drawPoints || drawPoints.length<2) return alert('Draw path first');
  const s = drawPoints.map(p=>p[0].toFixed(3)+','+p[1].toFixed(3)).join(';');
  await api('/path',{method:'POST',body:s});
  alert('Path sent');
};

/* radar scan + polling */
const scanBtn = document.getElementById('scan');
const scanLine = document.getElementById('scanLine');
let scanPoll = null;
scanBtn.onclick = async ()=>{
  scanLine.style.display='block';
  scanBtn.disabled = true;
  scanBtn.innerText='Scanning…';
  await api('/scan');
  if(!scanPoll) scanPoll = setInterval(pollScan,120);
};
document.getElementById('sonarToggle').onclick = async ()=>{
  const on = document.getElementById('sonarToggle').innerText.includes('ON');
  const m = on ? 'off' : 'on';
  await api('/sonarToggle?mode='+m);
  document.getElementById('sonarToggle').innerText = on ? 'Sonar OFF' : 'Sonar ON';
};

async function pollScan(){
  try{
    const r = await fetch('/scanResult'); const j = await r.json();
    document.getElementById('radDist').innerText = j.front + ' cm';
    placeDot('dotL', j.left, -60);
    placeDot('dotF', j.front, 0);
    placeDot('dotR', j.right, 60);
    if(j.done==1 || j.done===true){
      clearInterval(scanPoll); scanPoll=null;
      scanBtn.disabled=false; scanBtn.innerText='Scan';
      scanLine.style.display='none';
    }
  }catch(e){}
}

function placeDot(id, dist, ang){
  const el = document.getElementById(id);
  if(!el) return;
  if(!dist || dist<=0 || dist>400){ el.style.display='none'; return; }
  const maxR = 80; // pixel radius
  const capped = Math.min(dist,300);
  const r = (capped/300)*maxR;
  const t = ang*Math.PI/180;
  const cx = 100, cy = 5; // origin inside radar
  const x = cx + r*Math.cos(t) - 5;
  const y = r*Math.sin(t) + 0;
  el.style.left = x + 'px';
  el.style.bottom = y + 'px';
  el.style.display = 'block';
}

/* status polling */
async function pollStatus(){
  try{
    const r = await fetch('/status'); const j = await r.json();
    document.getElementById('radDist').innerText = j.distance + ' cm';
    document.getElementById('battery').innerText = j.batteryPct + '%';
    document.getElementById('conn').innerText = 'Connected';
    document.getElementById('modeToggle').innerText = (j.mode==0)?'AUTO':'MANUAL';
  }catch(e){
    document.getElementById('conn').innerText = 'No connection';
  }
}
setInterval(pollStatus,900); pollStatus();
</script>
</body>
</html>
)rawliteral";

// =========================
// === GLOBAL OBJECTS ====
// =========================
ESP8266WebServer server(80);
Servo scanServo;

// scan state machine
enum ScanPhase { IDLE, SWEEP };
volatile ScanPhase scanPhase = IDLE;
volatile bool scanningActive = false;
volatile bool scanResultReady = false;
int scanLeft = 400, scanFront = 400, scanRight = 400;
unsigned long lastServoStep = 0;
int sweepAngle = -90;
int sweepDir = 1; // +1 increasing

// auto state
enum AutoState { A_IDLE, A_DETECTED, A_REVERSING, A_SCANNING, A_TURNING, A_FORWARDING };
AutoState autoState = A_IDLE;
unsigned long autoStateAt = 0;

// sonar enable
volatile bool sonarEnabled = true;

// =========================
// === HELPER FUNCTIONS ====
// =========================
void pinSetup(){
  pinMode(IN1, OUTPUT); analogWrite(IN1, 0);
  pinMode(IN2, OUTPUT); analogWrite(IN2, 0);
  pinMode(IN3, OUTPUT); analogWrite(IN3, 0);
  pinMode(IN4, OUTPUT); analogWrite(IN4, 0);
  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_LEFT, OUTPUT); digitalWrite(LED_LEFT, LOW);
  pinMode(LED_RIGHT, OUTPUT); digitalWrite(LED_RIGHT, LOW);
}

// motorWrite: pwmVal positive -> forward pin pwm, negative -> backward pin pwm
void motorWrite(int pinF, int pinB, int pwmVal){
  if (pwmVal > 0){
    analogWrite(pinF, pwmVal);
    analogWrite(pinB, 0);
  } else if (pwmVal < 0){
    analogWrite(pinF, 0);
    analogWrite(pinB, -pwmVal);
  } else {
    analogWrite(pinF, 0);
    analogWrite(pinB, 0);
  }
}

// high-level mix: x = turn (-1..1), y = forward/back (-1..1)
void motorMix(float x, float y){
  float left = y + x;
  float right = y - x;
  float maxv = max(abs(left), abs(right));
  if (maxv > 1.0) { left /= maxv; right /= maxv; }
  int pwmL = (int)(left * 1023.0);
  int pwmR = (int)(right * 1023.0);
  motorWrite(IN1, IN2, pwmL);
  motorWrite(IN3, IN4, pwmR);
}

void stopCar(){ motorWrite(IN1, IN2, 0); motorWrite(IN3, IN4, 0); }

// ultrasonic read (blocking pulseIn with timeout)
long readUltrasonicCM(){
  if(!sonarEnabled) return 400;
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 20000); // timeout 20ms
  long cm = (dur * 0.034) / 2;
  if (cm <= 0 || cm > 400) return 400;
  return cm;
}

// =========================
// === SCAN STATE MACHINE ==
// =========================
void startScan(){
  if (scanPhase != IDLE) return;
  scanPhase = SWEEP;
  scanningActive = true;
  scanResultReady = false;
  scanLeft = scanFront = scanRight = 400;
  sweepAngle = -90; sweepDir = 1;
  scanServo.attach(SERVO_PIN);
  lastServoStep = 0;
}

void stepScan(){
  if (scanPhase == IDLE) return;
  unsigned long now = millis();
  if (now - lastServoStep < SERVO_STEP_MS) return;
  lastServoStep = now;

  // write servo: convert sweepAngle (-90..90) to 0..180
  int writeAngle = sweepAngle + 90;
  scanServo.write(writeAngle);
  delay(30); // small settling time for servo

  // read distance at specific key angles: -90, 0, 90
  if (sweepAngle == -90) scanLeft = readUltrasonicCM();
  else if (sweepAngle == 0) scanFront = readUltrasonicCM();
  else if (sweepAngle == 90) scanRight = readUltrasonicCM();

  // step
  sweepAngle += sweepDir * SERVO_STEP_DEG;
  if (sweepAngle > 90 && scanPhase == SWEEP){
    // finished sweep back (we stepped past -> finish)
    scanServo.write(90);
    delay(80);
    scanServo.detach();
    scanResultReady = true;
    scanningActive = false;
    scanPhase = IDLE;
    // ensure servo returns to center
    sweepAngle = -90;
  }
}

// =========================
// === AUTO STATE MACHINE ==
// =========================
void startReverse(){
  autoState = A_REVERSING;
  autoStateAt = millis();
  // start reversing with moderate PWM
  motorWrite(IN1, IN2, -1023);
  motorWrite(IN3, IN4, -1023);
}

void performDecisionAfterScan(){
  int L = scanLeft;
  int R = scanRight;   // inner wheel

  if (R>L) {
    // turn LEFT: only RIGHT motors forward
    startReverse();
    delay(1250);
    motorWrite(IN1, IN2, 1023);     // left motors stopped
    motorWrite(IN3, IN4, 0);   // right motors forward
  }
  else{
    startReverse();
    delay(1250);
    // turn RIGHT: only LEFT motors forward
    motorWrite(IN1, IN2, 0);   // left motors forward
    motorWrite(IN3, IN4, 1023);     // right motors stopped
  }

  autoState = A_TURNING;
  autoStateAt = millis();
}
void stepAuto(){
  if (autoState == A_IDLE){
    static unsigned long lastCheck = 0;
    unsigned long now = millis();
    if (now - lastCheck < AUTO_CHECK_MS) return;
    lastCheck = now;
    long d = readUltrasonicCM();
    if (d < OBSTACLE_THRESHOLD_CM){
      stopCar();
      autoState = A_DETECTED;
      autoStateAt = now;
    } else {
      // cruise forward slowly
      motorMix(0.0, 0.6);
    }
  }
  else if (autoState == A_DETECTED){
    // begin reversing
    startReverse();
  }
  else if (autoState == A_REVERSING){
    long cur = readUltrasonicCM();
    unsigned long now = millis();
    if (cur >= REVERSE_TARGET_CM){
      stopCar();
      delay(80);
      if (scanPhase == IDLE) startScan();
      autoState = A_SCANNING;
      autoStateAt = now;
    } else {
      if (now - autoStateAt > REVERSE_TIMEOUT_MS){
        stopCar();
        if (scanPhase == IDLE) startScan();
        autoState = A_SCANNING;
        autoStateAt = now;
      } else {
        // continue reversing (PWM already applied in startReverse), reassert to be safe
        motorWrite(IN1, IN2, -700);
        motorWrite(IN3, IN4, -700);
      }
    }
  }
  else if (autoState == A_SCANNING){
    if (scanResultReady){
      performDecisionAfterScan();
      scanResultReady = false;
    }
  }
  else if (autoState == A_TURNING){
    unsigned long now = millis();
    if (now - autoStateAt >= TURN_DURATION_MS){
      stopCar();
      autoState = A_FORWARDING;
      autoStateAt = now;
      motorMix(0.0, 0.6);
    }
  }
  else if (autoState == A_FORWARDING){
    unsigned long now = millis();
    if (now - autoStateAt >= FORWARD_AFTER_TURN_MS){
      stopCar();
      autoState = A_IDLE;
    }
  }
}

// =========================
// === HTTP HANDLERS =======
// =========================
void handleRoot(){
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus(){
  long d = readUltrasonicCM();
  int pct = 99; // placeholder battery (user removed battery monitor)
  String s = "{\"distance\":" + String(d) + ",\"mode\":" + String((autoState==A_IDLE && !scanningActive && autoState!=A_IDLE)?1:(autoState==A_IDLE?1:0)) + ",\"batteryPct\":" + String(pct) + "}";
  // simpler: mode numeric: 1 manual, 0 auto — we will define manual controlling variable by checking if autoState == A_IDLE and manualMode var optional
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", s);
}

volatile bool manualMode = true;
void handleMove(){
  if (!server.hasArg("d")) { server.send(400, "text/plain", "missing d"); return; }
  String d = server.arg("d");
  if (!manualMode) { stopCar(); server.send(200,"text/plain","auto"); return; }

  if (d == "F") motorMix(0.0, 1.0);      // forward
  else if (d == "B") motorMix(0.0, -1.0); // backward
  else if (d == "L") {
    // turn LEFT: only RIGHT motors forward
    motorWrite(IN1, IN2, 1023);     // left motors stopped
    motorWrite(IN3, IN4, 0);   // right motors forward
  }
  else if (d == "R") {
    // turn RIGHT: only LEFT motors forward
    motorWrite(IN1, IN2, 0);   // left motors forward
    motorWrite(IN3, IN4, 1023);     // right motors stopped
  }
  else if (d == "S") stopCar();

  server.send(200, "text/plain", "OK");
}
void handleMode(){
  if (!server.hasArg("set")) { server.send(400,"text/plain","missing"); return; }
  String m = server.arg("set");
  manualMode = (m == "manual");
  if (!manualMode) { autoState = A_IDLE; } else { stopCar(); autoState = A_IDLE; }
  server.send(200, "text/plain", "OK");
}

void handleStop(){
  stopCar();
  server.send(200,"text/plain","OK");
}

void handleScan(){
  startScan();
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"status\":\"started\"}");
}

void handleScanResult(){
  String res = "{";
  res += "\"left\":" + String(scanLeft) + ",";
  res += "\"front\":" + String(scanFront) + ",";
  res += "\"right\":" + String(scanRight) + ",";
  res += "\"done\":" + String((scanPhase==IDLE && !scanningActive && scanResultReady) ? 1 : (scanningActive?0:(scanResultReady?1:0)));
  res += "}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", res);
}

void handleSonarToggle(){
  if (!server.hasArg("mode")) { server.send(400,"text/plain","missing"); return; }
  String m = server.arg("mode");
  sonarEnabled = (m == "on");
  server.send(200,"text/plain","OK");
}

void handleRotate(){
  if (!server.hasArg("dir")) { server.send(400,"text/plain","missing"); return; }
  String d = server.arg("dir");

  if (!manualMode) { 
    server.send(200,"text/plain","auto"); 
    return; 
  }

  stopCar();
  delay(20);

  if (d == "left") {
    // LEFT rotation: left backwards, right forward
    motorWrite(IN1, IN2, 1023);     // left motors stopped
    motorWrite(IN3, IN4, 0); // RIGHT motors forward (your wiring reversed)
    delay(2000);
    stopCar();
  }

  else if (d == "right") {
    // RIGHT rotation: left forward, right backward
    motorWrite(IN1, IN2, 0);   // left motors forward
    motorWrite(IN3, IN4, 1023);      // RIGHT motors backward (your wiring reversed)
    delay(2000);
    stopCar();
  }

  else if (d == "360") {
    // full spin right
    motorWrite(IN1, IN2, 1023);    // left forward
    motorWrite(IN3, IN4, 0);    // right backward
    delay(5000);
    stopCar();
  }

  server.send(200,"text/plain","OK");
}

void handleLed(){
  if (!server.hasArg("effect")) { server.send(400,"text/plain","missing"); return; }
  String e = server.arg("effect");
  if (e=="off"){ digitalWrite(LED_LEFT, LOW); digitalWrite(LED_RIGHT, LOW); }
  else if (e=="blink"){ digitalWrite(LED_LEFT, HIGH); digitalWrite(LED_RIGHT, LOW); }
  else if (e=="warn"){ digitalWrite(LED_LEFT, HIGH); digitalWrite(LED_RIGHT, HIGH); }
  else if (e=="pulse"){ digitalWrite(LED_LEFT, HIGH); digitalWrite(LED_RIGHT, LOW); }
  server.send(200,"text/plain","OK");
}

void handlePath() {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "no data");
    return;
  }

  String body = server.arg("plain");   // "x1,y1;x2,y2;..."

  // ---- parse average X from the string ----
  float sumX = 0.0;
  int count = 0;

  int start = 0;
  while (start < body.length()) {
    int comma = body.indexOf(',', start);
    if (comma == -1) break;
    int semi = body.indexOf(';', start);
    if (semi == -1) semi = body.length();

    String sx = body.substring(start, comma);   // "0.123"
    float x = sx.toFloat();
    sumX += x;
    count++;

    if (semi >= body.length()) break;
    start = semi + 1;
  }

  if (count == 0) {
    server.send(400, "text/plain", "bad data");
    return;
  }

  float avgX = sumX / count;   // 0.0 .. 1.0, 0.5 is center

  // make sure we are in manual-style control
  manualMode = true;

  stopCar();   // start from stop

  // ---- decide motion from avgX ----
  if (avgX < 0.45) {
    // path mostly left -> curve left (right motors faster)
    motorWrite(IN1, IN2, FORWARD_PWM / 2);  // left slow
    motorWrite(IN3, IN4, FORWARD_PWM);      // right fast
  } else if (avgX > 0.55) {
    // path mostly right -> curve right (left motors faster)
    motorWrite(IN1, IN2, FORWARD_PWM);      // left fast
    motorWrite(IN3, IN4, FORWARD_PWM / 2);  // right slow
  } else {
    // path mostly centered -> straight
    motorMix(0.0, 0.8);                     // go straight
  }

  server.send(200, "text/plain", "OK");
}

// =========================
// === SETUP & LOOP =======
// =========================
void setup(){
  Serial.begin(115200);
  pinSetup();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP: "); Serial.println(myIP);

  // server routes
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/move", handleMove);
  server.on("/mode", handleMode);
  server.on("/stop", handleStop);
  server.on("/scan", handleScan);
  server.on("/scanResult", handleScanResult);
  server.on("/sonarToggle", handleSonarToggle);
  server.on("/rotate", handleRotate);
  server.on("/led", handleLed);
  server.on("/path", HTTP_POST, handlePath);
  server.begin();

  // attach servo centered initially and detach to avoid jitter
  scanServo.attach(SERVO_PIN);
  scanServo.write(90);
  delay(80);
  scanServo.detach();

  Serial.println("Nova X ready. Connect to AP and open 192.168.4.1");
}

void loop(){
  server.handleClient();

  // step scan if active
  stepScan();

  // step auto state machine (only runs if manualMode == false)
  if (!manualMode) stepAuto();

  // quick safety: if manual mode and nothing received, you might want to stop (UI sends repeated commands)
  // (left as-is — UI uses hold repeat events)
}