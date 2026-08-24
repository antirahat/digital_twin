#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// ==========================================
// 1. WI-FI CONFIGURATION
// ==========================================
const char* ssid     = "";  // Enter home Wi-Fi SSID, or leave blank for AP mode
const char* password = "";  // Enter home Wi-Fi password

const char* ap_ssid  = "ESP32_DigitalTwin";
const char* ap_pass  = "12345678";

IPAddress ap_local_IP(192, 168, 4, 1);
IPAddress ap_gateway(192, 168, 4, 1);
IPAddress ap_subnet(255, 255, 255, 0);

// ==========================================
// 2. HARDWARE PIN DEFINITIONS
// ==========================================
#define DHTPIN       4     // GPIO 4 connected to DHT22 DATA pin
#define DHTTYPE      DHT22 // Configured for DHT22
#define BUZZER_PIN   18    // GPIO 18 connected to Buzzer (+)
#define RELAY_PIN    26    // GPIO 26 connected to Relay IN (Signal Pin)
#define I2C_SDA      21    // ESP32 default I2C SDA (Shared by OLED & MPU6050)
#define I2C_SCL      22    // ESP32 default I2C SCL (Shared by OLED & MPU6050)

// ==========================================
// 3. HARDWARE OBJECTS
// ==========================================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define i2c_Address   0x3c

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;
DHT dht(DHTPIN, DHTTYPE);
WebServer server(80);
DNSServer dnsServer;
Preferences preferences;

// ==========================================
// 4. GLOBAL STATE & THRESHOLDS
// ==========================================
float currentTemp = 0.0;
float currentHum  = 0.0;
float yAccel      = 0.0; // Y-Axis Acceleration (m/s^2)
float yGyro       = 0.0; // Y-Axis Gyro (rad/s)
float motionScore = 0.0; // Combined Y-Axis Motion / Vibration metric

bool mpuConnected = false;
bool sensorError  = false;
bool relayState   = false;

// Default thresholds (Customizable from Web & Saved to Flash)
float tempThreshold   = 35.0; // °C
float humThreshold    = 85.0; // %
float motionThreshold = 4.0;  // m/s^2 motion magnitude limit

bool tempAlert   = false;
bool humAlert    = false;
bool motionAlert = false;
bool isAlertActive = false;
bool isAPMode    = false;
bool blinkState  = false;

unsigned long lastSensorReadTime = 0;
unsigned long lastBuzzerPingTime = 0;
String ipAddressStr = "192.168.4.1";

// ==========================================
// 5. HTML / CSS / JS WEB INTERFACE WITH LIVE OSCILLOSCOPE GRAPHS
// ==========================================
const char PAGE_INDEX[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32 Digital Twin Node</title>
  <style>
    :root {
      --bg: #0b0f19;
      --card-bg: #151e2e;
      --card-border: #23324a;
      --accent: #38bdf8;
      --accent-acc: #38bdf8;
      --accent-gyro: #a855f7;
      --accent-alert: #ef4444;
      --accent-ok: #10b981;
      --text: #f8fafc;
      --text-dim: #94a3b8;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    body { background: var(--bg); color: var(--text); padding: 16px; display: flex; justify-content: center; min-height: 100vh; }
    .container { max-width: 580px; width: 100%; display: flex; flex-direction: column; gap: 14px; }
    .header { text-align: center; padding: 6px 0; }
    .header h1 { font-size: 20px; color: var(--accent); letter-spacing: 0.5px; }
    .header p { font-size: 12px; color: var(--text-dim); }
    .card { background: var(--card-bg); border-radius: 12px; padding: 16px; box-shadow: 0 4px 20px rgba(0,0,0,0.4); border: 1px solid var(--card-border); }
    .status-badge { display: inline-block; padding: 6px 14px; border-radius: 20px; font-weight: bold; font-size: 13px; text-transform: uppercase; margin-bottom: 12px; }
    .status-normal { background: rgba(16, 185, 129, 0.15); color: var(--accent-ok); border: 1px solid var(--accent-ok); }
    .status-alert { background: rgba(239, 68, 68, 0.25); color: var(--accent-alert); border: 1px solid var(--accent-alert); animation: pulse 1.2s infinite; }
    @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.6; } }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    .metric-box { background: #0b0f19; padding: 12px; border-radius: 8px; text-align: center; border: 1px solid var(--card-border); transition: border-color 0.3s; }
    .metric-box.breached { border-color: var(--accent-alert); box-shadow: 0 0 10px rgba(239, 68, 68, 0.3); }
    .metric-box .label { font-size: 11px; color: var(--text-dim); text-transform: uppercase; margin-bottom: 4px; }
    .metric-box .value { font-size: 22px; font-weight: bold; color: var(--text); }
    .metric-box .unit { font-size: 11px; color: var(--text-dim); }
    
    .chart-container { margin-top: 10px; background: #070a12; border-radius: 8px; border: 1px solid var(--card-border); padding: 10px; position: relative; }
    .chart-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; font-size: 12px; }
    .chart-title { font-weight: bold; }
    .chart-legend { font-size: 11px; color: var(--text-dim); }
    canvas { width: 100%; height: 110px; display: block; border-radius: 4px; }

    .form-group { margin-bottom: 12px; }
    label { display: block; font-size: 12px; color: var(--text-dim); margin-bottom: 5px; font-weight: 500; }
    input[type="number"] { width: 100%; padding: 10px; border-radius: 6px; border: 1px solid #334155; background: #0b0f19; color: #fff; font-size: 15px; outline: none; }
    input[type="number"]:focus { border-color: var(--accent); }
    button { width: 100%; padding: 13px; border-radius: 6px; border: none; background: var(--accent); color: #0b0f19; font-size: 15px; font-weight: bold; cursor: pointer; transition: 0.2s; }
    button:hover { background: #7dd3fc; }
    #msg { font-size: 13px; text-align: center; margin-top: 10px; min-height: 18px; font-weight: bold; }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>DIGITAL TWIN NODE</h1>
      <p>Real-Time Condition Monitoring & Waveform Telemetry</p>
    </div>

    <!-- Live Status & Metric Cards -->
    <div class="card" style="text-align:center;">
      <div id="status-badge" class="status-badge status-normal">SYSTEM NORMAL</div>
      <div class="grid">
        <div class="metric-box" id="temp-box">
          <div class="label">Temperature</div>
          <div class="value" id="temp-val">--</div>
          <div class="unit">°C (Limit: <span id="temp-th-disp">--</span>°C)</div>
        </div>
        <div class="metric-box" id="hum-box">
          <div class="label">Humidity</div>
          <div class="value" id="hum-val">--</div>
          <div class="unit">% (Limit: <span id="hum-th-disp">--</span>%)</div>
        </div>
        <div class="metric-box" id="motion-box">
          <div class="label">Y-Axis Motion</div>
          <div class="value" id="motion-val">--</div>
          <div class="unit">m/s² (Limit: <span id="motion-th-disp">--</span>)</div>
        </div>
        <div class="metric-box" id="relay-box">
          <div class="label">Relay & Alert LED</div>
          <div class="value" id="relay-val" style="font-size: 18px; color: var(--accent-ok);">NORMAL</div>
          <div class="unit">GPIO 26 Status</div>
        </div>
      </div>
    </div>

    <!-- Real-time MPU6050 Graphs -->
    <div class="card">
      <h3 style="font-size: 14px; color: var(--accent); margin-bottom: 8px;">Real-Time MPU6050 Waveform Analysis</h3>
      
      <!-- Graph 1: Accelerometer Y-Axis -->
      <div class="chart-container">
        <div class="chart-header">
          <span class="chart-title" style="color: var(--accent-acc);">Y-Axis Acceleration (<span id="acc-curr">0.0</span> m/s²)</span>
          <span class="chart-legend">Threshold: <span id="acc-th-disp" style="color:var(--accent-alert);">4.0</span></span>
        </div>
        <canvas id="accChart"></canvas>
      </div>

      <!-- Graph 2: Gyroscope Y-Axis -->
      <div class="chart-container" style="margin-top: 12px;">
        <div class="chart-header">
          <span class="chart-title" style="color: var(--accent-gyro);">Y-Axis Gyroscope (<span id="gyro-curr">0.0</span> rad/s)</span>
          <span class="chart-legend">Angular Velocity</span>
        </div>
        <canvas id="gyroChart"></canvas>
      </div>
    </div>

    <!-- Threshold Configuration Card -->
    <div class="card">
      <h3 style="margin-bottom: 12px; font-size: 15px;">Configure Alert Thresholds</h3>
      <form id="threshold-form">
        <div class="form-group">
          <label for="temp_th">Max Temperature Threshold (°C)</label>
          <input type="number" step="0.5" id="temp_th" required>
        </div>
        <div class="form-group">
          <label for="hum_th">Max Humidity Threshold (%)</label>
          <input type="number" step="1" id="hum_th" required>
        </div>
        <div class="form-group">
          <label for="motion_th">Y-Axis Motion / Shake Threshold (m/s²)</label>
          <input type="number" step="0.5" id="motion_th" required>
        </div>
        <button type="submit" id="save-btn">Save Threshold Settings</button>
        <div id="msg"></div>
      </form>
    </div>
  </div>

  <script>
    const MAX_POINTS = 50;
    const accHistory = new Array(MAX_POINTS).fill(0);
    const gyroHistory = new Array(MAX_POINTS).fill(0);
    let activeMotionThreshold = 4.0;

    function renderChart(canvasId, dataArray, lineColor, fillGradientStart, minY, maxY, thresholdVal = null) {
      const canvas = document.getElementById(canvasId);
      if (!canvas) return;
      
      const dpr = window.devicePixelRatio || 1;
      const rect = canvas.getBoundingClientRect();
      
      if (canvas.width !== rect.width * dpr || canvas.height !== rect.height * dpr) {
        canvas.width = rect.width * dpr;
        canvas.height = rect.height * dpr;
      }

      const ctx = canvas.getContext('2d');
      ctx.save();
      ctx.scale(dpr, dpr);
      const w = rect.width;
      const h = rect.height;

      ctx.clearRect(0, 0, w, h);

      ctx.strokeStyle = '#1e293b';
      ctx.lineWidth = 1;
      for (let i = 1; i <= 3; i++) {
        const gy = (h / 4) * i;
        ctx.beginPath();
        ctx.moveTo(0, gy);
        ctx.lineTo(w, gy);
        ctx.stroke();
      }

      if (minY < 0 && maxY > 0) {
        const zeroY = h - ((0 - minY) / (maxY - minY)) * h;
        ctx.strokeStyle = '#334155';
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(0, zeroY);
        ctx.lineTo(w, zeroY);
        ctx.stroke();
        ctx.setLineDash([]);
      }

      if (thresholdVal !== null && thresholdVal <= maxY && thresholdVal >= minY) {
        const thY = h - ((thresholdVal - minY) / (maxY - minY)) * h;
        ctx.strokeStyle = 'rgba(239, 68, 68, 0.7)';
        ctx.setLineDash([5, 3]);
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(0, thY);
        ctx.lineTo(w, thY);
        ctx.stroke();
        ctx.setLineDash([]);
      }

      const step = w / (MAX_POINTS - 1);
      const points = [];
      for (let i = 0; i < dataArray.length; i++) {
        const val = Math.max(minY, Math.min(maxY, dataArray[i]));
        const y = h - ((val - minY) / (maxY - minY)) * (h - 8) - 4;
        points.push({ x: i * step, y: y });
      }

      const grad = ctx.createLinearGradient(0, 0, 0, h);
      grad.addColorStop(0, fillGradientStart);
      grad.addColorStop(1, 'rgba(11, 15, 25, 0.0)');

      ctx.beginPath();
      ctx.moveTo(points[0].x, h);
      points.forEach(p => ctx.lineTo(p.x, p.y));
      ctx.lineTo(points[points.length - 1].x, h);
      ctx.closePath();
      ctx.fillStyle = grad;
      ctx.fill();

      ctx.beginPath();
      ctx.strokeStyle = lineColor;
      ctx.lineWidth = 2;
      ctx.lineJoin = 'round';
      points.forEach((p, idx) => {
        if (idx === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      });
      ctx.stroke();

      const last = points[points.length - 1];
      ctx.beginPath();
      ctx.arc(last.x, last.y, 4, 0, Math.PI * 2);
      ctx.fillStyle = lineColor;
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 1.5;
      ctx.stroke();

      ctx.restore();
    }

    let userIsTyping = false;

    ['temp_th', 'hum_th', 'motion_th'].forEach(id => {
      const el = document.getElementById(id);
      el.addEventListener('focus', () => { userIsTyping = true; });
      el.addEventListener('blur', () => { userIsTyping = false; });
    });

    async function fetchData() {
      try {
        const res = await fetch('/api/data');
        if (!res.ok) return;
        const d = await res.json();
        
        document.getElementById('temp-val').innerText = d.temperature.toFixed(1);
        document.getElementById('hum-val').innerText = d.humidity.toFixed(1);
        document.getElementById('motion-val').innerText = d.y_motion.toFixed(1);

        document.getElementById('acc-curr').innerText = d.y_accel.toFixed(1);
        document.getElementById('gyro-curr').innerText = d.y_gyro.toFixed(2);

        document.getElementById('temp-th-disp').innerText = d.temp_th.toFixed(1);
        document.getElementById('hum-th-disp').innerText = d.hum_th.toFixed(0);
        document.getElementById('motion-th-disp').innerText = d.motion_th.toFixed(1);
        document.getElementById('acc-th-disp').innerText = d.motion_th.toFixed(1);

        activeMotionThreshold = d.motion_th;

        accHistory.shift();
        accHistory.push(d.y_accel);

        gyroHistory.shift();
        gyroHistory.push(d.y_gyro);

        const maxAcc = Math.max(8.0, activeMotionThreshold + 2.0);
        renderChart('accChart', accHistory, '#38bdf8', 'rgba(56, 189, 248, 0.35)', -maxAcc, maxAcc, activeMotionThreshold);
        renderChart('gyroChart', gyroHistory, '#a855f7', 'rgba(168, 85, 247, 0.35)', -5.0, 5.0);

        const badge = document.getElementById('status-badge');
        const tempBox = document.getElementById('temp-box');
        const humBox = document.getElementById('hum-box');
        const motionBox = document.getElementById('motion-box');
        const relayVal = document.getElementById('relay-val');

        if (d.temp_alert) tempBox.classList.add('breached'); else tempBox.classList.remove('breached');
        if (d.hum_alert) humBox.classList.add('breached'); else humBox.classList.remove('breached');
        if (d.motion_alert) motionBox.classList.add('breached'); else motionBox.classList.remove('breached');

        if (d.alert) {
          badge.className = 'status-badge status-alert';
          relayVal.innerText = 'BLINKING / ALERT';
          relayVal.style.color = 'var(--accent-alert)';
          if (d.motion_alert) {
            badge.innerText = 'ALERT: UNEXPECTED MOTION / SHAKE!';
          } else if (d.temp_alert) {
            badge.innerText = 'ALERT: HIGH TEMPERATURE!';
          } else {
            badge.innerText = 'ALERT: HIGH HUMIDITY!';
          }
        } else {
          badge.className = 'status-badge status-normal';
          badge.innerText = 'SYSTEM NORMAL';
          relayVal.innerText = 'NORMAL (OFF)';
          relayVal.style.color = 'var(--accent-ok)';
        }

        if (!userIsTyping) {
          const tInput = document.getElementById('temp_th');
          const hInput = document.getElementById('hum_th');
          const mInput = document.getElementById('motion_th');
          if (!tInput.value) tInput.value = d.temp_th;
          if (!hInput.value) hInput.value = d.hum_th;
          if (!mInput.value) mInput.value = d.motion_th;
        }
      } catch (err) {
        console.error('Fetch error:', err);
      }
    }

    document.getElementById('threshold-form').addEventListener('submit', async (e) => {
      e.preventDefault();
      const t = parseFloat(document.getElementById('temp_th').value);
      const h = parseFloat(document.getElementById('hum_th').value);
      const m = parseFloat(document.getElementById('motion_th').value);
      const msg = document.getElementById('msg');
      const btn = document.getElementById('save-btn');
      
      btn.innerText = 'Saving...';
      btn.disabled = true;

      try {
        const url = '/api/set?temp=' + encodeURIComponent(t) + '&hum=' + encodeURIComponent(h) + '&motion=' + encodeURIComponent(m);
        const res = await fetch(url);
        
        if (res.ok) {
          msg.style.color = '#10b981';
          msg.innerText = '✓ Settings Saved!';
          document.getElementById('temp-th-disp').innerText = t.toFixed(1);
          document.getElementById('hum-th-disp').innerText = h.toFixed(0);
          document.getElementById('motion-th-disp').innerText = m.toFixed(1);
          setTimeout(() => { msg.innerText = ''; }, 4000);
          fetchData();
        } else {
          throw new Error('HTTP Status ' + res.status);
        }
      } catch (err) {
        msg.style.color = '#ef4444';
        msg.innerText = '✗ Error saving: ' + err.message;
      } finally {
        btn.innerText = 'Save Threshold Settings';
        btn.disabled = false;
      }
    });

    setInterval(fetchData, 300);
    fetchData();
  </script>
</body>
</html>
)rawliteral";

// ==========================================
// 6. HELPER FUNCTIONS & ACTUATION (BUZZER + RELAY LED)
// ==========================================

// Trigger simultaneous Buzzer Ping and Relay LED Blink
void triggerAlertPing() {
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(RELAY_PIN, HIGH); // Turn Relay & LED ON
  relayState = true;
  delay(120);                    // 120ms sharp pulse
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);  // Turn Relay & LED OFF
  relayState = false;
}

void playBootAnimation() {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  for (int progress = 0; progress <= 100; progress += 4) {
    display.clearDisplay();

    display.setTextSize(1);
    display.setCursor(34, 4);
    display.print("Welcome to");

    display.setCursor(22, 16);
    display.print("Microprocessor");

    display.setCursor(22, 28);
    display.print("Course Project");

    display.drawRoundRect(14, 42, 100, 10, 3, SH110X_WHITE);

    int fillWidth = (progress * 94) / 100;
    if (fillWidth > 0) {
      display.fillRect(17, 45, fillWidth, 4, SH110X_WHITE);
    }

    display.setCursor(42, 55);
    display.printf("Loading %d%%", progress);

    display.display();
    delay(30);
  }
  delay(300);
}

void updateOLED() {
  display.clearDisplay();

  // If experiencing unexpected motion -> Show animated shaking alert screen!
  if (motionAlert) {
    int shakeX = random(-2, 3);
    int shakeY = random(-2, 3);

    display.fillRect(0, 0, 128, 12, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK, SH110X_WHITE);
    display.setTextSize(1);
    display.setCursor(16, 2);
    display.print("!! MOTION ALERT !!");

    display.setTextColor(SH110X_WHITE);
    display.setTextSize(1);
    display.setCursor(8 + shakeX, 16 + shakeY);
    display.print("System is");
    display.setCursor(8 + shakeX, 26 + shakeY);
    display.print("experiencing");
    display.setCursor(8 + shakeX, 36 + shakeY);
    display.print("unexpected motion!");

    display.drawLine(0, 48, 128, 48, SH110X_WHITE);
    display.setCursor(2, 52);
    display.printf("Y-Acc:%.1f | IP:%s", yAccel, ipAddressStr.c_str());
  } 
  else {
    display.setTextSize(1);
    display.setCursor(2, 2);
    display.setTextColor(SH110X_WHITE);
    display.print("DIGITAL TWIN");

    if (isAlertActive) {
      if (blinkState) {
        display.fillRoundRect(80, 0, 48, 12, 2, SH110X_WHITE);
        display.setTextColor(SH110X_BLACK, SH110X_WHITE);
        display.setCursor(84, 2);
        display.print("!ALERT");
      } else {
        display.drawRoundRect(80, 0, 48, 12, 2, SH110X_WHITE);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(84, 2);
        display.print("!ALERT");
      }
    } else {
      display.drawRoundRect(84, 0, 44, 12, 2, SH110X_WHITE);
      display.setTextColor(SH110X_WHITE);
      display.setCursor(90, 2);
      display.print("[ OK ]");
    }

    display.drawLine(0, 14, 128, 14, SH110X_WHITE);

    display.setTextColor(SH110X_WHITE);

    if (sensorError) {
      display.setCursor(4, 24);
      display.print("SENSOR READ ERROR!");
    } else {
      display.setCursor(2, 18);
      display.printf("T:%0.1fC", currentTemp);
      display.setCursor(68, 18);
      display.printf("H:%0.0f%%", currentHum);

      display.setCursor(2, 34);
      display.printf("Y-Acc:%0.1f", yAccel);
      display.setCursor(68, 34);
      display.printf("RLY:%s", isAlertActive ? "ALM" : "OK");
    }

    display.drawLine(0, 48, 128, 48, SH110X_WHITE);

    // Bottom Footer: Permanent IP Address
    display.setCursor(2, 52);
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.print("IP: ");
    display.print(ipAddressStr);
  }

  display.display();
  blinkState = !blinkState;
}

// ==========================================
// 7. WEB SERVER HANDLERS
// ==========================================

void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.send_P(200, "text/html", PAGE_INDEX);
}

void handleApiData() {
  String json = "{";
  json += "\"temperature\":" + String(currentTemp, 1) + ",";
  json += "\"humidity\":" + String(currentHum, 1) + ",";
  json += "\"y_accel\":" + String(yAccel, 2) + ",";
  json += "\"y_gyro\":" + String(yGyro, 2) + ",";
  json += "\"y_motion\":" + String(motionScore, 2) + ",";
  json += "\"temp_th\":" + String(tempThreshold, 1) + ",";
  json += "\"hum_th\":" + String(humThreshold, 1) + ",";
  json += "\"motion_th\":" + String(motionThreshold, 1) + ",";
  json += "\"temp_alert\":" + String(tempAlert ? "true" : "false") + ",";
  json += "\"hum_alert\":" + String(humAlert ? "true" : "false") + ",";
  json += "\"motion_alert\":" + String(motionAlert ? "true" : "false") + ",";
  json += "\"relay_state\":" + String(relayState ? "true" : "false") + ",";
  json += "\"alert\":" + String(isAlertActive ? "true" : "false");
  json += "}";
  
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handleApiSet() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");

  if (server.hasArg("temp")) {
    float newT = server.arg("temp").toFloat();
    if (newT > 0 && newT < 100) {
      tempThreshold = newT;
      preferences.putFloat("temp_th", tempThreshold);
    }
  }
  if (server.hasArg("hum")) {
    float newH = server.arg("hum").toFloat();
    if (newH > 0 && newH <= 100) {
      humThreshold = newH;
      preferences.putFloat("hum_th", humThreshold);
    }
  }
  if (server.hasArg("motion")) {
    float newM = server.arg("motion").toFloat();
    if (newM > 0) {
      motionThreshold = newM;
      preferences.putFloat("motion_th", motionThreshold);
    }
  }

  Serial.printf("[SETTINGS SAVED] Temp: %.1f C | Hum: %.1f %% | Motion: %.1f m/s^2\n", tempThreshold, humThreshold, motionThreshold);

  tempAlert   = (currentTemp >= tempThreshold);
  humAlert    = (currentHum >= humThreshold);
  motionAlert = (motionScore >= motionThreshold);
  isAlertActive = (tempAlert || humAlert || motionAlert);
  updateOLED();

  server.send(200, "text/plain", "OK");
}

void handleNotFound() {
  String uri = server.uri();
  if (uri.startsWith("/api/set")) {
    handleApiSet();
    return;
  }
  server.sendHeader("Location", String("http://") + ipAddressStr, true);
  server.send(302, "text/plain", "");
}

// ==========================================
// 8. SETUP & INITIALIZATION
// ==========================================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n======================================");
  Serial.println("  ESP32 Digital Twin Node Starting... ");
  Serial.println("======================================");

  // 1. Initialize Hardware Pins (Buzzer & Relay)
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);

  // 2. Initialize I2C Bus, OLED, MPU6050 & DHT
  Wire.begin(I2C_SDA, I2C_SCL);
  dht.begin();
  
  if (display.begin(i2c_Address, true)) {
    playBootAnimation();
  }

  // Initialize MPU6050
  if (mpu.begin()) {
    mpuConnected = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[OK] MPU6050 Initialized.");
  } else {
    mpuConnected = false;
    Serial.println("[WARNING] MPU6050 not found on I2C bus! Check wiring.");
  }

  // 3. Load Saved Thresholds from Flash Memory (Preferences)
  preferences.begin("twin_config", false);
  tempThreshold   = preferences.getFloat("temp_th", 35.0);
  humThreshold    = preferences.getFloat("hum_th", 85.0);
  motionThreshold = preferences.getFloat("motion_th", 4.0);

  // 4. Wi-Fi & Network Setup
  bool connectedSTA = false;

  if (strlen(ssid) > 0) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(10, 20);
    display.println("Connecting Wi-Fi...");
    display.setCursor(10, 35);
    display.println(ssid);
    display.display();

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("[WIFI] Connecting to ");
    Serial.println(ssid);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 15) {
      delay(500);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      connectedSTA = true;
      isAPMode = false;
      ipAddressStr = WiFi.localIP().toString();
      Serial.println("\n[WIFI] Connected to Station!");
    }
  }

  if (!connectedSTA) {
    isAPMode = true;
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(ap_local_IP, ap_gateway, ap_subnet);
    WiFi.softAP(ap_ssid, ap_pass, 1, 0, 4);
    
    ipAddressStr = WiFi.softAPIP().toString();
    Serial.println("\n[WIFI] Access Point Launched!");
    Serial.print("[WIFI] SSID: ");
    Serial.println(ap_ssid);
    Serial.print("[WIFI] Password: ");
    Serial.println(ap_pass);
    
    dnsServer.start(53, "*", ap_local_IP);
  }

  Serial.print("[WIFI] Web Dashboard URL: http://");
  Serial.println(ipAddressStr);

  if (MDNS.begin("digitaltwin")) {
    Serial.println("[MDNS] Responding at http://digitaltwin.local");
  }

  // 5. Setup Web Server Endpoints
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleApiData);
  server.on("/api/set", HTTP_ANY, handleApiSet);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("[SERVER] HTTP Server active on port 80");

  // Initial startup confirmation ping (triggers both Buzzer & Relay LED)
  triggerAlertPing();
}

// ==========================================
// 9. MAIN RUNTIME LOOP
// ==========================================

void loop() {
  if (isAPMode) {
    dnsServer.processNextRequest();
  }

  server.handleClient();

  unsigned long currentMillis = millis();

  // --- Step A: Read Sensors every 100ms (High-frequency sampling for smooth real-time telemetry) ---
  if (currentMillis - lastSensorReadTime >= 100) {
    lastSensorReadTime = currentMillis;

    // 1. Read MPU6050
    if (mpuConnected) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      yAccel = a.acceleration.y; // m/s^2
      yGyro  = g.gyro.y;         // rad/s

      motionScore = fabs(yAccel) + (fabs(yGyro) * 2.0);
      motionAlert = (motionScore >= motionThreshold);
    } else {
      motionAlert = false;
    }

    // 2. Read DHT22 every 1.5 seconds
    static unsigned long lastDhtTime = 0;
    if (currentMillis - lastDhtTime >= 1500) {
      lastDhtTime = currentMillis;

      float t = dht.readTemperature();
      float h = dht.readHumidity();

      if (isnan(t) || isnan(h)) {
        sensorError = true;
      } else {
        sensorError = false;
        currentTemp = t;
        currentHum = h;

        tempAlert = (currentTemp >= tempThreshold);
        humAlert  = (currentHum >= humThreshold);
      }
    }

    isAlertActive = (tempAlert || humAlert || motionAlert);

    // Refresh OLED
    updateOLED();
  }

  // --- Step B: Synchronized Buzzer & Relay LED Alert (Pings & Blinks every 2s during threshold breach) ---
  if (isAlertActive && !sensorError) {
    if (currentMillis - lastBuzzerPingTime >= 2000) {
      lastBuzzerPingTime = currentMillis;
      triggerAlertPing(); // Both Buzzer pings and Relay LED blinks together
    }
  } else {
    // Normal operation -> Keep both Buzzer and Relay LED silent / OFF
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(RELAY_PIN, LOW);
  }
}
