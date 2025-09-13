#include <M5Unified.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <DNSServer.h>
#include "filters.hpp"
#include "experiments.hpp"
#include "sound.hpp"

// تعريف الألوان المخصصة (أعيد بعد فصل الفلتر)
#define TEAL 0x0438

// فلاتر كالمان للمحاور
KalmanFilter axFilter; 
KalmanFilter ayFilter; 
KalmanFilter azFilter;

// نظام الصوت غير الحاجز: هيكل الحالة global حتى تستخدمه loop مبكراً
struct PendingTone { int freq = 0; unsigned long endMs = 0; bool active = false; };
PendingTone _toneState; 

// =================================================================
// إعدادات الشبكات
// =================================================================
const byte DNS_PORT = 53;
DNSServer dnsServer;
WebServer server(80);

#define EEPROM_SIZE 128
char station_ssid[32] = "";
char station_password[64] = "";

// =================================================================
// متغيرات إدارة الطاقة
// =================================================================
unsigned long lastActivityTime = 0;
const unsigned long sleepTimeout = 300000; // 5 دقائق بالمللي ثانية

// =================================================================
// ثوابت عامة (خاصة بالمعايرة فقط هنا)
// =================================================================
const int CALIBRATION_SAMPLES = 200; // الجاذبية المعرفة في experiments.cpp

// =================================================================
// تصريحات الدوال
// =================================================================
void handleMainPage(), handleProjectilePage(), handlePendulumPage(), handleFreefallPage(), handleFrictionPage();
void handleSimProjectilePage(), handleSimPendulumPage(), handleSimFreefallPage(), handleSimFrictionPage();
void handleStart(), handleReset(), handleResults(), handleSimProjectileCalc(), handleSimPendulumCalc(), handleSimFreefallCalc();
void calibrateIMU();
// الحفاظ مؤقتاً على playSound كغلاف لتوافق جزئي حتى إتمام النقل
void playSound(int freq, int duration);
void setupWifiManager(), loadCredentials(), saveCredentials();
void resetInternalState();
void enterLowPowerMode();

// =================================================================
// الدالة `setup()`
// =================================================================
void setup() {
    auto cfg = M5.config();
    M5.begin(cfg); 
    M5.Speaker.begin();
  // تدوير العرض ليظهر النص أفقياً (0 عمودي، 1 و 3 للوضع الأفقي حسب تفضيلك)
  // جرّب 1 أو 3 إذا كان الاتجاه معكوساً.
  M5.Display.setRotation(1);
    Serial.begin(115200);
    M5.Imu.begin();
    EEPROM.begin(EEPROM_SIZE);

    M5.BtnB.setHoldThresh(3000);

    M5.Display.fillScreen(BLACK);
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(0, 10);
    M5.Display.println("Initializing...");
    
  Sound::begin();
  // تفعيل تسلسل بدء التشغيل الرسمي
  Sound::trigger(Sound::Event::Startup);
    
  M5.Display.println("Calibrating...");
  Sound::trigger(Sound::Event::CalibrateStart);
  calibrateIMU();
  Sound::trigger(Sound::Event::CalibrateDone);
  M5.Display.println("Calibration Done.");
  delay(400);

    loadCredentials();

    if (strlen(station_ssid) > 0) {
        M5.Display.fillScreen(BLACK);
        M5.Display.setCursor(0, 10);
        M5.Display.printf("Connecting to:\n%s\n", station_ssid);
        WiFi.begin(station_ssid, station_password);
        int i = 0;
        while (WiFi.status() != WL_CONNECTED && i < 30) {
            delay(500);
            M5.Display.print(".");
            i++;
        }
    }

    if (WiFi.status() != WL_CONNECTED) {
        setupWifiManager();
    } else {
        M5.Display.fillScreen(BLACK);
        M5.Display.setCursor(0, 10);
        M5.Display.println("WiFi Connected!");
        M5.Display.print("IP: ");
        M5.Display.println(WiFi.localIP());
        
        server.on("/", HTTP_GET, handleMainPage);
        server.on("/projectile", HTTP_GET, handleProjectilePage);
        server.on("/pendulum", HTTP_GET, handlePendulumPage);
        server.on("/freefall", HTTP_GET, handleFreefallPage);
        server.on("/friction", HTTP_GET, handleFrictionPage);
        server.on("/sim_projectile", HTTP_GET, handleSimProjectilePage);
        server.on("/sim_pendulum", HTTP_GET, handleSimPendulumPage);
        server.on("/sim_freefall", HTTP_GET, handleSimFreefallPage);
        server.on("/sim_friction", HTTP_GET, handleSimFrictionPage);
        server.on("/calculate_projectile", HTTP_GET, handleSimProjectileCalc);
        server.on("/calculate_pendulum", HTTP_GET, handleSimPendulumCalc);
        server.on("/calculate_freefall", HTTP_GET, handleSimFreefallCalc);
        server.on("/start", HTTP_GET, handleStart);
        server.on("/reset", HTTP_GET, handleReset);
        server.on("/results", HTTP_GET, handleResults);
        server.begin();

        resetInternalState();
    }
}

// =================================================================
// الدالة `loop()`
// =================================================================
void loop() {
    M5.update();

  // إدارة إيقاف النغمة غير الحاجزة
  if (_toneState.active && millis() >= _toneState.endMs) {
    M5.Speaker.stop();
    _toneState.active = false;
  }

    if (M5.BtnB.wasHold()) {
        M5.Display.fillScreen(BLUE);
        M5.Display.setCursor(0, 80);
        M5.Display.println("Recalibrating...");
    Sound::trigger(Sound::Event::CalibrateStart);
        calibrateIMU();
    Sound::trigger(Sound::Event::CalibrateDone);
    delay(300);
        resetInternalState();
    }

    if (WiFi.getMode() == WIFI_AP) {
        dnsServer.processNextRequest();
    }
    server.handleClient();
    if (WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED) {
        if (activeExperiment == NONE && millis() - lastActivityTime > sleepTimeout) {
            enterLowPowerMode();
        }

    if (activeExperiment == PROJECTILE) projectileController();
    else if (activeExperiment == PENDULUM) pendulumController();
    else if (activeExperiment == FREEFALL) freefallController();
    else if (activeExperiment == FRICTION) frictionController();
    }
  // تحديث نظام الصوت غير الحاجز الجديد
  Sound::update();
    delay(1);
}

// =================================================================
// معالجات خادم الويب (Web Handlers)
// =================================================================
void handleMainPage() {
    resetInternalState();
    String html = R"rawliteral(
    <!DOCTYPE html><html lang="ar" dir="rtl"><head><meta charset="UTF-8"><title>المختبر الفيزيائي التفاعلي</title><meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="data:image/svg+xml,<svg xmlns=%22http://www.w3.org/2000/svg%22 viewBox=%220 0 100 100%22><text y=%22.9em%22 font-size=%2290%22>🔬</text></svg>">
    <link href="https://fonts.googleapis.com/css2?family=Tajawal:wght@400;500;700&display=swap" rel="stylesheet">
    <style>
      :root {
        --primary-blue: #1a237e;
        --secondary-blue: #3f51b5;
        --science-green: #00796b;
        --science-orange: #f57c00;
        --science-purple: #7b1fa2;
        --dark-bg: #0d1b2a;
      }
      * {
        box-sizing: border-box;
        margin: 0;
        padding: 0;
      }
      body {
        font-family: 'Tajawal', 'Segoe UI', sans-serif;
        text-align: center;
        background: linear-gradient(135deg, #0d1b2a 0%, #1b263b 100%);
        color: #e0e1dd;
        min-height: 100vh;
        padding: 20px;
        line-height: 1.6;
      }
      .container {
        max-width: 1000px;
        margin: 20px auto;
        padding: 30px;
        background: rgba(255, 255, 255, 0.05);
        backdrop-filter: blur(10px);
        border-radius: 20px;
        box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
        border: 1px solid rgba(255, 255, 255, 0.1);
        position: relative;
        overflow: hidden;
      }
      .container::before {
        content: "";
        position: absolute;
        top: 0;
        left: 0;
        right: 0;
        height: 4px;
        background: linear-gradient(90deg, var(--science-green), var(--secondary-blue), var(--science-orange));
      }
      h1 {
        color: #fff;
        font-size: 2.5rem;
        margin: 20px 0 30px;
        text-shadow: 0 2px 4px rgba(0,0,0,0.3);
        position: relative;
        display: inline-block;
      }
      h1::after {
        content: "";
        position: absolute;
        bottom: -10px;
        left: 25%;
        width: 50%;
        height: 3px;
        background: linear-gradient(90deg, transparent, var(--secondary-blue), transparent);
      }
      h2 {
        color: #e0e1dd;
        font-size: 1.8rem;
        margin: 25px 0 20px;
        padding-bottom: 15px;
        border-bottom: 2px solid rgba(255, 255, 255, 0.1);
        position: relative;
      }
      .main-container {
        display: flex;
        justify-content: center;
        gap: 30px;
        flex-wrap: wrap;
        margin: 40px 0;
      }
      .column {
        background: rgba(26, 35, 62, 0.7);
        border-radius: 16px;
        padding: 25px;
        width: 45%;
        min-width: 300px;
        box-shadow: 0 8px 16px rgba(0, 0, 0, 0.2);
        transition: transform 0.3s ease, box-shadow 0.3s ease;
        position: relative;
        overflow: hidden;
        border: 1px solid rgba(255, 255, 255, 0.08);
      }
      .column:hover {
        transform: translateY(-5px);
        box-shadow: 0 12px 24px rgba(0, 0, 0, 0.3);
      }
      .column::before {
        content: "";
        position: absolute;
        top: 0;
        left: 0;
        right: 0;
        height: 4px;
        background: linear-gradient(90deg, var(--science-green), var(--secondary-blue));
      }
      .exp-grid {
        display: grid;
        grid-template-columns: 1fr;
        gap: 18px;
      }
      .exp-button {
        display: flex;
        align-items: center;
        background: rgba(255, 255, 255, 0.08);
        color: #fff;
        padding: 20px;
        border-radius: 12px;
        font-size: 1.2rem;
        transition: all 0.3s ease;
        text-align: right;
        border: 1px solid rgba(255, 255, 255, 0.1);
        box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        position: relative;
        overflow: hidden;
        text-decoration: none;
      }
      .exp-button::before {
        content: "";
        position: absolute;
        top: 0;
        left: 0;
        width: 5px;
        height: 100%;
        background: var(--secondary-blue);
        transition: width 0.3s ease;
      }
      .exp-button:hover {
        background: rgba(63, 81, 181, 0.2);
        transform: translateX(-5px);
        box-shadow: 0 6px 12px rgba(0, 0, 0, 0.2);
      }
      .exp-button:hover::before {
        width: 8px;
      }
      .exp-button.sim-button {
        background: rgba(0, 121, 107, 0.15);
      }
      .exp-button.sim-button:hover {
        background: rgba(0, 121, 107, 0.25);
      }
      .exp-button .icon {
        margin-left: 15px;
        font-size: 1.8rem;
        min-width: 40px;
      }
      .exp-button .text {
        flex-grow: 1;
        text-align: right;
      }
      .exp-button:nth-child(1) { border-left: 4px solid #3f51b5; }
      .exp-button:nth-child(2) { border-left: 4px solid #4caf50; }
      .exp-button:nth-child(3) { border-left: 4px solid #ff9800; }
      .exp-button:nth-child(4) { border-left: 4px solid #e91e63; }
      .top-icon {
        position: absolute;
        top: 25px;
        width: 40px;
        height: 40px;
        border-radius: 50%;
        background: rgba(63, 81, 181, 0.2);
        color: #e0e1dd;
        display: flex;
        align-items: center;
        justify-content: center;
        font-size: 1.5rem;
        cursor: pointer;
        transition: all 0.3s ease;
        border: 1px solid rgba(255, 255, 255, 0.1);
        box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
      }
      .help-btn { left: 25px; }
      .mute-btn { right: 25px; }
      .top-icon:hover {
        background: rgba(63, 81, 181, 0.4);
        transform: scale(1.1);
      }
      footer {
        margin-top: 40px;
        font-size: 0.9rem;
        color: #a3b1c6;
        padding-top: 20px;
        border-top: 1px solid rgba(255, 255, 255, 0.1);
      }
      #helpModal {
        display: none;
        position: fixed;
        z-index: 1000;
        left: 0;
        top: 0;
        width: 100%;
        height: 100%;
        background: rgba(13, 27, 42, 0.9);
        backdrop-filter: blur(5px);
      }
      .modal-content {
        background: linear-gradient(135deg, #1b263b 0%, #0d1b2a 100%);
        margin: 10% auto;
        padding: 25px;
        border-radius: 20px;
        max-width: 450px;
        width: 90%;
        box-shadow: 0 10px 30px rgba(0, 0, 0, 0.4);
        border: 1px solid rgba(63, 81, 181, 0.3);
        position: relative;
        text-align: center;
      }
      .modal-content h2 { font-size: 1.5rem; }
      .close {
        position: absolute;
        top: 10px;
        left: 15px;
        color: #a3b1c6;
        font-size: 24px;
        font-weight: bold;
        cursor: pointer;
        transition: color 0.3s;
      }
      .close:hover {
        color: #fff;
      }
      .contact-info {
        background: rgba(255, 255, 255, 0.05);
        padding: 15px;
        border-radius: 12px;
        margin: 15px 0;
        text-align: center;
      }
      .contact-info p {
        margin: 10px 0;
        font-size: 0.9rem;
      }
      .sci-title {
        font-size: 1.1em;
        color: #64ffda;
        margin-top: 10px;
      }
      @media (max-width: 768px) {
        .main-container {
          flex-direction: column;
          align-items: center;
        }
        .column {
          width: 100%;
          max-width: 500px;
        }
        h1 {
          font-size: 2rem;
          margin-top: 30px;
        }
      }
    </style>
    </head>
    <body>
    <div class="container">
      <div class="help-btn top-icon" onmouseover="playHoverSound()" onclick="openHelpModal(event)">ℹ️</div>
      <div id="muteBtn" class="mute-btn top-icon" onmouseover="playHoverSound()" onclick="toggleMute(event)">🔊</div>
      <h1>المختبر الفيزيائي التفاعلي</h1>
      <div class="main-container">
        <div class="column">
          <h2>التجارب العملية</h2>
          <div class="exp-grid">
            <a href="/projectile" class="exp-button" onmouseover="playHoverSound()">
              <span class="text">تجربة المقذوفات</span>
              <span class="icon">🚀</span>
            </a>
            <a href="/pendulum" class="exp-button" onmouseover="playHoverSound()">
              <span class="text">تجربة البندول البسيط</span>
              <span class="icon">⏱️</span>
            </a>
            <a href="/freefall" class="exp-button" onmouseover="playHoverSound()">
              <span class="text">تجربة السقوط الحر</span>
              <span class="icon">🌍</span>
            </a>
            <a href="/friction" class="exp-button" onmouseover="playHoverSound()">
              <span class="text">تجربة الاحتكاك</span>
              <span class="icon">📐</span>
            </a>
          </div>
        </div>
        <div class="column">
          <h2>المحاكاة النظرية</h2>
          <div class="exp-grid">
            <a href="/sim_projectile" class="exp-button sim-button" onmouseover="playHoverSound()">
              <span class="text">محاكاة المقذوفات</span>
              <span class="icon">🚀</span>
            </a>
            <a href="/sim_pendulum" class="exp-button sim-button" onmouseover="playHoverSound()">
              <span class="text">محاكاة البندول</span>
              <span class="icon">⏱️</span>
            </a>
            <a href="/sim_freefall" class="exp-button sim-button" onmouseover="playHoverSound()">
              <span class="text">محاكاة السقوط الحر</span>
              <span class="icon">🌍</span>
            </a>
            <a href="/sim_friction" class="exp-button sim-button" onmouseover="playHoverSound()">
              <span class="text">محاكاة الاحتكاك</span>
              <span class="icon">📐</span>
            </a>
          </div>
        </div>
      </div>
      <footer>
        <p>&copy; حقوق الطبع والتوزيع 2023 - مختبرات وزارة التربية والتعليم سلطنة عمان</p>
        <p>الإصدار 2.1 | نظام تعليمي تفاعلي</p>
      </footer>
    </div>

    <div id="helpModal" style="display:none;">
      <div class="modal-content">
        <span class="close" onmouseover="playHoverSound()" onclick="closeHelpModal()">&times;</span>
        <h2>مركز المساعدة والدعم</h2>
        <div class="contact-info">
          <p>مبرمج ومطور النظام:</p>
          <p style="font-size: 1.2em; font-weight: bold; color: #64ffda; margin: 8px 0;">أحمد القصابي</p>
          <p>أخصائي صيانة أجهزة مخبرية</p>
          <p>مديرية التربية والتعليم - محافظة الداخلية</p>
          <div style="margin: 20px 0;">
            <p>للإستفسار والدعم الفني:</p>
            <p dir="ltr" style="font-family: monospace; font-size: 1.1em; letter-spacing: 1px;">
              +968 9984 8382
            </p>
          </div>
        </div>
      </div>
    </div>

    <script src="https://cdnjs.cloudflare.com/ajax/libs/tone/14.7.77/Tone.js"></script>
    <script>
      let isMuted = localStorage.getItem('isMuted') === 'true';
      const synth = new Tone.Synth().toDestination();

      function playHoverSound() {
        if (isMuted) return;
        try {
          if (Tone.context.state !== 'running') {
            Tone.context.resume();
          }
          synth.triggerAttackRelease("C5", "8n");
        } catch(e) { console.error("Could not play sound", e); }
      }

      function toggleMute(event) {
        event.preventDefault();
        isMuted = !isMuted;
        localStorage.setItem('isMuted', isMuted);
        updateMuteButton();
      }

      function updateMuteButton() {
        const muteIcon = document.getElementById('muteBtn');
        if (muteIcon) {
            muteIcon.textContent = isMuted ? '🔇' : '🔊';
        }
      }

      document.addEventListener('DOMContentLoaded', updateMuteButton);

      function openHelpModal(e) {
        e.preventDefault();
        document.getElementById("helpModal").style.display = "block";
      }
      function closeHelpModal() {
        document.getElementById("helpModal").style.display = "none";
      }
      window.onclick = function(event) {
        const modal = document.getElementById('helpModal');
        if (event.target == modal) {
          closeHelpModal();
        }
      }
    </script>
    </body>
    </html>
    )rawliteral";
    server.send(200, "text/html", html);
}

void handleProjectilePage() {
    resetInternalState();
    String html = R"rawliteral(
    <!DOCTYPE html><html lang="ar" dir="rtl"><head><meta charset="UTF-8"><title>تجربة المقذوفات</title><meta name="viewport" content="width=device-width, initial-scale=1"><link rel="icon" href="data:image/svg+xml,<svg xmlns=%22http://www.w3.org/2000/svg%22 viewBox=%220 0 100 100%22><text y=%22.9em%22 font-size=%2290%22>🔬</text></svg>"><style>body{font-family:'Segoe UI',sans-serif;text-align:center;margin:20px;background-color:#f0f2f5;}.container{max-width:600px;margin:auto;padding:20px;background-color:#fff;border-radius:12px;box-shadow:0 4px 12px rgba(0,0,0,.1);}h1,h2{color:#1a237e;}h3{color:#3f51b5;}input,button{padding:12px;margin:10px;font-size:16px;border-radius:8px;border:1px solid #ddd;}input{width:120px;text-align:center;}button{background-color:#3f51b5;color:#fff;border:none;cursor:pointer;transition:background-color .3s,transform .1s;}button:hover{background-color:#303f9f;}#resetBtn{background-color:#d32f2f;}#resetBtn:hover{background-color:#c62828;}.card{background-color:#f8f9fa;border-right:5px solid #3f51b5;padding:15px;margin:15px 0;border-radius:5px 0 0 5px;text-align:right;display:flex;justify-content:space-between;align-items:center;}.result-label{font-size:1.1em;color:#555;}.result-value{font-weight:700;color:#1a237e;font-size:1.2em;}.instructions{background-color:#fff8e1;border-right:5px solid #ffc107;padding:15px;margin:20px 0;border-radius:5px 0 0 5px;text-align:right;}.hidden{display:none;}a.back-link{display:inline-block;margin-top:20px;color:#555;text-decoration:none;}.what-if{background:#e0f2f1;border:1px solid #b2dfdb;padding:15px;margin-top:25px;border-radius:8px;}</style></head><body><div class="container"><h1>تجربة المقذوفات</h1><div id="inputSection"><h3>الخطوة 1: قياس السرعة الابتدائية</h3><form id="expForm"><div><label>الكتلة (كجم):</label><input type="number" step="0.01" id="mass" value="0.2" required></div><div><label>زاوية الإطلاق (°):</label><input type="number" id="angle" value="45" required></div><button type="button" onmouseover="playHoverSound()" onclick="startExperiment()">ابدأ التجربة</button></form></div><div id="waitingMsg" class="instructions hidden"><h2>🚀 استعد للقذف ...</h2><p>1. قم بقذف الجهاز لقياس سرعة الإطلاق.</p><p>2. حاول أن يكون مكان نزول الجهاز آمن.</p><p>3. ستظهر النتائج تلقائياً.</p></div><div id="results" class="hidden"><h2>📊 النتائج المحسوبة</h2><div class="card"><span class="result-label">السرعة الابتدائية المقاسة (V₀)</span><span class="result-value"><span id="v0">--</span> م/ث</span></div><div class="card"><span class="result-label">زاوية الإطلاق (θ)</span><span class="result-value"><span id="angle_res">--</span> °</span></div><div class="card" style="border-right-color:#4caf50"><span class="result-label">أقصى ارتفاع (h)</span><span class="result-value"><span id="sim_h">--</span> متر</span></div><div class="card" style="border-right-color:#2196f3"><span class="result-label">المدى الأفقي (R)</span><span class="result-value"><span id="sim_r">--</span> متر</span></div><div class="card" style="border-right-color:#ff9800"><span class="result-label">زمن التحليق (T)</span><span class="result-value"><span id="sim_t">--</span> ثانية</span></div></div><button id="resetBtn" onmouseover="playHoverSound()" onclick="resetExperiment()" class="hidden">إعادة التجربة</button><a href="/" onmouseover="playHoverSound()" class="back-link">&larr; العودة للقائمة الرئيسية</a></div><script src="https://cdnjs.cloudflare.com/ajax/libs/tone/14.7.77/Tone.js"></script><script>let resultInterval;const synth=new Tone.Synth().toDestination();function playHoverSound(){try{Tone.context.state!=="running"&&Tone.context.resume(),synth.triggerAttackRelease("C5","8n")}catch(t){console.error("Could not play sound",t)}}function startExperiment(){const t=document.getElementById("mass").value,e=document.getElementById("angle").value;if(!t||t<=0||!e&&0>e)return void alert("الرجاء إدخال قيم صحيحة.");document.getElementById("inputSection").classList.add("hidden"),document.getElementById("waitingMsg").classList.remove("hidden"),document.getElementById("results").classList.add("hidden"),document.getElementById("resetBtn").classList.add("hidden"),fetch(`/start?type=projectile&mass=${t}&angle=${e}`).then(t=>{if(!t.ok)throw new Error("Network response was not ok");return t.text()}).then(t=>{console.log("Experiment start request sent:",t),resultInterval=setInterval(checkResults,500)}).catch(t=>{console.error("Error starting experiment:",t),alert("حدث خطأ في بدء التجربة."),resetExperiment()})}function checkResults(){fetch("/results").then(t=>t.json()).then(t=>{"projectile"==t.type&&"done"===t.status&&(clearInterval(resultInterval),document.getElementById("waitingMsg").classList.add("hidden"),document.getElementById("results").classList.remove("hidden"),document.getElementById("resetBtn").classList.remove("hidden"),document.getElementById("v0").textContent=t.v0.toFixed(2),document.getElementById("angle_res").textContent=t.angle.toFixed(1),document.getElementById("sim_h").textContent=t.max_height.toFixed(2),document.getElementById("sim_r").textContent=t.range.toFixed(2),document.getElementById("sim_t").textContent=t.time.toFixed(2))}).catch(t=>{console.error("Error fetching results:",t),clearInterval(resultInterval)})}function resetExperiment(){location.reload();}</script></body></html>
    )rawliteral";
    server.send(200, "text/html", html);
}

void handlePendulumPage() {
    resetInternalState();
    String html = R"rawliteral(
    <!DOCTYPE html><html lang="ar" dir="rtl"><head><meta charset="UTF-8"><title>تجربة البندول البسيط</title><meta name="viewport" content="width=device-width, initial-scale=1"><link rel="icon" href="data:image/svg+xml,<svg xmlns=%22http://www.w3.org/2000/svg%22 viewBox=%220 0 100 100%22><text y=%22.9em%22 font-size=%2290%22>🔬</text></svg>"><style>body{font-family:'Segoe UI',sans-serif;text-align:center;margin:20px;background-color:#f0f2f5;}.container{max-width:600px;margin:auto;padding:20px;background-color:#fff;border-radius:12px;box-shadow:0 4px 12px rgba(0,0,0,.1);}h1{color:#1a237e;}h2{color:#3f51b5;}input,button{padding:12px;margin:10px;font-size:16px;border-radius:8px;border:1px solid #ddd;}input{width:120px;text-align:center;}button{background-color:#3f51b5;color:#fff;border:none;cursor:pointer;transition:background-color .3s;}button:hover{background-color:#303f9f;}#resetBtn{background-color:#d32f2f;}#resetBtn:hover{background-color:#c62828;}.card{background-color:#f8f9fa;border-right:5px solid #3f51b5;padding:15px;margin:15px 0;border-radius:5px 0 0 5px;text-align:right;display:flex;justify-content:space-between;align-items:center;}.result-label{font-size:1.1em;color:#555;}.result-value{font-weight:700;color:#1a237e;font-size:1.2em;}.instructions{background-color:#e1f5fe;border-right:5px solid #03a9f4;padding:15px;margin:20px 0;border-radius:5px 0 0 5px;text-align:right;}.hidden{display:none;}a.back-link{display:inline-block;margin-top:20px;color:#555;text-decoration:none;}</style></head><body><div class="container"><h1>تجربة البندول البسيط</h1><div id="inputSection"><form><div><label>طول الخيط (متر):</label><input type="number" id="length" step="0.01" value="0.5" required></div><div><label>عدد الاهتزازات:</label><input type="number" id="oscillations" step="1" value="10" required></div><button type="button" onmouseover="playHoverSound()" onclick="startExperiment()">ابدأ التجربة</button></form></div><div id="waitingMsg" class="instructions hidden"><h2>⏱️ جاري القياس...</h2><p>1. قم بتعليق الجهاز من خيط.</p><p>2. اجعله يتأرجح بشكل منتظم.</p><p>3. سيقوم الجهاز بحساب <span id="osc_target">10</span> اهتزازات كاملة. حافظ على ثبات الحركة.</p><p>الاهتزازات المكتملة: <span id="osc_count">0</span> / <span id="osc_target_disp">10</span></p></div><div id="results" class="hidden"><h2>📊 النتائج التجريبية</h2><div class="card"><span class="result-label">طول الخيط (L)</span><span class="result-value"><span id="length_res">--</span> متر</span></div><div class="card"><span class="result-label">الزمن الدوري (T)</span><span class="result-value"><span id="period">--</span> ثانية</span></div><div class="card"><span class="result-label">التردد (f)</span><span class="result-value"><span id="freq">--</span> هرتز</span></div><div class="card" style="border-right-color:#4caf50"><span class="result-label">عجلة الجاذبية المحسوبة (g)</span><span class="result-value"><span id="g_exp">--</span> م/ث²</span></div></div><button id="resetBtn" onmouseover="playHoverSound()" onclick="resetExperiment()" class="hidden">إعادة التجربة</button><a href="/" onmouseover="playHoverSound()" class="back-link">&larr; العودة للقائمة الرئيسية</a></div><script src="https://cdnjs.cloudflare.com/ajax/libs/tone/14.7.77/Tone.js"></script><script>let resultInterval;const synth=new Tone.Synth().toDestination();function playHoverSound(){try{Tone.context.state!=="running"&&Tone.context.resume(),synth.triggerAttackRelease("C5","8n")}catch(t){console.error("Could not play sound",t)}}function startExperiment(){const t=document.getElementById("length").value,e=document.getElementById("oscillations").value;if(!t||t<=0||!e||e<=0)return void alert("الرجاء إدخال قيم صحيحة للطول وعدد الاهتزازات.");document.getElementById("inputSection").classList.add("hidden"),document.getElementById("waitingMsg").classList.remove("hidden"),document.getElementById("results").classList.add("hidden"),document.getElementById("resetBtn").classList.add("hidden"),document.getElementById("osc_target").textContent=e,document.getElementById("osc_target_disp").textContent=e,fetch(`/start?type=pendulum&length=${t}&oscillations=${e}`).then(t=>{if(!t.ok)throw new Error("Network response was not ok");return t.text()}).then(t=>{console.log("Experiment start request sent:",t),resultInterval=setInterval(checkResults,250)}).catch(t=>{console.error("Error starting experiment:",t),alert("حدث خطأ في بدء التجربة."),resetExperiment()})}function checkResults(){fetch("/results").then(t=>t.json()).then(t=>{"pendulum"==t.type&&("running"===t.status?document.getElementById("osc_count").textContent=Math.floor(t.count/2):"done"===t.status&&(clearInterval(resultInterval),document.getElementById("waitingMsg").classList.add("hidden"),document.getElementById("results").classList.remove("hidden"),document.getElementById("resetBtn").classList.remove("hidden"),document.getElementById("length_res").textContent=t.length.toFixed(2),document.getElementById("period").textContent=t.period.toFixed(3),document.getElementById("freq").textContent=t.freq.toFixed(3),document.getElementById("g_exp").textContent=t.g.toFixed(2)))}).catch(t=>{console.error("Error fetching results:",t),clearInterval(resultInterval)})}function resetExperiment(){location.reload();}</script></body></html>
    )rawliteral";
    server.send(200, "text/html", html);
}

void handleFreefallPage() {
    resetInternalState();
    String html = R"rawliteral(
    <!DOCTYPE html><html lang="ar" dir="rtl"><head><meta charset="UTF-8"><title>تجربة السقوط الحر</title><meta name="viewport" content="width=device-width, initial-scale=1"><link rel="icon" href="data:image/svg+xml,<svg xmlns=%22http://www.w3.org/2000/svg%22 viewBox=%220 0 100 100%22><text y=%22.9em%22 font-size=%2290%22>🔬</text></svg>"><style>body{font-family:'Segoe UI',sans-serif;text-align:center;margin:20px;background-color:#f0f2f5;}.container{max-width:600px;margin:auto;padding:20px;background-color:#fff;border-radius:12px;box-shadow:0 4px 12px rgba(0,0,0,.1);}h1{color:#1a237e;}h2{color:#3f51b5;}input,button{padding:12px;margin:10px;font-size:16px;border-radius:8px;border:1px solid #ddd;}input{width:120px;text-align:center;}button{background-color:#3f51b5;color:#fff;border:none;cursor:pointer;transition:background-color .3s;}button:hover{background-color:#303f9f;}#resetBtn{background-color:#d32f2f;}#resetBtn:hover{background-color:#c62828;}.card{background-color:#f8f9fa;border-right:5px solid #3f51b5;padding:15px;margin:15px 0;border-radius:5px 0 0 5px;text-align:right;display:flex;justify-content:space-between;align-items:center;}.result-label{font-size:1.1em;color:#555;}.result-value{font-weight:700;color:#1a237e;font-size:1.2em;}.instructions{background-color:#e0f2f1;border-right:5px solid #009688;padding:15px;margin:20px 0;border-radius:5px 0 0 5px;text-align:right;}.hidden{display:none;}a.back-link{display:inline-block;margin-top:20px;color:#555;text-decoration:none;}</style></head><body><div class="container"><h1>تجربة السقوط الحر</h1><div id="inputSection"><form><div><label>مسافة السقوط (متر):</label><input type="number" id="distance" step="0.01" value="1.0" required></div><button type="button" onmouseover="playHoverSound()" onclick="startExperiment()">ابدأ التجربة</button></form></div><div id="waitingMsg" class="instructions hidden"><h2>🌍 استعد للسقوط...</h2><p>1. امسك الجهاز بثبات.</p><p>2. اتركه يسقط بحرية على سطح آمن.</p><p>3. ستظهر النتائج تلقائياً بعد اكتشاف الاصطدام.</p></div><div id="results" class="hidden"><h2>📊 النتائج التجريبية</h2><div class="card"><span class="result-label">زمن السقوط (t)</span><span class="result-value"><span id="time">--</span> ثانية</span></div><div class="card" style="border-right-color:#4caf50"><span class="result-label">عجلة الجاذبية المحسوبة (g)</span><span class="result-value"><span id="g_exp">--</span> م/ث²</span></div></div><button id="resetBtn" onmouseover="playHoverSound()" onclick="resetExperiment()" class="hidden">إعادة التجربة</button><a href="/" onmouseover="playHoverSound()" class="back-link">&larr; العودة للقائمة الرئيسية</a></div><script src="https://cdnjs.cloudflare.com/ajax/libs/tone/14.7.77/Tone.js"></script><script>let resultInterval;const synth=new Tone.Synth().toDestination();function playHoverSound(){try{Tone.context.state!=="running"&&Tone.context.resume(),synth.triggerAttackRelease("C5","8n")}catch(t){console.error("Could not play sound",t)}}function startExperiment(){const t=document.getElementById("distance").value;if(!t||t<=0)return void alert("الرجاء إدخال مسافة سقوط صحيحة.");document.getElementById("inputSection").classList.add("hidden"),document.getElementById("waitingMsg").classList.remove("hidden"),document.getElementById("results").classList.add("hidden"),document.getElementById("resetBtn").classList.add("hidden"),fetch(`/start?type=freefall&distance=${t}`).then(t=>{if(!t.ok)throw new Error("Network response was not ok");return t.text()}).then(t=>{console.log("Experiment start request sent:",t),resultInterval=setInterval(checkResults,100)}).catch(t=>{console.error("Error starting experiment:",t),alert("حدث خطأ في بدء التجربة."),resetExperiment()})}function checkResults(){fetch("/results").then(t=>t.json()).then(t=>{"freefall"==t.type&&"done"===t.status&&(clearInterval(resultInterval),document.getElementById("waitingMsg").classList.add("hidden"),document.getElementById("results").classList.remove("hidden"),document.getElementById("resetBtn").classList.remove("hidden"),document.getElementById("time").textContent=t.time.toFixed(3),document.getElementById("g_exp").textContent=t.g.toFixed(2))}).catch(t=>{console.error("Error fetching results:",t),clearInterval(resultInterval)})}function resetExperiment(){location.reload();}</script></body></html>
    )rawliteral";
    server.send(200, "text/html", html);
}

void handleFrictionPage() {
    resetInternalState();
    String html = R"rawliteral(
    <!DOCTYPE html><html lang="ar" dir="rtl"><head><meta charset="UTF-8"><title>تجربة الاحتكاك</title><meta name="viewport" content="width=device-width, initial-scale=1"><link rel="icon" href="data:image/svg+xml,<svg xmlns=%22http://www.w3.org/2000/svg%22 viewBox=%220 0 100 100%22><text y=%22.9em%22 font-size=%2290%22>🔬</text></svg>"><style>body{font-family:'Segoe UI',sans-serif;text-align:center;margin:20px;background-color:#f0f2f5;}.container{max-width:600px;margin:auto;padding:20px;background-color:#fff;border-radius:12px;box-shadow:0 4px 12px rgba(0,0,0,.1);}h1{color:#1a237e;}h2{color:#3f51b5;}button{padding:12px 25px;margin:10px;font-size:16px;border-radius:8px;border:1px solid #ddd;background-color:#3f51b5;color:#fff;cursor:pointer;transition:background-color .3s;}button:hover{background-color:#303f9f;}#resetBtn{background-color:#d32f2f;}#resetBtn:hover{background-color:#c62828;}.card{background-color:#f8f9fa;border-right:5px solid #f57c00;padding:15px;margin:15px 0;border-radius:5px 0 0 5px;text-align:right;display:flex;justify-content:space-between;align-items:center;}.result-label{font-size:1.1em;color:#555;}.result-value{font-weight:700;color:#1a237e;font-size:1.2em;}.instructions{background-color:#fff3e0;border-right:5px solid #f57c00;padding:15px;margin:20px 0;border-radius:5px 0 0 5px;text-align:right;}.hidden{display:none;}a.back-link{display:inline-block;margin-top:20px;color:#555;text-decoration:none;}</style></head><body><div class="container"><h1>تجربة الاحتكاك</h1><div id="inputSection"><button type="button" onmouseover="playHoverSound()" onclick="startExperiment()">ابدأ التجربة</button></div><div id="waitingMsg" class="instructions hidden"><h2>📐 قم بإمالة السطح...</h2><p>1. ضع الجهاز على سطح مستوٍ.</p><p>2. قم بإمالة السطح ببطء شديد.</p><p>3. ستظهر النتائج تلقائياً عند انزلاق الجهاز.</p><p>الزاوية الحالية: <span id="angle_live">0.0</span>°</p></div><div id="results" class="hidden"><h2>📊 النتائج التجريبية</h2><div class="card"><span class="result-label">الزاوية الحرجة (θ)</span><span class="result-value"><span id="angle_crit">--</span> °</span></div><div class="card" style="border-right-color:#4caf50"><span class="result-label">معامل الاحتكاك الساكن (μ)</span><span class="result-value"><span id="mu">--</span></span></div></div><button id="resetBtn" onmouseover="playHoverSound()" onclick="resetExperiment()" class="hidden">إعادة التجربة</button><a href="/" onmouseover="playHoverSound()" class="back-link">&larr; العودة للقائمة الرئيسية</a></div><script src="https://cdnjs.cloudflare.com/ajax/libs/tone/14.7.77/Tone.js"></script><script>let resultInterval;const synth=new Tone.Synth().toDestination();function playHoverSound(){try{Tone.context.state!=="running"&&Tone.context.resume(),synth.triggerAttackRelease("C5","8n")}catch(t){console.error("Could not play sound",t)}}function startExperiment(){document.getElementById("inputSection").classList.add("hidden"),document.getElementById("waitingMsg").classList.remove("hidden"),fetch("/start?type=friction").then(t=>{if(!t.ok)throw new Error("Network response was not ok");return t.text()}).then(t=>{console.log("Experiment start request sent:",t),resultInterval=setInterval(checkResults,100)}).catch(t=>{console.error("Error starting experiment:",t),alert("حدث خطأ في بدء التجربة."),resetExperiment()})}function checkResults(){fetch("/results").then(t=>t.json()).then(t=>{"friction"==t.type&&("running"===t.status?document.getElementById("angle_live").textContent=t.angle.toFixed(1):"done"===t.status&&(clearInterval(resultInterval),document.getElementById("waitingMsg").classList.add("hidden"),document.getElementById("results").classList.remove("hidden"),document.getElementById("resetBtn").classList.remove("hidden"),document.getElementById("angle_crit").textContent=t.angle.toFixed(2),document.getElementById("mu").textContent=t.mu.toFixed(3)))}).catch(t=>{console.error("Error fetching results:",t),clearInterval(resultInterval)})}function resetExperiment(){location.reload();}</script></body></html>
    )rawliteral";
    server.send(200, "text/html", html);
}

void handleSimProjectilePage() {
    resetInternalState();
    String html = R"rawliteral(
    <!DOCTYPE html><html lang="ar" dir="rtl"><head><meta charset="UTF-8"><title>محاكاة المقذوفات</title><meta name="viewport" content="width=device-width, initial-scale=1"><link rel="icon" href="data:image/svg+xml,<svg xmlns=%22http://www.w3.org/2000/svg%22 viewBox=%220 0 100 100%22><text y=%22.9em%22 font-size=%2290%22>🔬</text></svg>"><style>body{font-family:'Segoe UI',sans-serif;text-align:center;margin:20px;background-color:#f0f2f5;}.container{max-width:800px;margin:auto;padding:20px;background-color:#fff;border-radius:12px;box-shadow:0 4px 12px rgba(0,0,0,.1);}h1{color:#1a237e;}#controls{margin-bottom:20px;display:flex;justify-content:center;align-items:center;flex-wrap:wrap;}#controls div{margin:5px 15px;}input{width:80px;text-align:center;padding:8px;border-radius:5px;border:1px solid #ccc;}button{background-color:#00796b;color:#fff;border:none;cursor:pointer;padding:10px 20px;border-radius:5px;transition:background-color .3s;}button:hover{background-color:#004d40;}#results-container{display:flex;justify-content:space-around;margin-top:15px;flex-wrap:wrap;}#results-container div{background:#e0f2f1;padding:10px;border-radius:8px;margin:5px;min-width:150px;}canvas{border:1px solid #ccc;background-color:#f8f9fa;margin-top:20px;}a.back-link{display:inline-block;margin-top:20px;color:#555;text-decoration:none;}</style></head><body><div class="container"><h1>تجربة المقذوفات (محاكاة)</h1><div id="controls"><form onsubmit="runSimulation(event)"><div><label>السرعة الابتدائية (م/ث): </label><input type="number" id="v0" value="25" step="1"></div><div><label>زاوية الإطلاق (°): </label><input type="number" id="angle" value="45" step="1"></div><button type="submit" onmouseover="playHoverSound()">محاكاة</button></form></div><canvas id="simCanvas" width="760" height="400"></canvas><div id="results-container"><div><h4>زمن التحليق</h4><p id="time">-- s</p></div><div><h4>أقصى ارتفاع</h4><p id="height">-- m</p></div><div><h4>المدى الأفقي</h4><p id="range">-- m</p></div></div><a href="/" onmouseover="playHoverSound()" class="back-link">&larr; العودة للقائمة الرئيسية</a></div><script src="https://cdnjs.cloudflare.com/ajax/libs/tone/14.7.77/Tone.js"></script><script>const canvas=document.getElementById("simCanvas"),ctx=canvas.getContext("2d"),g=9.81;const synth=new Tone.Synth().toDestination();function playHoverSound(){try{Tone.context.state!=="running"&&Tone.context.resume(),synth.triggerAttackRelease("C5","8n")}catch(t){console.error("Could not play sound",t)}}function runSimulation(t){t.preventDefault();const e=parseFloat(document.getElementById("v0").value),n=parseFloat(document.getElementById("angle").value);fetch(`/calculate_projectile?v0=${e}&angle=${n}`).then(t=>t.json()).then(t=>{document.getElementById("time").textContent=t.time.toFixed(2)+" s",document.getElementById("height").textContent=t.max_height.toFixed(2)+" m",document.getElementById("range").textContent=t.range.toFixed(2)+" m",animateTrajectory(e,n,t.range,t.max_height,t.time)})}function animateTrajectory(t,e,n,a,i){ctx.clearRect(0,0,canvas.width,canvas.height);const o=t*Math.cos(e*Math.PI/180),d=t*Math.sin(e*Math.PI/180),r=canvas.width-40,l=canvas.height-40,s=Math.max(n,a),c=r/s,m=l/s;let u=0;function h(){if(u>i)return;const t=o*u,e=d*u-.5*g*u*u;ctx.clearRect(0,0,canvas.width,canvas.height),ctx.beginPath(),ctx.moveTo(20,l),ctx.lineTo(canvas.width-20,l),ctx.lineTo(20,l),ctx.lineTo(20,20),ctx.strokeStyle="#aaa",ctx.stroke();let n=20+t*c,a=l-e*m;ctx.beginPath(),ctx.arc(n,a,5,0,2*Math.PI),ctx.fillStyle="#d32f2f",ctx.fill(),u+=i/150,requestAnimationFrame(h)}h()}</script></body></html>
    )rawliteral";
    server.send(200, "text/html", html);
}

void handleSimPendulumPage() {
    resetInternalState();
    String html = R"rawliteral(
    <!DOCTYPE html><html lang="ar" dir="rtl"><head><meta charset="UTF-8"><title>محاكاة البندول</title><meta name="viewport" content="width=device-width, initial-scale=1"><link rel="icon" href="data:image/svg+xml,<svg xmlns=%22http://www.w3.org/2000/svg%22 viewBox=%220 0 100 100%22><text y=%22.9em%22 font-size=%2290%22>🔬</text></svg>"><style>body{font-family:'Segoe UI',sans-serif;text-align:center;margin:20px;background-color:#f0f2f5;}.container{max-width:600px;margin:auto;padding:20px;background-color:#fff;border-radius:12px;box-shadow:0 4px 12px rgba(0,0,0,.1);}h1{color:#1a237e;}#controls{margin-bottom:20px;display:flex;justify-content:center;align-items:center;flex-wrap:wrap;}#controls div{margin:5px 15px;}input{width:80px;text-align:center;padding:8px;border-radius:5px;border:1px solid #ccc;}button{background-color:#00796b;color:#fff;border:none;cursor:pointer;padding:10px 20px;border-radius:5px;transition:background-color .3s;}button:hover{background-color:#004d40;}#results-container{background:#e0f2f1;padding:10px;border-radius:8px;margin:5px auto;max-width:200px;}canvas{border:1px solid #ccc;background-color:#f8f9fa;margin-top:20px;}a.back-link{display:inline-block;margin-top:20px;color:#555;text-decoration:none;}</style></head><body><div class="container"><h1>محاكاة البندول البسيط</h1><div id="controls"><form onsubmit="runSimulation(event)"><div><label>طول الخيط (م): </label><input type="number" id="length" value="0.5" step="0.1"></div><div><label>الجاذبية (م/ث²): </label><input type="number" id="gravity" value="9.8" step="0.1"></div><button type="submit" onmouseover="playHoverSound()">محاكاة</button></form></div><canvas id="simCanvas" width="400" height="300"></canvas><div id="results-container"><h4>الزمن الدوري (T)</h4><p id="period">-- s</p></div><a href="/" onmouseover="playHoverSound()" class="back-link">&larr; العودة للقائمة الرئيسية</a></div><script src="https://cdnjs.cloudflare.com/ajax/libs/tone/14.7.77/Tone.js"></script><script>const canvas=document.getElementById("simCanvas"),ctx=canvas.getContext("2d");let animFrame;const synth=new Tone.Synth().toDestination();function playHoverSound(){try{Tone.context.state!=="running"&&Tone.context.resume(),synth.triggerAttackRelease("C5","8n")}catch(t){console.error("Could not play sound",t)}}function runSimulation(t){t.preventDefault();const e=parseFloat(document.getElementById("length").value),n=parseFloat(document.getElementById("gravity").value);fetch(`/calculate_pendulum?length=${e}&g=${n}`).then(t=>t.json()).then(t=>{document.getElementById("period").textContent=t.period.toFixed(3)+" s",cancelAnimationFrame(animFrame),animatePendulum(t.period)})}function animatePendulum(t){const e=canvas.width/2,n=20,a=120,o=Math.PI/4;let i=0;function d(){ctx.clearRect(0,0,canvas.width,canvas.height),ctx.beginPath(),ctx.moveTo(e,n),ctx.lineTo(e,n-10),ctx.strokeStyle="#555",ctx.stroke();const c=i*2*Math.PI/t,l=o*Math.cos(c),r=e+a*Math.sin(l),s=n+a*Math.cos(l);ctx.beginPath(),ctx.moveTo(e,n),ctx.lineTo(r,s),ctx.stroke(),ctx.beginPath(),ctx.arc(r,s,15,0,2*Math.PI),ctx.fillStyle="#d32f2f",ctx.fill(),i+=.016,animFrame=requestAnimationFrame(d)}d()}</script></body></html>
    )rawliteral";
    server.send(200, "text/html", html);
}

void handleSimFreefallPage() {
    resetInternalState();
    String html = R"rawliteral(
    <!DOCTYPE html><html lang="ar" dir="rtl"><head><meta charset="UTF-8"><title>محاكاة السقوط الحر</title><meta name="viewport" content="width=device-width, initial-scale=1"><link rel="icon" href="data:image/svg+xml,<svg xmlns=%22http://www.w3.org/2000/svg%22 viewBox=%220 0 100 100%22><text y=%22.9em%22 font-size=%2290%22>🔬</text></svg>"><style>body{font-family:'Segoe UI',sans-serif;text-align:center;margin:20px;background-color:#f0f2f5;}.container{max-width:600px;margin:auto;padding:20px;background-color:#fff;border-radius:12px;box-shadow:0 4px 12px rgba(0,0,0,.1);}h1{color:#1a237e;}#controls{margin-bottom:20px;display:flex;justify-content:center;align-items:center;flex-wrap:wrap;}#controls div{margin:5px 15px;}input{width:80px;text-align:center;padding:8px;border-radius:5px;border:1px solid #ccc;}button{background-color:#00796b;color:#fff;border:none;cursor:pointer;padding:10px 20px;border-radius:5px;transition:background-color .3s;}button:hover{background-color:#004d40;}#results-container{background:#e0f2f1;padding:10px;border-radius:8px;margin:5px auto;max-width:200px;}canvas{border:1px solid #ccc;background-color:#f8f9fa;margin-top:20px;}a.back-link{display:inline-block;margin-top:20px;color:#555;text-decoration:none;}</style></head><body><div class="container"><h1>محاكاة السقوط الحر</h1><div id="controls"><form onsubmit="runSimulation(event)"><div><label>مسافة السقوط (م): </label><input type="number" id="distance" value="10" step="1"></div><button type="submit" onmouseover="playHoverSound()">محاكاة</button></form></div><canvas id="simCanvas" width="200" height="400"></canvas><div id="results-container"><h4>زمن السقوط (t)</h4><p id="time">-- s</p></div><a href="/" onmouseover="playHoverSound()" class="back-link">&larr; العودة للقائمة الرئيسية</a></div><script src="https://cdnjs.cloudflare.com/ajax/libs/tone/14.7.77/Tone.js"></script><script>const canvas=document.getElementById("simCanvas"),ctx=canvas.getContext("2d"),g=9.81;let animFrame;const synth=new Tone.Synth().toDestination();function playHoverSound(){try{Tone.context.state!=="running"&&Tone.context.resume(),synth.triggerAttackRelease("C5","8n")}catch(t){console.error("Could not play sound",t)}}function runSimulation(t){t.preventDefault();const e=parseFloat(document.getElementById("distance").value);fetch(`/calculate_freefall?distance=${e}`).then(t=>t.json()).then(t=>{document.getElementById("time").textContent=t.time.toFixed(3)+" s",cancelAnimationFrame(animFrame),animateFall(t.time,e)})}function animateFall(t,e){const n=canvas.height-20;let a=0;function i(){if(a>t)return ctx.clearRect(0,0,canvas.width,canvas.height),ctx.beginPath(),ctx.arc(canvas.width/2,n,15,0,2*Math.PI),ctx.fillStyle="#d32f2f",ctx.fill(),void(animFrame=requestAnimationFrame(i));const s=a/t,l=20+s*(n-20);ctx.clearRect(0,0,canvas.width,canvas.height),ctx.beginPath(),ctx.arc(canvas.width/2,l,15,0,2*Math.PI),ctx.fillStyle="#3f51b5",ctx.fill(),a+=.016,animFrame=requestAnimationFrame(i)}i()}</script></body></html>
    )rawliteral";
    server.send(200, "text/html", html);
}

void handleSimFrictionPage() {
    resetInternalState();
    String html = R"rawliteral(
    <!DOCTYPE html><html lang="ar" dir="rtl"><head><meta charset="UTF-8"><title>محاكاة الاحتكاك</title><meta name="viewport" content="width=device-width, initial-scale=1"><link rel="icon" href="data:image/svg+xml,<svg xmlns=%22http://www.w3.org/2000/svg%22 viewBox=%220 0 100 100%22><text y=%22.9em%22 font-size=%2290%22>🔬</text></svg>"><style>body{font-family:'Segoe UI',sans-serif;text-align:center;margin:20px;background-color:#f0f2f5;}.container{max-width:800px;margin:auto;padding:20px;background-color:#fff;border-radius:12px;box-shadow:0 4px 12px rgba(0,0,0,.1);}h1{color:#1a237e;}#controls{margin-bottom:20px;display:flex;justify-content:center;align-items:center;flex-wrap:wrap;}#controls div{margin:5px 15px;}input[type=range]{width:150px;}#results-container{display:flex;justify-content:space-around;margin-top:15px;flex-wrap:wrap;}#results-container div{background:#fff3e0;padding:10px;border-radius:8px;margin:5px;min-width:150px;}canvas{border:1px solid #ccc;background-color:#f8f9fa;margin-top:20px;}a.back-link{display:inline-block;margin-top:20px;color:#555;text-decoration:none;}</style></head><body><div class="container"><h1>محاكاة الاحتكاك على سطح مائل</h1><div id="controls"><div><label>الزاوية (°): <span id="angle_val">30</span></label><br><input type="range" id="angle" min="0" max="90" value="30" oninput="updateSim()" onmouseover="playHoverSound()"></div><div><label>معامل الاحتكاك (μ): <span id="mu_val">0.7</span></label><br><input type="range" id="mu" min="0" max="1.5" value="0.7" step="0.01" oninput="updateSim()" onmouseover="playHoverSound()"></div></div><canvas id="simCanvas" width="500" height="300"></canvas><div id="results-container"><div><h4>قوة الجاذبية الموازية</h4><p id="fg_para">-- N</p></div><div><h4>أقصى قوة احتكاك</h4><p id="ff_max">-- N</p></div><div><h4>الحالة</h4><p id="status">--</p></div></div><a href="/" onmouseover="playHoverSound()" class="back-link">&larr; العودة للقائمة الرئيسية</a></div><script src="https://cdnjs.cloudflare.com/ajax/libs/tone/14.7.77/Tone.js"></script><script>const canvas=document.getElementById("simCanvas"),ctx=canvas.getContext("2d"),g=9.81,m=1;const angleSlider=document.getElementById("angle"),muSlider=document.getElementById("mu"),angleVal=document.getElementById("angle_val"),muVal=document.getElementById("mu_val"),fgParaEl=document.getElementById("fg_para"),ffMaxEl=document.getElementById("ff_max"),statusEl=document.getElementById("status");const synth=new Tone.Synth().toDestination();function playHoverSound(){try{Tone.context.state!=="running"&&Tone.context.resume(),synth.triggerAttackRelease("C5","8n")}catch(t){console.error("Could not play sound",t)}}function updateSim(){const t=parseFloat(angleSlider.value),e=parseFloat(muSlider.value);angleVal.textContent=t,muVal.textContent=e;const n=m*g*Math.cos(t*Math.PI/180),a=e*n,i=m*g*Math.sin(t*Math.PI/180);fgParaEl.textContent=i.toFixed(2)+" N",ffMaxEl.textContent=a.toFixed(2)+" N";let l="ثابت";i>a&&(l="متحرك"),statusEl.textContent=l,draw(t,l)}function draw(t,e){const n=t*Math.PI/180,a=canvas.width,i=canvas.height,l=50,o=i-50,c=l+(i-100)*Math.cos(n),d=o-(i-100)*Math.sin(n);ctx.clearRect(0,0,a,i),ctx.beginPath(),ctx.moveTo(l,o),ctx.lineTo(c,o),ctx.lineTo(l,d),ctx.closePath(),ctx.fillStyle="#d2b48c",ctx.fill();const s=a/2,r=o-25*Math.sin(n)-12.5*Math.cos(n);ctx.save(),ctx.translate(s,r),ctx.rotate(-n),ctx.fillStyle="متحرك"===e?"#e57373":"#64b5f6",ctx.fillRect(-25,-12.5,50,25),ctx.restore()}window.onload=updateSim;</script></body></html>
    )rawliteral";
    server.send(200, "text/html", html);
}


// =================================================================
// دوال التحكم بالتجارب
// =================================================================
void handleStart() {
    String type = server.arg("type");
    if (type == "projectile") {
        activeExperiment = PROJECTILE;
        experimentState = WAITING;
        proj_mass = server.arg("mass").toFloat();
        proj_angle_deg = server.arg("angle").toFloat();
    Sound::trigger(Sound::Event::ExperimentStartProjectile);
        M5.Display.fillScreen(TEAL); M5.Display.setCursor(0, 80); M5.Display.println("Projectile Exp.\nWaiting for throw...");
    } else if (type == "pendulum") {
        activeExperiment = PENDULUM;
        experimentState = WAITING;
        pend_string_length = server.arg("length").toFloat();
        pend_oscillations_to_measure = server.arg("oscillations").toInt();
    Sound::trigger(Sound::Event::ExperimentStartPendulum);
        M5.Display.fillScreen(TEAL); M5.Display.setCursor(0, 80); M5.Display.println("Pendulum Exp.\nWaiting for swing...");
    } else if (type == "freefall") {
        activeExperiment = FREEFALL;
        experimentState = WAITING;
        freefall_distance = server.arg("distance").toFloat();
    Sound::trigger(Sound::Event::ExperimentStartFreefall);
        M5.Display.fillScreen(TEAL); M5.Display.setCursor(0, 80); M5.Display.println("Free Fall Exp.\nWaiting for drop...");
    } else if (type == "friction") {
        activeExperiment = FRICTION;
        experimentState = RUNNING; // يبدأ القياس فوراً
    Sound::trigger(Sound::Event::ExperimentStartFriction);
        M5.Display.fillScreen(TEAL); M5.Display.setCursor(0, 80); M5.Display.println("Friction Exp.\nTilting...");
    }
    server.send(200, "text/plain", "Experiment started");
}

void handleReset() {
    resetInternalState();
    server.send(200, "text/plain", "Reset OK");
}

void handleResults() {
    String json = "{\"type\":\"";
    switch(activeExperiment) {
        case PROJECTILE: json += "projectile"; break;
        case PENDULUM:   json += "pendulum"; break;
        case FREEFALL:   json += "freefall"; break;
        case FRICTION:   json += "friction"; break;
        default:         json += "none"; break;
    }
    json += "\",\"status\":\"" + String(experimentState == DONE ? "done" : (experimentState == RUNNING ? "running" : "waiting")) + "\"";

    if (activeExperiment == PROJECTILE && experimentState == DONE) {
        float angle_rad = proj_angle_deg * PI / 180.0;
        float v0y = proj_V0 * sin(angle_rad);
        float v0x = proj_V0 * cos(angle_rad);
        float time_of_flight = (2 * v0y) / GRAVITY_CONST;
        float max_height = (v0y * v0y) / (2 * GRAVITY_CONST);
        float range = v0x * time_of_flight;

        json += ",\"v0\":" + String(proj_V0, 3) + ",\"angle\":" + String(proj_angle_deg, 1) + ",\"time\":" + String(time_of_flight, 3) + ",\"max_height\":" + String(max_height, 3) + ",\"range\":" + String(range, 3);
    }
    if (activeExperiment == PENDULUM) {
        if (experimentState == RUNNING) json += ",\"count\":" + String(pend_oscillation_count);
        else if (experimentState == DONE) json += ",\"length\":" + String(pend_string_length, 2) + ",\"period\":" + String(pend_period, 4) + ",\"freq\":" + String(pend_frequency, 4) + ",\"g\":" + String(pend_g_exp, 2);
    }
    if (activeExperiment == FREEFALL && experimentState == DONE) {
        json += ",\"time\":" + String(freefall_time, 3) + ",\"g\":" + String(freefall_g_exp, 2);
    }
    if (activeExperiment == FRICTION) {
        if (experimentState == RUNNING) json += ",\"angle\":" + String(fric_current_angle);
        else if (experimentState == DONE) json += ",\"angle\":" + String(fric_critical_angle) + ",\"mu\":" + String(fric_mu);
    }
    json += "}";
    server.send(200, "application/json", json);
}

void handleSimProjectileCalc() {
    float v0 = server.arg("v0").toFloat();
    float angle_deg = server.arg("angle").toFloat();
    float angle_rad = angle_deg * PI / 180.0;

    float v0y = v0 * sin(angle_rad);
    float v0x = v0 * cos(angle_rad);

    float time_of_flight = (2 * v0y) / GRAVITY_CONST;
    float max_height = (v0y * v0y) / (2 * GRAVITY_CONST);
    float range = v0x * time_of_flight;

    String json = "{";
    json += "\"time\":" + String(time_of_flight, 4) + ",";
    json += "\"max_height\":" + String(max_height, 4) + ",";
    json += "\"range\":" + String(range, 4);
    json += "}";
    server.send(200, "application/json", json);
}

void handleSimPendulumCalc() {
    float length = server.arg("length").toFloat();
    float g = server.arg("g").toFloat();
    if (length <= 0 || g <= 0) {
        server.send(400, "text/plain", "Invalid input");
        return;
    }
    float period = 2.0 * PI * sqrt(length / g);
    String json = "{\"period\":" + String(period, 4) + "}";
    server.send(200, "application/json", json);
}

void handleSimFreefallCalc() {
    float distance = server.arg("distance").toFloat();
    if (distance <= 0) {
        server.send(400, "text/plain", "Invalid input");
        return;
    }
    float time = sqrt((2.0 * distance) / GRAVITY_CONST);
    String json = "{\"time\":" + String(time, 4) + "}";
    server.send(200, "application/json", json);
}


// =================================================================
// دوال مساعدة
// =================================================================

void resetInternalState() {
    activeExperiment = NONE;
    experimentState = IDLE;
    lastActivityTime = millis();
  resetExperimentData();
    
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(0, 60);
    M5.Display.println("Ready!");
    M5.Display.println("Open in browser:");
    M5.Display.setTextColor(GREEN);
    M5.Display.println(WiFi.localIP());
    Serial.println("--- Internal State Reset ---");
}

void playSound(int freq, int duration) {
  if (freq <= 0 || duration <= 0) return;
  M5.Speaker.setVolume(255);
  M5.Speaker.tone(freq);
  _toneState.freq = freq;
  _toneState.endMs = millis() + (unsigned long)duration;
  _toneState.active = true;
}

void calibrateIMU() {
    float az_sum = 0.0;
    float ay_sum = 0.0;
    float ax_sum = 0.0;
    M5.Display.setCursor(0, 40);
    M5.Display.println("Keep device still...");
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        float ax, ay, az;
        M5.Imu.getAccelData(&ax, &ay, &az);
        az_sum += az;
        ay_sum += ay;
        ax_sum += ax;
        delay(5);
    }
    proj_g0 = az_sum / CALIBRATION_SAMPLES;
    pend_g0_y = ay_sum / CALIBRATION_SAMPLES;
    fric_g0_x = ax_sum / CALIBRATION_SAMPLES;
    Serial.printf("Accel offsets: Z=%.4f, Y=%.4f, X=%.4f\n", proj_g0, pend_g0_y, fric_g0_x);
}

void enterLowPowerMode() {
    M5.Display.sleep();
    WiFi.disconnect(true);
    M5.Power.lightSleep();
    // عند الاستيقاظ، سيتم إعادة تشغيل الجهاز تلقائياً
    ESP.restart();
}

// =================================================================
// دوال إدارة الواي فاي
// =================================================================

void setupWifiManager() {
    const char* ap_ssid = "M5-Experiment-Setup";
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid);
    M5.Display.fillScreen(BLUE);
    M5.Display.setCursor(10, 20);
    M5.Display.println("WiFi Setup Mode");
    M5.Display.setTextSize(1);
    M5.Display.printf("\n1. Connect to WiFi:\n   %s\n", ap_ssid);
    M5.Display.printf("\n2. Open browser to:\n   192.168.4.1\n");
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    server.on("/", HTTP_GET, []() {
        String html = R"rawliteral(
        <!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>WiFi Setup</title><style>body{font-family:sans-serif;text-align:center;background:#f0f2f5;}.container{max-width:400px;margin:20px auto;padding:20px;background:#fff;border-radius:10px;box-shadow:0 0 10px rgba(0,0,0,.1);}select,input,button{width:90%;padding:12px;margin:8px 0;border-radius:5px;border:1px solid #ccc;}button{background:#3f51b5;color:#fff;cursor:pointer;}</style></head><body><div class="container"><h1>WiFi Setup</h1><p>Choose a network and enter the password.</p><form action="/save" method="POST"><select id="ssid" name="ssid"></select><br><input type="password" name="password" placeholder="Password"><br><button type="submit">Save & Connect</button></form></div><script>window.onload=function(){fetch("/scan").then(r=>r.json()).then(d=>{let s=document.getElementById("ssid");d.forEach(n=>{let o=document.createElement("option");o.value=n.ssid;o.innerText=n.ssid+" ("+n.rssi+")";s.appendChild(o)})})};</script></body></html>)rawliteral";
        server.send(200, "text/html", html);
    });
    server.on("/scan", HTTP_GET, []() {
        int n = WiFi.scanNetworks();
        String json = "[";
        for (int i = 0; i < n; ++i) { if (i) json += ","; json += "{\"ssid\":\"" + WiFi.SSID(i) + "\",\"rssi\":" + WiFi.RSSI(i) + "}"; }
        json += "]";
        server.send(200, "application/json", json);
    });
    server.on("/save", HTTP_POST, []() {
        server.arg("ssid").toCharArray(station_ssid, sizeof(station_ssid));
        server.arg("password").toCharArray(station_password, sizeof(station_password));
        saveCredentials();
        server.send(200, "text/html", "<html><body><h1>Settings Saved!</h1><p>Rebooting...</p></body></html>");
        delay(1000);
        ESP.restart();
    });
    server.begin();
}

void loadCredentials() { EEPROM.get(0, station_ssid); EEPROM.get(32, station_password); }
void saveCredentials() { EEPROM.put(0, station_ssid); EEPROM.put(32, station_password); EEPROM.commit(); }