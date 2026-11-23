#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>

// ================================================================
// 1. HARDWARE & WIFI CONFIGURATION
// ================================================================
const char* ssid     = "Pendulum_Motor_Logger";
const char* password = "";

WebServer server(80);

// H-Bridge (IBT-2) & Optical Encoder Pinout
#define R_PWM     26
#define L_PWM     25
#define PIN_LM393 15

// ================================================================
// 2. ENCODER, FILTER & STEP-TEST VARIABLES
// ================================================================
const int ENCODER_SLOTS   = 4;
const int DATA_INTERVAL   = 50; // 20 Hz sampling
const int FILTER_WINDOW   = 4;  // Moving average window size

portMUX_TYPE timerMux              = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long pulseCount  = 0;
unsigned long lastCalcTime         = 0;

float rawRpm        = 0.0f;
float filteredOmega = 0.0f;
int currentPWM      = 0;

// Circular buffer for moving average filter
float readings[FILTER_WINDOW];
int readIndex = 0;
float total   = 0.0f;

// CSV logging state
bool isRecording                 = false;
bool stopRequest                 = false;
unsigned long recordingStartTime = 0;
File csvFile;

// Automated PWM step-sequence state
bool testActive                  = false;
bool testPhaseOn                 = false;
int testCurrentPWM               = 0;
unsigned long testPhaseStartTime = 0;
unsigned long phaseDuration      = 10000; // Default 10 s

// ================================================================
// 3. INTERRUPT SERVICE ROUTINE
// ================================================================
void IRAM_ATTR onSensorPulse() {
    portENTER_CRITICAL_ISR(&timerMux);
    pulseCount++;
    portEXIT_CRITICAL_ISR(&timerMux);
}

// ================================================================
// 4. WEB INTERFACE (PROGMEM)
// ================================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Automated Motor Step Analysis</title>
    <style>
        body { font-family: sans-serif; text-align: center; margin-top: 20px; background: #e0e0e0; }
        .box { background: #fff; margin: 15px auto; padding: 20px; width: 90%; max-width: 450px; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.15); }
        h2 { margin-top: 0; color: #333; }
        .data-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin: 20px 0; }
        .data-item { background: #f9f9f9; padding: 10px; border-radius: 5px; }
        .lbl { font-size: 0.9rem; color: #666; }
        .val { font-size: 1.4rem; font-weight: bold; color: #007BFF; }
        button { padding: 15px; font-size: 16px; width: 100%; margin: 5px 0; border: none; border-radius: 5px; cursor: pointer; color: white; font-weight: bold; transition: 0.2s; }
        input[type=number] { padding: 10px; font-size: 1.2rem; width: 60%; text-align: center; margin-bottom: 10px; border: 1px solid #ccc; border-radius: 5px; }
        .btn-start { background-color: #28a745; }
        .btn-stop { background-color: #dc3545; }
        .btn-set { background-color: #ff9800; width: 100%; }
        .btn-down { background-color: #0d6efd; text-decoration: none; display: block; padding: 15px; color: white; border-radius: 5px; margin-top: 10px; }
    </style>
</head>
<body>
    <div class="box">
        <h2>Step Settings</h2>
        <div class="lbl">Phase Duration (s)</div>
        <input type="number" id="durInput" min="1.0" max="10.0" step="0.1" value="10.0">
        <button class="btn-set" onclick="setDuration()">UPDATE</button>
    </div>

    <div class="box">
        <h2>Control</h2>
        <button id="btnMain" class="btn-start" onclick="toggleTest()">START TEST</button>
    </div>

    <div class="box">
        <h2>Telemetry</h2>
        <div class="data-grid">
            <div class="data-item"><div class="lbl">PWM</div><div id="pwmVal" class="val">0</div></div>
            <div class="data-item"><div class="lbl">Voltage</div><div id="voltVal" class="val">0.0 V</div></div>
            <div class="data-item" style="grid-column: span 2;"><div class="lbl">Phase Timer</div><div id="timerVal" class="val" style="color:#d63384">0 s</div></div>
        </div>
    </div>

    <div class="box">
        <h2>Export</h2>
        <a href="/download" class="btn-down" target="_blank">DOWNLOAD CSV</a>
    </div>

    <script>
        let testActive = false;

        function setDuration() {
            const val = document.getElementById("durInput").value;
            if (val < 1.0 || val > 10.0) {
                alert("Allowed range: 1.0 - 10.0 s");
                return;
            }
            fetch("/setDuration?val=" + Math.round(val * 1000)).then(r => {
                if (r.ok) alert("Updated");
            });
        }

        function toggleTest() {
            if (!testActive) {
                fetch("/startTest");
                fetch("/record?action=start");
            } else {
                fetch("/stopTest");
                document.getElementById("btnMain").innerText = "SAVING...";
                setTimeout(() => {
                    fetch("/record?action=stop");
                }, 500);
            }
        }

        setInterval(() => {
            fetch("/readData")
                .then(r => r.json())
                .then(data => {
                    document.getElementById("pwmVal").innerText = data.pwm;
                    document.getElementById("voltVal").innerText = ((data.pwm / 255.0) * 12.0).toFixed(1) + " V";
                    document.getElementById("timerVal").innerText = data.timer + " s";
                    testActive = (data.active == 1);

                    const btn = document.getElementById("btnMain");
                    if (testActive) {
                        btn.innerText = "STOP";
                        btn.className = "btn-stop";
                    } else {
                        btn.innerText = "START TEST";
                        btn.className = "btn-start";
                    }
                })
                .catch(e => console.log(e));
        }, 100);
    </script>
</body>
</html>
)rawliteral";

// ================================================================
// 5. MOTOR ACTUATION & SERVER HANDLERS
// ================================================================
void motorControl(int pwmVal) {
    if (pwmVal > 255) pwmVal = 255;
    if (pwmVal < 0) pwmVal = 0;

    analogWrite(R_PWM, pwmVal);
    analogWrite(L_PWM, 0);
    currentPWM = pwmVal;
}

void handleRoot() {
    server.send(200, "text/html", index_html);
}

void handleSetDuration() {
    if (server.hasArg("val")) {
        phaseDuration = server.arg("val").toInt();
        server.send(200, "text/plain", "OK");
    }
}

void handleStartTest() {
    testActive         = true;
    testCurrentPWM     = 15; // Initial PWM step (15 -> 75 in increments of 15)
    testPhaseOn        = true;
    testPhaseStartTime = millis();

    motorControl(testCurrentPWM);
    server.send(200, "text/plain", "Started");
}

void handleStopTest() {
    testActive = false;
    motorControl(0);
    server.send(200, "text/plain", "Stopped");
}

void handleReadData() {
    unsigned long seconds = testActive ? (millis() - testPhaseStartTime) / 1000 : 0;

    String json = "{";
    json += "\"pwm\":" + String(currentPWM) + ",";
    json += "\"active\":" + String(testActive ? 1 : 0) + ",";
    json += "\"timer\":" + String(seconds);
    json += "}";

    server.send(200, "application/json", json);
}

void handleRecord() {
    String action = server.arg("action");

    if (action == "start") {
        if (csvFile) csvFile.close();
        SPIFFS.remove("/dati.csv");

        csvFile = SPIFFS.open("/dati.csv", FILE_WRITE);
        if (!csvFile) {
            server.send(500, "text/plain", "SPIFFS Error");
            return;
        }

        csvFile.println("Time[s],Omega_Filt[rad/s],PWM_Input");
        recordingStartTime = millis();
        isRecording        = true;
        stopRequest        = false;

        // Reset moving average buffer
        for (int i = 0; i < FILTER_WINDOW; i++) {
            readings[i] = 0.0f;
        }
        readIndex = 0;
        total     = 0.0f;

        server.send(200, "text/plain", "Started");
    } else if (action == "stop") {
        if (isRecording) {
            stopRequest = true;
        }
        server.send(200, "text/plain", "Stopping...");
    }
}

void handleDownload() {
    if (SPIFFS.exists("/dati.csv")) {
        File file = SPIFFS.open("/dati.csv", FILE_READ);
        server.streamFile(file, "text/csv");
        file.close();
    } else {
        server.send(404, "text/plain", "Not found");
    }
}

// ================================================================
// 6. SETUP & MAIN LOOP
// ================================================================
void setup() {
    Serial.begin(115200);
    if (!SPIFFS.begin(true)) return;

    pinMode(R_PWM, OUTPUT);
    pinMode(L_PWM, OUTPUT);
    motorControl(0);

    for (int i = 0; i < FILTER_WINDOW; i++) {
        readings[i] = 0.0f;
    }

    pinMode(PIN_LM393, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_LM393), onSensorPulse, RISING);

    WiFi.softAP(ssid, password);

    server.on("/", handleRoot);
    server.on("/startTest", handleStartTest);
    server.on("/stopTest", handleStopTest);
    server.on("/setDuration", handleSetDuration);
    server.on("/readData", handleReadData);
    server.on("/record", handleRecord);
    server.on("/download", handleDownload);

    server.begin();
}

void loop() {
    server.handleClient();
    unsigned long currentTime = millis();

    // 1. Automated PWM step-response state machine
    if (testActive) {
        if (currentTime - testPhaseStartTime >= phaseDuration) {
            testPhaseStartTime = currentTime;

            if (testPhaseOn) {
                testPhaseOn = false;
                motorControl(0);
            } else {
                testCurrentPWM += 15;
                if (testCurrentPWM > 75) {
                    testActive = false;
                    motorControl(0);
                } else {
                    testPhaseOn = true;
                    motorControl(testCurrentPWM);
                }
            }
        }
    }

    // 2. Speed acquisition and moving average filtering
    if (currentTime - lastCalcTime >= DATA_INTERVAL) {
        portENTER_CRITICAL(&timerMux);
        unsigned long pulses = pulseCount;
        pulseCount = 0;
        portEXIT_CRITICAL(&timerMux);

        float dt = (float)(currentTime - lastCalcTime);
        if (dt > 0.0f) {
            rawRpm = ((float)pulses / ENCODER_SLOTS) * (60000.0f / dt);
        } else {
            rawRpm = 0.0f;
        }

        float instantRad = rawRpm * 0.10472f; // Convert RPM to rad/s

        total               = total - readings[readIndex];
        readings[readIndex] = instantRad;
        total               = total + readings[readIndex];
        readIndex           = (readIndex + 1) % FILTER_WINDOW;
        filteredOmega       = total / FILTER_WINDOW;

        // 3. Safe SPIFFS CSV logging
        if (isRecording && csvFile) {
            float relativeTime = (currentTime - recordingStartTime) / 1000.0f;
            csvFile.printf("%.3f,%.2f,%d\n", relativeTime, filteredOmega, currentPWM);

            if (stopRequest) {
                csvFile.flush();
                csvFile.close();
                isRecording = false;
                stopRequest = false;
            }
        }
        lastCalcTime = currentTime;
    }
}