#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <AS5600.h>
#include <SPIFFS.h>

// ================================================================
// 1. HARDWARE & CONTROL CONFIGURATION
// ================================================================
#define AS5600_SDA_PIN 21
#define AS5600_SCL_PIN 22

#define MOTOR_IN1 25
#define MOTOR_IN2 26

#define FC03_PIN      15
#define SLOTS_PER_REV 4
#define PWM_MAX       255

// Wi-Fi Access Point credentials
const char* apSSID     = "Pendulum_LQR_Controller";
const char* apPassword = "";

// Control loop timing (100 Hz)
const unsigned long samplePeriod_us = 10000; // 10 ms
const float dt                      = samplePeriod_us / 1000000.0f;

// LQR state-feedback gains (updated via web interface)
float K1 = 0.0f;
float K2 = 0.0f;
float K3 = 0.0f;
float K4 = 0.0f;

const float ANGLE_DEADBAND = 1.5f * (PI / 180.0f);
const float LIMIT_RAD      = 30.0f * (PI / 180.0f); // LQR active operating range

// ================================================================
// 2. GLOBAL VARIABLES & OBJECTS
// ================================================================
AS5600 as5600;
WebServer server(80);

// System state vector: [theta, theta_dot, alpha, alpha_dot]
float theta     = 0.0f;
float theta_dot = 0.0f;
float alpha     = 0.0f;
float alpha_dot = 0.0f;

// Controller & actuator state
bool controlActive            = false;
int current_pwm               = 0;
float theta_prev              = 0.0f;
float alpha_prev              = 0.0f;
float offset                  = 0.0f;
unsigned long lastControlTime = 0;

// Optical encoder pulse counters
volatile unsigned long pulseCount = 0;
unsigned long lastPulseCount      = 0;

// SPIFFS CSV data logger state
bool isRecording                 = false;
unsigned long recordingStartTime = 0;
File csvFile;

// ================================================================
// 3. WEB INTERFACE (PROGMEM)
// ================================================================
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <title>Reaction-Wheel Pendulum LQR</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: sans-serif; text-align: center; margin-top: 20px; background: #e0e0e0; }
        .box { background: #fff; margin: 15px auto; padding: 20px; width: 90%; max-width: 450px; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.15); }
        h2 { margin-top: 0; color: #333; font-size: 1.2rem; text-transform: uppercase; letter-spacing: 1px; }
        .val { font-size: 2.0rem; font-weight: bold; color: #007BFF; margin: 15px 0; }
        .grid-input { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; margin-bottom: 10px; }
        .input-group { text-align: left; }
        .lbl-small { font-size: 0.85rem; color: #666; margin-bottom: 5px; font-weight: bold; }
        input[type=number] { width: 100%; padding: 10px; font-size: 1.1rem; border: 1px solid #ccc; border-radius: 5px; box-sizing: border-box; }
        button { padding: 15px; font-size: 16px; width: 100%; margin: 5px 0; border: none; border-radius: 5px; cursor: pointer; color: white; font-weight: bold; transition: 0.2s; }
        .btn-zero { background-color: #ff9800; }
        .btn-start { background-color: #28a745; }
        .btn-stop { background-color: #dc3545; }
        .btn-set { background-color: #007bff; }
    </style>
</head>
<body>
    <!-- BLOCK 1: LQR GAIN TUNING -->
    <div class="box">
        <h2>LQR Feedback Gains</h2>
        <div class="grid-input">
            <div class="input-group">
                <div class="lbl-small">K1 (Theta)</div>
                <input type="number" id="k1" step="0.01" value="0">
            </div>
            <div class="input-group">
                <div class="lbl-small">K2 (Theta Dot)</div>
                <input type="number" id="k2" step="0.01" value="0">
            </div>
            <div class="input-group">
                <div class="lbl-small">K3 (Alpha)</div>
                <input type="number" id="k3" step="0.01" value="0">
            </div>
            <div class="input-group">
                <div class="lbl-small">K4 (Alpha Dot)</div>
                <input type="number" id="k4" step="0.01" value="0">
            </div>
        </div>
        <button class="btn-set" onclick="setGains()">UPDATE GAINS</button>
    </div>

    <!-- BLOCK 2: PENDULUM ANGLE -->
    <div class="box">
        <h2>Pendulum Angle</h2>
        <div id="ang" class="val">--&deg;</div>
        <button class="btn-zero" onclick="sendCmd('/zero')">SET ZERO REFERENCE</button>
    </div>

    <!-- BLOCK 3: CONTROLLER STATE -->
    <div class="box">
        <h2>Controller Status</h2>
        <div id="st" class="val" style="color:#666">--</div>
        <button id="btn-toggle" class="btn-start" onclick="sendCmd('/toggle')">ENABLE CONTROL</button>
    </div>

    <!-- BLOCK 4: MOTOR VOLTAGE -->
    <div class="box">
        <h2>Applied Voltage</h2>
        <div id="volt" class="val">-- V</div>
    </div>

    <!-- BLOCK 5: DATA LOGGER -->
    <div class="box">
        <h2>Data Logger</h2>
        <div id="recStatus" style="margin-bottom:10px; font-weight:bold; color:#666">Ready</div>
        <button id="btn-rec" class="btn-start" onclick="toggleRecord()">START LOGGING</button>
        <a href="/download" class="btn-down" target="_blank" style="text-decoration:none;">
            <button class="btn-set" style="background-color:#0d6efd;">DOWNLOAD CSV</button>
        </a>
    </div>

    <script>
        function setGains() {
            const k1 = document.getElementById("k1").value;
            const k2 = document.getElementById("k2").value;
            const k3 = document.getElementById("k3").value;
            const k4 = document.getElementById("k4").value;

            const url = "/setGains?k1=" + k1 + "&k2=" + k2 + "&k3=" + k3 + "&k4=" + k4;
            fetch(url).then(response => {
                if (response.ok) alert("LQR gains updated.");
                else alert("Error updating gains.");
            });
        }

        function sendCmd(url) {
            fetch(url).then(() => {
                if (url.includes("zero")) alert("Zero reference set.");
            });
        }

        function toggleRecord() {
            const btn = document.getElementById("btn-rec");
            const status = document.getElementById("recStatus");
            const action = btn.innerText.includes("START") ? "start" : "stop";

            fetch("/record?action=" + action).then(response => {
                if (response.ok) {
                    if (action === "start") {
                        btn.innerText = "STOP LOGGING";
                        btn.className = "btn-stop";
                        status.innerText = "RECORDING...";
                        status.style.color = "red";
                    } else {
                        btn.innerText = "START LOGGING";
                        btn.className = "btn-start";
                        status.innerText = "Recording complete. Ready to download.";
                        status.style.color = "green";
                    }
                }
            });
        }

        setInterval(() => {
            fetch("/data")
                .then(response => response.text())
                .then(text => {
                    const data = text.split(',');
                    if (data.length >= 3) {
                        document.getElementById("ang").innerText = data[0] + "°";
                        document.getElementById("volt").innerText = data[1] + " V";

                        const active = (parseInt(data[2]) === 1);
                        const btn = document.getElementById("btn-toggle");
                        const stLabel = document.getElementById("st");

                        if (active) {
                            btn.innerText = "DISABLE CONTROL";
                            btn.className = "btn-stop";
                            stLabel.innerText = "ACTIVE (LQR)";
                            stLabel.style.color = "#28a745";
                        } else {
                            btn.innerText = "ENABLE CONTROL";
                            btn.className = "btn-start";
                            stLabel.innerText = "INACTIVE";
                            stLabel.style.color = "#dc3545";
                        }
                    }
                })
                .catch(e => console.log(e));
        }, 100);
    </script>
</body>
</html>
)rawliteral";

// ================================================================
// 4. INTERRUPT, SENSOR & ACTUATOR UTILITIES
// ================================================================
void IRAM_ATTR countPulse() {
    pulseCount++;
}

float normalizeAndConvert(float angleRaw) {
    float a = angleRaw - offset;
    while (a < 0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    if (a > 180.0f) a -= 360.0f;
    return a * (PI / 180.0f);
}

void setMotorPWM(int pwmVal) {
    pwmVal = -pwmVal;

    // 1. Hardware saturation clamp
    if (pwmVal > PWM_MAX) pwmVal = PWM_MAX;
    if (pwmVal < -PWM_MAX) pwmVal = -PWM_MAX;

    // 2. Motor stiction deadband
    if (abs(pwmVal) < 15) {
        analogWrite(MOTOR_IN1, 0);
        analogWrite(MOTOR_IN2, 0);
        return;
    }

    // 3. H-Bridge direction and duty cycle actuation
    if (pwmVal > 0) {
        analogWrite(MOTOR_IN1, pwmVal);
        analogWrite(MOTOR_IN2, 0);
    } else {
        analogWrite(MOTOR_IN1, 0);
        analogWrite(MOTOR_IN2, abs(pwmVal));
    }
}

// ================================================================
// 5. REAL-TIME LQR CONTROL LOOP (100 Hz)
// ================================================================
void controlLoop() {
    unsigned long now_us = micros();

    if (now_us - lastControlTime >= samplePeriod_us) {
        float dt_real = (now_us - lastControlTime) / 1000000.0f;
        lastControlTime = now_us;

        // 1. Read pendulum angle from AS5600
        float rawDeg = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
        theta = normalizeAndConvert(rawDeg);

        // 2. Differentiate pendulum angular velocity with wrap-around handling
        float delta_theta = theta - theta_prev;
        if (delta_theta > PI) delta_theta -= TWO_PI;
        if (delta_theta < -PI) delta_theta += TWO_PI;
        theta_dot  = delta_theta / dt_real;
        theta_prev = theta;

        // 3. Compute flywheel angle increment from optical encoder pulses
        long pulses    = pulseCount - lastPulseCount;
        lastPulseCount = pulseCount;
        float motor_rps = ((float)pulses / (float)SLOTS_PER_REV) / dt_real;
        if (current_pwm < 0) motor_rps = -motor_rps;
        alpha = motor_rps * TWO_PI * dt_real;

        // 4. Differentiate flywheel angular velocity
        float delta_alpha = alpha - alpha_prev;
        alpha_dot  = delta_alpha / dt_real;
        alpha_prev = alpha;

        int target_pwm = 0;

        // 5. State-feedback control law
        if (controlActive) {
            // Case A: Inside linear operating region [-30 deg, +30 deg] -> LQR
            if (abs(theta) <= LIMIT_RAD) {
                if (abs(theta) < ANGLE_DEADBAND) {
                    target_pwm = 0;
                } else {
                    float u_raw = -(K1 * theta + K2 * theta_dot + K3 * alpha + K4 * alpha_dot);
                    float abs_u = abs(u_raw);

                    // Map voltage command [0..12 V] to PWM [15..255] to overcome static friction
                    if (abs_u > 0.1f) {
                        int base_pwm = 15 + (int)((abs_u / 12.0f) * (PWM_MAX - 15));
                        target_pwm   = (u_raw > 0.0f) ? base_pwm : -base_pwm;
                    } else {
                        target_pwm = 0;
                    }
                }
            }
            // Case B: Outside operating range (> 30 deg) -> Full-power recovery
            else {
                target_pwm = (theta > 0.0f) ? PWM_MAX : -PWM_MAX;
            }
        } else {
            target_pwm = 0;
        }

        // 6. Apply PWM command
        current_pwm = target_pwm;
        setMotorPWM(target_pwm);

        // 7. Log closed-loop telemetry to SPIFFS CSV
        if (isRecording && csvFile) {
            float t_sec   = (millis() - recordingStartTime) / 1000.0f;
            float ang_deg = theta * (180.0f / PI);
            float volt    = ((float)current_pwm / (float)PWM_MAX) * 12.0f;

            csvFile.printf("%.3f,%.2f,%.2f\n", t_sec, ang_deg, volt);
        }
    }
}

// ================================================================
// 6. WEB SERVER HANDLERS
// ================================================================
void handleRoot() {
    server.send(200, "text/html", webpage);
}

void handleSetGains() {
    if (server.hasArg("k1")) K1 = server.arg("k1").toFloat();
    if (server.hasArg("k2")) K2 = server.arg("k2").toFloat();
    if (server.hasArg("k3")) K3 = server.arg("k3").toFloat();
    if (server.hasArg("k4")) K4 = server.arg("k4").toFloat();

    Serial.print("Updated Gains -> K1: "); Serial.print(K1);
    Serial.print(" | K2: "); Serial.print(K2);
    Serial.print(" | K3: "); Serial.print(K3);
    Serial.print(" | K4: "); Serial.println(K4);

    server.send(200, "text/plain", "OK");
}

void handleZero() {
    offset     = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
    theta_prev = 0.0f;
    alpha_prev = 0.0f;
    server.send(200, "text/plain", "OK");
}

void handleToggle() {
    controlActive = !controlActive;

    if (controlActive) {
        float current_raw = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
        theta_prev        = normalizeAndConvert(current_raw);
        pulseCount        = 0;
        lastPulseCount    = 0;
        current_pwm       = 0;
    } else {
        setMotorPWM(0);
        current_pwm = 0;
    }

    server.send(200, "text/plain", controlActive ? "1" : "0");
}

void handleData() {
    float display_volt = (float)current_pwm * (12.0f / (float)PWM_MAX);

    String payload = String(theta * 180.0f / PI, 1) + "," +
                     String(display_volt, 1) + "," +
                     String(controlActive);

    server.send(200, "text/plain", payload);
}

void handleRecord() {
    String action = server.arg("action");

    if (action == "start") {
        if (csvFile) csvFile.close();
        SPIFFS.remove("/closed_loop_lqr.csv");

        csvFile = SPIFFS.open("/closed_loop_lqr.csv", FILE_WRITE);
        if (!csvFile) {
            server.send(500, "text/plain", "SPIFFS File Creation Error");
            return;
        }

        csvFile.println("Time,Angle,Voltage");
        recordingStartTime = millis();
        isRecording        = true;
        server.send(200, "text/plain", "Started");
    } else if (action == "stop") {
        isRecording = false;
        if (csvFile) {
            csvFile.close();
        }
        server.send(200, "text/plain", "Stopped");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

void handleDownload() {
    if (SPIFFS.exists("/closed_loop_lqr.csv")) {
        File file = SPIFFS.open("/closed_loop_lqr.csv", FILE_READ);
        server.streamFile(file, "text/csv");
        file.close();
    } else {
        server.send(404, "text/plain", "File not found");
    }
}

// ================================================================
// 7. SETUP & MAIN LOOP
// ================================================================
void setup() {
    Serial.begin(115200);

    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS mount failed");
        return;
    }

    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);

    pinMode(FC03_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(FC03_PIN), countPulse, RISING);

    Wire.begin(AS5600_SDA_PIN, AS5600_SCL_PIN);
    Wire.setClock(400000);
    if (!as5600.begin()) {
        Serial.println("AS5600 initialization failed");
        while (1);
    }
    as5600.setDirection(AS5600_CLOCK_WISE);
    offset = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;

    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSSID, apPassword);

    server.on("/", handleRoot);
    server.on("/zero", handleZero);
    server.on("/toggle", handleToggle);
    server.on("/data", handleData);
    server.on("/setGains", handleSetGains);
    server.on("/record", handleRecord);
    server.on("/download", handleDownload);

    server.begin();
    Serial.println("LQR Controller & Data Logger ready.");
}

void loop() {
    server.handleClient();
    controlLoop();
}