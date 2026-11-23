#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <AS5600.h>

// ================================================================
// 1. HARDWARE & SAMPLING CONFIGURATION
// ================================================================
AS5600 as5600;

#define AS5600_SDA_PIN 21
#define AS5600_SCL_PIN 22

// Wi-Fi Access Point credentials
const char* apSSID     = "Pendulum_Angle_Logger";
const char* apPassword = "";

WebServer server(80);

// Sampling settings (100 Hz)
const unsigned long samplePeriod_us = 10000; // 10 ms
unsigned long lastSample_us         = 0;
unsigned long startTime_ms          = 0;
float offset                        = 0.0;

// ================================================================
// 2. WEB INTERFACE (PROGMEM)
// ================================================================
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <title>AS5600 Data Logger</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: sans-serif; text-align: center; margin-top: 30px; background: #f4f4f4; }
        .box { background: #fff; margin: 20px auto; padding: 20px; width: 90%; max-width: 400px; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
        h2 { color: #333; }
        .val { font-size: 2.5rem; font-weight: bold; color: #007BFF; margin: 20px 0; }
        button { padding: 15px; font-size: 16px; width: 100%; margin: 10px 0; border: none; border-radius: 5px; cursor: pointer; color: white; font-weight: bold; }
        .btn-zero { background-color: #ff9800; }
        .btn-start { background-color: #28a745; }
        .btn-stop { background-color: #dc3545; }
        .lbl { color: #666; font-size: 0.9rem; }
    </style>
</head>
<body>
    <div class="box">
        <h2>AS5600 Data Logger</h2>
        <div class="lbl">Current Angle</div>
        <div id="ang" class="val">--&deg;</div>
        <button class="btn-zero" onclick="setZero()">SET ZERO REFERENCE</button>
    </div>

    <div class="box">
        <button class="btn-start" onclick="startLog()">START LOGGING (Reset Time)</button>
        <button class="btn-stop" onclick="stopLog()">STOP & DOWNLOAD CSV</button>
        <div style="margin-top:10px;">Status: <span id="status" style="font-weight:bold">Idle</span></div>
    </div>

    <script>
        let logging = false;
        let dataBuffer = "Time(ms),Angle(deg)\n";

        if (!!window.EventSource) {
            const source = new EventSource('/events');
            source.addEventListener('message', function(e) {
                const dataParts = e.data.split(',');
                const t = dataParts[0];
                const a = dataParts[1];

                document.getElementById("ang").innerHTML = a + "&deg;";
                if (logging) {
                    dataBuffer += t + "," + a + "\n";
                }
            }, false);
        }

        function setZero() {
            fetch("/set_zero").then(() => {
                const st = document.getElementById("status");
                st.textContent = "Zero reference set";
                st.style.color = "green";
                setTimeout(() => {
                    st.textContent = "Idle";
                    st.style.color = "black";
                }, 3000);
            });
        }

        function startLog() {
            fetch("/start_log")
                .then(response => {
                    if (response.ok) {
                        dataBuffer = "Time(ms),Angle(deg)\n";
                        logging = true;
                        const st = document.getElementById("status");
                        st.textContent = "RECORDING...";
                        st.style.color = "red";
                    } else {
                        document.getElementById("status").textContent = "Time reset error. Retry.";
                    }
                })
                .catch(() => {
                    document.getElementById("status").textContent = "Network error starting log.";
                });
        }

        function stopLog() {
            logging = false;
            const st = document.getElementById("status");
            st.textContent = "Recording Stopped";
            st.style.color = "red";
            downloadCSV();
            setTimeout(() => {
                st.textContent = "Idle";
                st.style.color = "black";
            }, 3000);
        }

        function downloadCSV() {
            const blob = new Blob([dataBuffer], { type: 'text/csv' });
            const url = window.URL.createObjectURL(blob);
            const a = document.createElement("a");
            a.href = url;
            a.download = "angle_identification.csv";
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
        }
    </script>
</body>
</html>
)rawliteral";

// ================================================================
// 3. UTILITY FUNCTIONS
// ================================================================
float normalizeAngle(float angleRaw) {
    float a = angleRaw - offset;
    while (a < 0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    if (a > 180.0f) a -= 360.0f;
    return a;
}

// ================================================================
// 4. WEB SERVER HANDLERS
// ================================================================
void handleRoot() {
    server.send(200, "text/html", webpage);
}

void handleStartLog() {
    startTime_ms = millis();
    server.send(200, "text/plain", "OK");
}

void handleZero() {
    offset = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
    server.send(200, "text/plain", "Zero Set");
}

void handleEvents() {
    WiFiClient client = server.client();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/event-stream");
    client.println("Cache-Control: no-cache");
    client.println("Connection: keep-alive");
    client.println();

    lastSample_us = micros();

    while (client.connected()) {
        unsigned long now_us = micros();

        if (now_us - lastSample_us >= samplePeriod_us) {
            lastSample_us += samplePeriod_us;
            unsigned long elapsedTime_ms = millis() - startTime_ms;

            float rawDeg = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
            float angle  = normalizeAngle(rawDeg);

            client.print("data: ");
            client.print(elapsedTime_ms);
            client.print(",");
            client.print(angle, 2);
            client.print("\n\n");
        }
        server.handleClient();
    }
}

// ================================================================
// 5. SETUP & MAIN LOOP
// ================================================================
void setupWiFi() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSSID, apPassword);
}

void setupAS5600() {
    Wire.begin(AS5600_SDA_PIN, AS5600_SCL_PIN);
    Wire.setClock(400000);
    as5600.begin(4);
    as5600.setDirection(AS5600_CLOCK_WISE);
    as5600.setOffset(offset);
}

void setupWebServer() {
    server.on("/", handleRoot);
    server.on("/events", handleEvents);
    server.on("/start_log", handleStartLog);
    server.on("/set_zero", handleZero);
    server.begin();
}

void setup() {
    Serial.begin(115200);
    setupAS5600();
    setupWiFi();
    setupWebServer();
    startTime_ms = millis();
}

void loop() {
    server.handleClient();
}