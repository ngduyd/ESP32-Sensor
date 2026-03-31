#include <Arduino.h>
#include <BH1750.h>
#include <PubSubClient.h>
#include <SensirionI2cScd4x.h>
#include <WiFi.h>
#include <WebServer.h>
#include <driver/i2s.h>
#include <esp_wifi.h>

#include "ConfigManager.h"

ConfigManager cfg;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WebServer setupServer(80);

SensirionI2cScd4x sensor;

BH1750 lightMeter;

#define SAMPLE_BUFFER_SIZE 512
#define SAMPLE_RATE 8000
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
#define I2S_MIC_SERIAL_CLOCK 13
#define I2S_MIC_LEFT_RIGHT_CLOCK 12
#define I2S_MIC_SERIAL_DATA 11

#define BOOT_BTN 0
#define BAT_PIN 7
#define DIVIDER_RATIO 2.0129

int16_t sensor_error = 0;

bool lastState = HIGH;
bool WifiMode = true;
bool isBusy = false;
struct APM10_Data {
    int pm1_0;
    int pm2_5;
    int pm10;
    bool valid;
};

constexpr uint32_t kConnectTimeoutMs = 15000; // stop trying after 15 seconds
constexpr uint32_t kRetryDelayMs = 1000;      // wait between status polls
constexpr char BleDeviceName[] = "ESP32";

i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// and don't mess around with this
i2s_pin_config_t i2s_mic_pins = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SERIAL_DATA};

bool wifiConnecting = false;
bool wifiConnected = false;
bool mqttOnlineAnnounced = false;
bool forceProvisioningMode = false;
String sensorTopic = "/unknown";
String commandTopic = "/ESP32/unknown/cmd";

constexpr uint32_t kBootHoldResetMs = 3000;
bool bootHoldArmed = false;
uint32_t bootHoldStartMs = 0;

bool hasProvisionedConfig() {
    return strlen(cfg.data.deviceId) > 0 && strlen(cfg.data.ssid) > 0;
}

void updateTopics() {
    sensorTopic = "/sensors/" + String(cfg.data.deviceId);
    commandTopic = sensorTopic + "/cmd";
}

bool parseMqttEndpoint(const String &input, char *hostOut, size_t hostOutSize, int &portOut) {
    String endpoint = input;
    endpoint.trim();
    if (endpoint.length() == 0) {
        return false;
    }

    String hostPart = endpoint;
    int parsedPort = portOut;

    int firstColon = endpoint.indexOf(':');
    int lastColon = endpoint.lastIndexOf(':');
    if (firstColon > 0 && firstColon == lastColon && firstColon < endpoint.length() - 1) {
        String portPart = endpoint.substring(lastColon + 1);
        portPart.trim();

        bool allDigits = true;
        for (size_t i = 0; i < portPart.length(); i++) {
            if (!isDigit(portPart[i])) {
                allDigits = false;
                break;
            }
        }

        if (allDigits) {
            int maybePort = portPart.toInt();
            if (maybePort > 0 && maybePort <= 65535) {
                hostPart = endpoint.substring(0, lastColon);
                hostPart.trim();
                parsedPort = maybePort;
            }
        }
    }

    if (hostPart.length() == 0) {
        return false;
    }

    strncpy(hostOut, hostPart.c_str(), hostOutSize - 1);
    hostOut[hostOutSize - 1] = '\0';
    portOut = parsedPort;
    return true;
}

void handleBootButtonHoldInLoop() {
    if (digitalRead(BOOT_BTN) == LOW) {
        if (!bootHoldArmed) {
            bootHoldArmed = true;
            bootHoldStartMs = millis();
            Serial.println("[RESET] BOOT pressed. Hold for 3 seconds to factory reset...");
            return;
        }

        if (millis() - bootHoldStartMs >= kBootHoldResetMs) {
            Serial.println("[RESET] Hold confirmed. Clearing saved configuration...");
            cfg.clear();
            delay(200);
            Serial.println("[RESET] Restarting into provisioning mode...");
            ESP.restart();
        }
        return;
    }

    if (bootHoldArmed) {
        Serial.println("[RESET] BOOT released before timeout.");
        bootHoldArmed = false;
    }
}

void startSetupPortal() {
    const char *apName = "ESP32-Setup";

    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_AP);
    IPAddress apIp(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(apIp, gateway, subnet);

    bool apStarted = WiFi.softAP(apName);

    IPAddress ip = WiFi.softAPIP();
    Serial.printf("[SETUP] AP start: %s\n", apStarted ? "OK" : "FAILED");
    Serial.println("[SETUP] Provisioning AP started");
    Serial.printf("[SETUP] SSID: %s\n", apName);
    Serial.println("[SETUP] PASS: (open network)");
    Serial.printf("[SETUP] Open http://%s\n", ip.toString().c_str());

    setupServer.on("/", HTTP_GET, []() {
        String page =
            "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>ESP32 Setup</title></head><body>"
            "<h2>ESP32 First Run Setup</h2>"
            "<p>Connect to ESP32-Setup, then open 192.168.4.1</p>"
            "<form method='POST' action='/save'>"
            "<label>ID Number:</label><br><input name='id' maxlength='32' required><br><br>"
            "<label>WiFi SSID:</label><br><input name='ssid' maxlength='31' required><br><br>"
            "<label>WiFi Password:</label><br><input name='pass' maxlength='31' required><br><br>"
            "<label>MQTT Server (IP/Host[:Port]):</label><br><input name='mqtt' maxlength='63' required><br><br>"
            "<button type='submit'>Save</button></form>"
            "</body></html>";
        setupServer.send(200, "text/html", page);
    });

    setupServer.on("/save", HTTP_POST, []() {
        if (!setupServer.hasArg("id") || !setupServer.hasArg("ssid") || !setupServer.hasArg("pass") || !setupServer.hasArg("mqtt")) {
            setupServer.send(400, "text/plain", "Missing required fields");
            return;
        }

        String id = setupServer.arg("id");
        String ssid = setupServer.arg("ssid");
        String pass = setupServer.arg("pass");
        String mqtt = setupServer.arg("mqtt");

        id.trim();
        ssid.trim();
        pass.trim();
        mqtt.trim();

        if (id.length() == 0 || ssid.length() == 0 || pass.length() < 8 || mqtt.length() == 0) {
            setupServer.send(400, "text/plain", "Invalid input. Password must be at least 8 characters.");
            return;
        }

        strncpy(cfg.data.deviceId, id.c_str(), sizeof(cfg.data.deviceId) - 1);
        cfg.data.deviceId[sizeof(cfg.data.deviceId) - 1] = '\0';
        strncpy(cfg.data.ssid, ssid.c_str(), sizeof(cfg.data.ssid) - 1);
        cfg.data.ssid[sizeof(cfg.data.ssid) - 1] = '\0';
        strncpy(cfg.data.pass, pass.c_str(), sizeof(cfg.data.pass) - 1);
        cfg.data.pass[sizeof(cfg.data.pass) - 1] = '\0';
        int mqttPort = 1883;
        if (!parseMqttEndpoint(mqtt, cfg.data.mqttHost, sizeof(cfg.data.mqttHost), mqttPort)) {
            setupServer.send(400, "text/plain", "Invalid MQTT server. Use host or host:port.");
            return;
        }
        cfg.data.mqttPort = mqttPort;

        if (!cfg.save()) {
            setupServer.send(500, "text/plain", "Failed to save config");
            return;
        }

        setupServer.send(200, "text/html", "Saved. Device will restart and connect to WiFi.");
        delay(1000);
        ESP.restart();
    });

    setupServer.begin();
    while (true) {
        setupServer.handleClient();
        delay(10);
    }
}

void setupWiFi() {
    Serial.println();
    Serial.println("=== WiFi setup ===");
    Serial.printf("Connecting to SSID: %s\n", cfg.data.ssid);
    // setup wifi station mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true, true);
    // clear previous connections
    delay(1000);
    WiFi.begin(cfg.data.ssid, cfg.data.pass);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_max_tx_power(44);
    uint32_t startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < kConnectTimeoutMs) {
        Serial.print(".");
        delay(1000);
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println();
        Serial.println("Failed to connect to WiFi!");
    } else {
        Serial.println();
        Serial.println("Connected to WiFi!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    }
}

void stopWiFi() {
    Serial.println("[WiFi] Stopping...");
    delay(50);
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    wifiConnecting = false;
    wifiConnected = false;
    delay(100);
    Serial.println("[WiFi] Stopped");
}

void asyncConnectWiFi(const char *ssid, const char *pass) {
    static unsigned long startTime = 0;
    if (!wifiConnecting && !wifiConnected) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, pass);
        esp_wifi_set_ps(WIFI_PS_NONE);
        esp_wifi_set_max_tx_power(34);
        startTime = millis();
        wifiConnecting = true;
        Serial.printf("[WiFi] Connecting to %s...\n", ssid);
    }
    if (wifiConnecting) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("[WiFi] ✅ Connected!");
            Serial.print("[WiFi] IP Address: ");
            Serial.println(WiFi.localIP());
            wifiConnecting = false;
            wifiConnected = true;
        } else if (millis() - startTime > kConnectTimeoutMs) {
            Serial.println("[WiFi] ❌ Timeout");
            wifiConnecting = false;
            wifiConnected = false;
        }
    }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
    String msg;
    for (int i = 0; i < length; i++)
        msg += (char)payload[i];
    Serial.println(msg);
    if (String(topic) == commandTopic) {
        if (msg == "disconnect") {
            // remove all stored wifi config and restart to BLE mode
            Serial.println("Received disconnect command via MQTT");
            cfg.clear();
            Serial.println("Cleared stored config, restarting...");
            ESP.restart();
        }
        strcpy(cfg.data.status, msg.c_str());
        cfg.save();
    }
}

void setupMQTT() {
    mqttClient.setServer(cfg.data.mqttHost, cfg.data.mqttPort);
    mqttClient.setCallback(mqttCallback);
}

void connectMQTT() {
    if (!mqttClient.connected()) {
        String clientId = "ESP32-" + String(cfg.data.deviceId);
        Serial.print("[MQTT] Connecting to MQTT broker...");
        if (mqttClient.connect(clientId.c_str(), sensorTopic.c_str(), 1, true, "offline")) {
            Serial.println("connected");
            mqttClient.subscribe(commandTopic.c_str());
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
        }
    }
}

float readBatteryVoltage() {
    float vbat = (analogReadMilliVolts(BAT_PIN) / 1000.0f) * DIVIDER_RATIO;
    return vbat;
}

void apm10_begin(uint8_t addr = 0x08) {
    Wire.beginTransmission(addr);
    Wire.write(0x00);
    Wire.write(0x10);
    Wire.write(0x05);
    Wire.write(0x00);
    Wire.write(0xF6);
    byte error = Wire.endTransmission();

    if (error == 0) {
        Serial.println("[APM10] Wake Command Sent. Waiting 5s for fan...");
    } else {
        Serial.print("[APM10] Wake Error: ");
        Serial.println(error);
    }

    delay(5000);
}

APM10_Data apm10_read(uint8_t addr = 0x08) {
    APM10_Data data = {0, 0, 0, false};

    Wire.beginTransmission(addr);
    Wire.write(0x03);
    Wire.write(0x00);

    if (Wire.endTransmission(true) != 0) {
        return data;
    }

    int qty = 30;
    if (Wire.requestFrom(addr, qty) == qty) {
        byte buf[30];
        Wire.readBytes(buf, qty);

        data.pm1_0 = (buf[0] << 8) | buf[1];
        data.pm2_5 = (buf[3] << 8) | buf[4];
        data.pm10 = (buf[9] << 8) | buf[10];

        if (data.pm2_5 != 0xFFFF) {
            data.valid = true;
        }
    }

    return data;
}

void loopMQTT() {
    if (WiFi.status() != WL_CONNECTED) {
        asyncConnectWiFi(cfg.data.ssid, cfg.data.pass);
        return;
    }
    if (!mqttClient.connected()) {
        connectMQTT();
    }
    mqttClient.loop();

    if (strcmp(cfg.data.status, "online") == 0) {
        bool dataReady = false;
        uint16_t co2Concentration = 0;
        float temperature = 0.0;
        float relativeHumidity = 0.0;
        uint32_t pressure = 0;
        // push data from sensors
        sensor.getDataReadyStatus(dataReady);
        float vbat = readBatteryVoltage();
        float lux = lightMeter.readLightLevel();
        int32_t raw_samples[SAMPLE_BUFFER_SIZE];
        size_t bytesRead = 0;
        i2s_read(I2S_NUM_0, (void *)raw_samples, SAMPLE_BUFFER_SIZE * sizeof(int32_t), &bytesRead, portMAX_DELAY);
        int32_t avg_level = 0;
        for (size_t i = 0; i < bytesRead / sizeof(int32_t); i++) {
            avg_level += abs(raw_samples[i]);
        }
        APM10_Data pmData = apm10_read();
        if (pmData.valid) {
            if (dataReady) {
                sensor_error = sensor.readMeasurement(co2Concentration, temperature, relativeHumidity);
                if (sensor_error == 0) {
                    Serial.printf("CO2: %d ppm, Temp: %.2f C, RH: %.2f %%, PM2.5: %d ug/m3, PM10: %d ug/m3, Lux: %.2f lx, Mic: %.2f\n",
                                  co2Concentration,
                                  temperature,
                                  relativeHumidity,
                                  pmData.pm2_5,
                                  pmData.pm10,
                                  lux,
                                  (float)avg_level / (bytesRead / sizeof(int32_t)));
                    Serial.printf("MQTT route: %s\n", sensorTopic.c_str());
                    char payload[200];
                    snprintf(payload, sizeof(payload),
                             "{\"co2\": %d, \"temp\": %.2f, \"rh\": %.2f, \"vbat\": %.2f, \"lux\": %.2f, \"mic\": %.2f, \"pm2_5\": %d, \"pm10\": %d}",
                             co2Concentration,
                             temperature,
                             relativeHumidity,
                             vbat,
                             lux,
                             (float)avg_level / (bytesRead / sizeof(int32_t)),
                             pmData.pm2_5,
                             pmData.pm10);
                    mqttClient.publish(sensorTopic.c_str(), payload);
                } else {
                    Serial.print("Sensor read error: ");
                    Serial.println(sensor_error);
                }
            }
        }
    }
}

void stopMQTT() {
    if (mqttClient.connected()) {
        mqttClient.publish(sensorTopic.c_str(), "offline", true);
        mqttClient.disconnect();
        Serial.println("MQTT disconnected");
    }
    mqttOnlineAnnounced = false;
}

void setup() {
    Serial.begin(115200);
    const uint32_t serialWaitStart = millis();
    while (!Serial && millis() - serialWaitStart < 2500) {
        delay(10);
    }
    delay(100);
    Serial.println("\n[BOOT] ESP32 starting...");
    pinMode(BOOT_BTN, INPUT_PULLUP);

    if (!cfg.load()) {
        Serial.println("[CFG] No valid saved config");
    } else {
        Serial.println("[CFG] Config loaded");
    }

    if (forceProvisioningMode || !hasProvisionedConfig()) {
        if (forceProvisioningMode) {
            Serial.println("[CFG] Entering provisioning mode due to BOOT reset.");
        }
        startSetupPortal();
    }

    updateTopics();
    Serial.printf("ID: %s\n", cfg.data.deviceId);
    Serial.printf("SSID: %s\n", cfg.data.ssid);
    Serial.printf("PASS: %s\n", cfg.data.pass);
    Serial.printf("MQTT: %s:%d\n", cfg.data.mqttHost, cfg.data.mqttPort);
    Serial.printf("Topic: %s\n", sensorTopic.c_str());

    Wire.begin(8, 9);
    sensor.begin(Wire, SCD41_I2C_ADDR_62);
    lightMeter.begin();
    apm10_begin(0x08);
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &i2s_mic_pins);
    Serial.println("Waking up sensor...");
    delay(50);
    sensor.stopPeriodicMeasurement();
    delay(500);
    sensor_error = sensor.wakeUp();
    sensor_error = sensor.startPeriodicMeasurement();
    if (sensor_error) {
        Serial.print("Sensor error during startup: ");
        Serial.println(sensor_error);
    }
}

void loop() {
    handleBootButtonHoldInLoop();

    asyncConnectWiFi(cfg.data.ssid, cfg.data.pass);
    if (wifiConnected) {
        if (!mqttClient.connected()) {
            setupMQTT();
            connectMQTT();
        }
        if (mqttClient.connected() && !mqttOnlineAnnounced) {
            mqttClient.publish((sensorTopic + "/status").c_str(), "online");
            strcpy(cfg.data.status, "online");
            cfg.save();
            mqttOnlineAnnounced = true;
        }
        loopMQTT();
    }
    delay(100);
}