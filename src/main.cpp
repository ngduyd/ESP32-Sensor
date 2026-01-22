#include <Arduino.h>
#include <BH1750.h>
#include <NimBLEDevice.h>
#include <PubSubClient.h>
#include <SensirionI2cScd4x.h>
#include <WiFi.h>
#include <driver/i2s.h>
#include <esp_wifi.h>

#include "ConfigManager.h"

ConfigManager cfg;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

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

enum SystemState {
    STATE_WIFI_ACTIVE,
    STATE_BLE_MODE,
    STATE_OFFLINE
};

SystemState currentState = STATE_WIFI_ACTIVE;

#define SERVICE_UUID "71941c3b-9666-4bef-82f0-1099e4cbc99e"
#define CHAR_SSID_UUID "8cbda693-fde4-49df-a2e9-1fd9ef3ac3d4"
#define CHAR_PASS_UUID "bce09c25-d280-42f0-8b67-bcf162a445b2"
#define CHAR_MQTT_UUID "dfb28fc6-a1ce-4b71-9480-c7d68a86d544"

constexpr uint32_t kConnectTimeoutMs = 15000; // stop trying after 15 seconds
constexpr uint32_t kRetryDelayMs = 1000;      // wait between status polls
constexpr char BleDeviceName[] = "ESP32";

void setupBLE();
void stopBLE();
void setupWiFi();
void stopWiFi();
void mqttCallback(char *topic, byte *payload, unsigned int length);

static NimBLEServer *g_bleServer = nullptr;
static NimBLEService *g_bleService = nullptr;
static NimBLECharacteristic *ssidChar = nullptr;
static NimBLECharacteristic *passChar = nullptr;
static NimBLECharacteristic *mqttChar = nullptr;

String tmpSSID;
String tmpPASS;
String tmpMQTT;

bool gotSSID = false;
bool gotPASS = false;
bool gotMQTT = false;
bool readyToConnect = false;

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

class ServerCallbacks : public NimBLEServerCallbacks {
public:
    void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
        Serial.printf("Connected from: %s\n", connInfo.getAddress().toString().c_str());
        server->getAdvertising()->stop();
    }
    void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
        Serial.printf("Disconnected, reason: %d\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

static ServerCallbacks g_serverCallbacks;

class ServerCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo) override { Serial.println("[BLE] Characteristic read"); }
    void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo) override {
        isBusy = true;
        std::string value = characteristic->getValue();
        std::string uuid = characteristic->getUUID().toString();
        if (uuid == CHAR_SSID_UUID) {
            tmpSSID = String(value.c_str());
            gotSSID = true;
        } else if (uuid == CHAR_PASS_UUID) {
            tmpPASS = String(value.c_str());
            gotPASS = true;
        } else if (uuid == CHAR_MQTT_UUID) {
            tmpMQTT = String(value.c_str());
            gotMQTT = true;
        }
        if (gotSSID && gotPASS && gotMQTT) {
            readyToConnect = true;
            Serial.println(tmpSSID);
            Serial.println(tmpPASS);
            Serial.println(tmpMQTT);
            strcpy(cfg.data.ssid, tmpSSID.c_str());
            strcpy(cfg.data.pass, tmpPASS.c_str());
            strcpy(cfg.data.mqttHost, tmpMQTT.c_str());
            Serial.println("[BLE] Configuration updated, saving to NVS...");
            cfg.save();
            gotSSID = false;
            gotPASS = false;
            gotMQTT = false;
        }
        isBusy = false;
    }
};

void setupBLE() {
    Serial.println();
    Serial.println("=== BLE setup ===");
    NimBLEDevice::init(BleDeviceName);

    g_bleServer = NimBLEDevice::createServer();
    g_bleServer->setCallbacks(&g_serverCallbacks);

    g_bleService = g_bleServer->createService("180A");

    NimBLECharacteristic *devNameChar = g_bleService->createCharacteristic("2A00", NIMBLE_PROPERTY::READ);

    ssidChar = g_bleService->createCharacteristic(CHAR_SSID_UUID,
                                                  NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    passChar = g_bleService->createCharacteristic(CHAR_PASS_UUID,
                                                  NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    mqttChar = g_bleService->createCharacteristic(CHAR_MQTT_UUID,
                                                  NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    devNameChar->setValue(BleDeviceName);

    Serial.printf("Current Config:\nSSID: %s\nPASS: %s\nMQTT Host: %s\n",
                  cfg.data.ssid,
                  cfg.data.pass,
                  cfg.data.mqttHost);

    ssidChar->setValue(cfg.data.ssid);
    passChar->setValue(cfg.data.pass);
    mqttChar->setValue(cfg.data.mqttHost);

    ssidChar->setCallbacks(new ServerCharacteristicCallbacks());
    passChar->setCallbacks(new ServerCharacteristicCallbacks());
    mqttChar->setCallbacks(new ServerCharacteristicCallbacks());

    g_bleService->start();

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->setName(BleDeviceName);

    advertising->addServiceUUID(g_bleService->getUUID());
    // advertising->setScanResponse(true);
    // advertising->setMinPreferred(0x06);
    // advertising->setMinPreferred(0x12);

    advertising->start();
    // NimBLEDevice::startAdvertising();

    Serial.println("[BLE] Advertising started");
}

void stopBLE() {
    Serial.println("Stopping BLE safely...");
    Serial.flush();

    if (!NimBLEDevice::isInitialized()) {
        Serial.println("BLE not initialized");
        return;
    }

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    if (adv && adv->isAdvertising()) {
        adv->stop();
        delay(300);
    }

    if (g_bleServer && g_bleServer->getConnectedCount() > 0) {
        std::vector<uint16_t> clients = g_bleServer->getPeerDevices();
        Serial.printf("Found %d client(s)\n", clients.size());

        for (uint16_t clientId : clients) {
            Serial.printf("Disconnecting client %d...\n", clientId);
            g_bleServer->disconnect(clientId);
            delay(200);
        }

        delay(500);
    }

    if (ssidChar)
        ssidChar->setCallbacks(nullptr);
    if (passChar)
        passChar->setCallbacks(nullptr);
    if (mqttChar)
        mqttChar->setCallbacks(nullptr);
    if (g_bleServer)
        g_bleServer->setCallbacks(nullptr);

    delay(200);

    NimBLEDevice::deinit(true);

    ssidChar = nullptr;
    passChar = nullptr;
    mqttChar = nullptr;
    g_bleService = nullptr;
    g_bleServer = nullptr;

    Serial.println("✅ BLE stopped successfully!");
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.flush();
}

bool wifiConnecting = false;
bool wifiConnected = false;

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

void setupMQTT() {
    mqttClient.setServer(cfg.data.mqttHost, cfg.data.mqttPort);
    mqttClient.setCallback(mqttCallback);
}

void connectMQTT() {
    if (!mqttClient.connected()) {
        Serial.print("[MQTT] Connecting to MQTT broker...");
        if (mqttClient.connect("ESP32Client", "client", "123456")) {
            Serial.println("connected");
            mqttClient.subscribe("ESP32/cmd");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
        }
    }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
    String msg;
    for (int i = 0; i < length; i++)
        msg += (char)payload[i];
    Serial.println(msg);
    if (String(topic) == "ESP32/cmd") {
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
        mqttClient.publish("ESP32/status", "offline");
        mqttClient.disconnect();
        Serial.println("MQTT disconnected");
    }
}

void handlerButton() {
    bool state = digitalRead(BOOT_BTN);
    if (state == LOW && lastState == HIGH) {
        delay(30);
        if (digitalRead(BOOT_BTN) == LOW) {
            Serial.println("Button pressed!");
            switch (currentState) {
            case STATE_WIFI_ACTIVE:
                stopWiFi();
                stopMQTT();
                delay(1000);
                setupBLE();
                currentState = STATE_BLE_MODE;
                break;

            case STATE_BLE_MODE:
                stopBLE();
                delay(1000);
                asyncConnectWiFi(cfg.data.ssid, cfg.data.pass);
                setupMQTT();
                currentState = STATE_OFFLINE;
                strcpy(cfg.data.status, "offline");
                cfg.save();
                break;

            case STATE_OFFLINE:
                currentState = STATE_WIFI_ACTIVE;
                strcpy(cfg.data.status, "online");
                cfg.save();
                break;

            default:
                break;
            }
        }
    }
    lastState = state;
}

void setup() {
    delay(5000);
    Serial.begin(115200);
    delay(100);
    pinMode(BOOT_BTN, INPUT_PULLUP);
    Serial.begin(115200);
    if (!cfg.load()) {
        Serial.println("⚠️ Using default config...");
        cfg.save();
    } else {
        Serial.println("✅ Config loaded.");
    }
    Serial.printf("SSID: %s\n", cfg.data.ssid);
    Serial.printf("PASS: %s\n", cfg.data.pass);
    Serial.printf("MQTT: %s:%d\n", cfg.data.mqttHost, cfg.data.mqttPort);

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
    handlerButton();
    switch (currentState) {
    case STATE_WIFI_ACTIVE:
        asyncConnectWiFi(cfg.data.ssid, cfg.data.pass);
        if (wifiConnected) {
            if (!mqttClient.connected()) {
                setupMQTT();
                mqttClient.publish("ESP32/cmd", "online");
                strcpy(cfg.data.status, "online");
                cfg.save();
            }
            loopMQTT();
        }
        break;
    case STATE_BLE_MODE:
        // nothing to do, just wait for config via BLE
        if (readyToConnect) {
            stopBLE();
            delay(1000);
            currentState = STATE_WIFI_ACTIVE;
            readyToConnect = false;
        }
        break;

    case STATE_OFFLINE:
        loopMQTT();
        break;

    default:
        break;
    }
    delay(100);
}