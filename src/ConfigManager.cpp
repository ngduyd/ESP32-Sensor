#include "ConfigManager.h"

ConfigManager::ConfigManager(const char* ns) : nvsNamespace(ns) {
    resetToDefault();
}

uint32_t ConfigManager::calcChecksum(const ConfigData &cfg) const {
    const uint8_t *ptr = reinterpret_cast<const uint8_t*>(&cfg);
    uint32_t sum = 0;
    for (size_t i = 0; i < sizeof(ConfigData) - sizeof(uint32_t); i++)
        sum += ptr[i];
    return sum;
}

bool ConfigManager::load() {
    prefs.begin(nvsNamespace, true); // read-only
    if (!prefs.isKey("cfg")) {
        prefs.end();
        Serial.println("[NVS] No config found");
        return false;
    }

    size_t cfgLen = prefs.getBytesLength("cfg");
    if (cfgLen != sizeof(ConfigData)) {
        prefs.end();
        Serial.println("[NVS] Config size mismatch");
        resetToDefault();
        return false;
    }

    prefs.getBytes("cfg", &data, sizeof(ConfigData));
    prefs.end();

    if (!isValid()) {
        Serial.println("[NVS] Invalid checksum -> reset to default");
        resetToDefault();
        return false;
    }

    Serial.println("[NVS] Config loaded successfully");
    return true;
}

bool ConfigManager::save() {
    data.checksum = calcChecksum(data);

    prefs.begin(nvsNamespace, false);
    size_t written = prefs.putBytes("cfg", &data, sizeof(ConfigData));
    prefs.end();

    if (written == sizeof(ConfigData)) {
        Serial.println("[NVS] Config saved successfully");
    } else {
        Serial.println("[NVS] Save failed");
    }
    return written == sizeof(ConfigData);
}

void ConfigManager::clear() {
    prefs.begin(nvsNamespace, false);
    prefs.clear();
    prefs.end();
    resetToDefault();
    Serial.println("[NVS] Config cleared");
}

void ConfigManager::resetToDefault() {
    memset(&data, 0, sizeof(data));
    strcpy(data.deviceId, "");
    strcpy(data.ssid, "");
    strcpy(data.pass, "");
    strcpy(data.mqttHost, "192.168.1.100");
    strcpy(data.status, "online");
    data.mqttPort = 1883;
    data.checksum = calcChecksum(data);
}

bool ConfigManager::isValid() const {
    return data.checksum == calcChecksum(data);
}
