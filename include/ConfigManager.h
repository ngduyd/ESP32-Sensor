#pragma once
#include <Arduino.h>
#include <Preferences.h>

struct ConfigData {
    char deviceId[33];
    char ssid[32];
    char pass[32];
    char mqttHost[64];
    int  mqttPort;
    char status[20];
    uint32_t checksum;
};

class ConfigManager {
public:
    ConfigManager(const char* ns = "config");
    bool load();                
    bool save();
    void clear();
    void resetToDefault();
    bool isValid() const;

    ConfigData data;

private:
    uint32_t calcChecksum(const ConfigData &cfg) const;
    Preferences prefs;
    const char* nvsNamespace;
};
