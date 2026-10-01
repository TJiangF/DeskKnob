#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <vector>
#include <string>

struct WifiEntry {
    String ssid;
    String pass;
    bool connected = false;
};

class WifiManager {
public:
    WifiManager();

    // NVS storage
    void loadStored();                          // 从 NVS 加载已存条目
    int  addEntry(const String& ssid, const String& pass);  // 返回索引
    void removeEntry(int idx);
    int  storedCount() const { return (int)stored.size(); }
    const WifiEntry& storedAt(int idx) const { return stored[idx]; }

    // Scan surrounding APs (blocking ~2s)
    int  scan();                                // 返回找到的 SSID 数
    int  scanCount() const { return (int)found.size(); }
    const String& scanAt(int idx) const { return found[idx]; }
    int  scanRSSI(int idx) const { return rssi[idx]; }

    // Try connect by SSID using stored pass; returns true on success
    bool connectBySSID(const String& ssid);
    // Try connect by stored index
    bool connectByIndex(int idx);

    bool isConnected() const { return WiFi.status() == WL_CONNECTED; }
    String currentSSID() const { return isConnected() ? WiFi.SSID() : String(""); }
    IPAddress currentIP() const { return WiFi.localIP(); }

    void disconnect();

private:
    Preferences prefs;
    std::vector<WifiEntry> stored;
    std::vector<String>   found;
    std::vector<int>      rssi;
    static const char* NVS_NAMESPACE;
};

#endif // WIFI_MANAGER_H