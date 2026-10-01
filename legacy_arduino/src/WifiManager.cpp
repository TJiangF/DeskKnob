#include "WifiManager.h"

const char* WifiManager::NVS_NAMESPACE = "deskknob";

WifiManager::WifiManager() {}

void WifiManager::loadStored() {
    stored.clear();
    prefs.begin(NVS_NAMESPACE, true);
    int count = prefs.getInt("count", 0);
    for (int i = 0; i < count; i++) {
        String keyS = "ssid_" + String(i);
        String keyP = "pass_" + String(i);
        WifiEntry e;
        e.ssid = prefs.getString(keyS.c_str(), "");
        e.pass = prefs.getString(keyP.c_str(), "");
        if (e.ssid.length() > 0) stored.push_back(e);
    }
    prefs.end();
}

int WifiManager::addEntry(const String& ssid, const String& pass) {
    // 检查是否已存在同 SSID，存在则更新密码
    for (size_t i = 0; i < stored.size(); i++) {
        if (stored[i].ssid == ssid) {
            stored[i].pass = pass;
            // 持久化所有
            prefs.begin(NVS_NAMESPACE, false);
            prefs.putInt("count", (int)stored.size());
            String keyP = "pass_" + String(i);
            prefs.putString(keyP.c_str(), pass);
            prefs.end();
            return (int)i;
        }
    }
    WifiEntry e; e.ssid = ssid; e.pass = pass;
    stored.push_back(e);
    int idx = (int)stored.size() - 1;
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putInt("count", (int)stored.size());
    String keyS = "ssid_" + String(idx);
    String keyP = "pass_" + String(idx);
    prefs.putString(keyS.c_str(), ssid);
    prefs.putString(keyP.c_str(), pass);
    prefs.end();
    return idx;
}

void WifiManager::removeEntry(int idx) {
    if (idx < 0 || idx >= (int)stored.size()) return;
    stored.erase(stored.begin() + idx);
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putInt("count", (int)stored.size());
    // 重新写所有
    for (size_t i = 0; i < stored.size(); i++) {
        String keyS = "ssid_" + String(i);
        String keyP = "pass_" + String(i);
        prefs.putString(keyS.c_str(), stored[i].ssid);
        prefs.putString(keyP.c_str(), stored[i].pass);
    }
    // 清掉最后一个
    String keyS = "ssid_" + String(stored.size());
    String keyP = "pass_" + String(stored.size());
    prefs.remove(keyS.c_str());
    prefs.remove(keyP.c_str());
    prefs.end();
}

int WifiManager::scan() {
    found.clear();
    rssi.clear();
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++) {
        String ssid = WiFi.SSID(i);
        if (ssid.length() == 0) continue;
        // 去重
        bool dup = false;
        for (size_t k = 0; k < found.size(); k++) {
            if (found[k] == ssid) { dup = true; break; }
        }
        if (dup) continue;
        found.push_back(ssid);
        rssi.push_back(WiFi.RSSI(i));
    }
    WiFi.scanDelete();
    return (int)found.size();
}

bool WifiManager::connectBySSID(const String& ssid) {
    // 优先用 stored 密码
    String pass = "";
    for (size_t i = 0; i < stored.size(); i++) {
        if (stored[i].ssid == ssid) { pass = stored[i].pass; break; }
    }
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
        delay(100);
    }
    return WiFi.status() == WL_CONNECTED;
}

bool WifiManager::connectByIndex(int idx) {
    if (idx < 0 || idx >= (int)stored.size()) return false;
    WiFi.mode(WIFI_STA);
    WiFi.begin(stored[idx].ssid.c_str(), stored[idx].pass.c_str());
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
        delay(100);
    }
    return WiFi.status() == WL_CONNECTED;
}

void WifiManager::disconnect() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}