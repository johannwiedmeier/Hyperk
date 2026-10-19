// File: src/wifi_recovery.cpp

#include <Arduino.h>
#include <WiFiUdp.h>
#include "main.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(HYPERK_WIFI_RECOVERY)
    #include <WiFi.h>
    #include <Update.h>
    #include <esp_wifi.h>
    #include <mutex>

namespace {
    std::recursive_mutex recoveryLock;
    bool recoveryEnabled = false;
    bool disconnected = false;
    uint32_t lastAttempt = 0;
    uint32_t retryDelay = 5000;
}

namespace WifiRecovery {
    void arm() {
        std::lock_guard<std::recursive_mutex> lock(recoveryLock);
        // Own sustained retries, including AP reason 8 which Arduino skips.
        WiFi.setAutoReconnect(false);
        recoveryEnabled = true;
        disconnected = false;
        retryDelay = 5000;
    }

    void suspend() {
        std::lock_guard<std::recursive_mutex> lock(recoveryLock);
        recoveryEnabled = false;
    }

    void processEvents(bool allowed) {
        static uint32_t lastCheck = 0;
        const uint32_t now = millis();
        if (uint32_t(now - lastCheck) < 250) return;
        lastCheck = now;

        // Never wait on a shutdown request in the streaming loop.
        std::unique_lock<std::recursive_mutex> lock(recoveryLock, std::try_to_lock);
        if (!lock.owns_lock() || !recoveryEnabled) return;
        const bool associated = WiFi.STA.connected();
        if (associated && WiFi.status() == WL_CONNECTED) {
            disconnected = false;
            retryDelay = 5000;
            return;
        }
        if ((WiFi.getMode() & WIFI_MODE_STA) == 0) {
            recoveryEnabled = false;
            return;
        }
        // Leave OTA, scans and an association awaiting DHCP alone.
        if (!allowed || associated || Update.isRunning() || WiFi.scanComplete() == WIFI_SCAN_RUNNING) {
            lastAttempt = now;
            return;
        }
        if (!disconnected) {
            disconnected = true;
            lastAttempt = now;
            return;
        }
        if (uint32_t(now - lastAttempt) < retryDelay) return;
        lastAttempt = now;
        retryDelay = retryDelay >= 30000 ? 60000 : retryDelay * 2;

        const esp_err_t result = esp_wifi_connect();
        Log::SERIAL_LOG("WiFi reconnect: ", int(result));
        (void)result;
    }
}

// Suspend retries before an intentional public WiFi.disconnect(), even if it fails.
// This linker adapter covers calls from the precompiled backend; with source
// access, call WifiRecovery::suspend() at those call sites instead.
extern "C" bool __real__ZN12WiFiSTAClass10disconnectEbbm(void*, bool, bool, unsigned long);
extern "C" bool __wrap__ZN12WiFiSTAClass10disconnectEbbm(void* self, bool off, bool erase, unsigned long timeout) {
    std::lock_guard<std::recursive_mutex> lock(recoveryLock);
    recoveryEnabled = false;
    return __real__ZN12WiFiSTAClass10disconnectEbbm(self, off, erase, timeout);
}
#else
namespace WifiRecovery {
    void arm() {}
    void suspend() {}
    void processEvents(bool) {}
}
#endif
