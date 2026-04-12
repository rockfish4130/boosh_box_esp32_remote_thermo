#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <esp_task_wdt.h>

// Build the existing Arduino sketch inside PlatformIO without forking the logic.
#include "../boosh_box_esp32_remote_thermo.ino"
