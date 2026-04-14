/*
Lava Lounge DJ Boosh Box Remote Thermo

Ported to PlatformIO for:
ESP32 ESP-32D WIFI Development Board Module CH340C With 0.96 OLED Screen
Yellow Blue Display with ESP32-N4XX module
*/

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <esp_bt.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <math.h>

#if __has_include("soc/rtc_wdt.h")
#include "soc/rtc_wdt.h"
#define BOOSH_HAS_RTC_WDT 1
#else
#define BOOSH_HAS_RTC_WDT 0
#endif

#include "wifi_credentials.h"

#define FW_VERSION "1.1.0"
#define FW_BUILD_DATE __DATE__
#define FW_BUILD_TIME __TIME__
#define SW_REV_CODE FW_VERSION
#define WDT_TIMEOUT 10

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN -1
#endif

#ifndef THERMISTOR_PIN1
#define THERMISTOR_PIN1 36
#endif

#ifndef THERMISTOR_PIN2
#define THERMISTOR_PIN2 39
#endif

#ifndef OLED_SDA_PIN
#define OLED_SDA_PIN 21
#endif

#ifndef OLED_SCL_PIN
#define OLED_SCL_PIN 22
#endif

#ifndef OLED_I2C_ADDR
#define OLED_I2C_ADDR 0x3C
#endif

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define DEBUG_DISPLAY_DIAGNOSTIC_UPDATE_PERIOD_MSEC 500
#define NETWORK_HOSTNAME "REMOTETHERMO"
#define TARGET_HOSTNAME "RPIBOOSH"
#define AP_SSID "FIRE_REMOTETHERMO"
#define AP_IP IPAddress(10, 1, 2, 3)
constexpr const char *kPrefsNamespace = "rthermo_cfg";
constexpr const char *kPrefsKeyAp     = "ap_en";
constexpr const char *kPrefsKeyUserSsid = "usr_ssid";
constexpr const char *kPrefsKeyUserPass = "usr_pass";
#define WIFI_CONNECTION_TIMEOUT_SECS 30
#define CPU_FREQ_MHZ 80

#define THERMISTOR_COUNT 2
#define THERMISTOR_MEAS_AVG_COUNT 16
#define THERMISTOR_MEAS_AVG_DELAY_USEC 100
#define THERMISTOR_MEAS_AVG_RIGHT_BIT_SHIFT 4

#define TEMP_HISTORY_SAMPLE_PERIOD_MSEC 120000UL
#define TEMP_HISTORY_DAYS 5
#define TEMP_HISTORY_CAPACITY ((TEMP_HISTORY_DAYS * 24UL * 60UL * 60UL * 1000UL) / TEMP_HISTORY_SAMPLE_PERIOD_MSEC)
#define TEMP_HISTORY_API_MAX_POINTS 480
#define SHORT_TERM_HISTORY_CAPACITY 600
#define OLED_PAGE_PERIOD_MSEC 3500UL
#define LOG_LINE_MAX_LEN 120
#define LOG_LINE_COUNT 96

struct TempHistoryPoint {
  unsigned long millisStamp;
  float t0;
  float t1;
};

enum OledPage {
  OLED_PAGE_TEMPS = 0,
  OLED_PAGE_WIFI = 1,
  OLED_PAGE_HTTP = 2,
  OLED_PAGE_SYSTEM = 3,
  OLED_PAGE_COUNT = 4
};

const uint8_t OLED_PAGE_SEQUENCE[] = {
  OLED_PAGE_TEMPS, OLED_PAGE_TEMPS, OLED_PAGE_TEMPS, OLED_PAGE_TEMPS, OLED_PAGE_TEMPS,
  OLED_PAGE_TEMPS, OLED_PAGE_TEMPS, OLED_PAGE_WIFI, OLED_PAGE_HTTP, OLED_PAGE_SYSTEM
};
const int OLED_PAGE_SEQUENCE_LEN = sizeof(OLED_PAGE_SEQUENCE) / sizeof(OLED_PAGE_SEQUENCE[0]);

hw_timer_t *myTimerBPMPulseBoosh = NULL;
WebServer webServer(80);
DNSServer dnsServer;
bool ap_enabled = false;
bool ap_active = false;
bool webServerStarted = false;
String user_wifi_ssid;
String user_wifi_pass;
HTTPClient http;
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int httpResponseCode = -999;
unsigned long currentMillis = 0;
bool enableSerial0DisplayUpdates = 0;
volatile unsigned long lastADCMeasurementTimeMsec = 0;
unsigned long lastDisplayDiagnosticSerialDumpTimeMsec = 0;
unsigned long timerPeriodADCISRMeasuSec = 3000000;
volatile bool flagNeedToPostNewTempMeasurement = 0;
volatile bool flagNeedToCalculateNewTempMeasurement = 0;
volatile bool flagNeedToSampleADC = 0;
unsigned long lastHttpPostTimeMSec = 0;
unsigned long lastHttpPostIntervalMSec = 10000;
unsigned long lastHttpReponseGoodTimestampMsec = 0;
unsigned long lastHttpReponseBadTimestampMsec = 0;
String urlBase = String("http://") + String(TARGET_HOSTNAME) + String(".local");
String urlPostfix = "/F";
String serverPath;
IPAddress DJBOOSHBOXIp = IPAddress(1, 1, 1, 1);
IPAddress lastDJBOOSHBOXIp = IPAddress(1, 1, 1, 1);
unsigned long lastmDNSLookupTimeStampMSec = 0;
unsigned long mDNSLookupTimeIntervalMSec = 15UL * 60UL * 1000UL;

const int ThermistorPins[THERMISTOR_COUNT] = {THERMISTOR_PIN1, THERMISTOR_PIN2};
int Vo;
volatile int adcRawReads[THERMISTOR_COUNT];
float R1 = 10000;
float logR2, R2, T[THERMISTOR_COUNT] = {NAN, NAN};
float c1 = 1.274219988e-03F, c2 = 2.171368266e-04F, c3 = 1.119659695e-07F;
float manualTempOffsetF = 11.17F;
esp_reset_reason_t bootResetReason = ESP_RST_UNKNOWN;

TempHistoryPoint tempHistory[TEMP_HISTORY_CAPACITY];
int tempHistoryCount = 0;
int tempHistoryNextIndex = 0;
unsigned long lastTempHistoryStoreMillis = 0;
TempHistoryPoint shortTermHistory[SHORT_TERM_HISTORY_CAPACITY];
int shortTermHistoryCount = 0;
int shortTermHistoryNextIndex = 0;
String logLines[LOG_LINE_COUNT];
int logLineNextIndex = 0;
int logLineCount = 0;
bool lastTempsWereValid = false;
int lastLoggedHttpResponseCode = -999999;

bool oledAvailable = false;
unsigned long lastOledPageChangeMsec = 0;
int oledPageIndex = OLED_PAGE_TEMPS;

IPAddress resolve_mdns_host(const char *host_name);
void loadConfig();
void saveConfig();
String buildConfigApiJson();
void sendApiError(int code, const char *msg);
void handleWebRoot();
void handleApiStatus();
void handleApiHistory();
void handleApiLogs();
void handleApiConfigGet();
void handleApiConfigPost();
void handleApiConfigAp();
void handleCaptivePortalRedirect();
void setupWebServer();
void setupApMode();
void stopApMode();
void serial0DebugCom();
void displayDiagnosticUpdate();
void updateOledDisplay();
void initOledDisplay();
void appendTemperatureHistory(float t0, float t1, unsigned long stampMillis);
float bufferMinTemp(TempHistoryPoint *buf, int bufCapacity, int bufCount, int bufNextIndex);
float bufferMaxTemp(TempHistoryPoint *buf, int bufCapacity, int bufCount, int bufNextIndex);
int getSampledBufferIndices(TempHistoryPoint *buf, int bufCapacity, int bufCount, int bufNextIndex, int *indices, int maxPoints);
int getSampledBufferIndicesForLookback(TempHistoryPoint *buf, int bufCapacity, int bufCount, int bufNextIndex, int *indices, int maxPoints, unsigned long lookbackMs);
bool isSsidVisible(const char *ssid);
bool connectToPreferredWifi();
String buildDashboardHtml();
String buildStatusJson(bool includeHistory);
String buildHistoryJson();
String buildLogJson();
String buildTemperaturePlotSvg(TempHistoryPoint *buf, int bufCapacity, int bufCount, int bufNextIndex, const char *emptyLabel, const char *timeAxisLabel);
String formatUptimeHMS(unsigned long ms);
String formatAgeSeconds(unsigned long sinceMillis);
String formatTempDisplay(float value);
String resetReasonToString(esp_reset_reason_t reason);
void appendLogLine(const String &line);
void logEvent(const String &line);
void logHttpEvent(const String &label, int code, const String &detail = "");

String buildVersionString() {
  return String(FW_VERSION) + " " + String(FW_BUILD_DATE) + " " + String(FW_BUILD_TIME);
}

String jsonEscape(const String &input) {
  String out;
  out.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); ++i) {
    char c = input[i];
    switch (c) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

String htmlEscape(const String &input) {
  String out;
  out.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); ++i) {
    char c = input[i];
    switch (c) {
      case '&':
        out += "&amp;";
        break;
      case '<':
        out += "&lt;";
        break;
      case '>':
        out += "&gt;";
        break;
      case '"':
        out += "&quot;";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

void loadConfig() {
  Preferences prefs;
  if (!prefs.begin(kPrefsNamespace, true)) {
    logEvent("[CFG] NVS open failed — using defaults");
    return;
  }
  ap_enabled      = prefs.getBool(kPrefsKeyAp, false);
  user_wifi_ssid  = prefs.getString(kPrefsKeyUserSsid, "");
  user_wifi_pass  = prefs.getString(kPrefsKeyUserPass, "");
  prefs.end();
  logEvent("[CFG] loaded ap_en=" + String(ap_enabled ? "true" : "false") +
           " usr_ssid=" + (user_wifi_ssid.length() > 0 ? user_wifi_ssid : "(none)"));
}

void saveConfig() {
  Preferences prefs;
  if (!prefs.begin(kPrefsNamespace, false)) {
    logEvent("[CFG] NVS write open failed");
    return;
  }
  prefs.putBool(kPrefsKeyAp, ap_enabled);
  prefs.putString(kPrefsKeyUserSsid, user_wifi_ssid);
  prefs.putString(kPrefsKeyUserPass, user_wifi_pass);
  prefs.end();
}

String buildConfigApiJson() {
  String j;
  j.reserve(128);
  j += "{";
  j += "\"ap_enabled\":" + String(ap_enabled ? "true" : "false") + ",";
  j += "\"ap_active\":"  + String(ap_active  ? "true" : "false") + ",";
  j += "\"wifi_ssid\":\"" + jsonEscape(user_wifi_ssid) + "\"";
  j += "}";
  return j;
}

void sendApiError(int code, const char *msg) {
  webServer.send(code, "application/json; charset=utf-8",
                 String("{\"ok\":false,\"error\":\"") + msg + "\"}");
}

String wifiEncryptionToString(wifi_auth_mode_t mode) {
  switch (mode) {
    case WIFI_AUTH_OPEN: return "open";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA";
    case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA+WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-EAP";
    case WIFI_AUTH_WPA3_PSK: return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2+WPA3";
    case WIFI_AUTH_WAPI_PSK: return "WAPI";
    default: return "unknown";
  }
}

String formatFloat1(float value) {
  if (!isfinite(value)) {
    return "N/A";
  }
  return String(value, 1);
}

void appendLogLine(const String &line) {
  String stamped = "[" + formatUptimeHMS(millis()) + "] " + line;
  if ((int)stamped.length() > LOG_LINE_MAX_LEN) {
    stamped = stamped.substring(0, LOG_LINE_MAX_LEN - 3) + "...";
  }
  logLines[logLineNextIndex] = stamped;
  logLineNextIndex = (logLineNextIndex + 1) % LOG_LINE_COUNT;
  if (logLineCount < LOG_LINE_COUNT) {
    logLineCount++;
  }
}

void logEvent(const String &line) {
  Serial.println(line);
  appendLogLine(line);
}

void logHttpEvent(const String &label, int code, const String &detail) {
  if ((code == lastLoggedHttpResponseCode) && detail.length() == 0) {
    return;
  }
  String line = label + " code=" + String(code);
  if (detail.length() > 0) {
    line += " " + detail;
  }
  logEvent(line);
  lastLoggedHttpResponseCode = code;
}

String formatTempDisplay(float value) {
  if (!isfinite(value)) {
    return "--";
  }
  return String((int)lroundf(value));
}

String resetReasonToString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "UNKNOWN";
  }
}

String formatUptimeHMS(unsigned long ms) {
  unsigned long totalSeconds = ms / 1000UL;
  unsigned long hours = totalSeconds / 3600UL;
  unsigned long minutes = (totalSeconds % 3600UL) / 60UL;
  unsigned long seconds = totalSeconds % 60UL;
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%lu:%02lu:%02lu", hours, minutes, seconds);
  return String(buffer);
}

String formatAgeSeconds(unsigned long sinceMillis) {
  if (sinceMillis == 0) {
    return "never";
  }
  unsigned long delta = millis() - sinceMillis;
  return String(delta / 1000.0F, 1) + " s";
}

float bufferMinTemp(TempHistoryPoint *buf, int bufCapacity, int bufCount, int bufNextIndex) {
  float minValue = 99999.0F;
  for (int i = 0; i < bufCount; ++i) {
    int idx = (bufNextIndex - bufCount + i + bufCapacity) % bufCapacity;
    minValue = min(minValue, min(buf[idx].t0, buf[idx].t1));
  }
  return (bufCount > 0) ? minValue : 0.0F;
}

float bufferMaxTemp(TempHistoryPoint *buf, int bufCapacity, int bufCount, int bufNextIndex) {
  float maxValue = -99999.0F;
  for (int i = 0; i < bufCount; ++i) {
    int idx = (bufNextIndex - bufCount + i + bufCapacity) % bufCapacity;
    maxValue = max(maxValue, max(buf[idx].t0, buf[idx].t1));
  }
  return (bufCount > 0) ? maxValue : 1.0F;
}

void appendTemperatureHistory(float t0, float t1, unsigned long stampMillis) {
  if (!isfinite(t0) || !isfinite(t1)) {
    return;
  }
  // Short-term buffer: every ADC sample, ring wraps at 600 points = 30 min at 3 s
  shortTermHistory[shortTermHistoryNextIndex].millisStamp = stampMillis;
  shortTermHistory[shortTermHistoryNextIndex].t0 = t0;
  shortTermHistory[shortTermHistoryNextIndex].t1 = t1;
  shortTermHistoryNextIndex = (shortTermHistoryNextIndex + 1) % SHORT_TERM_HISTORY_CAPACITY;
  if (shortTermHistoryCount < SHORT_TERM_HISTORY_CAPACITY) {
    shortTermHistoryCount++;
  }
  // Long-term buffer: one point per 2 minutes, ~5-day retention
  if ((tempHistoryCount == 0) || ((stampMillis - lastTempHistoryStoreMillis) >= TEMP_HISTORY_SAMPLE_PERIOD_MSEC)) {
    tempHistory[tempHistoryNextIndex].millisStamp = stampMillis;
    tempHistory[tempHistoryNextIndex].t0 = t0;
    tempHistory[tempHistoryNextIndex].t1 = t1;
    lastTempHistoryStoreMillis = stampMillis;
    tempHistoryNextIndex = (tempHistoryNextIndex + 1) % TEMP_HISTORY_CAPACITY;
    if (tempHistoryCount < TEMP_HISTORY_CAPACITY) {
      tempHistoryCount++;
    }
  }
}

int getSampledBufferIndices(TempHistoryPoint *buf, int bufCapacity, int bufCount, int bufNextIndex, int *indices, int maxPoints) {
  if (bufCount <= 0 || maxPoints <= 0) {
    return 0;
  }

  int count = bufCount;
  if (count <= maxPoints) {
    for (int i = 0; i < count; ++i) {
      indices[i] = (bufNextIndex - bufCount + i + bufCapacity) % bufCapacity;
    }
    return count;
  }

  for (int i = 0; i < maxPoints; ++i) {
    int logicalIndex = (long long)i * (count - 1) / (maxPoints - 1);
    indices[i] = (bufNextIndex - bufCount + logicalIndex + bufCapacity) % bufCapacity;
  }
  return maxPoints;
}

int getSampledBufferIndicesForLookback(TempHistoryPoint *buf, int bufCapacity, int bufCount, int bufNextIndex, int *indices, int maxPoints, unsigned long lookbackMs) {
  if (bufCount <= 0 || maxPoints <= 0) {
    return 0;
  }

  if (lookbackMs == 0) {
    return getSampledBufferIndices(buf, bufCapacity, bufCount, bufNextIndex, indices, maxPoints);
  }

  unsigned long nowMs = millis();
  unsigned long cutoff = (nowMs > lookbackMs) ? (nowMs - lookbackMs) : 0UL;
  int firstLogicalIndex = -1;
  int matchingCount = 0;

  for (int logicalIndex = 0; logicalIndex < bufCount; ++logicalIndex) {
    int idx = (bufNextIndex - bufCount + logicalIndex + bufCapacity) % bufCapacity;
    if (buf[idx].millisStamp >= cutoff) {
      if (firstLogicalIndex < 0) {
        firstLogicalIndex = logicalIndex;
      }
      matchingCount++;
    }
  }

  if (matchingCount <= 0) {
    return 0;
  }
  if (matchingCount <= maxPoints) {
    for (int i = 0; i < matchingCount; ++i) {
      int logicalIndex = firstLogicalIndex + i;
      indices[i] = (bufNextIndex - bufCount + logicalIndex + bufCapacity) % bufCapacity;
    }
    return matchingCount;
  }

  for (int i = 0; i < maxPoints; ++i) {
    int logicalIndex = (long long)i * (matchingCount - 1) / (maxPoints - 1);
    int absoluteLogicalIndex = firstLogicalIndex + logicalIndex;
    indices[i] = (bufNextIndex - bufCount + absoluteLogicalIndex + bufCapacity) % bufCapacity;
  }
  return maxPoints;
}

String buildTemperaturePlotSvg(TempHistoryPoint *buf, int bufCapacity, int bufCount, int bufNextIndex, const char *emptyLabel, const char *timeAxisLabel) {
  if (bufCount < 2) {
    return String("<div class='empty-graph'>") + emptyLabel + "</div>";
  }

  int sampledIndices[TEMP_HISTORY_API_MAX_POINTS];
  int sampledCount = getSampledBufferIndices(buf, bufCapacity, bufCount, bufNextIndex, sampledIndices, TEMP_HISTORY_API_MAX_POINTS);
  if (sampledCount < 2) {
    return String("<div class='empty-graph'>") + emptyLabel + "</div>";
  }

  const float width = 960.0F;
  const float height = 300.0F;
  const float leftPad = 56.0F;
  const float rightPad = 20.0F;
  const float topPad = 20.0F;
  const float bottomPad = 54.0F;
  const float innerWidth = width - leftPad - rightPad;
  const float innerHeight = height - topPad - bottomPad;

  unsigned long minTs = 0;
  unsigned long maxTs = 0;
  for (int i = 0; i < sampledCount; ++i) {
    int idx = sampledIndices[i];
    unsigned long ts = buf[idx].millisStamp;
    if (i == 0 || ts < minTs) minTs = ts;
    if (i == 0 || ts > maxTs) maxTs = ts;
  }

  float minTemp = bufferMinTemp(buf, bufCapacity, bufCount, bufNextIndex);
  float maxTemp = bufferMaxTemp(buf, bufCapacity, bufCount, bufNextIndex);
  if (maxTs <= minTs) {
    maxTs = minTs + 1UL;
  }
  if (maxTemp <= minTemp) {
    maxTemp = minTemp + 1.0F;
  }

  String t0Points;
  String t1Points;
  t0Points.reserve(sampledCount * 14);
  t1Points.reserve(sampledCount * 14);

  for (int i = 0; i < sampledCount; ++i) {
    int idx = sampledIndices[i];
    float x = leftPad + ((float)(buf[idx].millisStamp - minTs) / (float)(maxTs - minTs)) * innerWidth;
    float y0 = topPad + (1.0F - ((buf[idx].t0 - minTemp) / (maxTemp - minTemp))) * innerHeight;
    float y1 = topPad + (1.0F - ((buf[idx].t1 - minTemp) / (maxTemp - minTemp))) * innerHeight;
    if (i > 0) {
      t0Points += " ";
      t1Points += " ";
    }
    t0Points += String(x, 1) + "," + String(y0, 1);
    t1Points += String(x, 1) + "," + String(y1, 1);
  }

  String svg;
  svg.reserve(5000);
  svg += "<svg class='temp-plot' viewBox='0 0 960 300' role='img' aria-label='Temperature history plot'>";
  svg += "<rect x='56' y='20' width='884' height='226' fill='#121823' stroke='#2a3240' stroke-width='1'/>";

  for (int idx = 0; idx < 5; ++idx) {
    float frac = idx / 4.0F;
    float yVal = maxTemp - frac * (maxTemp - minTemp);
    float yPos = topPad + frac * innerHeight;
    svg += "<line x1='56' y1='" + String(yPos, 1) + "' x2='940' y2='" + String(yPos, 1) + "' stroke='rgba(255,255,255,0.12)' stroke-width='1'/>";
    svg += "<text x='48' y='" + String(yPos + 4.0F, 1) + "' text-anchor='end' fill='#9faec8' font-size='11'>" + String(yVal, 1) + "</text>";
  }

  unsigned long spanMs = maxTs - minTs;
  for (int idx = 0; idx < 5; ++idx) {
    float frac = idx / 4.0F;
    unsigned long tickTs = minTs + (unsigned long)(frac * spanMs);
    float xPos = leftPad + frac * innerWidth;
    String anchor = "middle";
    if (idx == 0) anchor = "start";
    if (idx == 4) anchor = "end";
    svg += "<line x1='" + String(xPos, 1) + "' y1='20' x2='" + String(xPos, 1) + "' y2='246' stroke='rgba(255,255,255,0.10)' stroke-width='1'/>";
    svg += "<text x='" + String(xPos, 1) + "' y='280' text-anchor='" + anchor + "' fill='#9faec8' font-size='11'>" + formatUptimeHMS(tickTs) + "</text>";
  }

  svg += "<polyline fill='none' stroke='#ff8b6b' stroke-width='2' points='" + t0Points + "'/>";
  svg += "<polyline fill='none' stroke='#7ce3b0' stroke-width='2' points='" + t1Points + "'/>";
  svg += "<line x1='56' y1='246' x2='940' y2='246' stroke='#2a3240' stroke-width='1'/>";
  svg += "<text x='64' y='34' fill='#ff8b6b' font-size='12'>t0</text>";
  svg += "<text x='92' y='34' fill='#7ce3b0' font-size='12'>t1</text>";
  svg += "<text x='498' y='294' text-anchor='middle' fill='#7d8ca4' font-size='10'>";
  svg += timeAxisLabel;
  svg += "</text>";
  svg += "</svg>";
  return svg;
}

String buildHistoryJson() {
  int sampledIndices[TEMP_HISTORY_API_MAX_POINTS];
  int sampledCount = getSampledBufferIndices(tempHistory, TEMP_HISTORY_CAPACITY, tempHistoryCount, tempHistoryNextIndex, sampledIndices, TEMP_HISTORY_API_MAX_POINTS);
  String json;
  json.reserve(16000);
  json += "{";
  json += "\"count\":" + String(tempHistoryCount) + ",";
  json += "\"capacity\":" + String(TEMP_HISTORY_CAPACITY) + ",";
  json += "\"samplePeriodSec\":" + String(TEMP_HISTORY_SAMPLE_PERIOD_MSEC / 1000UL) + ",";
  json += "\"returnedPoints\":" + String(sampledCount) + ",";
  json += "\"points\":[";
  for (int i = 0; i < sampledCount; ++i) {
    int idx = sampledIndices[i];
    if (i > 0) {
      json += ",";
    }
    json += "{";
    json += "\"millis\":" + String(tempHistory[idx].millisStamp) + ",";
    json += "\"uptime\":\"" + jsonEscape(formatUptimeHMS(tempHistory[idx].millisStamp)) + "\",";
    json += "\"t0\":" + String(tempHistory[idx].t0, 3) + ",";
    json += "\"t1\":" + String(tempHistory[idx].t1, 3);
    json += "}";
  }
  json += "]}";
  return json;
}

String buildLogJson() {
  String json;
  json.reserve(LOG_LINE_COUNT * 96);
  json += "{";
  json += "\"count\":" + String(logLineCount) + ",";
  json += "\"log\":\"";
  for (int i = 0; i < logLineCount; ++i) {
    int idx = (logLineNextIndex - logLineCount + i + LOG_LINE_COUNT) % LOG_LINE_COUNT;
    if (i > 0) {
      json += "\\n";
    }
    json += jsonEscape(logLines[idx]);
  }
  json += "\"}";
  return json;
}

String buildStatusJson(bool includeHistory) {
  String json;
  json.reserve(includeHistory ? 12000 : 5000);

  json += "{";
  json += "\"version\":{";
  json += "\"number\":\"" + jsonEscape(String(FW_VERSION)) + "\",";
  json += "\"buildDate\":\"" + jsonEscape(String(FW_BUILD_DATE)) + "\",";
  json += "\"buildTime\":\"" + jsonEscape(String(FW_BUILD_TIME)) + "\",";
  json += "\"build\":\"" + jsonEscape(buildVersionString()) + "\"";
  json += "},";

  json += "\"network\":{";
  json += "\"hostname\":\"" + jsonEscape(String(NETWORK_HOSTNAME)) + "\",";
  json += "\"targetHostname\":\"" + jsonEscape(String(TARGET_HOSTNAME)) + "\",";
  json += "\"localIp\":\"" + jsonEscape(WiFi.localIP().toString()) + "\",";
  json += "\"ssid\":\"" + jsonEscape(WiFi.SSID()) + "\",";
  json += "\"channel\":" + String(WiFi.channel()) + ",";
  json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"mac\":\"" + jsonEscape(WiFi.macAddress()) + "\",";
  json += "\"encryption\":\"" + jsonEscape(wifiEncryptionToString(WiFi.encryptionType(0))) + "\",";
  json += "\"resolvedTargetIp\":\"" + jsonEscape(DJBOOSHBOXIp.toString()) + "\",";
  json += "\"urlBase\":\"" + jsonEscape(urlBase) + "\"";
  json += "},";

  json += "\"timing\":{";
  json += "\"millis\":" + String(millis()) + ",";
  json += "\"uptimeHours\":" + String((float)millis() / 1000.0F / 60.0F / 60.0F, 3) + ",";
  json += "\"uptime\":\"" + jsonEscape(formatUptimeHMS(millis())) + "\",";
  json += "\"lastMeasurementAgeSec\":" + String((millis() - lastADCMeasurementTimeMsec) / 1000.0F, 3) + ",";
  json += "\"timerPeriodSeconds\":" + String(timerPeriodADCISRMeasuSec / 1000000.0F, 3) + ",";
  json += "\"httpPostIntervalSec\":" + String(lastHttpPostIntervalMSec / 1000.0F, 3);
  json += "},";

  json += "\"thermistors\":[";
  for (int i = 0; i < THERMISTOR_COUNT; ++i) {
    if (i > 0) {
      json += ",";
    }
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"gpio\":" + String(ThermistorPins[i]) + ",";
    json += "\"temperatureF\":";
    if (isfinite(T[i])) json += String(T[i], 3);
    else json += "null";
    json += ",";
    json += "\"temperatureRoundedF\":";
    if (isfinite(T[i])) json += String((int)lroundf(T[i]));
    else json += "null";
    json += ",";
    json += "\"adcRaw\":" + String(adcRawReads[i]);
    json += "}";
  }
  json += "],";

  json += "\"calibration\":{";
  json += "\"manualTempOffsetF\":" + String(manualTempOffsetF, 3) + ",";
  json += "\"r1Ohms\":" + String(R1, 3);
  json += "},";

  json += "\"http\":{";
  json += "\"responseCode\":" + String(httpResponseCode) + ",";
  json += "\"lastGetAgeSec\":" + String((millis() - lastHttpPostTimeMSec) / 1000.0F, 3) + ",";
  json += "\"lastGoodAgeSec\":";
  if (lastHttpReponseGoodTimestampMsec == 0) json += "null";
  else json += String((millis() - lastHttpReponseGoodTimestampMsec) / 1000.0F, 3);
  json += ",";
  json += "\"lastBadAgeSec\":";
  if (lastHttpReponseBadTimestampMsec == 0) json += "null";
  else json += String((millis() - lastHttpReponseBadTimestampMsec) / 1000.0F, 3);
  json += ",";
  json += "\"lastUrl\":\"" + jsonEscape(serverPath) + "\"";
  json += "}";

  json += ",";
  json += "\"reset\":{";
  json += "\"reason\":\"" + jsonEscape(resetReasonToString(bootResetReason)) + "\"";
  json += "}";

  if (includeHistory) {
    json += ",";
    json += "\"history\":" + buildHistoryJson();
  }

  json += "}";
  return json;
}

String buildDashboardHtml() {
  String html;
  html.reserve(18000);

  String localIp = WiFi.localIP().toString();
  String ssid = WiFi.SSID();
  String enc = wifiEncryptionToString(WiFi.encryptionType(0));

  html += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>REMOTETHERMO Dashboard</title>";
  html += "<style>";
  html += ":root{--bg:#0b0f14;--panel:#131a23;--panel2:#192230;--text:#edf3fb;--muted:#9aacbf;--accent:#ff8b6b;--accent2:#7ce3b0;--border:#263244;--line:#263244;}";
  html += "body{margin:0;font-family:Verdana,Arial,sans-serif;background:radial-gradient(circle at top,#1b2432 0,#0b0f14 50%);color:var(--text);}";
  html += ".wrap{max-width:1100px;margin:0 auto;padding:20px;}";
  html += ".hero{background:linear-gradient(135deg,#141c27,#0f141c);border:1px solid var(--border);border-radius:18px;padding:18px 20px;box-shadow:0 10px 40px rgba(0,0,0,0.28);}";
  html += ".hero h1{margin:0 0 8px;font-size:28px;letter-spacing:1px;color:#ffb39d;}";
  html += ".hero p{margin:0;color:var(--muted);line-height:1.5;}";
  html += ".chips{display:flex;flex-wrap:wrap;gap:10px;margin-top:14px;}";
  html += ".chip{padding:8px 12px;border-radius:999px;background:#0d141d;border:1px solid var(--border);font-size:13px;color:#dbe6f5;}";
  html += ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:14px;margin-top:16px;}";
  html += ".card{background:linear-gradient(180deg,var(--panel),var(--panel2));border:1px solid var(--border);border-radius:16px;padding:16px;box-shadow:0 8px 30px rgba(0,0,0,0.2);}";
  html += ".card h2{margin:0 0 12px;font-size:18px;color:#ffd0c3;}";
  html += ".temp-grid{display:grid;grid-template-columns:repeat(2,minmax(120px,1fr));gap:12px;}";
  html += ".temp{background:#0e141d;border:1px solid #243347;border-radius:14px;padding:14px;}";
  html += ".temp .label{font-size:12px;color:var(--muted);text-transform:uppercase;letter-spacing:1px;}";
  html += ".temp .value{font-size:38px;font-weight:bold;margin-top:8px;}";
  html += ".temp .sub{font-size:13px;color:#b8c6d8;margin-top:6px;}";
  html += ".kv{display:grid;grid-template-columns:160px 1fr;gap:8px;font-size:14px;padding:4px 0;border-bottom:1px solid rgba(255,255,255,0.05);}";
  html += ".kv .k{color:var(--muted);} .kv .v{color:#f3f7fd;word-break:break-word;}";
  html += ".mono{font-family:Consolas,Monaco,monospace;}";
  html += ".panel{background:linear-gradient(180deg,var(--panel),var(--panel2));border:1px solid var(--border);border-radius:16px;padding:16px;box-shadow:0 8px 30px rgba(0,0,0,0.2);}";
  html += "#log{background:#05080d;color:#b9ffd6;min-height:260px;max-height:420px;overflow:auto;font:13px/1.35 Consolas,monospace;border-radius:12px;padding:12px;border:1px solid #193022}";
  html += "#log pre{margin:0;white-space:pre-wrap}";
  html += ".graph-wrap{margin-top:18px;background:linear-gradient(180deg,#121823,#0d1219);border:1px solid var(--border);border-radius:16px;padding:14px;}";
  html += ".graph-wrap h2{margin:0 0 8px;font-size:18px;color:#ffd0c3;}";
  html += ".temp-plot{width:100%;height:auto;display:block;}";
  html += ".empty-graph{padding:28px 12px;color:var(--muted);font-family:Consolas,Monaco,monospace;}";
  html += "@media (max-width:700px){.hero h1{font-size:22px;}.temp .value{font-size:30px;}.kv{grid-template-columns:1fr;gap:2px;}}";
  html += "</style></head><body><div class='wrap'>";

  html += "<section class='hero'>";
  html += "<h1>REMOTE THERMO</h1>";
  html += "<p>Lava Lounge propane tank diagnostics for " + htmlEscape(String(NETWORK_HOSTNAME)) + ". ";
  html += "Version <span class='mono'>" + htmlEscape(buildVersionString()) + "</span></p>";
  html += "<div class='chips'>";
  html += "<span class='chip'>IP " + htmlEscape(localIp) + "</span>";
  html += "<span class='chip'>SSID " + htmlEscape(ssid) + "</span>";
  html += "<span class='chip'>RSSI " + String(WiFi.RSSI()) + " dBm</span>";
  html += "<span class='chip'>HTTP " + String(httpResponseCode) + "</span>";
  html += "<span class='chip'>ST " + String(shortTermHistoryCount) + "/" + String(SHORT_TERM_HISTORY_CAPACITY) + " (3s)</span>";
  html += "<span class='chip'>LT " + String(tempHistoryCount) + "/" + String(TEMP_HISTORY_CAPACITY) + " (2min)</span>";
  html += "<span class='chip'>AP " + String(ap_active ? "active" : "off") + "</span>";
  html += "</div></section>";

  html += "<div class='grid'>";

  html += "<section class='card'><h2>Temperatures</h2><div class='temp-grid'>";
  for (int i = 0; i < THERMISTOR_COUNT; ++i) {
    html += "<div class='temp'>";
    html += "<div class='label'>T" + String(i) + " / GPIO" + String(ThermistorPins[i]) + "</div>";
    html += "<div class='value' style='color:";
    html += (i == 0) ? "#ff8b6b'>" : "#7ce3b0'>";
    html += formatFloat1(T[i]) + "&deg;F</div>";
    html += "<div class='sub mono'>ADC " + String(adcRawReads[i]) + "</div>";
    html += "</div>";
  }
  html += "</div>";
  html += "<div class='kv'><div class='k'>Last measurement</div><div class='v mono'>" + formatAgeSeconds(lastADCMeasurementTimeMsec) + " ago</div></div>";
  html += "<div class='kv'><div class='k'>Timer period</div><div class='v mono'>" + String(timerPeriodADCISRMeasuSec / 1000000.0F, 3) + " s</div></div>";
  html += "<div class='kv'><div class='k'>Manual offset</div><div class='v mono'>" + String(manualTempOffsetF, 2) + " F</div></div>";
  html += "<div class='kv'><div class='k'>R1 resistor</div><div class='v mono'>" + String(R1, 0) + " ohm</div></div>";
  html += "</section>";

  html += "<section class='card'><h2>HTTP / mDNS</h2>";
  html += "<div class='kv'><div class='k'>Last response code</div><div class='v mono'>" + String(httpResponseCode) + "</div></div>";
  html += "<div class='kv'><div class='k'>Last GET</div><div class='v mono'>" + formatAgeSeconds(lastHttpPostTimeMSec) + " ago</div></div>";
  html += "<div class='kv'><div class='k'>Last GOOD</div><div class='v mono'>" + formatAgeSeconds(lastHttpReponseGoodTimestampMsec) + " ago</div></div>";
  html += "<div class='kv'><div class='k'>Last BAD</div><div class='v mono'>" + formatAgeSeconds(lastHttpReponseBadTimestampMsec) + " ago</div></div>";
  html += "<div class='kv'><div class='k'>URL base</div><div class='v mono'>" + htmlEscape(urlBase) + "</div></div>";
  html += "<div class='kv'><div class='k'>Last URL</div><div class='v mono'>" + htmlEscape(serverPath) + "</div></div>";
  html += "<div class='kv'><div class='k'>mDNS target</div><div class='v mono'>" + htmlEscape(String(TARGET_HOSTNAME)) + " =&gt; " + htmlEscape(DJBOOSHBOXIp.toString()) + "</div></div>";
  html += "</section>";

  html += "<section class='card'><h2>Network</h2>";
  html += "<div class='kv'><div class='k'>Hostname</div><div class='v mono'>" + htmlEscape(String(NETWORK_HOSTNAME)) + "</div></div>";
  html += "<div class='kv'><div class='k'>Local IP</div><div class='v mono'>" + htmlEscape(localIp) + "</div></div>";
  html += "<div class='kv'><div class='k'>SSID</div><div class='v mono'>" + htmlEscape(ssid) + "</div></div>";
  html += "<div class='kv'><div class='k'>Channel</div><div class='v mono'>" + String(WiFi.channel()) + "</div></div>";
  html += "<div class='kv'><div class='k'>Encryption</div><div class='v mono'>" + htmlEscape(enc) + "</div></div>";
  html += "<div class='kv'><div class='k'>RSSI</div><div class='v mono'>" + String(WiFi.RSSI()) + " dBm</div></div>";
  html += "<div class='kv'><div class='k'>MAC</div><div class='v mono'>" + htmlEscape(WiFi.macAddress()) + "</div></div>";
  html += "</section>";

  html += "<section class='card'><h2>System</h2>";
  html += "<div class='kv'><div class='k'>Version</div><div class='v mono'>" + htmlEscape(buildVersionString()) + "</div></div>";
  html += "<div class='kv'><div class='k'>Build code</div><div class='v mono'>" + htmlEscape(String(SW_REV_CODE)) + "</div></div>";
  html += "<div class='kv'><div class='k'>Reset reason</div><div class='v mono'>" + htmlEscape(resetReasonToString(bootResetReason)) + "</div></div>";
  html += "<div class='kv'><div class='k'>Millis</div><div class='v mono'>" + String(millis()) + "</div></div>";
  html += "<div class='kv'><div class='k'>Uptime</div><div class='v mono'>" + formatUptimeHMS(millis()) + "</div></div>";
  html += "<div class='kv'><div class='k'>API</div><div class='v mono'>/api/status, /api/history, /api/logs</div></div>";
  html += "</section>";

  html += "</div>";

  html += "<div class='grid' style='margin-top:16px'>";
  html += "<section class='panel'><h2>Serial Console</h2><div id='log'><pre id='log-text'>";
  for (int i = 0; i < logLineCount; ++i) {
    int idx = (logLineNextIndex - logLineCount + i + LOG_LINE_COUNT) % LOG_LINE_COUNT;
    html += htmlEscape(logLines[idx]);
    if (i + 1 < logLineCount) {
      html += "\n";
    }
  }
  html += "</pre></div></section>";

  html += "<section class='panel'><h2>Node Config</h2>";
  html += "<form id='config-form'>";
  html += "<div style='border-top:1px solid var(--border);padding-top:10px;display:grid;gap:8px'>";
  html += "<span style='color:var(--muted);font-size:13px'>WiFi Fallback</span>";
  html += "<div class='kv'><div class='k'>SSID</div><div class='v'><input id='cfg-wifi-ssid' style='width:100%;box-sizing:border-box;background:#0d141d;color:#f3f7fd;border:1px solid var(--border);border-radius:6px;padding:6px 8px;font-size:13px' placeholder='leave blank to disable'></div></div>";
  html += "<div class='kv'><div class='k'>Password</div><div class='v'><input id='cfg-wifi-pass' type='password' style='width:100%;box-sizing:border-box;background:#0d141d;color:#f3f7fd;border:1px solid var(--border);border-radius:6px;padding:6px 8px;font-size:13px' placeholder=''></div></div>";
  html += "</div>";
  html += "<div style='border-top:1px solid var(--border);padding-top:10px;display:flex;align-items:center;gap:10px'>";
  html += "<input type='checkbox' id='cfg-ap' style='width:18px;height:18px;margin:0;cursor:pointer;accent-color:var(--accent)'>";
  html += "<span style='color:var(--muted);font-size:14px'>Enable WiFi AP &mdash; SSID: <code>" + String(AP_SSID) + "</code>, IP <code>10.1.2.3</code></span>";
  html += "</div>";
  html += "<div style='display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin-top:10px'>";
  html += "<button type='submit' style='padding:8px 16px;background:var(--accent);color:#0b0f14;border:none;border-radius:8px;font-size:14px;cursor:pointer;font-weight:bold'>Save Config</button>";
  html += "<span id='config-status' style='font-size:13px;color:var(--muted)'></span>";
  html += "</div>";
  html += "</form></section>";
  html += "</div>";

  html += "<section class='graph-wrap'><h2>T0 / T1 Temperature History</h2>";
  html += "<div class='kv'><div class='k'>Short-term buffer</div><div class='v mono'>" + String(shortTermHistoryCount) + "/" + String(SHORT_TERM_HISTORY_CAPACITY) + " pts, 1 per 3 s, 30-min window</div></div>";
  html += "<div class='kv'><div class='k'>Long-term buffer</div><div class='v mono'>" + String(tempHistoryCount) + "/" + String(TEMP_HISTORY_CAPACITY) + " pts, 1 per 2 min, ~5 days retained</div></div>";
  html += "<h2 style='margin-top:16px'>Last 30 Minutes</h2>";
  html += buildTemperaturePlotSvg(shortTermHistory, SHORT_TERM_HISTORY_CAPACITY, shortTermHistoryCount, shortTermHistoryNextIndex, "Need at least 2 temperature samples.", "uptime (last 30 min)");
  html += "<h2 style='margin-top:20px'>All Stored Data</h2>";
  html += buildTemperaturePlotSvg(tempHistory, TEMP_HISTORY_CAPACITY, tempHistoryCount, tempHistoryNextIndex, "Need at least 2 stored temperature samples.", "uptime (full retained history)");
  html += "</section>";

  html += "<script>";
  html += "async function fetchJson(url,opt){const r=await fetch(url,Object.assign({cache:'no-store'},opt));if(!r.ok)throw new Error(await r.text());return r.json();}";
  html += "function syncField(id,v){const e=document.getElementById(id);if(e&&document.activeElement!==e)e.value=v||'';}";
  html += "async function refreshLogs(){const d=await fetchJson('/api/logs');const t=document.getElementById('log-text');const b=document.getElementById('log');if(t&&b){t.textContent=d.log||'';b.scrollTop=b.scrollHeight;}}";
  html += "async function refreshConfig(){const d=await fetchJson('/api/config');syncField('cfg-wifi-ssid',d.wifi_ssid);const cb=document.getElementById('cfg-ap');if(cb&&document.activeElement!==cb)cb.checked=!!d.ap_enabled;}";
  html += "document.getElementById('cfg-ap').addEventListener('change',async(e)=>{await fetchJson('/api/config/ap',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'value='+(e.target.checked?'true':'false')});});";
  html += "document.getElementById('config-form').addEventListener('submit',async(e)=>{e.preventDefault();const st=document.getElementById('config-status');st.textContent='Saving...';const b=new URLSearchParams();const s=document.getElementById('cfg-wifi-ssid').value.trim();if(s)b.set('wifi_ssid',s);else b.set('wifi_ssid','');const p=document.getElementById('cfg-wifi-pass').value;if(p)b.set('wifi_pass',p);try{await fetchJson('/api/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b.toString()});st.textContent='Saved';document.getElementById('cfg-wifi-pass').value='';}catch(err){st.textContent='Error: '+err.message;}});";
  html += "setInterval(()=>{refreshLogs().catch(()=>{});},3000);";
  html += "setInterval(()=>{refreshConfig().catch(()=>{});},5000);";
  html += "refreshLogs().catch(()=>{});refreshConfig().catch(()=>{});";
  html += "</script>";
  html += "</div></body></html>";
  return html;
}

void IRAM_ATTR IRQdoADCMeasurement() {
  if (STATUS_LED_PIN >= 0) {
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
  }
  flagNeedToSampleADC = 1;
}

void initOledDisplay() {
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  oledAvailable = oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
  if (!oledAvailable) {
    Serial.println("OLED init failed");
    return;
  }
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.setTextSize(1);
  oled.println("REMOTE THERMO");
  oled.println(buildVersionString());
  oled.display();
}

void drawOledTempsPage() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("T0");
  oled.setCursor(0, 32);
  oled.print("T1");

  oled.setTextSize(4);
  oled.setCursor(18, 0);
  oled.print(formatTempDisplay(T[0]));
  oled.print("F");
  oled.setCursor(18, 32);
  oled.print(formatTempDisplay(T[1]));
  oled.print("F");
  oled.display();
}

void drawOledWifiPage() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("WiFi");
  oled.println(WiFi.SSID());
  oled.print("IP ");
  oled.println(WiFi.localIP());
  oled.print("RSSI ");
  oled.print(WiFi.RSSI());
  oled.println(" dBm");
  oled.print("CH ");
  oled.println(WiFi.channel());
  oled.display();
}

void drawOledHttpPage() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("HTTP / mDNS");
  oled.print("CODE ");
  oled.println(httpResponseCode);
  oled.print("GOOD ");
  oled.println(formatAgeSeconds(lastHttpReponseGoodTimestampMsec));
  oled.print("BAD  ");
  oled.println(formatAgeSeconds(lastHttpReponseBadTimestampMsec));
  oled.print("IP ");
  oled.println(DJBOOSHBOXIp);
  oled.display();
}

void drawOledSystemPage() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("System");
  oled.println(FW_VERSION);
  oled.println(String(FW_BUILD_DATE));
  oled.println(String(FW_BUILD_TIME));
  oled.print("UP ");
  oled.println(formatUptimeHMS(millis()));
  oled.println(resetReasonToString(bootResetReason));
  oled.print("CPU ");
  oled.print(getCpuFrequencyMhz());
  oled.println("MHz");
  oled.display();
}

void updateOledDisplay() {
  if (!oledAvailable) {
    return;
  }

  if ((millis() - lastOledPageChangeMsec) > OLED_PAGE_PERIOD_MSEC) {
    lastOledPageChangeMsec = millis();
    oledPageIndex = (oledPageIndex + 1) % OLED_PAGE_SEQUENCE_LEN;
  }

  switch (OLED_PAGE_SEQUENCE[oledPageIndex]) {
    case OLED_PAGE_TEMPS:
      drawOledTempsPage();
      break;
    case OLED_PAGE_WIFI:
      drawOledWifiPage();
      break;
    case OLED_PAGE_HTTP:
      drawOledHttpPage();
      break;
    case OLED_PAGE_SYSTEM:
      drawOledSystemPage();
      break;
    default:
      drawOledTempsPage();
      break;
  }
}

bool isSsidVisible(const char *ssid) {
  int networkCount = WiFi.scanNetworks(false, true);
  if (networkCount <= 0) {
    return false;
  }

  bool found = false;
  for (int i = 0; i < networkCount; ++i) {
    if (WiFi.SSID(i) == String(ssid)) {
      found = true;
      break;
    }
  }
  WiFi.scanDelete();
  return found;
}

bool connectToPreferredWifi() {
  struct CandidateNetwork {
    const char *ssid;
    const char *pass;
  };

  CandidateNetwork primary = {BOOSH_WIFI_SSID_LL, BOOSH_WIFI_PASS_LL};
  CandidateNetwork fallback = {BOOSH_WIFI_SSID_MW, BOOSH_WIFI_PASS_MW};

  CandidateNetwork ordered[2];
  int orderedCount = 0;

  if (isSsidVisible(primary.ssid)) {
    ordered[orderedCount++] = primary;
    logEvent(String("Preferred SSID visible: ") + primary.ssid);
  }

  if (isSsidVisible(fallback.ssid)) {
    if (orderedCount == 0) {
      logEvent(String("Preferred SSID not visible. Falling back to ") + fallback.ssid);
    } else {
      logEvent(String("Fallback SSID also visible: ") + fallback.ssid);
    }
    ordered[orderedCount++] = fallback;
  }

  if (orderedCount == 0) {
    logEvent("No configured SSIDs visible during scan.");
    ordered[orderedCount++] = primary;
    ordered[orderedCount++] = fallback;
  } else if (orderedCount == 1 && String(ordered[0].ssid) != String(primary.ssid)) {
    ordered[orderedCount++] = primary;
  } else if (orderedCount == 1 && String(ordered[0].ssid) != String(fallback.ssid)) {
    ordered[orderedCount++] = fallback;
  }

  for (int i = 0; i < orderedCount; ++i) {
    logEvent(String("Connecting to SSID ") + ordered[i].ssid);
    WiFi.disconnect(true, true);
    delay(250);
    WiFi.begin(ordered[i].ssid, ordered[i].pass);

    int attempts = 0;
    while ((WiFi.status() != WL_CONNECTED) && (attempts < (WIFI_CONNECTION_TIMEOUT_SECS * 2))) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      logEvent(String("Connected to preferred order SSID: ") + WiFi.SSID());
      return true;
    }

    logEvent(String("Failed to connect to ") + ordered[i].ssid);
  }

  // User-defined WiFi fallback (stored in NVS)
  if (user_wifi_ssid.length() > 0) {
    logEvent("Trying user WiFi fallback ssid=" + user_wifi_ssid);
    WiFi.disconnect(true, true);
    delay(250);
    WiFi.begin(user_wifi_ssid.c_str(), user_wifi_pass.c_str());
    int attempts = 0;
    while ((WiFi.status() != WL_CONNECTED) && (attempts < (WIFI_CONNECTION_TIMEOUT_SECS * 2))) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      logEvent("Connected to user WiFi ssid=" + WiFi.SSID());
      return true;
    }
    logEvent("Failed to connect to user WiFi ssid=" + user_wifi_ssid);
  }

  return false;
}

// ── Web handlers ─────────────────────────────────────────────────────────────

void handleWebRoot() {
  esp_task_wdt_reset();
  webServer.send(200, "text/html; charset=utf-8", buildDashboardHtml());
}

void handleApiStatus() {
  esp_task_wdt_reset();
  webServer.send(200, "application/json; charset=utf-8", buildStatusJson(true));
}

void handleApiHistory() {
  webServer.send(200, "application/json; charset=utf-8", buildHistoryJson());
}

void handleApiLogs() {
  webServer.send(200, "application/json; charset=utf-8", buildLogJson());
}

void handleApiConfigGet() {
  webServer.send(200, "application/json; charset=utf-8", buildConfigApiJson());
}

void handleApiConfigPost() {
  bool changed = false;
  if (webServer.hasArg("wifi_ssid")) {
    user_wifi_ssid = webServer.arg("wifi_ssid");
    user_wifi_ssid.trim();
    changed = true;
    logEvent("[CFG] wifi_ssid set: " + (user_wifi_ssid.length() > 0 ? user_wifi_ssid : "(cleared)"));
  }
  if (webServer.hasArg("wifi_pass") && webServer.arg("wifi_pass").length() > 0) {
    user_wifi_pass = webServer.arg("wifi_pass");
    changed = true;
    logEvent("[CFG] wifi_pass updated");
  }
  if (!changed) {
    sendApiError(400, "expected wifi_ssid and/or wifi_pass");
    return;
  }
  saveConfig();
  webServer.send(200, "application/json; charset=utf-8",
                 "{\"ok\":true,\"config\":" + buildConfigApiJson() + "}");
}

void handleApiConfigAp() {
  if (!webServer.hasArg("value")) {
    sendApiError(400, "missing value");
    return;
  }
  String v = webServer.arg("value");
  v.toLowerCase();
  ap_enabled = (v == "true" || v == "1" || v == "yes" || v == "on");
  saveConfig();
  logEvent("[CFG] ap_enabled=" + String(ap_enabled ? "true" : "false"));
  webServer.send(200, "application/json; charset=utf-8",
                 "{\"ok\":true,\"config\":" + buildConfigApiJson() + "}");
}

void handleCaptivePortalRedirect() {
  webServer.sendHeader("Location", "http://10.1.2.3/");
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.send(302, "text/plain", "");
}

void setupWebServer() {
  if (webServerStarted) return;
  webServer.on("/",            HTTP_GET,  handleWebRoot);
  webServer.on("/api/status",  HTTP_GET,  handleApiStatus);
  webServer.on("/api/history", HTTP_GET,  handleApiHistory);
  webServer.on("/api/logs",    HTTP_GET,  handleApiLogs);
  webServer.on("/api/config",  HTTP_GET,  handleApiConfigGet);
  webServer.on("/api/config",  HTTP_POST, handleApiConfigPost);
  webServer.on("/api/config/ap", HTTP_POST, handleApiConfigAp);
  webServer.begin();
  webServerStarted = true;
  logEvent("HTTP server listening on port 80");
}

void setupApMode() {
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_AP);
  }
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID);
  delay(100);
  dnsServer.start(53, "*", AP_IP);
  ap_active = true;
  logEvent(String("[AP] started SSID: ") + AP_SSID + " IP: 10.1.2.3");
  setupWebServer();
  // Captive portal detection probes for iOS / Android / Windows
  webServer.on("/generate_204",       HTTP_GET, handleCaptivePortalRedirect);
  webServer.on("/hotspot-detect.html",HTTP_GET, handleCaptivePortalRedirect);
  webServer.on("/ncsi.txt",           HTTP_GET, handleCaptivePortalRedirect);
  webServer.on("/connecttest.txt",    HTTP_GET, handleCaptivePortalRedirect);
  webServer.onNotFound(handleCaptivePortalRedirect);
}

void stopApMode() {
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.mode(WIFI_STA);
  }
  ap_active = false;
  logEvent("[AP] stopped");
}

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  bootResetReason = esp_reset_reason();
  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  if (STATUS_LED_PIN >= 0) {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
  }

  pinMode(THERMISTOR_PIN1, INPUT);
  pinMode(THERMISTOR_PIN2, INPUT);
  analogReadResolution(12);

  delay(200);
  Serial.begin(115200);
  delay(10);

  logEvent("Boot reset=" + resetReasonToString(bootResetReason));
  logEvent("CPU MHz=" + String(getCpuFrequencyMhz()));

  loadConfig();

  esp_bt_controller_disable();
  esp_bt_mem_release(ESP_BT_MODE_BTDM);
  logEvent("Bluetooth disabled");

  initOledDisplay();

  WiFi.setHostname(NETWORK_HOSTNAME);
  if (connectToPreferredWifi()) {
    logEvent("WiFi connected ssid=" + WiFi.SSID() + " ip=" + WiFi.localIP().toString() + " rssi=" + String(WiFi.RSSI()));
  } else {
    logEvent("WiFi connection failed");
    if (!ap_enabled) {
      ap_enabled = true;
      saveConfig();
      logEvent("[AP] auto-enabled: no WiFi network found");
    }
  }

  int mDNSStatus = MDNS.begin(NETWORK_HOSTNAME);
  if (!mDNSStatus) {
    logEvent("Error starting mDNS");
  } else {
    logEvent(String("mDNS OK ") + String(NETWORK_HOSTNAME));
  }

  setupWebServer();

  if (ap_enabled) {
    setupApMode();
  }

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  logEvent("Task WDT enabled timeout=" + String(WDT_TIMEOUT) + "s");

  myTimerBPMPulseBoosh = timerBegin(0, 80, true);
  timerAttachInterrupt(myTimerBPMPulseBoosh, &IRQdoADCMeasurement, false);
  timerAlarmWrite(myTimerBPMPulseBoosh, timerPeriodADCISRMeasuSec, true);
  timerAlarmEnable(myTimerBPMPulseBoosh);
  logEvent("ADC timer armed periodSec=" + String(timerPeriodADCISRMeasuSec / 1000000.0F, 3));
}

void loop() {
  currentMillis = millis();
  esp_task_wdt_reset();

  serial0DebugCom();
  webServer.handleClient();
  if (ap_active) dnsServer.processNextRequest();
  if (ap_enabled && !ap_active) setupApMode();
  else if (!ap_enabled && ap_active) stopApMode();
  updateOledDisplay();

  if (((currentMillis - lastDisplayDiagnosticSerialDumpTimeMsec) > DEBUG_DISPLAY_DIAGNOSTIC_UPDATE_PERIOD_MSEC) ||
      (currentMillis < lastDisplayDiagnosticSerialDumpTimeMsec)) {
    displayDiagnosticUpdate();
  }

  if (flagNeedToSampleADC) {
    noInterrupts();
    flagNeedToSampleADC = 0;
    interrupts();

    lastADCMeasurementTimeMsec = millis();
    Vo = 0;
    for (int i = 0; i < THERMISTOR_COUNT; ++i) {
      adcRawReads[i] = 0;
      for (int j = 0; j < THERMISTOR_MEAS_AVG_COUNT; ++j) {
        adcRawReads[i] += analogRead(ThermistorPins[i]);
        delayMicroseconds(THERMISTOR_MEAS_AVG_DELAY_USEC);
      }
      adcRawReads[i] = adcRawReads[i] >> THERMISTOR_MEAS_AVG_RIGHT_BIT_SHIFT;
    }

    flagNeedToCalculateNewTempMeasurement = 1;
  }

  if (flagNeedToCalculateNewTempMeasurement) {
    int rawSnapshot[THERMISTOR_COUNT];
    for (int i = 0; i < THERMISTOR_COUNT; ++i) {
      rawSnapshot[i] = adcRawReads[i];
    }
    flagNeedToCalculateNewTempMeasurement = 0;

    for (int i = 0; i < THERMISTOR_COUNT; ++i) {
      Vo = rawSnapshot[i];
      if (Vo > 0) {
        R2 = R1 * (4095.0F / (float)Vo - 1.0F);
        if (R2 > 0) {
          logR2 = logf(R2);
          if (isfinite(logR2)) {
            T[i] = 1.0F / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2);
            T[i] = T[i] - 273.15F;
            T[i] = (T[i] * 9.0F) / 5.0F + 32.0F;
            T[i] += manualTempOffsetF;
          }
        }
      }
    }
    bool tempsValid = isfinite(T[0]) && isfinite(T[1]);
    if (tempsValid && !lastTempsWereValid) {
      logEvent("Temperature sampling valid t0=" + formatFloat1(T[0]) + "F t1=" + formatFloat1(T[1]) + "F");
    } else if (!tempsValid && lastTempsWereValid) {
      logEvent("Temperature sampling invalid adc0=" + String(adcRawReads[0]) + " adc1=" + String(adcRawReads[1]));
    }
    lastTempsWereValid = tempsValid;
    appendTemperatureHistory(T[0], T[1], millis());
    flagNeedToPostNewTempMeasurement = 1;
  }

  bool shouldRefreshMdns = (lastmDNSLookupTimeStampMSec == 0) ||
                           ((millis() - lastmDNSLookupTimeStampMSec) > mDNSLookupTimeIntervalMSec) ||
                           (httpResponseCode <= 0 && (millis() - lastHttpPostTimeMSec) < 30000UL &&
                            (millis() - lastmDNSLookupTimeStampMSec) > 30000UL);

  if (shouldRefreshMdns) {
    esp_task_wdt_reset();
    lastmDNSLookupTimeStampMSec = millis();
    DJBOOSHBOXIp = resolve_mdns_host(TARGET_HOSTNAME);

    if ((DJBOOSHBOXIp != IPAddress(1, 1, 1, 1)) && (DJBOOSHBOXIp != IPAddress(1, 1, 1, 2)) && (DJBOOSHBOXIp != IPAddress(1, 1, 1, 3))) {
      lastDJBOOSHBOXIp = DJBOOSHBOXIp;
      urlBase = String("http://") + DJBOOSHBOXIp.toString();
      logEvent("mDNS GOOD ip=" + DJBOOSHBOXIp.toString() + " url=" + urlBase);
    } else {
      if ((lastDJBOOSHBOXIp != IPAddress(1, 1, 1, 1)) && (lastDJBOOSHBOXIp != IPAddress(1, 1, 1, 2)) && (lastDJBOOSHBOXIp != IPAddress(1, 1, 1, 3))) {
        urlBase = String("http://") + lastDJBOOSHBOXIp.toString();
        logEvent("mDNS BAD revert cached ip=" + lastDJBOOSHBOXIp.toString() + " url=" + urlBase);
      } else {
        urlBase = String("http://") + String(TARGET_HOSTNAME) + String(".local");
        logEvent("mDNS BAD revert hostname url=" + urlBase);
      }
    }
  }

  if (flagNeedToPostNewTempMeasurement && ((millis() - lastHttpPostTimeMSec) > lastHttpPostIntervalMSec)) {
    if (!isfinite(T[0]) || !isfinite(T[1])) {
      logEvent("Skip HTTP post until both temperatures are valid");
      flagNeedToPostNewTempMeasurement = 0;
      return;
    }

    serverPath = urlBase + "/updatetemp?";
    for (int i = 0; i < THERMISTOR_COUNT; ++i) {
      serverPath += "t";
      serverPath += i;
      serverPath += "=";
      serverPath += String((int)T[i]);
      serverPath += "&";
    }

    Serial.print("HTTP GET: ");
    Serial.println(serverPath);

    http.begin(serverPath.c_str());
    httpResponseCode = http.GET();
    Serial.print("HTTP httpResponseCode: ");
    Serial.println(httpResponseCode);
    http.end();
    lastHttpPostTimeMSec = millis();

    if (httpResponseCode > 0) {
      lastHttpReponseGoodTimestampMsec = millis();
      logHttpEvent("HTTP GET", httpResponseCode, "OK");
    } else {
      lastHttpReponseBadTimestampMsec = millis();
      logHttpEvent("HTTP GET", httpResponseCode, "FAIL");
    }

    flagNeedToPostNewTempMeasurement = 0;
  }
}

void serial0DebugCom() {
  if (Serial.peek() == 'r') {
    Serial.read();
    Serial.print("timerPeriodADCISRMeasuSec was , ");
    Serial.print(timerPeriodADCISRMeasuSec);
    timerPeriodADCISRMeasuSec = (60000000UL / Serial.parseInt());
    Serial.print(" timerPeriodADCISRMeasuSec updated to ");
    Serial.println(timerPeriodADCISRMeasuSec);

    timerAlarmDisable(myTimerBPMPulseBoosh);
    timerAlarmWrite(myTimerBPMPulseBoosh, timerPeriodADCISRMeasuSec, true);
    timerAlarmEnable(myTimerBPMPulseBoosh);
  }

  if (Serial.peek() == 'x') {
    enableSerial0DisplayUpdates ^= 1;
    Serial.println("enableSerial0DisplayUpdates toggled");
  }

  if (Serial.peek() == 'o') {
    Serial.read();
    Serial.print("manualTempOffsetF was , ");
    Serial.print(manualTempOffsetF);
    manualTempOffsetF = Serial.parseFloat();
    Serial.print(" manualTempOffsetF updated to ");
    Serial.println(manualTempOffsetF);
  }

  if (Serial.peek() == 'f') {
    Serial.read();
    Serial.print("R1 was , ");
    Serial.print(R1);
    R1 = Serial.parseInt();
    Serial.print(" R1 updated to ");
    Serial.println(R1);
  }

  if (Serial.peek() == 'h') {
    Serial.print("last httpResponseCode: ");
    Serial.print(httpResponseCode);
    Serial.print(" at ");
    Serial.print(millis() - lastHttpPostTimeMSec);
    Serial.print(" mSec ago to URL:");
    Serial.println(serverPath);
  }

  if (Serial.peek() == 'w') {
    Serial.println("wifi info:");
    Serial.print(NETWORK_HOSTNAME);
    Serial.print(", at ");
    Serial.print(WiFi.localIP());
    Serial.print(", on SSID ");
    Serial.print(WiFi.SSID().c_str());
    Serial.print(", on channel ");
    Serial.print(WiFi.channel());
    Serial.print(", RSSI ");
    Serial.println(WiFi.RSSI());
    Serial.print("dBm, status ");
    switch (WiFi.status()) {
      case WL_IDLE_STATUS: Serial.println("WL_IDLE_STATUS"); break;
      case WL_NO_SSID_AVAIL: Serial.println("WL_NO_SSID_AVAIL"); break;
      case WL_SCAN_COMPLETED: Serial.println("WL_SCAN_COMPLETED"); break;
      case WL_CONNECTED: Serial.println("WL_CONNECTED"); break;
      case WL_CONNECT_FAILED: Serial.println("WL_CONNECT_FAILED"); break;
      case WL_CONNECTION_LOST: Serial.println("WL_CONNECTION_LOST"); break;
      case WL_DISCONNECTED: Serial.println("WL_DISCONNECTED"); break;
      default: Serial.println("UNKNOWN"); break;
    }
    Serial.print("ESP Board MAC Address: ");
    Serial.println(WiFi.macAddress());
  }

  while (Serial.available() > 0) {
    Serial.read();
  }
}

void displayDiagnosticUpdate() {
  if (enableSerial0DisplayUpdates) {
    lastDisplayDiagnosticSerialDumpTimeMsec = millis();

    Serial.print("mils:");
    Serial.print(currentMillis);
    Serial.print(", LED:");
    if (STATUS_LED_PIN >= 0) {
      Serial.print((digitalRead(STATUS_LED_PIN) == HIGH));
    } else {
      Serial.print("n/a");
    }

    Serial.print(", uptime:");
    Serial.print(formatUptimeHMS(millis()));
    Serial.print(", http:");
    Serial.print(httpResponseCode);

    for (int i = 0; i < THERMISTOR_COUNT; ++i) {
      Serial.print(", temp ");
      Serial.print(i);
      Serial.print("=");
      Serial.print(T[i]);
      Serial.print("F adc=");
      Serial.print(adcRawReads[i]);
    }

    Serial.println("");
  }
}

IPAddress resolve_mdns_host(const char *host_name) {
  Serial.print("Query A: ");
  Serial.println(host_name);

  esp_ip4_addr_t addr;
  addr.addr = 0;
  esp_err_t err = mdns_query_a(host_name, 5000, &addr);

  if (err) {
    if (err == ESP_ERR_NOT_FOUND) {
      Serial.print("Query Failed: ");
      Serial.println(err);
      return IPAddress(1, 1, 1, 3);
    }

    Serial.print("Query Failed: ");
    Serial.println(err);
    return IPAddress(1, 1, 1, 2);
  }

  Serial.println(String("Query returned: ") + IPAddress(addr.addr).toString());
  return IPAddress(addr.addr);
}
