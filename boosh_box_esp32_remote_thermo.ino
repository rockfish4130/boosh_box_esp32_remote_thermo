/*
Lava Lounge DJ Boosh Box Remote Thermo

Used for monitoring temperature of fuel depo tanks. 
2 100kOhm thermisters in a 100k + 100k divider across 3.3V
pins:
5 (ADC1_0)
8 (ADC1_3)

Expose temperatures via a HTTP web interface

Measure temperatures once per second i an timer ISR

Using board 'lolin32' from platform in folder: C:\Users\Matt\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.11
Using core 'esp32' from platform in folder: C:\Users\Matt\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.11
Board type: WEMOS LOLIN32 
CPU 80MHz
Debug level: none
Flash 80MHz
2024.03.15 MSW

Clone library https://github.com/espressif/arduino-esp32.git to ../arduino-esp32

Arduino IDE libs:
Adafruit SSD1306 2.5.9 https://github.com/adafruit/Adafruit_SSD1306
Adafruit GFX Library 1.11.9 https://github.com/adafruit/Adafruit-GFX-Library

Arduino IDE board libs:
Arduino ESP32 Boards 2.0.13
esp32 by Espressif 2.0.11


Memory usage:

 */

#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <esp_task_wdt.h>
#include "soc/rtc_wdt.h" // 2025: file missing after updated esp32 3.0.7, was built with 2.0.11
#include "wifi_credentials.h" // Intentionally omitted from Git for security. Defines BOOSH_WIFI_SSID and BOOSH_WIFI_PASS.

#define SW_REV_CODE "ESP32_2025_02_01_0030" // Echo a rev code on Serial0 for identifying version in the field
#define WDT_TIMEOUT 10     // define a 10 seconds WDT (Watch Dog Timer) TODO: debug


#define LED_BUILTIN 5 //ESP32 module LED

#define DEBUG_DISPLAY_DIAGNOSTIC_UPDATE_PERIOD_MSEC 500 // Update period in msec when dumping status to Serial

#define NETWORK_HOSTNAME "REMOTETHERMO" // name of this node
#define TARGET_HOSTNAME "RPIBOOSH" //name of node to which this node will be sending HTTP requests with temperature data
#define WIFI_CONNECTION_TIMEOUT_SECS 30

// IRQ timers
hw_timer_t *myTimerBPMPulseBoosh = NULL; // used for hardware timer/interrupt for BPM pulsing

// Globals
WiFiServer server(80);
WiFiMulti wifiMulti;

int httpResponseCode = -999; // make this a global so we can dump it to the web intfc

// state variables, flags, timestamps
unsigned long currentMillis = 0;
bool enableSerial0DisplayUpdates = 0; // Flag to dump hardware states to Serial0 for debug/dev
unsigned long lastADCMeasurementTimeMsec = 0; // debug heatbeat pulse timestamp

unsigned long lastDisplayDiagnosticSerialDumpTimeMsec = 0 ; // timestamp of last displayDiagnosticUpdate

unsigned long timerPeriodADCISRMeasuSec = 3000000; // 3 seconds between ADC measurements in the ISR

bool flagNeedToPostNewTempMeasurement = 0; // ISR sets this, main loop handles the HTTP GET
bool flagNeedToCalculateNewTempMeasurement = 0; // ISR sets this, main loop handles floating point, log etc

unsigned long lastHttpPostTimeMSec = 0; // mSec timestamp of last time temperatures were HTTP GET to DJ BOOSH BOX
unsigned long lastHttpPostIntervalMSec = 10000; // mSec interval to actually send 
unsigned long lastHttpReponseGoodTimestampMsec = 0;
unsigned long lastHttpReponseBadTimestampMsec = 0;

String urlBase = String("http://") + String(TARGET_HOSTNAME) + String(".local"); // TODO: switch this to hostname using mDNS
String urlPostfix = "/F";
String serverPath; //full String serverPath to be built up in loop(). made a global for dump to web UI, Serial0 etc.
HTTPClient http;
IPAddress DJBOOSHBOXIp = IPAddress(1, 1, 1, 1);
IPAddress lastDJBOOSHBOXIp = IPAddress(1, 1, 1, 1);

IPAddress resolve_mdns_host(const char* host_name);  // to resolve host IPs
unsigned long lastmDNSLookupTimeStampMSec = 0;
unsigned long mDNSLookupTimeIntervalMSec = 60 * 1000; // set to 60 for production
#define THERMISTOR_COUNT 2


// #define THERMISTOR_PIN1 36   // module pins (GPIO36) and (GPIO36) connected to thermister 100+100k divider circuit across 3.3V
// #define THERMISTOR_PIN2 39

// https://iotlearner.com/2024/03/12/esp32-pinout-guide-understand-every-pin/
#define THERMISTOR_PIN1 A0   // module pins (GPIO36) and (GPIO36) connected to thermister 100+100k divider circuit across 3.3V
#define THERMISTOR_PIN2 A3

#define THERMISTOR_MEAS_AVG_COUNT 16 // number of ADC measurements to perform and average, MUST be a power of 2 as we use int right shifts to avoid floating point in the ISR!
#define THERMISTOR_MEAS_AVG_DELAY_USEC 100 // uSec between measurements
#define THERMISTOR_MEAS_AVG_RIGHT_BIT_SHIFT 4 // right shift 4 = div16


int ThermistorPins[THERMISTOR_COUNT] = {THERMISTOR_PIN1, THERMISTOR_PIN2};
//should be volatile
int Vo;
volatile int adcRawReads[THERMISTOR_COUNT];
float R1 = 10000; //10000 != the 100kR used...
float logR2, R2, T[THERMISTOR_COUNT];
//float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
// https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation for thermister

// Manual calibration of specific thermistor
// https://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm
// 	ref P1 T F	ref P2 TF	Tc Ohms	ref average C
// Tcouple sealed metal	70	70	11610	21.11111111
// Tcouple sealed metal	37	37	28700	2.777777778
// Tcouple sealed metal	111	111	4260	43.88888889
// Calculated Steinhart-Hart model coefficients
float c1 = 1.274219988e-03, c2 = 2.171368266e-04, c3 = 1.119659695e-07;
// Calculated β model coefficients:
// R(25C) = 9720.21 Ohm
// β = 4008.03K


float manualTempOffsetF = 11.17; // Yes, it's nonlinear and not this simple, but we're adding this offset even after calibarting the Steinhart-Hart coeffs

void IRAM_ATTR IRQdoADCMeasurement() {
  int i;
  int j;
  lastADCMeasurementTimeMsec = millis();
  
  digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN)); // toggle LED

  Vo = 0;
  
  
  for(i=0;i<THERMISTOR_COUNT;++i) {
    adcRawReads[i] = 0;
    for(j=0;j<THERMISTOR_MEAS_AVG_COUNT;++j) {
      adcRawReads[i] += analogRead(ThermistorPins[i]);
      delayMicroseconds(THERMISTOR_MEAS_AVG_DELAY_USEC);
    }
    adcRawReads[i] = adcRawReads[i]>>THERMISTOR_MEAS_AVG_RIGHT_BIT_SHIFT;
  }
  // no floating point math in the ISR

  flagNeedToPostNewTempMeasurement = 1;
  flagNeedToCalculateNewTempMeasurement = 1;
}

void setup()
{
    int cntWifiTimeout = 0;
    
    pinMode(LED_BUILTIN, OUTPUT);      // set the LED pin mode

    delay(200);

    Serial.begin(115200);

    delay(10);
    
    wifiMulti.addAP(BOOSH_WIFI_SSID_MW, BOOSH_WIFI_PASS_MW);
    wifiMulti.addAP(BOOSH_WIFI_SSID_LL, BOOSH_WIFI_PASS_LL);

    WiFi.setHostname(NETWORK_HOSTNAME);

    cntWifiTimeout = 0;
    while ((wifiMulti.run() != WL_CONNECTED) && (cntWifiTimeout<(WIFI_CONNECTION_TIMEOUT_SECS * 2))) {
        delay(500);
        Serial.print(".");
        cntWifiTimeout++;
    }

    Serial.println("WiFi connected.");
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.SSID());
    Serial.println(WiFi.RSSI());

    int mDNSStatus = MDNS.begin(NETWORK_HOSTNAME);
    if (!mDNSStatus) {
        Serial.println("Error starting mDNS");
    }
    else {
        Serial.println(String("mDNS OK ") + String(NETWORK_HOSTNAME));
    }
  
    server.begin();

    rtc_wdt_protect_off(); //https://stackoverflow.com/questions/51750377/how-to-disable-interrupt-watchdog-in-esp32-or-increase-isr-time-limit
    rtc_wdt_disable(); //https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/wdts.html

    // IRQ
    // at 1 second, toggle right hand GPIOs
    myTimerBPMPulseBoosh = timerBegin(0, 80, true);
    timerAttachInterrupt(myTimerBPMPulseBoosh, &IRQdoADCMeasurement, false);
    timerAlarmWrite(myTimerBPMPulseBoosh, timerPeriodADCISRMeasuSec, true); 


    timerAlarmEnable(myTimerBPMPulseBoosh);
    

}

void loop(){
  int i;
  String url;
  char cstr[10] = "";

  currentMillis = millis(); // save the current timer value

  // Debug monitor Serial0 for bench debug
  serial0DebugCom();

  serviceWebInterface();

  // Debug heatbeat pulse: every 200ms or so, toggle LED_BUILTIN for a visual heartbeat inside the plastic case
  if ( ((currentMillis - lastDisplayDiagnosticSerialDumpTimeMsec ) > DEBUG_DISPLAY_DIAGNOSTIC_UPDATE_PERIOD_MSEC) || (currentMillis < lastDisplayDiagnosticSerialDumpTimeMsec ) ) {
    displayDiagnosticUpdate();
    // visualHeartbeat();
  }

  if (flagNeedToCalculateNewTempMeasurement) {
    // do the floating point math here, not in the ISR
    for(i=0;i<THERMISTOR_COUNT;++i) {
      Vo = adcRawReads[i];
      if ( Vo > 0 ) {
        R2 = R1 * (4095.0 / (float)Vo - 1.0); //(12-bit ADC max 0xfff)
        // guard against expcetion of log of 0 or negative
        if ( R2 > 0 ) {
          logR2 = log(R2);
          if ( logR2 > 0 ) {
            // Serial.println("in logR2>0");
            T[i] = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
            T[i] = T[i] - 273.15;
            T[i] = (T[i] * 9.0)/ 5.0 + 32.0;
            T[i] += manualTempOffsetF; 
          }
        }
      }
    }
    flagNeedToCalculateNewTempMeasurement = 0;
  }

  if (((millis() - lastmDNSLookupTimeStampMSec) > mDNSLookupTimeIntervalMSec) || (lastmDNSLookupTimeStampMSec==0))  {
    lastmDNSLookupTimeStampMSec = millis();
  // periodically look up IP, just in case of change over serval days of DJBOOSHBOX potentially being power cycled
    
    DJBOOSHBOXIp = resolve_mdns_host(TARGET_HOSTNAME);
    Serial.println("in mDNSLookupTimeIntervalMSec...");

    // if good IP lookup
    if ( (DJBOOSHBOXIp!=IPAddress(1, 1, 1, 1)) && (DJBOOSHBOXIp!=IPAddress(1, 1, 1,2)) && (DJBOOSHBOXIp!=IPAddress(1, 1, 1, 3)) ) {
      lastDJBOOSHBOXIp = DJBOOSHBOXIp;
      urlBase = String("http://") + DJBOOSHBOXIp.toString();
      Serial.print("mDNS lookup GOOD, URL=");
      Serial.println(urlBase);
    } else {
      if ( (lastDJBOOSHBOXIp!=IPAddress(1, 1, 1, 1)) && (lastDJBOOSHBOXIp!=IPAddress(1, 1, 1,2)) && (lastDJBOOSHBOXIp!=IPAddress(1, 1, 1, 3)) ) {
         //else revert to last good IP lookup
         urlBase = String("http://") + lastDJBOOSHBOXIp.toString();
         Serial.print("mDNS lookup BAD revert to cached IP, URL=");
         Serial.println(urlBase);
      } else {
         // else revert to hostname
        urlBase = String("http://") + String(TARGET_HOSTNAME) + String(".local");
        Serial.print("mDNS lookup BAD, revert to hostname, URL=");
        Serial.println(urlBase);
      }
 
    }
    Serial.println("DONE mDNSLookupTimeIntervalMSec...");



  }
    

  if (flagNeedToPostNewTempMeasurement && ( (millis() - lastHttpPostTimeMSec) > lastHttpPostIntervalMSec )) {
    

    // just use the native hostname in the URL and let mDNS handle the resolution
    // urlBase = String("http://") + DJBOOSHBOXIP.toString();

    serverPath = urlBase + "/updatetemp?";
    for(i=0;i<THERMISTOR_COUNT;++i) {
        // httpPost(urlBase + "/" + String(int(T[i])) + urlPostfix, beatChangeEffectsData);
        // Serial.print("HTTP POSTed ")
        // Serial.println(urlBase + String(int(T[i])) + urlPostfix));
        serverPath = serverPath + "t";
        serverPath = serverPath + i;
        serverPath = serverPath + "=";
        
        serverPath = serverPath + String(int(T[i])) ; 
        serverPath = serverPath + "&";
        
      }
      
      Serial.print("HTTP GET: ");
      Serial.println(serverPath);
      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());

      // If your need Node-RED/server authentication, insert user and password below
      //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");

      // Send HTTP GET request
      httpResponseCode = http.GET();
      Serial.print("HTTP httpResponseCode: ");
      Serial.println(httpResponseCode);
      // Free resources
      http.end();
      lastHttpPostTimeMSec = millis();

      if (httpResponseCode > 0) {
        lastHttpReponseGoodTimestampMsec = millis();
      } else {
        lastHttpReponseBadTimestampMsec = millis();
      }

      flagNeedToPostNewTempMeasurement = 0;
  }
 
}

void serviceWebInterface() {
  // Handle web interface
  int i;
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            // the content of the HTTP response follows the header:
            client.print("<head><title>REMOTETHERMO Web Interface</title></head><body bgcolor=\"black\"><h3 style=\"font-family:verdana;color:Tomato\">LAVA LOUNGE REMOTE THERMO BOOSH ESP32</h3>");
            client.print("<p style=\"font-family:verdana;color:white;font-size:100%\">");
            
                    
            for(i=0;i<THERMISTOR_COUNT;++i) {
              client.print(" temp at thermistor # ");
              client.print(i);
              client.print(" = ");
              client.print(T[i]);
              client.print(" F,");
              client.print(" ADC raw read code ");
              client.print(adcRawReads[i]);
              client.print(", ");
              

              client.print(" measured ");
              client.print((millis()-lastADCMeasurementTimeMsec)/1000.0);
              
              client.print(" seconds ago <br>");
            }

            // timerPeriodADCISRMeasuSec
            client.print("<br> timerPeriodADCISRMeasuSec ");
            client.print(timerPeriodADCISRMeasuSec/1000000);
            client.print(" seconds between ADC thermistor ISR measurements<br> ");

            
            // manualTempOffsetF
            client.print("<br> manual offset: ");
            client.print(manualTempOffsetF);
            client.print(" F degrees<br> ");

            //lastHttpPostTimeMSec
            // httpResponseCode = -999; 
            client.print("<br> last httpResponseCode: ");
            client.print(httpResponseCode);
            client.print(" at ");
            client.print(millis()-lastHttpPostTimeMSec);
            client.print(" mSec ago to URL:<br>");
            //serverPath
            client.print(serverPath);
            

            //good, bad http last times
            // lastHttpReponseBadTimestampMsec = 0;
            client.print(" <br><br>");
            client.print("last HTTP responses:");
            client.print("<br>last GOOD HTTP response: ");
            client.print((millis()-lastHttpReponseGoodTimestampMsec)/1000.0/60.0);
            client.print(" mins ago ");
            client.print("<br>last BAD HTTP response: ");
            client.print((millis()-lastHttpReponseBadTimestampMsec)/1000.0/60.0);
            client.print(" mins ago <br>");
            
            


            client.print(" <br><br>");
            client.print("mDNS lookup of ");
            client.print(TARGET_HOSTNAME);
            client.print(" last resolved to ");
            client.print(DJBOOSHBOXIp.toString());
            


            
                    
            
            
            
            client.print("<br> millis ");
            client.print(millis());
            client.print(" mSec<br>");

            client.print("<br> Uptime ");
            client.print(float(millis())/1000.0F/60.0F/60.0F);
            client.print(" hrs <br>");
            
            client.print(NETWORK_HOSTNAME);
            client.print(", at ");
            client.print(WiFi.localIP());
            client.print(", on SSID ");
            client.print(WiFi.SSID().c_str());
            client.print(", on channel ");
            client.print(WiFi.channel());
            client.print(", enc: ");
            switch (WiFi.encryptionType(0)){
              case WIFI_AUTH_OPEN:
                  client.print("open");
                  break;
              case WIFI_AUTH_WEP:
                  client.print("WEP");
                  break;
              case WIFI_AUTH_WPA_PSK:
                  client.print("WPA");
                  break;
              case WIFI_AUTH_WPA2_PSK:
                  client.print("WPA2");
                  break;
              case WIFI_AUTH_WPA_WPA2_PSK:
                  client.print("WPA+WPA2");
                  break;
              case WIFI_AUTH_WPA2_ENTERPRISE:
                  client.print("WPA2-EAP");
                  break;
              case WIFI_AUTH_WPA3_PSK:
                  client.print("WPA3");
                  break;
              case WIFI_AUTH_WPA2_WPA3_PSK:
                  client.print("WPA2+WPA3");
                  break;
              case WIFI_AUTH_WAPI_PSK:
                  client.print("WAPI");
                  break;
              default:
                  client.print("unknown");
            }
            client.print(" at ");
            client.print(WiFi.RSSI());
            client.print(" dBm, MAC ");
            client.print(WiFi.macAddress());
            client.print("<br>");
            client.print("build ");
            client.print(SW_REV_CODE);
            client.print("<br>");

            // The HTTP response ends with another blank line:
            
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(5, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(5, LOW);                // GET /L turns the LED off
        }
        // web easter egg
        if (currentLine.endsWith("MSW")) {
          unsigned int pattern = currentLine.substring(currentLine.length()-4,currentLine.length()-3)[0] - '0';
          if ((pattern>=0) && (pattern<THERMISTOR_COUNT)) {
            Serial.print("WEB INTERFACE MSW  "); 
            Serial.println(pattern); 
            
          }
          
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
}


//Debug monitor Serial0 for bench debug
void serial0DebugCom() {
 
  // r223, r400, l10, etc for timerPeriodADCISRMeasuSec in BPM units
  if (Serial.peek() == 'r') {       
    Serial.read();
    Serial.print("timerPeriodADCISRMeasuSec was , "); 
    Serial.print(timerPeriodADCISRMeasuSec); 
    timerPeriodADCISRMeasuSec = (60000000UL/ Serial.parseInt() ); //convert to BPM
    Serial.print(" timerPeriodADCISRMeasuSec updated to "); 
    Serial.println(timerPeriodADCISRMeasuSec); 

    timerAlarmDisable(myTimerBPMPulseBoosh);
    // update the timer IRQ
    timerAlarmWrite(myTimerBPMPulseBoosh, timerPeriodADCISRMeasuSec, true); 
    timerAlarmEnable(myTimerBPMPulseBoosh);
    
  }
  
  // x to toggle enableSerial0DisplayUpdates
  if (Serial.peek() == 'x') {       
    enableSerial0DisplayUpdates ^= 1;
    Serial.println("enableSerial0DisplayUpdates toggled"); 
  }

  //o11.2, o10, o-1.15, etc to set manualTempOffsetF
  if (Serial.peek() == 'o') {       
    Serial.read();
    Serial.print("manualTempOffsetF was , "); 
    Serial.print(manualTempOffsetF); 
    manualTempOffsetF = Serial.parseFloat() ; 
    Serial.print(" manualTempOffsetF updated to "); 
    Serial.println(manualTempOffsetF); 
  }


  //f10000, f10000, f1000, etc to set R1 resistor divider circuit value
  if (Serial.peek() == 'f') {       
    Serial.read();
    Serial.print("R1 was , "); 
    Serial.print(R1); 
    R1 = Serial.parseInt() ; //convert to BPM
    Serial.print(" R1 updated to "); 
    Serial.println(R1); 

    timerAlarmDisable(myTimerBPMPulseBoosh);
    // update the timer IRQ
    timerAlarmWrite(myTimerBPMPulseBoosh, timerPeriodADCISRMeasuSec, true); 
    timerAlarmEnable(myTimerBPMPulseBoosh);
    
  }

  // h to dump http info
  if (Serial.peek() == 'h') {
    Serial.print("last httpResponseCode: ");
    Serial.print(httpResponseCode);
    Serial.print(" at ");
    Serial.print(millis()-lastHttpPostTimeMSec);
    Serial.print(" mSec ago to URL:");
    Serial.print(serverPath);
    Serial.println("");
    
  }
  // w to dump WiFi info
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
    Serial.print("dBm ");
    Serial.print(", status ");
    switch(WiFi.status()) {
      case WL_IDLE_STATUS: 
        Serial.println("WL_IDLE_STATUS");
        break;
      case WL_NO_SSID_AVAIL:
        Serial.println("WL_NO_SSID_AVAIL");
        break;
      case WL_SCAN_COMPLETED:
        Serial.println("WL_SCAN_COMPLETED");
        break;
      case WL_CONNECTED:
        Serial.println("WL_CONNECTED");
        break;
      case WL_CONNECT_FAILED:
        Serial.println("WL_CONNECT_FAILED");
        break;
      case WL_CONNECTION_LOST:
        Serial.println("WL_CONNECTION_LOST");
        break;
      case WL_DISCONNECTED:
        Serial.println("WL_DISCONNECTED");
        break;
    }
    Serial.print("ESP Board MAC Address:  ");
    Serial.println(WiFi.macAddress());
    
  }


  while (Serial.available() > 0) {
    Serial.read();
  }

}

void displayDiagnosticUpdate() {
  int i;
  
  if( enableSerial0DisplayUpdates ) {
    lastDisplayDiagnosticSerialDumpTimeMsec = millis();
  
    Serial.print("mils:");
    Serial.print(currentMillis);
    Serial.print(", LED:");
    Serial.print((digitalRead(LED_BUILTIN) == HIGH));
    
    Serial.print(",");
    Serial.print(String(timerPeriodADCISRMeasuSec));
      
    Serial.print(", meas  ");
    Serial.print((millis()-lastADCMeasurementTimeMsec)/1000.0);
    
    for(i=0;i<THERMISTOR_COUNT;++i) {
      Serial.print(", temp at index ");
      Serial.print(i);
      Serial.print(" = ");
      Serial.print(T[i]);
      Serial.print(" F,");
      
      Serial.print(" analogRead of ");
      Serial.print(ThermistorPins[i]);
      Serial.print(" = ");
      Serial.print(adcRawReads[i]);
    }
    Serial.print(" = ");
    Serial.print(adcRawReads[i]);

    Serial.println("");
  }
}

// to resolve host IPs
IPAddress resolve_mdns_host(const char* host_name) {
    Serial.print("Query A: ");
    Serial.println(host_name);
    // Serial.println(".local");

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
