# Remote Thermo PlatformIO Port

This project was developed as an Arduino IDE sketch for a `WEMOS LOLIN32` and has been prepared for PlatformIO with a generic `esp32dev` target.

## Target board assumptions

- Board listing: `ESP32 ESP-32D WIFI Development Board Module CH340C With 0.96 OLED Screen Yellow Blue Display`
- PlatformIO board: `esp32dev`
- Flash / CPU baseline: ESP32, 4 MB flash, Arduino framework
- Built-in OLED wiring from the listing image: `I2C 0x3C`, `GPIO21 = SDA`, `GPIO22 = SCL`
- Thermistor inputs are mapped to `GPIO36` and `GPIO39`, which are ADC1 input-only pins and closest to the original `A0` / `A3` intent

## Layout

- `src/main.cpp` wraps the existing `.ino` file so the same sketch logic is reused in PlatformIO.
- `platformio.ini` adds `-I .` so the existing root-level `wifi_credentials.h` remains usable without duplicating secrets.

## Notes

- No OLED code was added during this port. The built-in display wiring is documented so it can be enabled later without redoing the board mapping work.
- `STATUS_LED_PIN` is set to `-1` in PlatformIO because this board listing does not clearly expose a dedicated user LED. If you confirm one exists, override it in `platformio.ini`.
- The sketch now tolerates the `soc/rtc_wdt.h` header being absent, which avoids a hard build failure on newer ESP32 Arduino cores.

## Verification

- PlatformIO build succeeded for environment `esp32-oled`.
- Firmware upload succeeded to `COM12`.
- The attached device identified itself during upload as `ESP32-D0WD-V3`.

## Power Notes

- Firmware now forces the ESP32 CPU clock to `80 MHz`.
- Bluetooth is disabled and controller memory is released.
- Periodic mDNS refresh is reduced to boot, every 15 minutes, or shortly after an HTTP failure instead of constant 1-minute refresh.
- In bench testing after these changes, observed current draw dropped to roughly `60 mA` average or lower at about `3.7 V`.
