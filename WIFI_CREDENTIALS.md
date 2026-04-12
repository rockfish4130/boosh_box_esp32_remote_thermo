# WiFi Credentials

`wifi_credentials.h` contains live network credentials and must never be committed, copied into Git history, or pushed to any remote.

## Required local file

Create `wifi_credentials.h` in the project root next to `boosh_box_esp32_remote_thermo.ino`.

Use this format:

```c
// Do not commit wifi credentials to git.
#define BOOSH_WIFI_SSID_MW "your-ssid-1"
#define BOOSH_WIFI_PASS_MW "your-password-1"

#define BOOSH_WIFI_SSID_LL "your-ssid-2"
#define BOOSH_WIFI_PASS_LL "your-password-2"
```

## Notes

- The file is intentionally Git-ignored.
- Generate or provide this file locally before building or uploading firmware.
- Keep the macro names exactly as shown because the firmware expects these identifiers.
