# TD42Tacho — V3.6.1 RELEASE

ESP32-based tachometer + boost gauge for TD42 (and similar setups), with OLED display, web configuration, and firmware update support.

## What’s new in V3.6.1 RELEASE
- Service Mode (4× power-cycle trigger) + 10 minute timeout
- Wi-Fi is OFF in normal driving mode (faster + more stable)
- On-screen **SERVICE** indicator flashes when Service Mode is active
- Boost graph bottom row: **RPM (left)** + **PSI (right)** in matching font
- Independent RPM/PSI flashing; if both flash at once they flash opposite
- Update/config pages cleaned (no weird encoding characters)
- ESP32 Arduino core v3 compatible

---

## Hardware
- ESP32 (tested with standard ESP32 Dev Module / WROOM-class boards)
- OLED (U8g2 supported; typical 128×64 I2C OLED)
- VR conditioner / VR sensor input (RPM)
- Optional speed pulse input

> Pin mapping and wiring depends on your exact VR conditioner and OLED type. Check the defines at the top of the `.ino`.

---

## Build environment
- Arduino IDE
- ESP32 board package (Arduino-ESP32 core v3 supported)
- Libraries used:
  - U8g2
  - WiFi / WebServer (built-in with ESP32 core)
  - Preferences (NVS)
  - HTTPClient + Update (for manifest-based flashing)
  - ArduinoOTA (used in Service Mode only)

---

## Wi-Fi behavior (important)
### Normal Mode (default)
- Wi-Fi is **OFF**
- Gauge runs fastest/cleanest for driving

### Service Mode
Service Mode enables hotspot + web UI + update tools for a limited time.

**Enter Service Mode:**
1. Power ON/OFF the gauge **4 times** (quickly)
2. On the 4th boot it enables Service Mode

**What you’ll see:**
- OLED flashes **SERVICE**
- Hotspot appears: `td42tach`
- Service Mode lasts **10 minutes**, then Wi-Fi shuts down automatically

---

## Web UI
Connect to the hotspot and open:
- `http://192.168.4.1/` (dashboard)
- `http://192.168.4.1/config` (configuration)
- `http://192.168.4.1/update` (firmware updates)

---

## Firmware updates via GitHub (Manifest method)
This firmware supports updating from a **manifest.json** hosted on GitHub (raw or release asset).

### Manifest format
Create a JSON file like this:

```json
{
  "version": "3.6.1",
  "bin_url": "https://YOUR_HOSTED_URL/TD42Tacho_V3_6_1.bin",
  "notes": "Optional release notes here"
}
