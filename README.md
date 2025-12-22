TD42Tacho V3.6.1 RELEASE — Feature list
Core gauge

VR RPM input with smoothing + stable RPM display

OLED bar/graph display modes (RPM / Boost)

Boost display with PSI readout

Peak/Trip tracking (max RPM trip, trip km) + reset buttons from web UI

Engine hours counting (only above a running RPM threshold)

Display/UI

Boost graph bottom row shows RPM (left) and PSI (right) (same font size)

RPM and PSI flashing are independent

If both flashes are active at once, they flash opposite each other (RPM on while PSI off, then swap)

Boot/logo display holds for ~2 seconds (less “blink and miss it”)

SERVICE indicator flashes on-screen when Service Mode is active

Wi-Fi & Service Mode

Normal running: Wi-Fi OFF (fastest/stablest driving mode)

Service Mode: enter by 4× power cycles within the boot window

Service Mode starts AP hotspot (SSID td42tach) + web UI + update tools

Auto-timeout: Service Mode shuts down after 10 minutes and returns to Wi-Fi OFF

Web UI (config + updates)

Config page for tuning gauge behavior (bar RPM, redline, graph mode, etc.)

STA connect/disconnect for internet access when doing updates

Firmware update system using a manifest URL (great for GitHub hosting)

Live status polling endpoints for STA connection status

TD42Tacho V3.6.1 RELEASE — Bugfix report / changes

Fixed ESP32 core v3 compile break: WiFi.setAutoConnect() removed → handled safely (core v3 compatible).

Fixed STA connect lock/stall: prevents “wifi:sta is connecting, cannot set config” by disconnecting cleanly before applying new STA config.

Fixed update status endpoint build issue: removed the extern vs static function mismatch for routeUpdateStatus().

Removed weird web-page characters caused by encoding/emoji/unicode; replaced with plain ASCII and enforced proper page charset.

Improved “Connected / Connecting…” reporting for update flow (status polling + cache busting).

Added 2-second boot/logo hold for nicer startup UX.

Added on-screen SERVICE indicator so you can confirm Service Mode without Serial Monitor.

Kept update features available only during Service Mode (normal driving stays lightweight).
