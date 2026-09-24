# Pico: the 2 Hz USB debug printf can block core 0 for up to 1 s, which trips the 250 ms watchdog and resets the thruster path

Part of the 2026-09-24 ecosystem review, round 2 (see ../README.md).

## Summary
The Pico's `loop()` (core 0, which also clocks DShot out) prints about 400 bytes of debug output to USB CDC every 500 ms. In arduino-pico, `SerialUSB::write()` **blocks** while a host has the port open (`tud_cdc_connected()`) and the TX buffer is full. It gives up only after 1 s with no progress. The hardware watchdog is 250 ms. A laptop that holds the Pico's USB port open without reading it (a paused terminal, a crashed logging script, a serial monitor stuck behind a modal dialog) therefore stalls core 0 until the watchdog resets the Pico. The ESCs then lose signal and need a neutral window to re-arm (draft 24).

There is a related interlock. Bluejay enters its **bootloader** if the signal line stays high for about 150 ms, and in bidirectional mode the DShot pin idles high with a pull-up. A Pico stall that leaves the lines high for more than 150 ms sends the ESCs into the bootloader, which costs extra re-arm time. The 250 ms watchdog currently bounds this. **Do not raise `wdt_begin(250)` as a "fix".**

## Evidence
At commit `f1d3ba9`.

The print, on core 0, every 500 ms (`src/pico/main.cpp:396-411`):
```cpp
    if (now - dbg_ms >= 500) {
        dbg_ms = now;
        Serial.printf("link=%d armed=%d bidir=%d rpm_mode=%d loop=%d | ",
                      link_ok, armed, s_bidir, rpm_mode, g_rpm_loop);
        for (int i = 0; i < TL_NUM_THRUSTERS; i++) {
            ...
            Serial.printf("M%d[cmd=%d out=%d rpm=%d pres=%d e=%u d=%u c=%u n=%u] ",
```
The watchdog (`src/pico/main.cpp:178`):
```cpp
    rp2040.wdt_begin(250);                          // 250 ms hardware watchdog (margin vs a
```
The blocking write (`cores/rp2040/SerialUSB.cpp:156-177` @ `earlephilhower/arduino-pico@7a00f15`):
```cpp
    if (tud_cdc_connected() || _ss.ignoreFlowControl) {
        for (size_t i = 0; i < length;) {
            ...
            } else {
                tud_task();
                tud_cdc_write_flush();
                if (!tud_cdc_connected() ||
                        (!tud_cdc_write_available() && time_us_64() > last_avail_time + 1'000'000 /* 1 second */)) {
                    break;
```
The bidir pin idles pulled up (`src/bidir_dshot_x1.cpp:83`, `:131` @ `bastian2001/pico-bidir-dshot@b46f543`):
```cpp
	gpio_set_pulls(pin, true, false);
```
Bluejay's bootloader entry (`src/Bluejay.asm:496-508` @ `bird-sanctuary/bluejay@0368d11`):
```asm
    ; If input signal is high for about ~150ms, enter bootloader mode
    ...
input_high_check:
    jnb  RTX_BIT, bootloader_done       ; If low is detected, skip bootloader check
```

## Failure scenario
1. On the bench (or tethered in the pool), someone has `screen /dev/ttyACM1` open on the Pico to watch the `e=`/`d=` counters, then suspends it with Ctrl-S or the laptop sleeps. The host keeps the CDC port "connected" but stops reading.
2. The TX FIFO fills. The next `Serial.printf` spins in `write()`. Core 0 stops clocking DShot and stops feeding the watchdog.
3. At 250 ms the watchdog resets the Pico. During the stall, the bidir lines idled high. If the stall had lasted more than about 150 ms before the reset released the pins, the ESCs jumped to the bootloader.
4. After the reboot, the ESCs need about 300 ms of neutral to re-arm. Any thruster with a steady non-zero demand stays dead (draft 24), and the ESP32 reports it only on the OLED.

*The exact timing of the pins during a watchdog reset was not measured.*

## Suggested fix
- Write debug output only when it fits: `if (Serial.availableForWrite() >= len) Serial.write(buf, len);`, formatting into a local buffer first. Or set `Serial.ignoreFlowControl(true)` so writes drop instead of block (check the arduino-pico semantics). Or compile the debug print out of flight builds.
- Better, move the debug print to core 1 (which already owns the UART link), or give it a `snprintf` + non-blocking path.
- On any detected stall or fault, **drive the DShot pins low** (or send DShot 0 / 1048) rather than leaving them floating high, so an ESC can never be walked into its bootloader by the Pico's failure mode.
- Keep `wdt_begin(250)`. Add a comment that it also guards the Bluejay 150 ms bootloader window.

## How to verify (bench)
Props off, ESCs powered:
1. Open the Pico's USB port with a script that opens it and never reads (`python -c "import serial,time; s=serial.Serial('/dev/ttyACM1'); time.sleep(60)"`).
2. **Today:** within a few seconds the ESP32 OLED shows "PICO REBOOT#n", repeating, and the ESCs play their arming tones repeatedly.
3. **After:** no reboots. Uptime in the telemetry frame increases monotonically.

## Severity: Medium
A common bench habit resets the thruster controller. In the water it needs a USB host attached to the Pico, which is uncommon in flight, but the consequence (draft 24) is a silent loss of thrust.

## Related
- Draft 24 in this batch (Pico/ESC loss is invisible, and the ESC re-arm window).
- Round-1 draft 17 (Pico RPM PI windup).

---
_Generated by [Claude Code](https://claude.ai/code)_
