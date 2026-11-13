# Level 1: Dual-Core Tasks, Health and Watchdog

**Team Core32 (T04)** · FastRoboX 1.0 Grand Finale · ESP32-S3-CAM

The firmware runs every job as a separate FreeRTOS task on both CPU cores, with no blocking waits. A task watchdog restarts the board if any task freezes.

## Evidence

- Git tag: `level-01`
- Documentation and video: [Google Drive](https://drive.google.com/drive/folders/1NhekVwZNdqFo9HvLmIQ_xfB4g_q1X2Ag?usp=sharing)

## Hardware and pins

| Module | Pins (GPIO) |
|---|---|
| DHT11 | DATA 41 · VCC 45 · GND 3 |
| Flame sensor | DO 42 · VCC 3.3V · GND 2 |
| RC522 RFID | SDA 21 · SCK 47 · MOSI 38 · MISO 39 · RST 40 |
| Active buzzer | + 1 · − 2 |
| Touch panic pad | 14 (from Level 3) |
| RGB LED | 48 (onboard) |

## Tasks

| Task | Core | Period |
|---|---|---|
| ledTask | 1 | 50 ms (2 Hz heartbeat) |
| sensorTask | 1 | 100 ms (flame), 5 s (DHT11) |
| mainTask | 1 | 50 ms |
| rfidTask | 0 | 200 ms |
| statusTask | 0 | 1 s |
| serialTask | 0 | 20 ms |

Task watchdog timeout: 5 s.

## Build and upload

1. Arduino IDE with **Arduino-ESP32 core 3.x**. Select the board **ESP32S3 Dev Module**.
2. Install these libraries: **MFRC522** and **DHT sensor library** (Adafruit).
3. Open the `.ino` file, then upload it.
4. Open the Serial Monitor at **115200 baud**, with the line ending set to **Newline**.

## Serial commands

`help` · `status` · `health` · `tasks` · `ver` · `hang`

The test-only commands `reboot` and `crash` force the SOFTWARE and CRASH reset reasons.

## Expected output

```
[EVENT] BOOT reason=POWER_ON
[STATUS] up=12s T=26.1C RH=53% HI=- flame=0 door=LOCKED state=SAFE trend=- rssi=- heap=316k psram=8189k chip=33.2C fw=1.0.0
[CARD] uid=A1B2C3D4
```


