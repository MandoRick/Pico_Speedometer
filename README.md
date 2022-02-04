# Pico_Speedometer

Hall Sensor (3) Based Speedometer/Odometer/Tachometer for Raspi Pico with Round Display (GC9A01A)

Using Earle F. Philhower, III "Raspberry Pi Pico Arduino core for all RP2040 boards" in Arduino IDE (https://github.com/earlephilhower/arduino-pico)

Notes on installing Philhower Core: https://learn.adafruit.com/rp2040-arduino-with-the-earlephilhower-core/installing-the-earlephilhower-core

display GC9A01A pins:

TFT_CLK (2)

TFT_DIN (3)

TFT_DC (4)

TFT_CS (5)

TFT_RST (6)

TFT_DOUT (7)

hall-effect sensor pins:

SENSOR_PIN1 (10)

SENSOR_PIN2 (11)

SENSOR_PIN3 (12)

bridged pins:

FLAG_PIN_FILE_LOCK_OUT (14) bridged to FLAG_PIN_FILE_LOCK_IN (15)

FLAG_PIN_FILE_UNLOCK_OUT (16) bridged to FLAG_PIN_FILE_UNLOCK_IN (17)

button pins:

BUTTON_PIN_LEFT (8)

BUTTON_PIN_RIGHT (9)

BUTTON_PIN_EMERGENCY (13)
