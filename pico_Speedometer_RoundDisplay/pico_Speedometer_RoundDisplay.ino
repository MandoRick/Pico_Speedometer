/*-data-type------size---------description-----
  boolean        (8 bit)   -  [true/false]
  byte           (8 bit)   -  [0-255] unsigned number
  char           (8 bit)   -  [-128 to 127] signed number
  unsigned char  (8 bit)   -  [-128 to 127] signed number
  word           (16 bit)  -  [0-65535] unsigned number
  unsigned int   (16 bit)  -  [0-65535] unsigned number
  int            (16 bit)  -  [-32768 to 32767] signed number
  unsigned long  (32 bit)  -  [0-4,294,967,295] unsigned number usually for millis
  long           (32 bit)  -  [-2,147,483,648 to 2,147,483,647] signed number
  float          (32 bit)  -  [-3.4028235E38 to 3.4028235E38] signed number
  uint8_t        (8 bit)   -  [0-255] unsigned number
  int8_t         (8 bit)   -  [-127 - 127] signed number
  uint16_t       (16 bit)  -  [0-65,535] unsigned number
  int16_t        (16 bit)  -  [-32,768 - 32,767] signed number
  uint32_t       (32 bit)  -  [0-4,294,967,295] unsigned number
  int32_t        (32 bit)  -  [-2,147,483,648 - 2,147,483,647] signed number
  uint64_t       (64 bit)  -  [0-18,446,744,073,709,551,615] unsigned number
  int64_t        (64 bit)  -  [−9,223,372,036,854,775,808 - 9,223,372,036,854,775,807] signed number
  --------------------------------------------
  camelCase                -  anything that changes
  snake_case               -  variable's that are exclusive in a function
  Snake_Case               -  CLASS/struct exclusave varables/functions
  iNVERTEDcAMELcASE        -  outside code that is being accessed [database]
  SNake_CAse               -  duplicate varables inside the case function [frequently used in library names]
  ALL_CAPS                 -  const varable names or defines
  -------------by-jediRick--------------------
  VCC Power (3.3V / 5V input)
  GND Ground (GND)
  DIN Spi data input (MOSI)
  CLK Spi clock input (SCLK)
  CS  Chip selection, low active (CE0)
  DC  Data/Command selection high for data, low for command (GpiO)
  RST Reset, low active (GpiO)
  BL  Backlight (GpiO)
*/

#include <SPI.h>
#include <Adafruit_GFX.h>
#include "Adafruit_GC9A01A.h"

#define TFT_CLK (2)
#define TFT_DIN (3)
#define TFT_DC (4)
#define TFT_CS (5)
#define TFT_RST (6)
#define TFT_DOUT (7)
#define SENSOR_PIN1 (10)
#define SENSOR_PIN2 (11)
#define SENSOR_PIN3 (12)
#define FLAG_PIN_FILE_LOCK_OUT (14)
#define FLAG_PIN_FILE_LOCK_IN (15)
#define FLAG_PIN_FILE_UNLOCK_OUT (16)
#define FLAG_PIN_FILE_UNLOCK_IN (17)
#define wheelDiam 28  //in cm
#define pi 3.141592

#define myDebug 1     // 1 ON 0 OFF
#if myDebug == 1
#define debugln(x) Serial.println(x)
#define debug(x) Serial.print(x)
#else
#define debugln(x)
#define debug(x)
#endif

#define BUTTON_PIN_LEFT (8)
#define BUTTON_PIN_RIGHT (9)
#define BUTTON_PIN_EMERGENCY (13)

#define flasherPinOff (LOW)
#define flasherPinOn (HIGH)

volatile bool buttonBoolLeft = false;
volatile bool buttonBoolRight = false;
volatile bool buttonBoolEmergency = false;
volatile bool sensorBool1 = false;
volatile bool sensorBool2 = false;
volatile bool sensorBool3 = false;
volatile bool variableOwnershipFlag = false;
volatile bool buttonAction1 = false;
volatile bool buttonAction2 = false;
volatile bool buttonAction3 = false;
long turnSignalDebounceTime = 250;   //in millis
volatile int8_t stripeStepAmount = 1;
volatile int16_t racingStripeCurrent = 110;
volatile int16_t racingStripeOld = 110;
volatile unsigned long turnSignalDebouncePreviousMicrosLeft;
volatile unsigned long turnSignalDebouncePreviousMicrosRight;
volatile unsigned long emergencySignalDebouncePreviousMicros;
float rotationTime = 0;
float revsPerMin = 0;
float finalSpeedKph = 0;
float distanceTraveled = 0;
volatile unsigned long currentMicros = 0;
volatile unsigned long previousMicros = 0;
volatile int rpmRotationAngleCurrent = 0;
volatile int rpmRotationAngleOld = 359;
volatile int rpmIndDotCurrentX = 0;
volatile int rpmIndDotCurrentY = 0;
volatile int rpmIndDotOldX = 0;
volatile int rpmIndDotOldY = 0;
volatile int speedDigitSpacerCurrent = 105;
volatile int speedDigitSpacerOld = 105;
volatile int displayRpmCurrent = 0;
volatile int displayRpmOld = 0;
volatile int displaySpeedKphCurrent = 0;
volatile int displaySpeedKphOld = 99;
volatile float displayDistanceKmCurrent = 0;
volatile float displayDistanceKmOld = 99;
volatile int rotationCount = 0;
volatile int flasherPinCountNew = 0;
volatile int flasherPinCountOld = 0;
const uint8_t flasherPinsLeft[]PROGMEM = {18, 19, 20, 21};
const uint8_t flasherPinsRight[]PROGMEM = {22, 26, 27, 28};

//------- first timer stuff --------
volatile unsigned long turnSignalTimerCurrentMillis;
const unsigned long turnSignalTimerOnTime = 100;
const unsigned long turnSignalTimerOffTime = 100;
volatile unsigned long turnSignalTimerPrevioustMillis = 0;
unsigned long turnSignalTimerInterval = turnSignalTimerOnTime;
volatile boolean turnSignalTimerTriggerState = true;
//------- end first timer stuff --------

//------- version stuff --------
const String p_project = "Pico_Speedometer";
const uint8_t version_hi = 0;
const uint8_t version_lo = 1;
void versionPrint(void) {
  debug("RicksWorx: ");
  debugln(p_project);
  debug("Version: ");
  debug(version_hi);
  debug(".");
  debugln(version_lo);
}
//------- end version stuff --------

Adafruit_GC9A01A tft(TFT_CS, TFT_DC, TFT_DIN, TFT_CLK, TFT_RST, TFT_DOUT);

void setup() {
  delay(2000);
  Serial.begin(115200);
  debugln("Serial started");
  versionPrint();
  attachInterrupt(digitalPinToInterrupt(FLAG_PIN_FILE_LOCK_IN), triggerFlagBusy, RISING);
  attachInterrupt(digitalPinToInterrupt(FLAG_PIN_FILE_UNLOCK_IN), triggerFlagIdle, RISING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_LEFT), triggerButtonLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_RIGHT), triggerButtonRight, RISING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_EMERGENCY), triggerButtonEmergency, RISING);
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(flasherPinsLeft[i], OUTPUT);
    pinMode(flasherPinsRight[i], OUTPUT);
  }
  setupTft();
}

void setup1() {
  delay(2000);
  pinMode(FLAG_PIN_FILE_LOCK_OUT, OUTPUT);
  pinMode(FLAG_PIN_FILE_UNLOCK_OUT, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN1), triggerSensor1, RISING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN2), triggerSensor2, RISING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN3), triggerSensor3, RISING);
}

void loop() {
  updateVariables();
  checkButtonActions();
  checkTurnSignalTimer();
  renderTft();
}

void loop1() {
  checkSensors();
}

void checkTurnSignalTimer() {
  if (buttonBoolLeft || buttonBoolRight || buttonBoolEmergency) {
    turnSignalTimerCurrentMillis = millis();
    if ((unsigned long)(turnSignalTimerCurrentMillis - turnSignalTimerPrevioustMillis) >= turnSignalTimerInterval) {
      if (turnSignalTimerTriggerState) {
        turnSignalTimerInterval = turnSignalTimerOffTime;
        if (buttonBoolLeft) {
          digitalWrite(flasherPinsLeft[flasherPinCountNew], flasherPinOff);
        }
        if (buttonBoolRight) {
          digitalWrite(flasherPinsRight[flasherPinCountNew], flasherPinOff);
        }
        if (buttonBoolEmergency) {
          digitalWrite(flasherPinsRight[flasherPinCountNew], flasherPinOff);
          digitalWrite(flasherPinsLeft[flasherPinCountNew], flasherPinOff);
        }
      } else {
        turnSignalTimerInterval = turnSignalTimerOnTime;
        if (buttonBoolLeft) {
          digitalWrite(flasherPinsLeft[flasherPinCountOld], flasherPinOff);
          digitalWrite(flasherPinsLeft[flasherPinCountNew], flasherPinOn);
        }
        if (buttonBoolRight) {
          digitalWrite(flasherPinsRight[flasherPinCountOld], flasherPinOff);
          digitalWrite(flasherPinsRight[flasherPinCountNew], flasherPinOn);
        }
        if (buttonBoolEmergency) {
          digitalWrite(flasherPinsRight[flasherPinCountOld], flasherPinOff);
          digitalWrite(flasherPinsLeft[flasherPinCountOld], flasherPinOff);
          digitalWrite(flasherPinsRight[flasherPinCountNew], flasherPinOn);
          digitalWrite(flasherPinsLeft[flasherPinCountNew], flasherPinOn);
        }
        flasherPinCountOld = flasherPinCountNew;
        flasherPinCountNew++;
        if (flasherPinCountNew > 3) {
          flasherPinCountNew = 0;
        }
      }
      turnSignalTimerTriggerState = !turnSignalTimerTriggerState;
      turnSignalTimerPrevioustMillis = turnSignalTimerCurrentMillis;
    }
  } else {
    buttonBoolLeft = false;
    buttonBoolRight = false;
    buttonBoolEmergency = false;
    for (uint8_t i = 0; i < 4; i++) {
      digitalWrite(flasherPinsLeft[i], flasherPinOff);
      digitalWrite(flasherPinsRight[i], flasherPinOff);
    }
    flasherPinCountNew = 0;
  }
}

void triggerButtonLeft() {
  buttonAction1 = true;
}

void triggerButtonRight() {
  buttonAction2 = true;
}

void triggerButtonEmergency() {
  buttonAction3 = true;
}

void triggerSensor1() {
  sensorBool1 = true;
}

void triggerSensor2() {
  sensorBool2 = true;
}

void triggerSensor3() {
  sensorBool3 = true;
}

void triggerFlagBusy() {
  variableOwnershipFlag = true;
}

void triggerFlagIdle() {
  variableOwnershipFlag = false;
}

void checkSensors() {
  if (sensorBool1 && sensorBool2 && sensorBool3) {
    sensorBool1 = false;
    sensorBool2 = false;
    sensorBool3 = false;
    countRotation();
  }
}

void checkButtonActions() {
  if (buttonAction1 == true) {
    buttonAction1 = false;
    if ((long)(micros() - turnSignalDebouncePreviousMicrosLeft) >= turnSignalDebounceTime * 1000) {
      buttonBoolLeft = !buttonBoolLeft;
      buttonBoolRight = false;
      buttonBoolEmergency = false;
      turnSignalDebouncePreviousMicrosLeft = micros();
    }
    for (uint8_t i = 0; i < 4; i++) {
      digitalWrite(flasherPinsLeft[i], flasherPinOff);
      digitalWrite(flasherPinsRight[i], flasherPinOff);
    }
  }
  if (buttonAction2 == true) {
    buttonAction2 = false;
    if ((long)(micros() - turnSignalDebouncePreviousMicrosRight) >= turnSignalDebounceTime * 1000) {
      buttonBoolRight = !buttonBoolRight;
      buttonBoolLeft = false;
      buttonBoolEmergency = false;
      turnSignalDebouncePreviousMicrosRight = micros();
    }
    for (uint8_t i = 0; i < 4; i++) {
      digitalWrite(flasherPinsLeft[i], flasherPinOff);
      digitalWrite(flasherPinsRight[i], flasherPinOff);
    }
  }
  if (buttonAction3 == true) {
    buttonAction3 = false;
    if ((long)(micros() - emergencySignalDebouncePreviousMicros) >= turnSignalDebounceTime * 1000) {
      buttonBoolEmergency = !buttonBoolEmergency;
      buttonBoolRight = false;
      buttonBoolLeft = false;
      emergencySignalDebouncePreviousMicros = micros();
    }
    for (uint8_t i = 0; i < 4; i++) {
      digitalWrite(flasherPinsLeft[i], flasherPinOff);
      digitalWrite(flasherPinsRight[i], flasherPinOff);
    }
  }
}

void countRotation() {
  digitalWrite(FLAG_PIN_FILE_UNLOCK_OUT, LOW);
  digitalWrite(FLAG_PIN_FILE_LOCK_OUT, HIGH);  //lock variables being written to
  currentMicros = micros();
  rotationTime = currentMicros - previousMicros;
  rotationCount++;
  previousMicros = currentMicros;
  digitalWrite(FLAG_PIN_FILE_LOCK_OUT, LOW);  //unlock variables
  digitalWrite(FLAG_PIN_FILE_UNLOCK_OUT, HIGH);
}

void setupTft() {
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(GC9A01A_BLACK);
  //tft.drawCircle(120, 120, 120, GC9A01A_RED);
  tft.fillCircle(120, 120, 100, GC9A01A_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(GC9A01A_BLACK);
  //      (peakX, peakY, botLeftX, botLeftY, botRightX, botRightY, color)
  tft.fillTriangle(120, 120, 5, 240, 235, 240, GC9A01A_BLACK);
  tft.fillTriangle(120, 22, 130, 2, 110, 2, GC9A01A_GREEN);
  tft.setCursor(117, 4); tft.println("0");
  tft.setCursor(118, 4); tft.println("0");
  tft.fillTriangle(154, 28, 170, 13, 151, 6, GC9A01A_GREEN);
  tft.setCursor(155, 12); tft.println("1");
  tft.setCursor(156, 12); tft.println("1");
  tft.fillTriangle(183, 45, 202, 36, 188, 23, GC9A01A_GREEN);
  tft.setCursor(189, 30); tft.println("2");
  tft.setCursor(190, 30); tft.println("2");
  tft.fillTriangle(205, 71, 227, 70, 217, 52, GC9A01A_GREEN);
  tft.setCursor(214, 60); tft.println("3");
  tft.setCursor(215, 60); tft.println("3");
  tft.fillTriangle(216, 103, 238, 110, 234, 90, GC9A01A_GREEN);
  tft.setCursor(227, 98); tft.println("4");
  tft.setCursor(228, 98); tft.println("4");
  tft.fillTriangle(216, 137, 234, 151, 238, 131, GC9A01A_GREEN);
  tft.setCursor(227, 137); tft.println("5");
  tft.setCursor(228, 137); tft.println("5");
  tft.fillTriangle(205, 169, 217, 188, 227, 170, GC9A01A_GREEN);
  tft.setCursor(214, 173); tft.println("6");
  tft.setCursor(215, 173); tft.println("6");
  tft.fillTriangle(183, 195, 188, 217, 204, 203, GC9A01A_GREEN);
  tft.setCursor(188, 202); tft.println("7");
  tft.setCursor(189, 202); tft.println("7");
  tft.fillTriangle(153, 212, 151, 234, 170, 227, GC9A01A_GREEN);
  tft.setCursor(155, 220); tft.println("8");
  tft.setCursor(156, 220); tft.println("8");
  tft.fillTriangle(120, 218, 110, 238, 130, 238, GC9A01A_GREEN);
  tft.setCursor(118, 227); tft.println("9");
  tft.setCursor(119, 227); tft.println("9");
  tft.fillTriangle(87, 212, 70, 227, 89, 234, GC9A01A_GREEN);
  tft.setCursor(75, 222); tft.println("10");
  tft.setCursor(76, 222); tft.println("10");
  tft.fillTriangle(57, 195, 37, 204, 52, 217, GC9A01A_GREEN);
  tft.setCursor(42, 202); tft.println("11");
  tft.setCursor(43, 202); tft.println("11");
  tft.fillTriangle(35, 169, 13, 170, 23, 188, GC9A01A_GREEN);
  tft.setCursor(19, 171); tft.println("12");
  tft.setCursor(18, 171); tft.println("12");
  tft.fillTriangle(24, 137, 2, 130, 6, 151, GC9A01A_GREEN);
  tft.setCursor(4, 137); tft.println("13");
  tft.setCursor(5, 137); tft.println("13");
  tft.fillTriangle(24, 103, 6, 89, 2, 109, GC9A01A_RED);
  tft.setCursor(5, 98); tft.println("14");
  tft.setCursor(6, 98); tft.println("14");
  tft.fillTriangle(35, 71, 23, 52, 13, 70, GC9A01A_RED);
  tft.setCursor(18, 63); tft.println("15");
  tft.setCursor(19, 63); tft.println("15");
  tft.fillTriangle(57, 45, 52, 23, 37, 36, GC9A01A_RED);
  tft.setCursor(42, 32); tft.println("16");
  tft.setCursor(43, 32); tft.println("16");
  tft.fillTriangle(87, 28, 89, 6, 70, 13, GC9A01A_RED);
  tft.setCursor(76, 12); tft.println("17");
  tft.setCursor(77, 12); tft.println("17");
  tft.setTextSize(2);
  tft.setTextColor(GC9A01A_WHITE);
  tft.setCursor(105, 75);
  tft.println("KPH");
  tft.setTextSize(1);
  tft.setTextColor(GC9A01A_RED);
  tft.setCursor(95, 175);
  tft.println("RPM x 100");
  tft.setTextSize(2);
  tft.setTextColor(GC9A01A_MAGENTA);
  tft.setCursor(85, 150);
  tft.println("km:");
  //tft.drawRect(120, 120, 130, 150, GC9A01A_RED);
  //tft.drawRect(120, 120, 130, 150, GC9A01A_RED);
}

void updateVariables() {
  if (variableOwnershipFlag == false) {
    revsPerMin = 60000000 / rotationTime;
    finalSpeedKph = wheelDiam * revsPerMin * 0.001885;
    distanceTraveled = ((wheelDiam * pi) * rotationCount) * 0.00001;
    displayRpmCurrent = revsPerMin;
    displaySpeedKphCurrent = finalSpeedKph;
    displayDistanceKmCurrent = distanceTraveled;
    rpmRotationAngleCurrent = map(displayRpmCurrent, 0, 1799, 0, 359);
    //debugln("rpm: " + (String)displayRpmCurrent);
    //debugln("speed: " + (String)displaySpeedKphCurrent);
    //debugln("distance: " + (String)displayDistanceKmCurrent);
  }
}

void renderTft() {
  racingStripeCurrent = racingStripeCurrent + stripeStepAmount;
  if (racingStripeCurrent <= 90 || racingStripeCurrent >= 150) {
    stripeStepAmount = -stripeStepAmount;
  }
  tft.fillRect(racingStripeOld, 65, 6, 6, GC9A01A_BLACK);
  tft.fillRect(racingStripeCurrent, 65, 6, 6, GC9A01A_YELLOW);
  racingStripeOld = racingStripeCurrent;
  if (displaySpeedKphCurrent >= 999) {
    displaySpeedKphCurrent = 0;
  }
  if (displaySpeedKphCurrent != displaySpeedKphOld) {
    tft.setTextSize(6);
    if (displaySpeedKphCurrent >= 0 && displaySpeedKphCurrent <= 9) {
      speedDigitSpacerCurrent = 105;
    } else if (displaySpeedKphCurrent >= 9 && displaySpeedKphCurrent <= 99) {
      speedDigitSpacerCurrent = 85;
    } else if (displaySpeedKphCurrent >= 99 && displaySpeedKphCurrent <= 999) {
      speedDigitSpacerCurrent = 70;
    }
    tft.setTextColor(GC9A01A_BLACK);
    tft.setCursor(speedDigitSpacerOld, 100);
    tft.println(displaySpeedKphOld);
    tft.setTextColor(GC9A01A_WHITE);
    tft.setCursor(speedDigitSpacerCurrent, 100);
    tft.println(displaySpeedKphCurrent);
  }
  rpmIndDotCurrentY = 120 + 80 * (-cos(rpmRotationAngleCurrent * pi / 180));
  rpmIndDotCurrentX = 120 + 80 * (sin(rpmRotationAngleCurrent * pi / 180));
  if (rpmRotationAngleCurrent != rpmRotationAngleOld) {
    tft.fillCircle(rpmIndDotOldX, rpmIndDotOldY, 12, GC9A01A_BLACK);
    tft.fillCircle(rpmIndDotCurrentX, rpmIndDotCurrentY, 12, GC9A01A_CYAN);
  }
  if (displayDistanceKmCurrent != displayDistanceKmOld) {
    tft.setTextSize(2);
    tft.setTextColor(GC9A01A_BLACK);
    tft.setCursor(115, 150);
    tft.println(displayDistanceKmOld);
    tft.setTextColor(GC9A01A_MAGENTA);
    tft.setCursor(115, 150);
    tft.println(displayDistanceKmCurrent);
  }
  rpmIndDotOldX = rpmIndDotCurrentX;
  rpmIndDotOldY = rpmIndDotCurrentY;
  speedDigitSpacerOld = speedDigitSpacerCurrent;
  displaySpeedKphOld = displaySpeedKphCurrent;
  displayDistanceKmOld = displayDistanceKmCurrent;
  rpmRotationAngleOld = rpmRotationAngleCurrent;
  if (rpmRotationAngleCurrent >= 360) {
    rpmRotationAngleCurrent = 0;
  };
}
