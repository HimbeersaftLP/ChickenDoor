#include <avr/wdt.h>
#include <Arduino.h>
#include <EEPROM.h>

constexpr auto DEBUG_MODE = true;

/* PINS */
constexpr auto PIN_BUTTON = 2;
constexpr auto PIN_LIGHT_SENSOR = A0;

constexpr auto PIN_MOTOR_OPEN = 3;   // Connect relay so that when it closes, the motor opens the door
constexpr auto PIN_MOTOR_CLOSE = 4;

constexpr auto PIN_SWITCH_TOP = 5;
constexpr auto PIN_SWITCH_BOTTOM = 6;

/* SETTINGS */
constexpr auto TOLERANCE_OPEN = 20;
constexpr auto TOLERANCE_CLOSE = TOLERANCE_OPEN;
constexpr auto MAX_OPEN_CLOSE_TIME = 30000; // Maximum amount of time opening/closing is allowed to take in milliseconds

/* CONSTANTS */
constexpr auto DOOR_STATUS_UNKNOWN = 2;
constexpr auto DOOR_STATUS_OPEN = 1;
constexpr auto DOOR_STATUS_CLOSED = 0;

/* CODE */
int savedLightLevel;
uint8_t doorStatus = DOOR_STATUS_UNKNOWN;

bool isButtonPressed() { return digitalRead(PIN_BUTTON) == LOW; }

// cc: https://forum.arduino.cc/index.php?topic=100231.msg751705#msg751705
void setSavedLightLevel() {
  byte lowByte = ((savedLightLevel >> 0) & 0xFF);
  byte highByte = ((savedLightLevel >> 8) & 0xFF);
  EEPROM.update(0, lowByte);
  EEPROM.update(1, highByte);
}

void getSavedLightLevel() {
  byte lowByte = EEPROM.read(0);
  byte highByte = EEPROM.read(1);
  savedLightLevel = ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

void processButton(int lightLevel) {
  digitalWrite(LED_BUILTIN, HIGH);
  savedLightLevel = lightLevel;
  setSavedLightLevel();
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
}

void enterErrorState() {
  digitalWrite(PIN_MOTOR_OPEN, LOW);
  digitalWrite(PIN_MOTOR_CLOSE, LOW);
  while (true)
  {
    wdt_reset();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    if (digitalRead(PIN_SWITCH_TOP) == LOW) {
      delay(500);
    }
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    if (digitalRead(PIN_SWITCH_BOTTOM) == LOW) {
      delay(500);
    }
  }
}

void waitForLow(uint8_t pin) {
  unsigned long startTime = millis();
  while (digitalRead(pin) != LOW) {
    if (millis() - startTime > MAX_OPEN_CLOSE_TIME) {
      enterErrorState();
    }
    wdt_reset();
  }
}

void initDoorStatus() {
  if (digitalRead(PIN_SWITCH_TOP) == LOW && digitalRead(PIN_SWITCH_BOTTOM) == HIGH) {
    doorStatus = DOOR_STATUS_OPEN;
  } else if (digitalRead(PIN_SWITCH_TOP) == HIGH && digitalRead(PIN_SWITCH_BOTTOM) == LOW) {
    doorStatus = DOOR_STATUS_CLOSED;
  } else if (digitalRead(PIN_SWITCH_TOP) == LOW && digitalRead(PIN_SWITCH_BOTTOM) == LOW) {
    enterErrorState();
  }
}

void openDoor() {
  digitalWrite(PIN_MOTOR_OPEN, HIGH);
  waitForLow(PIN_SWITCH_TOP);
  digitalWrite(PIN_MOTOR_OPEN, LOW);
  doorStatus = DOOR_STATUS_OPEN;
}

void closeDoor() {
  digitalWrite(PIN_MOTOR_CLOSE, HIGH);
  waitForLow(PIN_SWITCH_BOTTOM);
  digitalWrite(PIN_MOTOR_CLOSE, LOW);
  doorStatus = DOOR_STATUS_CLOSED;
}

void setup() {
  wdt_enable(WDTO_4S);
  if (DEBUG_MODE)
    Serial.begin(115200);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_SWITCH_TOP, INPUT_PULLUP);
  pinMode(PIN_SWITCH_BOTTOM, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_MOTOR_OPEN, OUTPUT);
  pinMode(PIN_MOTOR_CLOSE, OUTPUT);

  Serial.println("Hello world!");
  Serial.print("Saved light level: ");
  getSavedLightLevel();
  Serial.println(savedLightLevel);
  Serial.print("Current light level: ");
  Serial.println(analogRead(PIN_LIGHT_SENSOR));

  initDoorStatus();
}

void loop() {
  wdt_reset();
  int lightLevel = analogRead(PIN_LIGHT_SENSOR);
  Serial.print(lightLevel);
  Serial.print('\t');
  if (lightLevel < savedLightLevel - TOLERANCE_CLOSE && doorStatus != DOOR_STATUS_CLOSED) {
    Serial.print("Closing... ");
    closeDoor();
    Serial.println("done");
  } else if (lightLevel > savedLightLevel + TOLERANCE_OPEN && doorStatus != DOOR_STATUS_OPEN) {
    Serial.print("Opening... ");
    openDoor();
    Serial.println("done");
  } /*else {
    Serial.println("Doing nothing");
  }*/
  if (isButtonPressed())
    processButton(lightLevel);
}