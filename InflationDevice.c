#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pin Definitions
const int powerBtnPin = 2;
const int alertBtnPin = 3;
const int redLedPin = 4;
const int greenLedPin = 5;
const int buzzerPin = 6;
const int airPumpRelayPin = 7;
const int solenoidRelayPin = 8;
const int pressureSensorPin = A0;  // Analog pin for pressure sensor
const int adjustPressurePin = A1;   // Analog pin for pressure adjustment knob

// LCD Display Setup
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address for the LCD

// Constants
const int maxPressure = 10;  // Maximum pressure in Bar
const float pressureTolerance = 0.2;  // Tolerance range for pressure matching
const int releaseRate = 1;  // Bar per minute for controlled release
const int buzzerFreq = 420;  // Frequency for the buzzer

void setup() {
  pinMode(powerBtnPin, INPUT_PULLUP);
  pinMode(alertBtnPin, INPUT_PULLUP);
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(airPumpRelayPin, OUTPUT);
  pinMode(solenoidRelayPin, OUTPUT);

  lcd.begin(16, 2);
  lcd.clear();
}

float readPressureSensor() {
  int sensorValue = analogRead(pressureSensorPin);
  return (sensorValue / 1023.0) * maxPressure;
}

float readDesiredPressure() {
  int sensorValue = analogRead(adjustPressurePin);
  return (sensorValue / 1023.0) * maxPressure;
}

void activateBuzzer() {
  tone(buzzerPin, buzzerFreq, 100); // Buzzer on for 100ms
  delay(200); // Pause before next tone
  noTone(buzzerPin); // Buzzer off
}

void deactivateBuzzer() {
  noTone(buzzerPin); // Stop buzzer
}

void controlledRelease() {
  digitalWrite(solenoidRelayPin, HIGH);
  delay(1000); // Simulate release time
  digitalWrite(solenoidRelayPin, LOW);
}

void displayPressure(float desiredPressure, float currentPressure) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Desired: ");
  lcd.print(desiredPressure);
  lcd.print(" Bar");
  lcd.setCursor(0, 1);
  lcd.print("Current: ");
  lcd.print(currentPressure);
  lcd.print(" Bar");
}

void loop() {
  static bool systemActive = false;
  static float desiredPressure = 0;
  static float currentPressure = 0;

  if (digitalRead(powerBtnPin) == LOW) {
    systemActive = true;
  }

  if (systemActive) {
    desiredPressure = readDesiredPressure();
    currentPressure = readPressureSensor();
    
    float pressureDiff = abs(currentPressure - desiredPressure);

    if (pressureDiff <= pressureTolerance) {
      digitalWrite(redLedPin, LOW);
      digitalWrite(greenLedPin, HIGH);
      digitalWrite(airPumpRelayPin, LOW);
      deactivateBuzzer();

      if (digitalRead(alertBtnPin) == LOW) {
        controlledRelease();
        systemActive = false;
      }
    } else {
      digitalWrite(greenLedPin, LOW);
      digitalWrite(redLedPin, HIGH);

      if (currentPressure < desiredPressure) {
        digitalWrite(airPumpRelayPin, HIGH);  // Increase pressure
      } else {
        digitalWrite(airPumpRelayPin, LOW);   // Pressure too high
        activateBuzzer();
        controlledRelease();
      }
    }

    displayPressure(desiredPressure, currentPressure);
  } else {
    digitalWrite(redLedPin, LOW);
    digitalWrite(greenLedPin, LOW);
    digitalWrite(airPumpRelayPin, LOW);
    digitalWrite(solenoidRelayPin, LOW);
    deactivateBuzzer();
    lcd.clear();
  }

  delay(100);  // Small delay to prevent tight polling
}
