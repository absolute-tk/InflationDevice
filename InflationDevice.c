#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD I2C address (commonly 0x27 or 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16x2 display

// Pin definitions
const int potPin = A0; // Potentiometer input
const int pressurePin = A1; // Pressure sensor input
const int pumpRelayPin = 2; // Relay for pump
const int solenoidRelayPin = 3; // Relay for solenoid
const int greenLEDPin = 9; // Green LED
const int redLEDPin = 10; // Red LED
const int buzzerPin = 11; // Buzzer
const int switchPin = 8; // ON/OFF switch

// Variables
float desiredPressure = 0.0; // Pressure set by potentiometer (in kPa)
float currentPressure = 0.0; // Pressure read from the sensor (in kPa)
float pressureTolerance = 0.5; // Allowable range ±0.5 kPa
int lastPotValue = -1; // Store the last potentiometer value
bool pumpStopped = false; // Track pump state
bool solenoidActivated = false; // Track solenoid state
bool systemInitialized = false; // Ensure system does not activate on startup

void setup() {
  // Initialize pins
  pinMode(pumpRelayPin, OUTPUT);
  pinMode(solenoidRelayPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  displayStartupMessage();
}

void loop() {
  // Check if system is turned OFF
  if (digitalRead(switchPin) == HIGH) {
    resetSystem(); // Ensure everything stops
    systemInitialized = false; // Reset initialization flag
    return; // Wait until the switch is turned back ON
  }

  // Read desired pressure from potentiometer
  int potValue = analogRead(potPin);
  desiredPressure = map(potValue, 0, 1023, 0, 5000) / 100.0; // Map to 0.0 - 50.0 kPa

  // Skip system operation if the potentiometer is not set
  if (desiredPressure <= 0) {
    resetSystem();
    lcd.setCursor(0, 0);
    lcd.print("Set pressure");
    lcd.setCursor(0, 1);
    lcd.print("to start...");
    delay(500); // Short delay to allow display to update
    return;
  }

  // Initialize the system only when the switch is first turned ON
  if (!systemInitialized) {
    resetSystem(); // Reset all relays and indicators
    lastPotValue = potValue; // Initialize last potentiometer value
    systemInitialized = true; // Mark system as initialized
  }

  // Detect if the potentiometer value has changed
  if (potValue != lastPotValue) {
    resetSystem(); // Reset system when knob is adjusted
    lastPotValue = potValue; // Store the new potentiometer value
    pumpStopped = false; // Allow the pump to restart
    solenoidActivated = false; // Reset solenoid state
  }

  // Read current pressure from sensor
  int sensorValue = analogRead(pressurePin);
  currentPressure = map(sensorValue, 0, 1023, 0, 40); // Assuming sensor gives 0-40 kPa

  // Display pressure values on LCD
  displayPressureValues();

  // Calculate the lower and upper bounds for acceptable pressure
  float lowerBound = desiredPressure - pressureTolerance;
  float upperBound = desiredPressure + pressureTolerance;

  // Control pump and solenoid based on pressure
  if (!pumpStopped) {
    if (currentPressure < lowerBound) {
      activatePump();
      deactivateSolenoid(); // Ensure solenoid is off
    } else if (currentPressure >= lowerBound && currentPressure <= upperBound) {
      deactivatePump(); // Stop pump when pressure is within range
      pumpStopped = true; // Mark pump as stopped
      if (!solenoidActivated) {
        delay(3000); // Wait 3 seconds before activating solenoid
        activateSolenoid();
      }
    } else if (currentPressure > upperBound) {
      deactivatePump(); // Stop pump when overpressure occurs
      deactivateSolenoid(); // Ensure solenoid is off
      pumpStopped = true;
    }
  }

  // Control LEDs
  if (currentPressure >= lowerBound && currentPressure <= upperBound) {
    digitalWrite(greenLEDPin, HIGH); // Turn green LED on
    digitalWrite(redLEDPin, LOW); // Turn red LED off
  } else {
    digitalWrite(greenLEDPin, LOW); // Turn green LED off
    digitalWrite(redLEDPin, HIGH); // Turn red LED on
  }

  delay(100); // Short delay for stability
}

// Display startup message
void displayStartupMessage() {
  lcd.clear();
  lcd.print("    BME SWU");
  delay(3000);
  lcd.clear();
}

// Reset the system to initial state
void resetSystem() {
  digitalWrite(pumpRelayPin, LOW);
  digitalWrite(solenoidRelayPin, LOW);
  digitalWrite(greenLEDPin, LOW);
  digitalWrite(redLEDPin, LOW);
  digitalWrite(buzzerPin, LOW); // Ensure buzzer is off
  lcd.clear();
  lcd.print("System OFF");
}

// Display pressure values on LCD
void displayPressureValues() {
  lcd.setCursor(0, 0);
  lcd.print("Set: ");
  lcd.print(desiredPressure, 1); // Display with 1 decimal place
  lcd.print(" kPa");

  lcd.setCursor(0, 1);
  lcd.print("Cur: ");
  lcd.print(currentPressure, 1); // Display with 1 decimal place
  lcd.print(" kPa");
}

// Activate the pump to increase pressure
void activatePump() {
  digitalWrite(pumpRelayPin, HIGH); // Turn pump on
}

// Deactivate the pump
void deactivatePump() {
  digitalWrite(pumpRelayPin, LOW); // Turn pump off
}

// Activate the solenoid
void activateSolenoid() {
  digitalWrite(solenoidRelayPin, HIGH); // Turn solenoid on
  solenoidActivated = true;
}

// Deactivate the solenoid
void deactivateSolenoid() {
  digitalWrite(solenoidRelayPin, LOW); // Turn solenoid off
  solenoidActivated = false;
}
