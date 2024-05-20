#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define the I2C address and LCD size
const int I2C_ADDRESS = 0x27;
const int LCD_COLUMNS = 16;
const int LCD_ROWS = 2;

// Define the analog pins for the voltage divider circuits
const int VOLTAGE_DIVIDER_PIN_BATTERY1 = A0;
const int VOLTAGE_DIVIDER_PIN_BATTERY2 = A1;
const int LDR_PIN = A3;

// LDR and relay module pins
const int RELAY_PIN_BATTERY1 = 8;
const int RELAY_PIN_BATTERY2 = 9;
const int RELAY_PIN = 7;

// LCD Display
LiquidCrystal_I2C lcd(I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);

void setup() {
  // Setup Serial Monitor
  Serial.begin(9600);
  Serial.println("Battery Monitoring & Light Control");

  // Initialize LCD Display
  lcd.init(); // Initialize the LCD
  lcd.backlight(); // Turn on backlight

  // Display "Power On" message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Power On");
  delay(2000); // Display "Power On" message for 2 seconds

  // Initialize relay pins
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Initially turn off relay

  pinMode(RELAY_PIN_BATTERY1, OUTPUT);
  digitalWrite(RELAY_PIN_BATTERY1, LOW); // Initially turn off Battery 1 relay

  pinMode(RELAY_PIN_BATTERY2, OUTPUT);
  digitalWrite(RELAY_PIN_BATTERY2, LOW); // Initially turn off Battery 2 relay

  // Setup LDR and Relay pin
  pinMode(LDR_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
}

void loop() {

    // Read light level from LDR
  int lightLevel = analogRead(LDR_PIN);

  // Adjust the threshold value according to your environment
  int threshold = 500; // Adjust this value based on ambient light conditions

  if (lightLevel < threshold) {
    // If light level is below threshold, turn on relay module
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    // If light level is above threshold, turn off relay module
    digitalWrite(RELAY_PIN, LOW);
  }
  delay(500);
  
  // Read voltage from voltage divider for Battery 1
  int adcValueBattery1 = analogRead(VOLTAGE_DIVIDER_PIN_BATTERY1);
  float batteryPercentageBattery1 = (adcValueBattery1 / 1023.0) * 100.0;

  // Read voltage from voltage divider for Battery 2
  int adcValueBattery2 = analogRead(VOLTAGE_DIVIDER_PIN_BATTERY2);
  float batteryPercentageBattery2 = (adcValueBattery2 / 1023.0) * 100.0;

  // Control Battery 1 relay based on battery percentage
  if (batteryPercentageBattery1 < 20.0 && batteryPercentageBattery2 < 20.0) {
    digitalWrite(RELAY_PIN_BATTERY1, LOW); // Turn off Battery 1 relay
    digitalWrite(RELAY_PIN_BATTERY2, LOW); // Turn off Battery 2 relay
  } else if (batteryPercentageBattery1 < 20.0 && batteryPercentageBattery2 > 20.0) {
    digitalWrite(RELAY_PIN_BATTERY1, LOW); // Turn off Battery 1 relay
    digitalWrite(RELAY_PIN_BATTERY2, HIGH); // Turn on Battery 2 relay
  } else if (batteryPercentageBattery1 > 20.0 && batteryPercentageBattery2 < 20.0) {
    digitalWrite(RELAY_PIN_BATTERY1, HIGH); // Turn on Battery 1 relay
    digitalWrite(RELAY_PIN_BATTERY2, LOW); // Turn off Battery 2 relay
  } else {
    digitalWrite(RELAY_PIN_BATTERY1, HIGH); // Turn on Battery 1 relay
    digitalWrite(RELAY_PIN_BATTERY2, LOW); // Turn off Battery 2 relay
  }

  // Display battery level percentages and other messages on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Battery 1: ");
  lcd.print(batteryPercentageBattery1);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("Battery 2: ");
  lcd.print(batteryPercentageBattery2);
  lcd.print("%");

  // Short delay before next message
  delay(4000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Batch 5");
  lcd.setCursor(0, 1);
  lcd.print("2020-2024 batch");

  // Short delay before next message
  delay(4000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Jettison, Manoj");
  lcd.setCursor(0, 1);
  lcd.print("Libin, Siva");

  // Short delay before next message
  delay(4000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("JEMLS");

  // Short delay before next loop
  delay(4000);
}
