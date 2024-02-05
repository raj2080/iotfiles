#include <Servo.h>
#include <LiquidCrystal_I2C.h>

Servo servoMotor;
Servo gateServo;       // Create a servo object to control the servo motor
const int gateIRSensor = 12; // IR sensor for gate
const int slot1SensorPin = 9;
const int slot2SensorPin = 8;
const int slot3SensorPin = 7;
const int slot4SensorPin = 6;
const int IR_SENSOR_PIN = 2;
const int SERVO_PIN = 3;

LiquidCrystal_I2C lcd(0x27, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

int totalSlots = 4;  // Total number of parking slots

// Array to store the state of each parking slot sensor
int slotSensorState[4] = {1, 1, 1, 1};  // Initial state: all slots empty

bool isGateOpening = false;  // Flag to prevent continuous gate opening

void setup() {
  Serial.begin(9600);
  pinMode(gateIRSensor, INPUT);
  pinMode(slot1SensorPin, INPUT);
  pinMode(slot2SensorPin, INPUT);
  pinMode(slot3SensorPin, INPUT);
  pinMode(slot4SensorPin, INPUT);
  gateServo.attach(11);  // Attach the servo to digital pin 11

  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(SERVO_PIN, OUTPUT);

  servoMotor.attach(SERVO_PIN);


  lcd.init();
  lcd.clear();
  lcd.backlight();      // Make sure the backlight is on
  
  // Print a message on both lines of the LCD.
  lcd.setCursor(4, 0);   // Set cursor to character 2 on line 0
  lcd.print("ShadanPark ");
  
  lcd.setCursor(2, 1);   // Move cursor to character 2 on line 1
  lcd.print("Empty-Slots: ");

  // Print the initial number of empty slots
  lcd.print(totalSlots);
}

void loop() {
  int gateSensorValue = digitalRead(gateIRSensor);
  int slot1Value = digitalRead(slot1SensorPin);
  int slot2Value = digitalRead(slot2SensorPin);
  int slot3Value = digitalRead(slot3SensorPin);
  int slot4Value = digitalRead(slot4SensorPin);

  // Check for changes in the state of each slot sensor
  checkSlotState(0, slot1Value);
  checkSlotState(1, slot2Value);
  checkSlotState(2, slot3Value);
  checkSlotState(3, slot4Value);

  if (!isGateOpening && (totalSlots > 0) && (slotSensorState[0] == 1 || slotSensorState[1] == 1 || slotSensorState[2] == 1 || slotSensorState[3] == 1) && gateSensorValue == 0) {
    // Gate should be open if an object is detected and there is an available parking slot
    controlGate();
    Serial.println("Gate Opened!");
  } else {
    // Gate should be closed
    gateServo.write(80);  // Adjust the angle as needed to close the gate
    Serial.println("Gate Closed!");
  }

  if (digitalRead(IR_SENSOR_PIN) == HIGH) {
    // If IR sensor detects an object
    servoMotor.write(90);  // Rotate servo to 90 degrees (or your desired position)
    delay(1000);           // Wait for 1 second (adjust as needed)
  } else {
    servoMotor.write(0);   // Rotate servo to initial position (or any other position)
  }

  // Print parking slot status
  printSlotStatus("Slot 1", slotSensorState[0]);
  printSlotStatus("Slot 2", slotSensorState[1]);
  printSlotStatus("Slot 3", slotSensorState[2]);
  printSlotStatus("Slot 4", slotSensorState[3]);

  delay(500);  // Delay between readings
}

void controlGate() {
  isGateOpening = true;  // Set the flag to indicate gate opening
  // Open the gate
  gateServo.write(0);  // Adjust the angle as needed to open the gate
  
  // Check for objects while opening the gate
  while (digitalRead(gateIRSensor) == 0 ) {
    // Continue opening the gate until an object is detected
    delay(50);  // Adjust the delay based on your needs
  }

  // Close the gate
  gateServo.write(0);  // Adjust the angle as needed to close the gate
  delay(3000);  // Add a delay before the next check (adjust as needed)
  isGateOpening = false;  // Reset the flag after gate is closed
}

void printSlotStatus(const char* slotName, int sensorValue) {
  if (sensorValue == 0) {
    Serial.print(slotName);
    Serial.println(": Occupied");
  } else {
    Serial.print(slotName);
    Serial.println(": Empty");
  }
}

void checkSlotState(int slotIndex, int sensorValue) {
  // Check if the sensor state has changed
  if (sensorValue != slotSensorState[slotIndex]) {
    // Update the sensor state
    slotSensorState[slotIndex] = sensorValue;

    // Update the totalSlots based on sensor state
    if (sensorValue == 0) {
      // Car parked, reduce totalSlots
      totalSlots--;
      if (totalSlots < 0) {
        totalSlots = 0;  // Ensure totalSlots doesn't go below 0
      }
      updateLCD();
    } else {
      // Car removed, increase totalSlots
      totalSlots++;
      if (totalSlots > 4) {
        totalSlots = 4;  // Ensure totalSlots doesn't exceed 4
      }
      updateLCD();
    }
  }
}

void updateLCD() {
  // Update the LCD with the current number of empty slots
  lcd.setCursor(13, 1);
  lcd.print("    ");  // Clear the previous number
  lcd.setCursor(13, 1);
  lcd.print(totalSlots);
}