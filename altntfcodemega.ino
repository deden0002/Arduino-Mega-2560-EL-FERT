#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ACS712.h>

const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

byte rowPins[ROWS] = {31, 33, 35, 37};
byte colPins[COLS] = {39, 41, 43};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

const int pwmPin = 4; // PWM pin
int dutyCycle = 0;
String inputString = "";

// #define SENSOR_PIN A3
const int SENSOR_PIN = A3;
#define RELAY_PIN 9
#define BUZZER_PIN 10

LiquidCrystal_I2C lcd(0x27, 16, 2);

int moistureThreshold = 60;

#define ACS_PIN A1
#define NUM_READINGS 10

ACS712 sensorACS(ACS712_30A, ACS_PIN);

int analogPin = A2;
float Vmodul = 0.0;
float hasil = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;
int value = 0;
float tegangandc;

#define trigPin 5
#define echoPin 6
#define jarak 30
int tinggi;
float arus;

unsigned long lastMoistureCheckTime = 0;
unsigned long lastSensorReadingTime = 0;
unsigned long lastDisplayUpdateTime = 0;
int currentMoistureLevel = 0;

// Variables for DC voltage sensor
float adc_voltage = 0.0;
float in_voltage = 0.0;
float ref_voltage = 5.0;
int adc_value = 0;

void setup() {
  pinMode(pwmPin, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(analogPin, INPUT);
  Serial.begin(115200);
  Serial3.begin(115200);
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("EL-FERT");
  lcd.setCursor(0, 1);
  lcd.print("TA2324.01.002");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Masukkan");
  lcd.setCursor(0, 1);
  lcd.print("Nilai Tegangan:");
  Serial.println("Program dimulai.");
}

void loop() {
  checkKeypad();
  checkMoistureLevel();
  checkUltrasonicDistance();
  readSensorValues();
  receivePWMValueFromESP8266();
}

void checkKeypad() {
  char key = keypad.getKey();
  
  if (key) {
    if (key >= '0' && key <= '9') {
      inputString += key;
      Serial.print("Masukkan angka: ");
      Serial.println(inputString);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Input: ");
      lcd.print(inputString);
      
    } else if (key == '#') {
      if (inputString.length() > 0) {
        int inputVal = inputString.toInt();
        dutyCycle = mapKeypadInputToPWM(inputVal);
        analogWrite(pwmPin, dutyCycle);
        Serial.print("Duty Cycle baru: ");
        Serial.println(dutyCycle);

        sendPWMValueToESP8266(dutyCycle);

        // Display the entered value on the LCD
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Nilai Masukkan:");
        lcd.setCursor(0, 1);
        lcd.print(inputVal);
      }
      inputString = "";

    } else if (key == '*') {
      inputString = "";
      Serial.println("Input direset");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Input direset");
      delay(1000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Masukkan");
      lcd.setCursor(0, 1);
      lcd.print("Nilai Tegangan:");
    }
  }
}

int mapKeypadInputToPWM(int input) {
  switch (input) {
    case 1: return 11;
    case 2: return 23;
    case 3: return 35;
    case 4: return 48;
    case 5: return 61;
    case 6: return 73;
    case 7: return 85;
    case 8: return 97;
    case 9: return 109;
    case 10: return 122;
    case 11: return 134;
    case 12: return 146;
    case 13: return 158;
    case 14: return 170;
    case 15: return 182;
    case 16: return 194;
    case 17: return 206;
    case 18: return 218;
    case 19: return 230;
    case 20: return 242;
    case 21: return 254;
    case 22: return 256;
    case 23: return 256;
    case 24: return 256;
    default: return 0;
  }
}

void checkMoistureLevel() {
  unsigned long currentTime = millis();
  if (currentTime - lastMoistureCheckTime >= 1000) {
    lastMoistureCheckTime = currentTime;
    
    currentMoistureLevel = map(analogRead(SENSOR_PIN), 0, 1023, 100, 0);
  
    Serial.print("Kelembaban Tanah: ");
    Serial.print(currentMoistureLevel);
    Serial.println(" %");

    if (currentMoistureLevel < moistureThreshold) {
      Serial.println("Penyiraman dinyalakan");
      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);
      delay(1000);  // Hanya untuk bunyi buzzer sementara
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
    } else {
      Serial.println("Penyiraman dimatikan");
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}

void checkUltrasonicDistance() {
  unsigned long currentTime = millis();
  if (currentTime - lastSensorReadingTime >= 1000) {
    lastSensorReadingTime = currentTime;
    
    long duration, gape;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    gape = (duration / 2) / 29.0;
    tinggi = jarak - gape;
    
    Serial.print("Tinggi: ");
    Serial.print(tinggi);
    Serial.println(" CM");
  }
}

void readSensorValues() {
  unsigned long currentTime = millis();
  if (currentTime - lastSensorReadingTime >= 1000) {
    lastSensorReadingTime = currentTime;
    
    // Baca nilai dari sensor tegangan DC
    adc_value = analogRead(analogPin);
    adc_voltage  = (adc_value * ref_voltage) / 1024.0;
    in_voltage = adc_voltage / (R2 / (R1 + R2));
    tegangandc = in_voltage;
    Serial.print("Tegangan DC: ");
    Serial.print(tegangandc);
    Serial.println(" V");

    // Baca nilai arus dari sensor ACS712
    float totalCurrent = 0;
    for (int i = 0; i < NUM_READINGS; i++) {
      totalCurrent += sensorACS.getCurrentDC();
      delay(10);
    }
    float averagedCurrent = totalCurrent / NUM_READINGS;
    arus = averagedCurrent;
    Serial.print("Arus: ");
    Serial.print(arus);
    Serial.println(" mA");

    // Buat pesan dan kirim ke ESP8266
    String message = String(currentMoistureLevel) + ";" + String(tegangandc) + ";" + String(arus) + ";" + String(tinggi) + ";" + String(dutyCycle);
    Serial3.println(message);

    if (Serial3.available()) {
      String receivedMessage = Serial3.readStringUntil('\n');
      Serial.print("Pesan diterima dari ESP8266: ");
      Serial.println(receivedMessage);
    }
  }
}

void sendPWMValueToESP8266(int pwmValue) {
  String message = String("PWM:") + String(pwmValue);
  Serial3.println(message);
}

void receivePWMValueFromESP8266() {
  if (Serial3.available()) {
    String receivedMessage = Serial3.readStringUntil('\n');
    if (receivedMessage.startsWith("PWM:")) {
      int pwmValue = receivedMessage.substring(4).toInt();
      dutyCycle = pwmValue;
      analogWrite(pwmPin, dutyCycle);
      Serial.print("PWM value updated from ESP8266: ");
      Serial.println(dutyCycle);
    }
  }
}
