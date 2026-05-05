#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===== LCD =====
LiquidCrystal_I2C lcd(0x27, 16, 2); 
unsigned long lastLcdUpdate = 0;
const unsigned long LCD_INTERVAL = 500; 

// ===== RFID =====
#define SS_1 53
#define RST_1 22
#define SS_2 48
#define RST_2 23 
MFRC522 rfid1(SS_1, RST_1);
MFRC522 rfid2(SS_2, RST_2);

// ===== SERVOS =====
Servo servoA;
Servo servoB;
#define SERVO_A_PIN 11
#define SERVO_B_PIN 12

#define SERVO_A_CLOSED 90    
#define SERVO_A_OPEN   0   
#define SERVO_B_CLOSED 0   
#define SERVO_B_OPEN   90    

// ===== SENSORS =====
#define DHT_A    24
#define DHT_B    26
#define DHTTYPE  DHT11
DHT dhtA(DHT_A, DHTTYPE);
DHT dhtB(DHT_B, DHTTYPE);
#define MQ_A A0
#define MQ_B A1

// ===== PINS =====
#define BUZZER 8   // ACTIVE LOW
#define LED    6
#define BUTTON 5

// ===== THRESHOLDS =====
#define TEMP_LIMIT  55    
#define GAS_LIMIT_A 550   
#define GAS_LIMIT_B 1000  

// ===== AUTHORIZED UIDS & PERMISSIONS =====
// Card 1: Zone A | Card 2: Zone B | Card 3 & 4: Both
const byte CARD_Z1_ONLY[4] = {0x1A, 0xA7, 0x0F, 0x06};
const byte CARD_Z2_ONLY[4] = {0x40, 0x2B, 0xD6, 0x61};
const byte CARD_BOTH_1[4]  = {0x7D, 0x64, 0xFA, 0x05};
const byte CARD_BOTH_2[4]  = {0x50, 0x3E, 0x8C, 0x61};

// ===== STATE VARIABLES =====
unsigned long lastBeep = 0;
bool buzzerActive = false;
bool overrideActive = false; 
bool lastButtonState = HIGH;

void openA()  { servoA.write(SERVO_A_OPEN); }
void closeA() { servoA.write(SERVO_A_CLOSED); }
void openB()  { servoB.write(SERVO_B_OPEN); }
void closeB() { servoB.write(SERVO_B_CLOSED); }

void lcdPrint(String l1, String l2 = "") {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(l1);
  lcd.setCursor(0, 1); lcd.print(l2);
}

// Check if UID matches a specific card
bool compareUID(byte *scanned, const byte *authorized) {
  for (int i = 0; i < 4; i++) {
    if (scanned[i] != authorized[i]) return false;
  }
  return true;
}

// Permission Logic: Zone 1 = A, Zone 2 = B
bool checkAccess(MFRC522 &rfid, int targetZone) {
  byte *uid = rfid.uid.uidByte;
  if (targetZone == 1) {
    return (compareUID(uid, CARD_Z1_ONLY) || compareUID(uid, CARD_BOTH_1) || compareUID(uid, CARD_BOTH_2));
  } else if (targetZone == 2) {
    return (compareUID(uid, CARD_Z2_ONLY) || compareUID(uid, CARD_BOTH_1) || compareUID(uid, CARD_BOTH_2));
  }
  return false;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  lcd.init();
  lcd.backlight();
  
  SPI.begin();
  rfid1.PCD_Init();
  rfid2.PCD_Init();

  servoA.attach(SERVO_A_PIN);
  servoB.attach(SERVO_B_PIN);
  
  overrideActive = false; 
  closeA(); 
  closeB(); 

  dhtA.begin();
  dhtB.begin();
  
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH); // Silent
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  lastButtonState = digitalRead(BUTTON); 
  lcdPrint("System Ready");
}

void loop() {
  float t1 = dhtA.readTemperature();
  float t2 = dhtB.readTemperature();
  int g1 = analogRead(MQ_A);
  int g2 = analogRead(MQ_B);

  bool unsafe = (t1 > TEMP_LIMIT || t2 > TEMP_LIMIT || g1 > GAS_LIMIT_A || g2 > GAS_LIMIT_B);
  digitalWrite(LED, unsafe);

  // BUZZER LOGIC: Continuous beep for Override OR unsafe pulse
  if (overrideActive) {
    if (millis() - lastBeep > 200) { // Fast continuous beep for override
      buzzerActive = !buzzerActive;
      digitalWrite(BUZZER, buzzerActive ? LOW : HIGH);
      lastBeep = millis();
    }
  } else if (unsafe) {
    if (millis() - lastBeep > 400) { // Slower beep for hazard alert
      buzzerActive = !buzzerActive;
      digitalWrite(BUZZER, buzzerActive ? LOW : HIGH);
      lastBeep = millis();
    }
  } else {
    digitalWrite(BUZZER, HIGH); // Silent
  }

  // LCD Update
  if (millis() - lastLcdUpdate > LCD_INTERVAL) {
    if (overrideActive) {
      lcdPrint("OVERRIDE ACTIVE", "BUZZER BEEPING");
    } else if (unsafe) {
      lcdPrint("!! ALERT !!", "UNSAFE STATE");
    } else {
      lcdPrint("A:" + String((int)t1) + "C G:" + String(g1), 
               "B:" + String((int)t2) + "C G:" + String(g2));
    }
    lastLcdUpdate = millis();
  }

  // BUTTON OVERRIDE
  bool currentButton = digitalRead(BUTTON);
  if (lastButtonState == HIGH && currentButton == LOW) {
    delay(50);
    if (digitalRead(BUTTON) == LOW) {
      overrideActive = !overrideActive;
      if (overrideActive) {
        openA(); openB();
      } else {
        closeA(); closeB();
      }
    }
  }
  lastButtonState = currentButton;

  // ZONE A RFID
  digitalWrite(SS_2, HIGH); digitalWrite(SS_1, LOW);
  if (rfid1.PICC_IsNewCardPresent() && rfid1.PICC_ReadCardSerial()) {
    if (!unsafe && !overrideActive && checkAccess(rfid1, 1)) {
       lcdPrint("Zone A", "Granted");
       openA(); delay(3000); closeA();
    } else {
       lcdPrint("Zone A", "Access Denied");
    }
    rfid1.PICC_HaltA(); rfid1.PCD_StopCrypto1();
  }

  // ZONE B RFID
  digitalWrite(SS_1, HIGH); digitalWrite(SS_2, LOW);
  if (rfid2.PICC_IsNewCardPresent() && rfid2.PICC_ReadCardSerial()) {
    if (!unsafe && !overrideActive && checkAccess(rfid2, 2)) {
       lcdPrint("Zone B", "Granted");
       openB(); delay(3000); closeB();
    } else {
       lcdPrint("Zone B", "Access Denied");
    }
    rfid2.PICC_HaltA(); rfid2.PCD_StopCrypto1();
  }
}
