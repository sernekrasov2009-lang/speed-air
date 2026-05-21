#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Preferences.h>
#include <math.h>
#include "esp_timer.h"

// ===== ПИНЫ =====
#define TFT_CS     5
#define TFT_RST    4
#define TFT_DC     14

#define VALVE_PIN  27
#define SOL_PIN    26
#define TRIG_PIN   13 // Тактовая кнопка (замыкает на GND)

#define ENC_S1     32
#define ENC_S2     33
#define ENC_KEY    25

#define THERM_PIN  34
#define BAT_PIN    35

#define IR1_PIN    19
#define IR2_PIN    18 // I2C pin 21 свободен

#define ENC_STEPS 4

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Preferences prefs;

// ===== НАСТРОЙКИ =====
float sensorDist = 40.0;
float bulletMass = 0.25;
int valveOpenTime = 15;
int solPushTime = 50;
int solPullTime = 50;
int solSealTime = 10;
int burstCount = 3;
int burstDelay = 50;
int fireMode = 0; // 0: Single, 1: Burst, 2: Auto
int maxTemp = 50;
float warnVolt = 3.5;

// ===== СОСТОЯНИЕ =====
float currentSpeed = 0.0;
float currentEnergy = 0.0;
float battVoltage = 0.0;
float currentTemp = 0.0;

volatile int64_t t1 = 0, t2 = 0;
volatile bool chronoReady = false;
volatile int encoderPos = 0;

bool triggerWasReleased = true;
int burstsRemaining = 0; 

// Для фильтров и UI
unsigned long lastUiUpdate = 0;
bool needUiUpdate = false;
bool isFirstRead = true;
bool lastOverheatState = false;

// Цвета
#define COL_BG      0x0000
#define COL_ACCENT  0x07FF
#define COL_TEXT    0xFFFF
#define COL_FRAME   0x3186
#define COL_WARN    0xF800

int uiState = 0; // 0: main, 1: menu, 2: edit
int menuIndex = 0;
int lastEnc = 0;

// ===== FSM =====
enum FireState {IDLE, VALVE_ON, SOL_ON, SOL_WAIT, SOL_SEAL, DELAY_STATE};
FireState fireState = IDLE;
unsigned long stateTime = 0;

const int menuCount = 12;
String menuItems[menuCount] = {
  "[ EXIT ]", "MODE", "MASS (g)", "VALVE (ms)",
  "SOL PUSH", "SOL RET", "SOL SEAL", 
  "BURST CNT", "BURST DLY", "DIST (mm)", "MAX TEMP", "LOW V"
};

// ===== ISR ПРЕРЫВАНИЯ =====
void IRAM_ATTR isrChrono1() { t1 = esp_timer_get_time(); }
void IRAM_ATTR isrChrono2() { if (t1 > 0) { t2 = esp_timer_get_time(); chronoReady = true; } }
void IRAM_ATTR isrEncoder() {
  static uint8_t old_AB = 0;
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  old_AB <<= 2; old_AB |= (digitalRead(ENC_S1) << 1) | digitalRead(ENC_S2);
  encoderPos += enc_states[(old_AB & 0x0f)];
  if(encoderPos > 10000) encoderPos = 10000;
  if(encoderPos < -10000) encoderPos = -10000;
}

// ===== SETUP =====
void setup() {
  pinMode(VALVE_PIN, OUTPUT); digitalWrite(VALVE_PIN, LOW);
  pinMode(SOL_PIN, OUTPUT); digitalWrite(SOL_PIN, LOW);
  
  pinMode(TRIG_PIN, INPUT_PULLUP);
  
  pinMode(IR1_PIN, INPUT_PULLDOWN);
  pinMode(IR2_PIN, INPUT_PULLDOWN);
  pinMode(ENC_S1, INPUT_PULLUP);
  pinMode(ENC_S2, INPUT_PULLUP);
  pinMode(ENC_KEY, INPUT_PULLUP);

  attachInterrupt(IR1_PIN, isrChrono1, RISING);
  attachInterrupt(IR2_PIN, isrChrono2, RISING);
  attachInterrupt(ENC_S1, isrEncoder, CHANGE);
  attachInterrupt(ENC_S2, isrEncoder, CHANGE);

  prefs.begin("gun_conf", false);
  sensorDist = prefs.getFloat("dist", 100.0);
  bulletMass = prefs.getFloat("mass", 0.25);
  valveOpenTime = prefs.getInt("valve", 15);
  solPushTime = prefs.getInt("push", 50);
  solPullTime = prefs.getInt("pull", 50);
  solSealTime = prefs.getInt("seal", 10);
  burstCount = prefs.getInt("bcnt", 3);
  burstDelay = prefs.getInt("bdly", 50);
  fireMode = prefs.getInt("mode", 0);
  maxTemp = prefs.getInt("mtemp", 50);
  warnVolt = prefs.getFloat("volt", 3.5);

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  drawMainScreen();
}

// ===== MAIN LOOP =====
void loop() {
  handleEncoder();
  
  if (uiState == 0) {
    processChrono();
    readSensors();
    handleTriggerAndFSM();
    
    // Обновление UI: по таймеру 200мс ИЛИ при новом выстреле
    if (millis() - lastUiUpdate > 200 || needUiUpdate) {
      updateMainData();
      lastUiUpdate = millis();
      needUiUpdate = false;
    }
  }
}
// ===== ХРОНОГРАФ =====
void processChrono() {
  if (chronoReady) {
    int64_t dt = t2 - t1;
    if (dt > 80 && dt < 500000) { 
      float t_sec = dt / 1000000.0;
      currentSpeed = (sensorDist / 1000.0) / t_sec;
      currentEnergy = 0.5 * (bulletMass / 1000.0) * currentSpeed * currentSpeed;
      needUiUpdate = true;
    }
    t1 = 0;
    chronoReady = false;
  }
}

// ===== СЕНСОРЫ =====
void readSensors() {
  float rawBatt = (analogRead(BAT_PIN) / 4095.0) * 3.3 * 2.0;
  
  float rawTemp = currentTemp;
  int rawADC = analogRead(THERM_PIN);
  if (rawADC > 0) {
    float r = 10000.0 / (4095.0 / (float)rawADC - 1.0);
    float t = log(r / 10000.0) / 3950.0 + 1.0 / (273.15 + 25.0);
    rawTemp = 1.0 / t - 273.15;
  }

  if (isFirstRead) {
    battVoltage = rawBatt;
    currentTemp = rawTemp;
    isFirstRead = false;
  } else {
    battVoltage = battVoltage * 0.9 + rawBatt * 0.1;
    currentTemp = currentTemp * 0.9 + rawTemp * 0.1;
  }
}

// ===== ЛОГИКА FSM =====
void startShot() {
  digitalWrite(VALVE_PIN, HIGH);
  fireState = VALVE_ON;
  stateTime = millis();
}

void handleTriggerAndFSM() {
  // Аварийная защита (Перегрев или зависание клапана > 2 сек)
  if (currentTemp > maxTemp || (fireState != IDLE && millis() - stateTime > 2000)) {
    fireState = IDLE;
    digitalWrite(VALVE_PIN, LOW);
    digitalWrite(SOL_PIN, LOW);
    burstsRemaining = 0;
    if (currentTemp > maxTemp) return; 
  }

  static unsigned long lastTrigTime = 0;
  bool isPressed = (digitalRead(TRIG_PIN) == LOW);
  
  // Антидребезг нажатия
  if (fireState == IDLE && isPressed && triggerWasReleased && (millis() - lastTrigTime > 50)) {
    lastTrigTime = millis();
    triggerWasReleased = false;

    if (fireMode == 0) {
      startShot();
    } 
    else if (fireMode == 1) {
      burstsRemaining = burstCount - 1;
      startShot();
    } 
    else if (fireMode == 2) {
      startShot();
    }
  }
  
  // Антидребезг отпускания
  if (!isPressed && !triggerWasReleased && (millis() - lastTrigTime > 50)) {
    lastTrigTime = millis();
    triggerWasReleased = true;
    if (fireMode == 2) burstsRemaining = 0;
  }

  // Автомат состояний
  switch (fireState) {
    case IDLE: 
      break;

    case VALVE_ON:
      if (millis() - stateTime >= valveOpenTime) {
        digitalWrite(VALVE_PIN, LOW);
        digitalWrite(SOL_PIN, HIGH);
        fireState = SOL_ON;
        stateTime = millis();
      }
      break;

    case SOL_ON:
      if (millis() - stateTime >= solPushTime) {
        digitalWrite(SOL_PIN, LOW);
        fireState = SOL_WAIT;
        stateTime = millis();
      }
      break;

    case SOL_WAIT:
      if (millis() - stateTime >= solPullTime) {
        fireState = SOL_SEAL;
        stateTime = millis();
      }
      break;

    case SOL_SEAL:
      if (millis() - stateTime >= solSealTime) {
        if (burstsRemaining > 0) {
          burstsRemaining--;
          fireState = DELAY_STATE;
          stateTime = millis();
        } else if (fireMode == 2 && isPressed) {
          fireState = DELAY_STATE;
          stateTime = millis();
        } else {
          fireState = IDLE;
        }
      }
      break;

    case DELAY_STATE:
      if (millis() - stateTime >= burstDelay) {
        startShot();
      }
      break;
  }
}

// ===== UI: MAIN SCREEN =====
void drawMainScreen() {
  tft.fillScreen(COL_BG);
  tft.fillRect(0, 0, 160, 14, 0x000F); 
  tft.setTextColor(COL_ACCENT); tft.setTextSize(1);
  tft.setCursor(5, 3); tft.print("SMART SYSTEM");
  
  tft.drawRoundRect(5, 20, 150, 45, 6, COL_FRAME);
  tft.setCursor(12, 26); tft.setTextColor(0x7BEF); tft.print("VELOCITY m/s");
  
  tft.drawRoundRect(5, 70, 150, 35, 6, COL_FRAME);
  tft.setCursor(12, 76); tft.setTextColor(0x07E0); tft.print("ENERGY Joule");
  
  lastOverheatState = false; // Сброс стейта для корректной отрисовки
  updateMainData();
}

void updateMainData() {
  tft.setTextSize(3); tft.setCursor(15, 38);
  tft.setTextColor(COL_TEXT, COL_BG); tft.print(currentSpeed, 1); tft.print("  ");
  tft.setTextSize(2); tft.setCursor(15, 84);
  tft.setTextColor(COL_TEXT, COL_BG); tft.print(currentEnergy, 2); tft.print("  ");

  bool currentOverheatState = (currentTemp > maxTemp);
  
  if (currentOverheatState != lastOverheatState) {
    tft.fillRect(5, 110, 150, 16, COL_BG); 
    if (currentOverheatState) {
      tft.fillRoundRect(5, 110, 150, 16, 4, COL_WARN);
      tft.setTextSize(1); tft.setTextColor(COL_TEXT); 
      tft.setCursor(20, 114); tft.print("!!! OVERHEAT !!!"); 
    }
    lastOverheatState = currentOverheatState;
  }

  if (!currentOverheatState) {
    tft.setTextSize(1); tft.setCursor(10, 115);
    tft.setTextColor(0x7BEF, COL_BG); tft.print("MODE: ");
    tft.print((fireMode==0)?"SINGLE ":(fireMode==1)?"BURST  ":"AUTO   ");
  }

  tft.setCursor(95, 3);
  bool blink = (millis() / 500) % 2 == 0;
  if (battVoltage < warnVolt && blink) tft.setTextColor(COL_WARN, 0x000F);
  else tft.setTextColor(ST7735_YELLOW, 0x000F);
  
  tft.print(battVoltage, 1); tft.print("V ");
  
  if (currentTemp > maxTemp - 5) tft.setTextColor(ST7735_ORANGE, 0x000F);
  else tft.setTextColor(ST7735_GREEN, 0x000F);
  tft.print((int)currentTemp); tft.print("C  ");
}

// ===== UI: MENU =====
void drawMenu() {
  tft.fillRect(0, 14, 160, 114, COL_BG); 
  tft.setTextSize(1);

  int startItem = 0;
  if (menuIndex >= 6) startItem = menuIndex - 5;

  for (int i = 0; i < 6; i++) {
    int idx = startItem + i;
    if (idx >= menuCount) break;

    int y = 20 + (i * 16);  
    if (idx == menuIndex) {  
      tft.fillRect(5, y - 4, 150, 14, (uiState == 2) ? COL_WARN : 0x2104);  
      tft.setTextColor(COL_TEXT);  
    } else {  
      tft.setTextColor(0x7BEF);  
    }  

    tft.setCursor(10, y); tft.print(menuItems[idx]);  
      
    tft.setCursor(100, y);  
    switch(idx) {
      case 1: tft.print((fireMode==0)?"SNG":(fireMode==1)?"BST":"AUTO"); break;
      case 2: tft.print(bulletMass, 2); break;
      case 3: tft.print(valveOpenTime); break;
      case 4: tft.print(solPushTime); break;
      case 5: tft.print(solPullTime); break;
      case 6: tft.print(solSealTime); break;
      case 7: tft.print(burstCount); break;
      case 8: tft.print(burstDelay); break;
      case 9: tft.print(sensorDist, 0); break;
      case 10: tft.print(maxTemp); break;
      case 11: tft.print(warnVolt, 1); break;
    }
  }
}

// ===== ENCODER LOGIC =====
void handleEncoder() {
  int cp = encoderPos / ENC_STEPS;
  int d = cp - lastEnc;

  if (d != 0) {
    lastEnc = cp;
    if (uiState == 1) { 
      menuIndex = constrain(menuIndex + d, 0, menuCount - 1); 
      drawMenu(); 
    }
    else if (uiState == 2) {
      switch(menuIndex){
        case 1: fireMode = constrain(fireMode+d, 0, 2); break;
        case 2: bulletMass = constrain(bulletMass+d*0.01, 0.1, 5.0); break;
        case 3: valveOpenTime = constrain(valveOpenTime+d, 1, 200); break;
        case 4: solPushTime = constrain(solPushTime+d, 1, 500); break;
        case 5: solPullTime = constrain(solPullTime+d, 1, 500); break;
        case 6: solSealTime = constrain(solSealTime+d, 1, 200); break;
        case 7: burstCount = constrain(burstCount+d, 2, 50); break;
        case 8: burstDelay = constrain(burstDelay+d*5, 10, 1000); break;
        case 9: sensorDist = constrain(sensorDist+d*5, 10, 500); break;
        case 10: maxTemp = constrain(maxTemp+d, 30, 100); break;
        case 11: warnVolt = constrain(warnVolt+d*0.1, 3.0, 4.2); break;
      }
      drawMenu();
    }
  }

  static unsigned long lastBtn = 0;
  if (digitalRead(ENC_KEY) == LOW && millis() - lastBtn > 300) {
    lastBtn = millis();

    if (uiState == 0) { uiState = 1; drawMenu(); }
    else if (uiState == 1) {
      if (menuIndex == 0) { uiState = 0; drawMainScreen(); }
      else { uiState = 2; drawMenu(); }
    }
    else if (uiState == 2) {
      saveSettings();
      uiState = 1;
      drawMenu();
    }
  }
}
// ===== MEMORY =====
void saveSettings() {
  prefs.putFloat("dist", sensorDist);
  prefs.putFloat("mass", bulletMass);
  prefs.putInt("valve", valveOpenTime);
  prefs.putInt("push", solPushTime);
  prefs.putInt("pull", solPullTime);
  prefs.putInt("seal", solSealTime);
  prefs.putInt("bcnt", burstCount);
  prefs.putInt("bdly", burstDelay);
  prefs.putInt("mode", fireMode);
  prefs.putInt("mtemp", maxTemp);
  prefs.putFloat("volt", warnVolt);
}