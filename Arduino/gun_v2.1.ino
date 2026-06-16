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
#define TRIG_PIN   13
#define ENC_S1     32
#define ENC_S2     33
#define ENC_KEY    25
#define THERM_PIN  34
#define BAT_PIN    35
#define IR1_PIN    19
#define IR2_PIN    21   

// НОВЫЕ ПИНЫ ДЛЯ ШАГОВИКА (Драйвер ULN2003)
#define STEP_IN1   16
#define STEP_IN2   17
#define STEP_IN3   18
#define STEP_IN4   22

#define ENC_STEPS 4

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Preferences prefs;

// ===== НАСТРОЙКИ =====
float sensorDist    = 40.0;
float bulletMass    = 0.25;
int   valveOpenTime = 15;
int   valveDelay    = 20;   
int   solPushTime   = 50;
int   solPullTime   = 50;
int   solSealTime   = 10;
int   burstCount    = 3;
int   burstDelay    = 50;
int   fireMode      = 0;    
int   fireOrder     = 0;    
int   maxTemp       = 50;
float warnVolt      = 3.5;
int   stepsPerBall  = 128;  // НОВОЕ: Количество шагов для подачи 1 шара (подбирается в меню)

// ===== СОСТОЯНИЕ =====
float currentSpeed  = 0.0;
float currentEnergy = 0.0;
float battVoltage   = 0.0;
float currentTemp   = 0.0;

volatile int64_t t1 = 0, t2 = 0;
volatile bool chronoReady = false;
volatile int  encoderPos  = 0;

bool triggerWasReleased = true;
int  burstsRemaining    = 0;

unsigned long lastUiUpdate = 0;
bool needUiUpdate = false;
bool isFirstRead  = true;

// ===== ШАГОВЫЙ ДВИГАТЕЛЬ (ФОНОВЫЕ ПЕРЕМЕННЫЕ) =====
volatile int stepsRemaining = 0;
unsigned long lastStepTime = 0;
int currentStepPhase = 0;
const unsigned long stepInterval = 2500; // Микросекунды между шагами (2.5 мс). Регулирует скорость мотора.

// ===== ЦВЕТА =====
#define COL_BG     0x0000
#define COL_ACCENT 0x07FF
#define COL_TEXT   0xFFFF
#define COL_FRAME  0x3186
#define COL_WARN   0xF800

int uiState   = 0;
int menuIndex = 0;
int lastEnc   = 0;

// ===== FSM =====
enum FireState { IDLE, VALVE_ON, BETWEEN, SOL_ON, SOL_WAIT, SOL_SEAL, BURST_DELAY };
FireState fireState = IDLE;
unsigned long stateTime = 0;

// ===== МЕНЮ (Теперь 15 пунктов) =====
const int menuCount = 15;
String menuItems[menuCount] = {
  "[ EXIT ]", "MODE",      "ORDER",
  "MASS (g)", "VALVE (ms)","VALVE DLY",
  "SOL PUSH", "SOL RET",   "SOL SEAL",
  "BURST CNT","BURST DLY", "STEPS/BAL", // НОВЫЙ ПУНКТ
  "DIST (mm)","MAX TEMP",  "LOW V"
};

// ===== ISR =====
void IRAM_ATTR isrChrono1() { t1 = esp_timer_get_time(); }
void IRAM_ATTR isrChrono2() { if (t1 > 0) { t2 = esp_timer_get_time(); chronoReady = true; } }
void IRAM_ATTR isrEncoder() {
  static uint8_t old_AB = 0;
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  old_AB <<= 2; old_AB |= (digitalRead(ENC_S1) << 1) | digitalRead(ENC_S2);
  encoderPos += enc_states[(old_AB & 0x0f)];
  if (encoderPos >  10000) encoderPos =  10000;
  if (encoderPos < -10000) encoderPos = -10000;
}

// ===== SETUP =====
void setup() {
  pinMode(VALVE_PIN, OUTPUT); digitalWrite(VALVE_PIN, LOW);
  pinMode(SOL_PIN,   OUTPUT); digitalWrite(SOL_PIN,   LOW);
  
  // Инициализация пинов шаговика
  pinMode(STEP_IN1, OUTPUT); digitalWrite(STEP_IN1, LOW);
  pinMode(STEP_IN2, OUTPUT); digitalWrite(STEP_IN2, LOW);
  pinMode(STEP_IN3, OUTPUT); digitalWrite(STEP_IN3, LOW);
  pinMode(STEP_IN4, OUTPUT); digitalWrite(STEP_IN4, LOW);

  pinMode(TRIG_PIN,  INPUT_PULLUP);
  pinMode(IR1_PIN,   INPUT_PULLDOWN);
  pinMode(IR2_PIN,   INPUT_PULLDOWN);
  pinMode(ENC_S1,    INPUT_PULLUP);
  pinMode(ENC_S2,    INPUT_PULLUP);
  pinMode(ENC_KEY,   INPUT_PULLUP);

  attachInterrupt(IR1_PIN, isrChrono1, RISING);
  attachInterrupt(IR2_PIN, isrChrono2, RISING);
  attachInterrupt(ENC_S1,  isrEncoder, CHANGE);
  attachInterrupt(ENC_S2,  isrEncoder, CHANGE);

  prefs.begin("gun_conf", false);
  sensorDist    = prefs.getFloat("dist",    40.0);
  bulletMass    = prefs.getFloat("mass",    0.25);
  valveOpenTime = prefs.getInt("valve",     15);
  valveDelay    = prefs.getInt("valvDly",   20);
  solPushTime   = prefs.getInt("push",      50);
  solPullTime   = prefs.getInt("pull",      50);
  solSealTime   = prefs.getInt("seal",      10);
  burstCount    = prefs.getInt("bcnt",      3);
  burstDelay    = prefs.getInt("bdly",      50);
  fireMode      = prefs.getInt("mode",      0);
  fireOrder     = prefs.getInt("order",     0);
  maxTemp       = prefs.getInt("mtemp",     50);
  warnVolt      = prefs.getFloat("volt",    3.5);
  stepsPerBall  = prefs.getInt("stepsB",    128); // Загрузка шагов

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  drawMainScreen();
}

// ===== LOOP =====
void loop() {
  handleEncoder();
  
  if (uiState == 0) {
    handleStepper(); // Фоновое управление подачей шаров
    processChrono();
    readSensors();
    handleTriggerAndFSM();
    
    if (millis() - lastUiUpdate > 200 || needUiUpdate) {
      updateMainData();
      lastUiUpdate = millis();
      needUiUpdate = false;
    }
  }
}

// ===== ФОНОВОЕ УПРАВЛЕНИЕ ШАГОВИКОМ (БЕЗ DELAY) =====
void handleStepper() {
  if (stepsRemaining > 0) {
    unsigned long currentMicros = micros();
    if (currentMicros - lastStepTime >= stepInterval) {
      lastStepTime = currentMicros;
      
      // Полношаговый режим для максимального крутящего момента
      switch(currentStepPhase) {
        case 0: digitalWrite(STEP_IN1, HIGH); digitalWrite(STEP_IN2, HIGH); digitalWrite(STEP_IN3, LOW);  digitalWrite(STEP_IN4, LOW);  break;
        case 1: digitalWrite(STEP_IN1, LOW);  digitalWrite(STEP_IN2, HIGH); digitalWrite(STEP_IN3, HIGH); digitalWrite(STEP_IN4, LOW);  break;
        case 2: digitalWrite(STEP_IN1, LOW);  digitalWrite(STEP_IN2, LOW);  digitalWrite(STEP_IN3, HIGH); digitalWrite(STEP_IN4, HIGH); break;
        case 3: digitalWrite(STEP_IN1, HIGH); digitalWrite(STEP_IN2, LOW);  digitalWrite(STEP_IN3, LOW);  digitalWrite(STEP_IN4, HIGH); break;
      }
      
      currentStepPhase++;
      if (currentStepPhase > 3) currentStepPhase = 0;
      stepsRemaining--;

      // Как только шаги кончились - отключаем питание катушек, чтобы не сжечь мотор
      if (stepsRemaining == 0) {
        digitalWrite(STEP_IN1, LOW);
        digitalWrite(STEP_IN2, LOW);
        digitalWrite(STEP_IN3, LOW);
        digitalWrite(STEP_IN4, LOW);
      }
    }
  }
}

// ===== ХРОНОГРАФ =====
void processChrono() {
  if (chronoReady) {
    int64_t dt = t2 - t1;
    if (dt > 80 && dt < 500000) {
      float t_sec = dt / 1000000.0;
      currentSpeed  = (sensorDist / 1000.0) / t_sec;
      currentEnergy = 0.5 * (bulletMass / 1000.0) * currentSpeed * currentSpeed;
      needUiUpdate  = true;
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
    battVoltage = rawBatt; currentTemp = rawTemp;
    isFirstRead = false;
  } else {
    battVoltage = battVoltage * 0.9 + rawBatt * 0.1;
    currentTemp = currentTemp * 0.9 + rawTemp * 0.1;
  }
}

// ===== FSM СТРЕЛЬБЫ =====
void startShot() {
  // Добавляем шаги мотору при каждом выстреле
  stepsRemaining += stepsPerBall; 

  if (fireOrder == 0) {
    digitalWrite(VALVE_PIN, HIGH);
    fireState = VALVE_ON;
  } else {
    digitalWrite(SOL_PIN, HIGH);
    fireState = SOL_ON;
  }
  stateTime = millis();
}

void handleTriggerAndFSM() {
  if (currentTemp > maxTemp || (fireState != IDLE && millis() - stateTime > 2000)) {
    fireState = IDLE;
    digitalWrite(VALVE_PIN, LOW);
    digitalWrite(SOL_PIN,   LOW);
    burstsRemaining = 0;
    if (currentTemp > maxTemp) return;
  }

  static unsigned long lastTrigTime = 0;
  bool isPressed = (digitalRead(TRIG_PIN) == LOW);

  if (fireState == IDLE && isPressed && triggerWasReleased && (millis() - lastTrigTime > 50)) {
    lastTrigTime = millis();
    triggerWasReleased = false;
    if      (fireMode == 0) { startShot(); }
    else if (fireMode == 1) { burstsRemaining = burstCount - 1; startShot(); }
    else if (fireMode == 2) { startShot(); }
  }

  if (!isPressed && !triggerWasReleased && (millis() - lastTrigTime > 50)) {
    lastTrigTime = millis();
    triggerWasReleased = true;
    if (fireMode == 2) burstsRemaining = 0;
  }

  switch (fireState) {
    case IDLE: break;

    case VALVE_ON:
      if (millis() - stateTime >= (unsigned long)valveOpenTime) {
        digitalWrite(VALVE_PIN, LOW);
        stateTime = millis();
        if (fireOrder == 0) {
          fireState = BETWEEN;
        } else {
          if      (burstsRemaining > 0)           { burstsRemaining--; fireState = BURST_DELAY; }
          else if (fireMode == 2 && isPressed)     { fireState = BURST_DELAY; }
          else                                     { fireState = IDLE; }
        }
      }
      break;

    case BETWEEN:
      if (millis() - stateTime >= (unsigned long)valveDelay) {
        stateTime = millis();
        if (fireOrder == 0) {
          digitalWrite(SOL_PIN, HIGH);
          fireState = SOL_ON;
        } else {
          digitalWrite(VALVE_PIN, HIGH);
          fireState = VALVE_ON;
        }
      }
      break;

    case SOL_ON:
      if (millis() - stateTime >= (unsigned long)solPushTime) {
        digitalWrite(SOL_PIN, LOW);
        fireState = SOL_WAIT;
        stateTime = millis();
      }
      break;

    case SOL_WAIT:
      if (millis() - stateTime >= (unsigned long)solPullTime) {
        fireState = SOL_SEAL;
        stateTime = millis();
      }
      break;

    case SOL_SEAL:
      if (millis() - stateTime >= (unsigned long)solSealTime) {
        stateTime = millis();
        if (fireOrder == 0) {
          if      (burstsRemaining > 0)           { burstsRemaining--; fireState = BURST_DELAY; }
          else if (fireMode == 2 && isPressed)     { fireState = BURST_DELAY; }
          else                                     { fireState = IDLE; }
        } else {
          fireState = BETWEEN;
        }
      }
      break;

    case BURST_DELAY:
      if (millis() - stateTime >= (unsigned long)burstDelay) {
        startShot();
      }
      break;
  }
}

// ===== UI: ГЛАВНЫЙ ЭКРАН =====
void drawMainScreen() {
  tft.fillScreen(COL_BG);
  tft.fillRect(0, 0, 160, 14, 0x000F);
  tft.setTextColor(COL_ACCENT); tft.setTextSize(1);
  tft.setCursor(5, 3); tft.print("SMART SYSTEM");
  tft.drawRoundRect(5, 20, 150, 45, 6, COL_FRAME);
  tft.setCursor(12, 26); tft.setTextColor(0x7BEF); tft.print("VELOCITY m/s");
  tft.drawRoundRect(5, 70, 150, 35, 6, COL_FRAME);
  tft.setCursor(12, 76); tft.setTextColor(0x07E0); tft.print("ENERGY Joule");
  updateMainData();
}

void updateMainData() {
  tft.setTextSize(3); tft.setCursor(15, 38);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.print(currentSpeed, 1); tft.print("  ");

  tft.setTextSize(2); tft.setCursor(15, 84);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.print(currentEnergy, 2); tft.print("  ");

  tft.fillRect(5, 110, 150, 16, COL_BG);
  if (currentTemp > maxTemp) {
    tft.fillRoundRect(5, 110, 150, 16, 4, COL_WARN);
    tft.setTextSize(1); tft.setTextColor(COL_TEXT);
    tft.setCursor(20, 114); tft.print("!!! OVERHEAT !!!");
  } else {
    tft.setTextSize(1); tft.setCursor(10, 115);
    tft.setTextColor(0x7BEF, COL_BG);
    tft.print("MODE: ");
    tft.print((fireMode==0)?"SINGLE":(fireMode==1)?"BURST ":"AUTO  ");
    tft.print((fireOrder==0)?" V>S":" S>V");
  }

  tft.setCursor(95, 3);
  bool blink = (millis() / 500) % 2 == 0;
  if (battVoltage < warnVolt && blink) tft.setTextColor(COL_WARN, 0x000F);
  else                                 tft.setTextColor(ST7735_YELLOW, 0x000F);
  tft.print(battVoltage, 1); tft.print("V ");

  if (currentTemp > maxTemp - 5) tft.setTextColor(ST7735_ORANGE, 0x000F);
  else                           tft.setTextColor(ST7735_GREEN,  0x000F);
  tft.print((int)currentTemp); tft.print("C  ");
}

// ===== UI: МЕНЮ =====
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
    tft.setCursor(105, y);
    switch (idx) {
      case 1:  tft.print((fireMode==0)?"SNG":(fireMode==1)?"BST":"AUT"); break;
      case 2:  tft.print((fireOrder==0)?"V->S":"S->V"); break;
      case 3:  tft.print(bulletMass, 2); break;
      case 4:  tft.print(valveOpenTime); break;
      case 5:  tft.print(valveDelay); break;
      case 6:  tft.print(solPushTime); break;
      case 7:  tft.print(solPullTime); break;
      case 8:  tft.print(solSealTime); break;
      case 9:  tft.print(burstCount); break;
      case 10: tft.print(burstDelay); break;
      case 11: tft.print(stepsPerBall); break; // Вывод шагов на экран
      case 12: tft.print(sensorDist, 0); break;
      case 13: tft.print(maxTemp); break;
      case 14: tft.print(warnVolt, 1); break;
    }
  }
}

// ===== ЭНКОДЕР =====
void handleEncoder() {
  int cp = encoderPos / ENC_STEPS;
  int d  = cp - lastEnc;

  if (d != 0) {
    lastEnc = cp;
    if (uiState == 1) {
      menuIndex = constrain(menuIndex + d, 0, menuCount - 1);
      drawMenu();
    } else if (uiState == 2) {
      switch (menuIndex) {
        case 1:  fireMode      = constrain(fireMode + d, 0, 2); break;
        case 2:  fireOrder     = constrain(fireOrder + d, 0, 1); break;
        case 3:  bulletMass    = constrain(bulletMass + d * 0.01f, 0.1f, 5.0f); break;
        case 4:  valveOpenTime = constrain(valveOpenTime + d, 1, 200); break;
        case 5:  valveDelay    = constrain(valveDelay    + d, 0, 500); break;
        case 6:  solPushTime   = constrain(solPushTime   + d, 1, 500); break;
        case 7:  solPullTime   = constrain(solPullTime   + d, 1, 500); break;
        case 8:  solSealTime   = constrain(solSealTime   + d, 1, 200); break;
        case 9:  burstCount    = constrain(burstCount    + d, 2, 50); break;
        case 10: burstDelay    = constrain(burstDelay    + d * 5, 10, 1000); break;
        case 11: stepsPerBall  = constrain(stepsPerBall  + d * 4, 1, 2048); break; // Шаг настройки +-4
        case 12: sensorDist    = constrain(sensorDist    + d * 5, 10, 500); break;
        case 13: maxTemp       = constrain(maxTemp       + d, 30, 100); break;
        case 14: warnVolt      = constrain(warnVolt      + d * 0.1f, 3.0f, 4.2f); break;
      }
      drawMenu();
    }
  }

  static unsigned long lastBtn = 0;
  if (digitalRead(ENC_KEY) == LOW && millis() - lastBtn > 300) {
    lastBtn = millis();
    if (uiState == 0)      { uiState = 1; drawMenu(); }
    else if (uiState == 1) {
      if (menuIndex == 0)  { uiState = 0; drawMainScreen(); }
      else                 { uiState = 2; drawMenu(); }
    }
    else if (uiState == 2) { saveSettings(); uiState = 1; drawMenu(); }
  }
}

// ===== СОХРАНЕНИЕ =====
void saveSettings() {
  prefs.putFloat("dist",   sensorDist);
  prefs.putFloat("mass",   bulletMass);
  prefs.putInt("valve",    valveOpenTime);
  prefs.putInt("valvDly",  valveDelay);
  prefs.putInt("push",     solPushTime);
  prefs.putInt("pull",     solPullTime);
  prefs.putInt("seal",     solSealTime);
  prefs.putInt("bcnt",     burstCount);
  prefs.putInt("bdly",     burstDelay);
  prefs.putInt("mode",     fireMode);
  prefs.putInt("order",    fireOrder);
  prefs.putInt("mtemp",    maxTemp);
  prefs.putFloat("volt",   warnVolt);
  prefs.putInt("stepsB",   stepsPerBall); // Сохранение шагов
}
