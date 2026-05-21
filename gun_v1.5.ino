#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Preferences.h>

// ================= НАСТРОЙКИ ПИНОВ =================
#define TFT_CS     5
#define TFT_RST    4
#define TFT_DC     2
#define PIN_VALVE  25
#define PIN_TRIG   34
#define PIN_BATT   35
#define PIN_SENS_1 32
#define PIN_SENS_2 33
#define PIN_ENC_A  26
#define PIN_ENC_B  27
#define PIN_ENC_SW 14

#define ENC_STEPS  4

// ================= ПЕРЕМЕННЫЕ =================
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Preferences prefs;

float sensorDist = 50.0, bulletMass = 0.25, currentSpeed = 0.0, currentEnergy = 0.0, battVoltage = 0.0;
int trigThreshold = 2000, valveOpenTime = 10, burstDelay = 50, burstCount = 3, fireMode = 0;
bool triggerPressed = false, triggerWasReleased = true;

volatile unsigned long time1 = 0, time2 = 0;
volatile bool shotReady = false;
volatile int encoderPos = 0;
int lastMenuPos = 0, uiState = 0, menuIndex = 0;
unsigned long lastButtonPress = 0;
const int menuItemsCount = 7;
String menuNames[] = {"EXIT", "FIRE MODE", "DIST (mm)", "TRIG THR", "VALVE(ms)", "BRST DLY", "BRST CNT"};

// Цвета
#define COL_BG      0x0000
#define COL_ACCENT  0x07FF 
#define COL_TEXT    0xFFFF
#define COL_FRAME   0x3186

// ================= СИСТЕМА ПРЕРЫВАНИЙ =================
void IRAM_ATTR isrChrono1() { time1 = micros(); }
void IRAM_ATTR isrChrono2() { if (time1 > 0) { time2 = micros(); shotReady = true; } }
void IRAM_ATTR isrEncoder() {
  static uint8_t old_AB = 0;
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  old_AB <<= 2; old_AB |= (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  encoderPos += enc_states[(old_AB & 0x0f)];
}

void setup() {
  pinMode(PIN_VALVE, OUTPUT); digitalWrite(PIN_VALVE, LOW);
  pinMode(PIN_SENS_1, INPUT_PULLDOWN); pinMode(PIN_SENS_2, INPUT_PULLDOWN);
  pinMode(PIN_ENC_A, INPUT_PULLUP); pinMode(PIN_ENC_B, INPUT_PULLUP); pinMode(PIN_ENC_SW, INPUT_PULLUP);
  attachInterrupt(PIN_SENS_1, isrChrono1, RISING); attachInterrupt(PIN_SENS_2, isrChrono2, RISING);
  attachInterrupt(PIN_ENC_A, isrEncoder, CHANGE); attachInterrupt(PIN_ENC_B, isrEncoder, CHANGE);

  prefs.begin("gun_settings", false);
  sensorDist = prefs.getFloat("dist", 50.0); trigThreshold = prefs.getInt("trig", 2000);
  valveOpenTime = prefs.getInt("valve", 10); burstDelay = prefs.getInt("delay", 50);
  burstCount = prefs.getInt("burst", 3); fireMode = prefs.getInt("mode", 0);

  tft.initR(INITR_BLACKTAB); tft.setRotation(1);
  drawMainScreen();
}

void loop() {
  checkEncoder();
  if (uiState == 0) {
    checkChrono();
    checkTriggerAndFire();
    readBattery();
  }
  delay(10);
}

// ================= ОТРИСОВКА ГЛАВНОГО ЭКРАНА =================
void drawMainScreen() {
  tft.fillScreen(COL_BG);
  
  // Шапка
  tft.fillRect(0, 0, 160, 14, 0x000F); 
  tft.setTextColor(COL_ACCENT); tft.setTextSize(1);
  tft.setCursor(5, 3); tft.print("SMART SYSTEM HUD");

  // Блок скорости (закругленный)
  tft.drawRoundRect(5, 20, 150, 45, 6, COL_FRAME);
  tft.setCursor(12, 26); tft.setTextColor(COL_ACCENT); tft.print("VELOCITY m/s");

  // Блок энергии (закругленный)
  tft.drawRoundRect(5, 70, 150, 35, 6, COL_FRAME);
  tft.setCursor(12, 76); tft.setTextColor(0x07E0); tft.print("ENERGY Joule");

  // Режим стрельбы внизу
  tft.setTextColor(0x7BEF);
  tft.setCursor(10, 115); tft.print("MODE:");

  updateMainData();
}

void updateMainData() {
  // Скорость (перерисовываем только цифры)
  tft.setTextSize(3); tft.setCursor(15, 38);
  tft.setTextColor(COL_TEXT, COL_BG); 
  tft.print(currentSpeed, 1); tft.print("  ");

  // Энергия
  tft.setTextSize(2); tft.setCursor(15, 84);
  tft.setTextColor(COL_TEXT, COL_BG); 
  tft.print(currentEnergy, 2); tft.print("  ");

  // Режим
  tft.setTextSize(1); tft.setCursor(45, 115);
  tft.setTextColor(COL_ACCENT, COL_BG);
  if(fireMode == 0) tft.print("SINGLE      ");
  else if(fireMode == 1) tft.print("BURST       ");
  else tft.print("AUTO        ");
  // Батарея в шапке
  tft.setTextColor(ST7735_YELLOW, 0x000F);
  tft.setCursor(125, 3); tft.print(battVoltage, 1); tft.print("V");
}

// ================= МЕНЮ (ИСПРАВЛЕННОЕ) =================
void drawMenu() {
  tft.fillRect(0, 14, 160, 114, COL_BG);
  tft.setTextColor(ST7735_ORANGE); tft.setCursor(10, 20); tft.print("--- CONFIG ---");

  for (int i = 0; i < menuItemsCount; i++) {
    int y = 35 + (i * 12);
    if (i == menuIndex) { 
      tft.fillRect(5, y - 2, 150, 11, (uiState == 2) ? 0xF800 : 0x2104); 
      tft.setTextColor(COL_TEXT); 
    } else { 
      tft.setTextColor(0x7BEF); 
    }
    
    tft.setCursor(10, y); tft.print(menuNames[i]);
    
    // Значения (Теперь всё на своих местах)
    tft.setCursor(110, y);
    if (i == 1) tft.print((fireMode==0)?"SNG":(fireMode==1)?"BST":"AUTO");
    else if (i == 2) tft.print(sensorDist, 0);
    else if (i == 3) tft.print(trigThreshold);
    else if (i == 4) tft.print(valveOpenTime);
    else if (i == 5) tft.print(burstDelay); // Тот самый BRST DLY
    else if (i == 6) tft.print(burstCount);
  }
}

// ================= ЛОГИКА (БЕЗ ИЗМЕНЕНИЙ) =================
void checkTriggerAndFire() {
  int tv = analogRead(PIN_TRIG);
  triggerPressed = (tv > trigThreshold);
  if (triggerPressed && triggerWasReleased) {
    if (fireMode <= 1) { 
      int s = (fireMode == 0) ? 1 : burstCount;
      for(int i=0; i<s; i++){ fireValve(); if(s>1) delay(burstDelay); }
      triggerWasReleased = false;
    }
  }
  if (triggerPressed && fireMode == 2) { fireValve(); delay(burstDelay); triggerWasReleased = false; }
  if (tv < (trigThreshold - 200)) triggerWasReleased = true;
}

void fireValve() { 
  time1 = 0; time2 = 0; shotReady = false; 
  digitalWrite(PIN_VALVE, HIGH); delay(valveOpenTime); digitalWrite(PIN_VALVE, LOW); 
}

void checkChrono() {
  if (shotReady) {
    unsigned long td = time2 - time1; 
    if (td > 0 && td < 1000000) { 
      currentSpeed = (sensorDist / 1000.0) / (td / 1000000.0); 
      currentEnergy = (bulletMass / 1000.0) * (currentSpeed * currentSpeed) / 2.0;
      updateMainData();
    }
    shotReady = false;
  }
}

void readBattery() { 
  static unsigned long lb = 0; 
  if (millis() - lb > 5000) { 
    battVoltage = (analogRead(PIN_BATT) / 4095.0) * 6.6; 
    lb = millis(); updateMainData(); 
  } 
}

void checkEncoder() {
  int cp = encoderPos / ENC_STEPS;
  int d = cp - lastMenuPos;
  if (d != 0) {
    lastMenuPos = cp;
    if (uiState == 1) { menuIndex = constrain(menuIndex + d, 0, menuItemsCount - 1); drawMenu(); } 
    else if (uiState == 2) {
      if (menuIndex == 1) fireMode = constrain(fireMode + d, 0, 2);
      if (menuIndex == 2) sensorDist += d;
      if (menuIndex == 3) trigThreshold = constrain(trigThreshold + (d * 50), 0, 4000);
      if (menuIndex == 4) valveOpenTime = constrain(valveOpenTime + d, 1, 100);
      if (menuIndex == 5) burstDelay = constrain(burstDelay + (d * 5), 10, 500);
      if (menuIndex == 6) burstCount = constrain(burstCount + d, 2, 20);
      drawMenu();
    }
  }
  if (digitalRead(PIN_ENC_SW) == LOW && millis() - lastButtonPress > 300) {
    lastButtonPress = millis();
    if (uiState == 0) { uiState = 1; drawMenu(); } 
    else if (uiState == 1) { if (menuIndex == 0) { uiState = 0; drawMainScreen(); } else { uiState = 2; drawMenu(); } } 
    else if (uiState == 2) { uiState = 1; saveSettings(); drawMenu(); }
  }
}

void saveSettings() { 
  prefs.putFloat("dist", sensorDist); prefs.putInt("trig", trigThreshold); 
  prefs.putInt("valve", valveOpenTime); prefs.putInt("delay", burstDelay); prefs.putInt("burst", burstCount); 
}