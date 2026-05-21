// // --- ПИНЫ ---
// #define TFT_CS     5
// #define TFT_RST    4
// #define TFT_DC     14
// #define VALVE_PIN  27
// #define SOL_PIN    26
// #define TRIG_PIN   13
// #define ENC_S1     32
// #define ENC_S2     33
// #define ENC_KEY    25
// #define THERM_PIN  34
// #define BAT_PIN    35
// #define IR1_PIN    19
// #define IR2_PIN    21


#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Preferences.h>
#include <math.h>

// --- ПИНЫ (ПРОВЕРЕНО ДЛЯ ESP32) ---
#define TFT_CS     5
#define TFT_RST    4
#define TFT_DC     14
#define VALVE_PIN  27
#define SOL_PIN    26
#define TRIG_PIN   13
#define ENC_S1     32
#define ENC_S2     33
#define ENC_KEY    25
#define THERM_PIN  34  // ADC6 (Input only)
#define BAT_PIN    35  // ADC7 (Input only)
#define IR1_PIN    19
#define IR2_PIN    21

#define ENC_STEPS  4

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Preferences prefs;

// Настройки
float sensorDist = 50.0, bulletMass = 0.25;
int trigThreshold = 2000, valveOpenTime = 10, burstDelay = 50, burstCount = 3, fireMode = 0;
int solPushTime = 30, solPullTime = 100, maxTemp = 50;

// Состояние
float currentSpeed = 0.0, currentEnergy = 0.0, battVoltage = 0.0, currentTemp = 0.0;
bool triggerPressed = false, triggerWasReleased = true;
volatile unsigned long time1 = 0, time2 = 0;
volatile bool shotReady = false;
volatile int encoderPos = 0;
int lastMenuPos = 0, uiState = 0, menuIndex = 0;
unsigned long lastButtonPress = 0;

const int menuItemsCount = 11;
String menuNames[] = {"[ EXIT ]", "FIRE MODE", "DIST (mm)", "MASS (g)", "SOL PUSH", "SOL WAIT", "MAX TEMP", "TRIG THR", "VALVE (ms)", "BURST DLY", "BURST CNT"};

#define COL_BG      0x0000
#define COL_ACCENT  0x07FF 
#define COL_TEXT    0xFFFF
#define COL_FRAME   0x3186

// ================= ЛОГИКА И ПРЕРЫВАНИЯ =================
void IRAM_ATTR isrChrono1() { time1 = micros(); }
void IRAM_ATTR isrChrono2() { if (time1 > 0) { time2 = micros(); shotReady = true; } }
void IRAM_ATTR isrEncoder() {
  static uint8_t old_AB = 0;
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  old_AB <<= 2; old_AB |= (digitalRead(ENC_S1) << 1) | digitalRead(ENC_S2);
  encoderPos += enc_states[(old_AB & 0x0f)];
}

void setup() {
  pinMode(VALVE_PIN, OUTPUT); digitalWrite(VALVE_PIN, LOW);
  pinMode(SOL_PIN, OUTPUT); digitalWrite(SOL_PIN, LOW);
  pinMode(IR1_PIN, INPUT_PULLDOWN); pinMode(IR2_PIN, INPUT_PULLDOWN);
  pinMode(ENC_S1, INPUT_PULLUP); pinMode(ENC_S2, INPUT_PULLUP); pinMode(ENC_KEY, INPUT_PULLUP);
  
  attachInterrupt(IR1_PIN, isrChrono1, RISING); attachInterrupt(IR2_PIN, isrChrono2, RISING);
  attachInterrupt(ENC_S1, isrEncoder, CHANGE); attachInterrupt(ENC_S2, isrEncoder, CHANGE);

  prefs.begin("gun_conf", false);
  sensorDist = prefs.getFloat("dist", 50.0); bulletMass = prefs.getFloat("mass", 0.25);
  solPushTime = prefs.getInt("sPush", 30); solPullTime = prefs.getInt("sWait", 100);
  maxTemp = prefs.getInt("mTemp", 50); trigThreshold = prefs.getInt("trig", 2000);
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
    readSensors();
  }
  delay(1);
}

// ================= СЕНСОРЫ (NTC 10K) =================
void readSensors() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1500) {
    // Батарея
    battVoltage = (analogRead(BAT_PIN) / 4095.0) * 3.3 * 2.0; // Делитель 1:1

    // Термистор NTC 10k (Коэффициент B=3950)
    int raw = analogRead(THERM_PIN);
    if (raw > 0) {
      float resistance = 10000.0 / (4095.0 / (float)raw - 1.0);
      float steinhart;
      steinhart = resistance / 10000.0;     // (R/Ro)
      steinhart = log(steinhart);           // ln(R/Ro)
      steinhart /= 3950.0;                  // 1/B * ln(R/Ro)
      steinhart += 1.0 / (25.0 + 273.15);   // + (1/To)
      steinhart = 1.0 / steinhart;          // Инвертируем
      currentTemp = steinhart - 273.15;     // В Цельсии
    }
    
    lastUpdate = millis();
    if (uiState == 0) updateMainData();
  }
}

// ================= ЛОГИКА СТРЕЛЬБЫ =================
void fireSequence() {
  if (currentTemp > (float)maxTemp) return;
  digitalWrite(VALVE_PIN, HIGH); delay(valveOpenTime); digitalWrite(VALVE_PIN, LOW);
  digitalWrite(SOL_PIN, HIGH); delay(solPushTime); digitalWrite(SOL_PIN, LOW);
  delay(solPullTime);

  time1 = 0; time2 = 0; shotReady = false; 
}

void checkTriggerAndFire() {
  int tv = analogRead(TRIG_PIN);
  triggerPressed = (tv > trigThreshold);
  
  if (triggerPressed && triggerWasReleased) {
    if (fireMode == 0) { fireSequence(); triggerWasReleased = false; }
    else if (fireMode == 1) {
      for(int i=0; i<burstCount; i++) { fireSequence(); if(i < burstCount-1) delay(burstDelay); }
      triggerWasReleased = false;
    }
  }
  if (triggerPressed && fireMode == 2) { fireSequence(); delay(burstDelay); triggerWasReleased = false; }
  if (tv < (trigThreshold - 200)) triggerWasReleased = true;
}

// ================= ЭКРАН И МЕНЮ =================
void updateMainData() {
  tft.setTextSize(3); tft.setCursor(15, 38);
  tft.setTextColor(COL_TEXT, COL_BG); tft.print(currentSpeed, 1); tft.print("  ");

  tft.setTextSize(2); tft.setCursor(15, 84);
  tft.setTextColor(COL_TEXT, COL_BG); tft.print(currentEnergy, 2); tft.print("  ");

  tft.setTextSize(1); tft.setCursor(10, 115);
  if (currentTemp > (float)maxTemp) { tft.setTextColor(ST7735_RED, COL_BG); tft.print("!!! OVERHEAT !!!"); }
  else {
    tft.setTextColor(0x7BEF, COL_BG); tft.print("MODE: "); 
    tft.print((fireMode==0)?"SINGLE ":(fireMode==1)?"BURST  ":"AUTO   ");
  }
  
  tft.setTextColor(ST7735_YELLOW, 0x000F); tft.setCursor(95, 3); 
  tft.print(battVoltage, 1); tft.print("V ");
  if (currentTemp > (float)maxTemp-5) tft.setTextColor(ST7735_ORANGE, 0x000F);
  else tft.setTextColor(ST7735_GREEN, 0x000F);
  tft.print((int)currentTemp); tft.print("C");
}

void drawMainScreen() {
  tft.fillScreen(COL_BG);
  tft.fillRect(0, 0, 160, 14, 0x000F); 
  tft.setTextColor(COL_ACCENT); tft.setTextSize(1);
  tft.setCursor(5, 3); tft.print("SMART SYSTEM");
  tft.drawRoundRect(5, 20, 150, 45, 6, COL_FRAME);
  tft.setCursor(12, 26); tft.print("VELOCITY m/s");
  tft.drawRoundRect(5, 70, 150, 35, 6, COL_FRAME);
  tft.setCursor(12, 76); tft.setTextColor(0x07E0); tft.print("ENERGY Joule");
  updateMainData();
}

void drawMenu() {
  tft.fillRect(0, 14, 160, 114, COL_BG);
  tft.setTextSize(1);
  
  // Логика прокрутки: показываем окно из 6 элементов
  int startItem = 0;
  if (menuIndex >= 6) startItem = menuIndex - 5; 

  for (int i = 0; i < 6; i++) {
    int idx = startItem + i;
    if (idx >= menuItemsCount) break;

    int y = 20 + (i * 16);
    if (idx == menuIndex) {
      tft.fillRect(5, y - 4, 150, 14, (uiState == 2) ? 0xF800 : 0x2104);
      tft.setTextColor(COL_TEXT);
    } else {
      tft.setTextColor(0x7BEF);
    }

    tft.setCursor(10, y); tft.print(menuNames[idx]);
    
    // Значения
    tft.setCursor(110, y);
    if (idx == 1) tft.print((fireMode==0)?"SNG":(fireMode==1)?"BST":"AUTO");
    else if (idx == 2) tft.print(sensorDist, 0);
    else if (idx == 3) tft.print(bulletMass, 2);
    else if (idx == 4) tft.print(solPushTime);
    else if (idx == 5) tft.print(solPullTime);
    else if (idx == 6) tft.print(maxTemp);
    else if (idx == 7) tft.print(trigThreshold);
    else if (idx == 8) tft.print(valveOpenTime);
    else if (idx == 9) tft.print(burstDelay);
    else if (idx == 10) tft.print(burstCount);
  }
}

// ================= УПРАВЛЕНИЕ =================
void checkChrono() {
  if (shotReady) {
    unsigned long td = time2 - time1; 
    if (td > 50 && td < 1000000) { 
      currentSpeed = (sensorDist / 1000.0) / (td / 1000000.0); 
      currentEnergy = (bulletMass / 1000.0) * (currentSpeed * currentSpeed) / 2.0;
      updateMainData();
    }
    shotReady = false;
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
      if (menuIndex == 2) sensorDist = constrain(sensorDist + d, 1, 500);
      if (menuIndex == 3) bulletMass = constrain(bulletMass + (d * 0.01), 0.01, 10.0);
      if (menuIndex == 4) solPushTime = constrain(solPushTime + d, 1, 1000);
      if (menuIndex == 5) solPullTime = constrain(solPullTime + (d * 5), 1, 2000);
      if (menuIndex == 6) maxTemp = constrain(maxTemp + d, 20, 100);
      if (menuIndex == 7) trigThreshold = constrain(trigThreshold + (d * 50), 0, 4000);
      if (menuIndex == 8) valveOpenTime = constrain(valveOpenTime + d, 1, 500);
      if (menuIndex == 9) burstDelay = constrain(burstDelay + (d * 5), 10, 1000);
      if (menuIndex == 10) burstCount = constrain(burstCount + d, 2, 50);
      drawMenu();
    }
  }
  if (digitalRead(ENC_KEY) == LOW && millis() - lastButtonPress > 300) {
    lastButtonPress = millis();
    if (uiState == 0) { uiState = 1; drawMenu(); } 
    else if (uiState == 1) { if (menuIndex == 0) { uiState = 0; drawMainScreen(); } else { uiState = 2; drawMenu(); } } 
    else if (uiState == 2) { uiState = 1; saveSettings(); drawMenu(); }
  }
}

void saveSettings() { 
  prefs.putFloat("dist", sensorDist); prefs.putFloat("mass", bulletMass);
  prefs.putInt("sPush", solPushTime); prefs.putInt("sWait", solPullTime);
  prefs.putInt("mTemp", maxTemp); prefs.putInt("trig", trigThreshold); 
  prefs.putInt("valve", valveOpenTime); prefs.putInt("delay", burstDelay); 
  prefs.putInt("burst", burstCount); prefs.putInt("mode", fireMode);
}