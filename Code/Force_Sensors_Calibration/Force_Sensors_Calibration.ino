#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <math.h>

#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

const int PIN_L = A0;
const int PIN_R = A1;

struct CalData {
  uint32_t magic;
  float M_L, B_L;
  float M_R, B_R;
  float tot_gain;
};
const uint32_t CAL_MAGIC = 0xA17F5F5Ful;
const int EEPROM_ADDR = 0;

CalData cal = {CAL_MAGIC, 182.0f, 353.7f, 124.0f, 371.3f, 1.0f};

const int N_AVG = 10;
int bufL[N_AVG] = {0}, bufR[N_AVG] = {0};
int idxAvg = 0, filled = 0, sumL = 0, sumR = 0;

float tareL = 0.0f, tareR = 0.0f;
unsigned long lastLcd = 0;

int readADCavg(int pin) {
  long acc = 0;
  for (int i = 0; i < 20; i++) { acc += analogRead(pin); delay(2); }
  return (int)(acc / 20);
}
void lcdClearLine(uint8_t row){
  lcd.setCursor(0,row); lcd.print("                ");
}
void lcdTop(const String &s) { lcdClearLine(0); lcd.setCursor(0,0); lcd.print(s); }
void lcdBot(const String &s) { lcdClearLine(1); lcd.setCursor(0,1); lcd.print(s); }

void loadCal() {
  CalData tmp;
  EEPROM.get(EEPROM_ADDR, tmp);
  if (tmp.magic == CAL_MAGIC && isfinite(tmp.M_L) && isfinite(tmp.M_R)) {
    cal = tmp;
    if (!(cal.tot_gain > 0.2f && cal.tot_gain < 5.0f)) cal.tot_gain = 1.0f;
  }
}
void saveCal() { EEPROM.put(EEPROM_ADDR, cal); }

void tareNow() {
  int aL = readADCavg(PIN_L);
  int aR = readADCavg(PIN_R);
  float kgL = (aL - cal.B_L) / cal.M_L;
  float kgR = (aR - cal.B_R) / cal.M_R;
  tareL = kgL; tareR = kgR;
}

void fitLineLR(const float *x, const int *yL, const int *yR, int n,
               float &ML, float &BL, float &MR, float &BR) {
  double Sx = 0, Sx2 = 0, SyL = 0, SyR = 0, SxSyL = 0, SxSyR = 0;
  for (int i = 0; i < n; i++) {
    double xi = x[i];
    Sx += xi; Sx2 += xi * xi;
    SyL += yL[i]; SyR += yR[i];
    SxSyL += xi * yL[i]; SxSyR += xi * yR[i];
  }
  double denom = (n * Sx2 - Sx * Sx);
  if (denom == 0) denom = 1;
  ML = (float)((n * SxSyL - Sx * SyL) / denom);
  BL = (float)((SyL - ML * Sx) / n);
  MR = (float)((n * SxSyR - Sx * SyR) / denom);
  BR = (float)((SyR - MR * Sx) / n);
}

void runCalibration() {
  const int NPTS = 3;
  float W[NPTS] = {0.5f, 1.0f, 1.5f};
  int YL[NPTS], YR[NPTS];

  lcdTop("CAL: place 0.5kg"); lcdBot("Press '1' to cap");
  Serial.println(F("[CAL] Place 0.5 kg, press '1' to capture"));
  while (true) { if (Serial.available() && Serial.read() == '1') { YL[0] = readADCavg(PIN_L); YR[0] = readADCavg(PIN_R); break; } }

  lcdTop("CAL: place 1.0kg"); lcdBot("Press '2' to cap");
  Serial.println(F("[CAL] Place 1.0 kg, press '2' to capture"));
  while (true) { if (Serial.available() && Serial.read() == '2') { YL[1] = readADCavg(PIN_L); YR[1] = readADCavg(PIN_R); break; } }

  lcdTop("CAL: place 1.5kg"); lcdBot("Press '3' to cap");
  Serial.println(F("[CAL] Place 1.5 kg, press '3' to capture"));
  while (true) { if (Serial.available() && Serial.read() == '3') { YL[2] = readADCavg(PIN_L); YR[2] = readADCavg(PIN_R); break; } }

  float ML, BL, MR, BR;
  fitLineLR(W, YL, YR, NPTS, ML, BL, MR, BR);

  cal.M_L = ML; cal.B_L = BL;
  cal.M_R = MR; cal.B_R = BR;
  cal.magic = CAL_MAGIC;
  saveCal();

  tareL = 0; tareR = 0;
  Serial.println(F("[CAL] Done. New params:"));
  Serial.print(F("  Left:  M=")); Serial.print(cal.M_L, 2); Serial.print(F("  B=")); Serial.println(cal.B_L, 2);
  Serial.print(F("  Right: M=")); Serial.print(cal.M_R, 2); Serial.print(F("  B=")); Serial.println(cal.B_R, 2);

  lcdTop("CAL saved ✓"); lcdBot("t=Zero  g/k=Gain");
  delay(1000);
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(5000);

  Wire.begin();
  lcd.init();
  lcd.backlight();

  lcdTop("FSR Scale v1.3");
  lcdBot("Loading cal...");
  loadCal();
  delay(600);

  for (int i = 0; i < N_AVG; i++) {
    int rl = analogRead(PIN_L);
    int rr = analogRead(PIN_R);
    bufL[i] = rl; bufR[i] = rr; sumL += rl; sumR += rr;
    delay(5);
  }
  filled = N_AVG;

  lcdTop("t=Tare c=Cal");
  lcdBot("g/k=Gain, p=par");
  delay(700);
}

void loop() {
  int rl = analogRead(PIN_L);
  int rr = analogRead(PIN_R);
  sumL -= bufL[idxAvg]; sumR -= bufR[idxAvg];
  bufL[idxAvg] = rl;   bufR[idxAvg] = rr;
  sumL += bufL[idxAvg]; sumR += bufR[idxAvg];
  idxAvg = (idxAvg + 1) % N_AVG;

  float adcL = sumL / float(filled);
  float adcR = sumR / float(filled);

  float kgL = (adcL - cal.B_L) / cal.M_L - tareL; if (kgL < 0) kgL = 0;
  float kgR = (adcR - cal.B_R) / cal.M_R - tareR; if (kgR < 0) kgR = 0;

  float kgSum = kgL + kgR;
  float kgSumDisp = kgSum * cal.tot_gain;

  if (millis() - lastLcd > 250) {
    lastLcd = millis();

    lcdClearLine(0);
    lcd.setCursor(0,0);
    lcd.print("L:");   lcd.print(kgL, 2);
    lcd.print(" R:");  lcd.print(kgR, 2);

    lcdClearLine(1);
    lcd.setCursor(0,1);
    lcd.print("SUM:"); lcd.print(kgSumDisp, 2); lcd.print("kg");
  }

  if (Serial.available()) {
    char c = Serial.read();

    if (c == 't' || c == 'T') {
      tareNow();
      lcdTop("Tare set (0 kg)");
      delay(800);
    }

    if (c == 'c' || c == 'C') {
      while (Serial.available()) Serial.read();
      runCalibration();
    }

    if (c == 'p' || c == 'P') {
      Serial.println(F("[PARAMS]"));
      Serial.print(F("L: M=")); Serial.print(cal.M_L, 4); Serial.print(F("  B=")); Serial.println(cal.B_L, 2);
      Serial.print(F("R: M=")); Serial.print(cal.M_R, 4); Serial.print(F("  B=")); Serial.println(cal.B_R, 2);
      Serial.print(F("TOT_GAIN=")); Serial.println(cal.tot_gain, 4);
      lcdTop("Sent: params"); delay(600);
    }

    if (c == 'g' || c == 'G') {
      float L = (readADCavg(PIN_L) - cal.B_L) / cal.M_L - tareL; if (L < 0) L = 0;
      float R = (readADCavg(PIN_R) - cal.B_R) / cal.M_R - tareR; if (R < 0) R = 0;
      float sum = L + R;
      if (sum > 0.01f) {
        cal.tot_gain = 1.0f / sum;
        saveCal();
        lcdTop("Gain set: 1.00kg"); delay(1200);
      } else {
        lcdTop("Put weight first"); delay(1000);
      }
    }

    if (c == 'k' || c == 'K') {
      float ref = Serial.parseFloat();
      float L = (readADCavg(PIN_L) - cal.B_L) / cal.M_L - tareL; if (L < 0) L = 0;
      float R = (readADCavg(PIN_R) - cal.B_R) / cal.M_R - tareR; if (R < 0) R = 0;
      float sum = L + R;
      if (ref > 0.05f && sum > 0.01f) {
        cal.tot_gain = ref / sum;
        saveCal();
        lcdTop("Gain saved"); delay(1000);
      } else {
        lcdTop("Bad ref/sum"); delay(800);
      }
    }

    if (c == 'r' || c == 'R') {
      cal.tot_gain = 1.0f; saveCal();
      lcdTop("Gain reset (1x)"); delay(800);
    }

    if (c == 's' || c == 'S') {
      Serial.print(F("kgL=")); Serial.print(kgL,3);
      Serial.print(F(" kgR=")); Serial.print(kgR,3);
      Serial.print(F(" SUM=")); Serial.print(kgSum,3);
      Serial.print(F(" SUM*G=")); Serial.println(kgSumDisp,3);
    }

    if (c == 'u' || c == 'U') {
      float L = (readADCavg(PIN_L) - cal.B_L) / cal.M_L - tareL; if (L < 0) L = 0;
      float R = (readADCavg(PIN_R) - cal.B_R) / cal.M_R - tareR; if (R < 0) R = 0;
      float sum = L + R;
      Serial.print(F("[UNSCALED SUM] ")); Serial.println(sum,3);
    }
  }

  delay(10);
}