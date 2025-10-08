#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Servo.h>
#include <math.h>

#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

const int PIN_L = A0;
const int PIN_R = A1;
const int SERVO_PIN = 9;

const float G_N_PER_KG = 9.80665f;
const float MAX_FORCE_N = 15.0f;

struct CalData {
  uint32_t magic;
  float M_L, B_L;
  float M_R, B_R;
  float tot_gain;
};
const uint32_t CAL_MAGIC = 0xA17F5F5Ful;
const int CAL_EE_ADDR = 0;
CalData cal = {CAL_MAGIC, 162.0f, 247.67f, 124.0f, 244.0f, 1.0f};

struct EP {
  uint32_t magic;
  int usOpen;
  int usClose;
};
const uint32_t EP_MAGIC = 0x454E444Fu;
const int EP_ADDR = 128;
EP ep = {EP_MAGIC, 2100, 544};

Servo gripper;
bool armed = false;
int curUs = 1500;

const int N_AVG = 8;
int bufL[N_AVG]={0}, bufR[N_AVG]={0};
int idx=0, filled=0, sumL=0, sumR=0;

float tareL=0, tareR=0;
unsigned long lastLCD=0;

inline int clampToEndpoints(int us){
  int lo = min(ep.usClose, ep.usOpen), hi = max(ep.usClose, ep.usOpen);
  return constrain(us, lo, hi);
}
void loadCal(){
  CalData t; EEPROM.get(CAL_EE_ADDR, t);
  if (t.magic==CAL_MAGIC && isfinite(t.M_L) && isfinite(t.M_R)) {
    cal=t; if(!(cal.tot_gain>0.2f && cal.tot_gain<5.0f)) cal.tot_gain=1.0f;
  }
}
void loadEP(){ EP t; EEPROM.get(EP_ADDR, t); if (t.magic==EP_MAGIC) ep=t; }

void lcdClearLine(uint8_t row){ lcd.setCursor(0,row); lcd.print("                "); }
void lcdTop(const String&s){ lcdClearLine(0); lcd.setCursor(0,0); lcd.print(s); }
void lcdBot(const String&s){ lcdClearLine(1); lcd.setCursor(0,1); lcd.print(s); }

void attachIfNeeded(){ if(!armed){ gripper.attach(SERVO_PIN); gripper.writeMicroseconds(curUs); armed=true; } }
void detachIfNeeded(){ if(armed){ gripper.detach(); armed=false; } }

void rampToUs(int target){
  target = clampToEndpoints(target);
  int dir = (target > curUs) ? +1 : -1;
  while (curUs != target){
    curUs += dir;
    if (armed) gripper.writeMicroseconds(curUs);
    delay(6);
  }
}

int readADCavg(int pin){
  long acc=0; for(int i=0;i<16;i++){ acc+=analogRead(pin); delay(1);} return (int)(acc/16);
}

void tareNow(){
  int aL=readADCavg(PIN_L), aR=readADCavg(PIN_R);
  float kgL=(aL - cal.B_L)/cal.M_L;
  float kgR=(aR - cal.B_R)/cal.M_R;
  if(kgL<0) kgL=0; if(kgR<0) kgR=0;
  tareL=kgL; tareR=kgR;
  lcdTop("Tare done");
}

float readForceN(){
  int rL=analogRead(PIN_L), rR=analogRead(PIN_R);
  sumL-=bufL[idx]; sumR-=bufR[idx];
  bufL[idx]=rL;    bufR[idx]=rR;
  sumL+=bufL[idx]; sumR+=bufR[idx];
  idx=(idx+1)%N_AVG;

  float adcL = sumL / float(max(filled,1));
  float adcR = sumR / float(max(filled,1));
  float kgL = (adcL - cal.B_L)/cal.M_L - tareL; if(kgL<0) kgL=0;
  float kgR = (adcR - cal.B_R)/cal.M_R - tareR; if(kgR<0) kgR=0;
  float kgSumDisp = (kgL + kgR) * cal.tot_gain;
  return kgSumDisp * G_N_PER_KG;
}

void setup(){
  Serial.begin(9600);
  Serial.setTimeout(4000);

  Wire.begin();
  lcd.init(); lcd.backlight();

  lcdTop("Gripper Run (N)");
  lcdBot("Loading...");
  loadCal();
  loadEP();
  delay(400);

  for(int i=0;i<N_AVG;i++){
    int L=analogRead(PIN_L), R=analogRead(PIN_R);
    bufL[i]=L; bufR[i]=R; sumL+=L; sumR+=R; delay(2);
  }
  filled=N_AVG;

  curUs = (ep.usOpen + ep.usClose) / 2;

  lcdTop("READY");
  lcdBot("");
}

void loop(){
  float Nsum = readForceN();

  if (millis()-lastLCD > 250){
    lastLCD = millis();
    lcdClearLine(0); lcd.setCursor(0,0);
    lcd.print("F:"); lcd.print(Nsum,1); lcd.print("N ");
    lcd.print(armed ? "A" : "D");

    lcdClearLine(1); lcd.setCursor(0,1);
    lcd.print("u:"); lcd.print(curUs);
    lcd.print(" O:"); lcd.print(ep.usOpen);
    lcd.print(" C:"); lcd.print(ep.usClose);
  }

  if (Serial.available()){
    char c = Serial.read();

    if (c=='a' || c=='A') attachIfNeeded();
    if (c=='x' || c=='X') detachIfNeeded();

    if (c=='o' || c=='O') rampToUs(ep.usOpen);
    if (c=='c' || c=='C'){
      if (Nsum <= MAX_FORCE_N) rampToUs(ep.usClose);
      else lcdTop("Force limit hit");
    }

    if (c=='[') rampToUs(curUs - 2);
    if (c==']'){
      int next = clampToEndpoints(curUs + 2);
      bool closeIsLower = (ep.usClose < ep.usOpen);
      bool towardClose = closeIsLower ? (next < curUs) : (next > curUs);
      if (!(towardClose && Nsum > MAX_FORCE_N)) rampToUs(next);
    }
    if (c=='-') rampToUs(curUs - 10);
    if (c=='+'){
      int next = clampToEndpoints(curUs + 10);
      bool closeIsLower = (ep.usClose < ep.usOpen);
      bool towardClose = closeIsLower ? (next < curUs) : (next > curUs);
      if (!(towardClose && Nsum > MAX_FORCE_N)) rampToUs(next);
    }

    if (c=='u' || c=='U'){ int v=Serial.parseInt(); if(v>0) rampToUs(v); }
    if (c=='t' || c=='T') tareNow();

    if (c=='p' || c=='P'){
      Serial.println(F("[STATUS]"));
      Serial.print(F("OPEN(us)=")); Serial.println(ep.usOpen);
      Serial.print(F("CLOSE(us)=")); Serial.println(ep.usClose);
      Serial.print(F("u=")); Serial.println(curUs);
      Serial.print(F("Force(N)=")); Serial.println(Nsum,2);
    }
  }

  delay(5);
}