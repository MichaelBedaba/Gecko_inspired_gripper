#include <Servo.h>
#include <EEPROM.h>

Servo s;
const int SERVO_PIN = 9;

struct EP {
  uint32_t magic;
  int usOpen;
  int usClose;
};
const uint32_t EP_MAGIC = 0x454E444Fu;
const int EP_ADDR = 128;

EP ep;

bool armed = false;
int curUs = 1500;
int msPerStep = 6;
const int FINE = 2;
const int COARSE = 10;

void loadEP(){
  EEPROM.get(EP_ADDR, ep);
  if (ep.magic != EP_MAGIC) { ep.magic = EP_MAGIC; ep.usOpen = 1500; ep.usClose = 1500; }
}
void saveEP(){ ep.magic = EP_MAGIC; EEPROM.put(EP_ADDR, ep); }

void printEP(const char* tag=nullptr){
  if(tag){ Serial.print(tag); Serial.print(" "); }
  Serial.print(F("curUs=")); Serial.print(curUs);
  Serial.print(F("  OPEN(us)=")); Serial.print(ep.usOpen);
  Serial.print(F("  CLOSE(us)=")); Serial.println(ep.usClose);
}

void rampTo(int target){
  target = constrain(target, 500, 2500);
  int step = (target > curUs) ? +1 : -1;
  while (curUs != target){
    curUs += step;
    if (armed) { s.writeMicroseconds(curUs); delay(msPerStep); }
    else delay(1);
  }
}

void armNow(){
  if (armed) return;
  s.attach(SERVO_PIN);
  s.writeMicroseconds(curUs);
  armed = true;
  printEP("[ARMED]");
}

void disarmNow(){
  if (!armed) return;
  s.detach();
  armed = false;
  Serial.println(F("[DISARMED]"));
}

void setup(){
  Serial.begin(9600);
  loadEP();
  curUs = (ep.usOpen + ep.usClose) / 2;
  printEP("[BOOT]");
}

void loop(){
  if (Serial.available()){
    char c = Serial.read();

    if (c=='a' || c=='A') armNow();
    if (c=='x' || c=='X') disarmNow();

    if (c=='[') rampTo(curUs - FINE);
    if (c==']') rampTo(curUs + FINE);
    if (c=='-') rampTo(curUs - COARSE);
    if (c=='+') rampTo(curUs + COARSE);

    if (c=='u' || c=='U'){ int v = Serial.parseInt(); if (v>0) rampTo(v); printEP("[u]"); }

    if (c=='o' || c=='O'){ if (c=='O') { ep.usOpen = curUs; printEP("[SET OPEN]"); }
                           else if (armed) rampTo(ep.usOpen); }
    if (c=='c' || c=='C'){ if (c=='C') { ep.usClose = curUs; printEP("[SET CLOSE]"); }
                           else if (armed) rampTo(ep.usClose); }

    if (c=='S'){ saveEP(); Serial.println(F("[SAVED]")); }
    if (c=='p' || c=='P') printEP("[p]");
  }
}