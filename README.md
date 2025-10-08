# Development and Experimental Evaluation of Gecko-Inspired Robotic Gripper

## Code Procedures
1) **Force_Sensors_Calibration** → To calibrate FSRs with known weights; update M/B in code.
2) **Setting_Servo_Limits** → To set safe OPEN/CLOSE pulses (e.g., 2100/1500 μs) and save to EEPROM.
3) **Gripper_Operation_Testing** → To run integrated test: grip, read forces, show on LCD, log results.

## Required Libraries
- LiquidCrystal_I2C, Wire, EEPROM

## Wiring (Arduino Uno)
- Servo PWM → D9 (servo power from external 6 V, common GND)
- FSR Left → A0 (voltage divider 4.7k-ohm)
- FSR Right → A1 (voltage divider 4.7k-ohm)
- LCD SDA → A4, LCD SCL → A5

## Procedure

 **Power On:**
     -	Servo remains detached (no movement on startup).
     -	Arduino loads previous calibration and servo limit values from EEPROM.
 **Load Configuration:**
     -	Retrieve FSR calibration constants (M, B) and servo endpoint limits (OPEN/CLOSE µs).
 **Sensor Reading:**
     -	Continuously read force from Left (A0) and Right (A1) FSRs.
     -	Convert analog readings to Force (N) using calibration data.
     -	If total force exceeds predefined maximum → block servo closing.
 **Servo Activation (ARM):**
     -	Attach servo to enable motion.
     -	Manually adjust servo pulse width to control jaw opening/clamping.
     -	LCD displays real-time values:
             o	Total Force (N)
             o	Current servo pulse (µs)
             o	System status (ARM / DISARM)
 **Data Recording:**
     -	Record readings for each test object and jaw type.
     -	Include parameters such as object ID, pulse (µs), forces (N), slip, and surface marks.
 **Iteration:**
     -	Replace current jaw type (Gecko / Soft / Hard).
     -	Repeat measurement and recording for each configuration.
     -	Compare performance results across jaw types.



## By:
     Michael George - M00979224
     Supervisor: Dr. Judhi Prasetyo
     MSc Robotics, Middlesex University Dubai  

