#include <Arduino.h>
/*******************************************************************************
 * IO DEFINITION                                                                *
 *******************************************************************************/

// PWM is connected to pin 3 and 5.
#define PIN_PWM_R 3
#define PIN_PWM_L 5

// PWN controler
#define PIN_CH1_CONTROLER 6 //prawy joystick, sterowanie na boki
#define PIN_CH2_CONTROLER 7 //prawy joystick, predkosc prawych kol
#define PIN_CH3_CONTROLER 8 //lewy joystick, predkosc lewych kol
#define PIN_CH4_CONTROLER 9 //lewy joystick, sterowanie na boki
#define PIN_CH5_CONTROLER 10 //obracanie sie lazika
#define PIN_CH6_CONTROLER 11 //szybkosc autoatycznego obracania sie lazika odnoscie CH5 
#define PIN_CH7_CONTROLER 12 
#define PIN_CH8_CONTROLER 13

// DIR is connected to pin 2 and 4.
#define PIN_DIR_R 2
#define PIN_DIR_L 4

//photoresistor
#define R_PIN A0
#define L_PIN A1

int defaultLightIntensity = 0;
int rPinLightIntensity = 0;
int lPinLightIntensity = 0;

#define BASIC_CONTROLER_INPUT 1500
#define MAX_CONTROLER_INPUT 1900
#define MIN_CONTROLER_INPUT 1100

int speedR = 0;
int speedL = 0;

int ch5Signal;
int ch6Signal;
int ch7Signal;

int autoModStatus = 0;

/*******************************************************************************
 * FUNCTIONS                                                                    *
 *******************************************************************************/

// The setup routine runs once when you press reset.
void setup() {
  Serial.begin(9600);
  // Initialize the PWM and DIR pins as digital outputs.
  pinMode(PIN_PWM_R, OUTPUT);
  pinMode(PIN_PWM_L, OUTPUT);
  pinMode(PIN_DIR_R, OUTPUT);
  pinMode(PIN_DIR_L, OUTPUT);

  pinMode(PIN_CH1_CONTROLER, INPUT);
  pinMode(PIN_CH2_CONTROLER, INPUT);
  pinMode(PIN_CH3_CONTROLER, INPUT);
  pinMode(PIN_CH4_CONTROLER, INPUT);
  pinMode(PIN_CH5_CONTROLER, INPUT);
  pinMode(PIN_CH6_CONTROLER, INPUT);
  pinMode(PIN_CH7_CONTROLER, INPUT);
  pinMode(PIN_CH8_CONTROLER, INPUT);

  digitalWrite(PIN_DIR_R, HIGH);
  digitalWrite(PIN_DIR_L, HIGH);

  pinMode(R_PIN, INPUT);
  pinMode(L_PIN, INPUT);
  defaultLightIntensity = (analogRead(R_PIN) + analogRead(L_PIN))/ 2;
}

// The loop routine runs over and over again forever.
void loop() {
    //Serial.println(pulseIn(PIN_CH4_CONTROLER, HIGH));
    speedR = pulseIn(PIN_CH2_CONTROLER, HIGH);
    speedL = pulseIn(PIN_CH3_CONTROLER, HIGH);
    ch5Signal = pulseIn(PIN_CH5_CONTROLER, HIGH);
    ch6Signal = pulseIn(PIN_CH6_CONTROLER, HIGH);
    ch7Signal = pulseIn(PIN_CH7_CONTROLER, HIGH);

    //Ustawienia automatycznego dzialania lazika
    if(ch7Signal < 1300)
      autoModStatus = 1;
    else
      autoModStatus = 0;

    //zatrzymanie lazika jesli kontroler jest wylaczony
    if(speedR < 930 || speedR > 2070 || speedL < 930 || speedL > 2070 || ((ch7Signal > 1300 && ch7Signal < 1700) || ch7Signal < 800))
    {
      analogWrite(PIN_PWM_R, 0);
      analogWrite(PIN_PWM_L, 0);
    }
    else {
      if(autoModStatus == 0)
      {
        if(speedR >= (BASIC_CONTROLER_INPUT - 50) && speedR <= (BASIC_CONTROLER_INPUT + 50) && speedL >= (BASIC_CONTROLER_INPUT - 50) && speedL <= (BASIC_CONTROLER_INPUT + 50)){
          rPinLightIntensity = analogRead(R_PIN);
          lPinLightIntensity = analogRead(L_PIN);
          
          if(rPinLightIntensity > defaultLightIntensity + 50){
            digitalWrite(PIN_DIR_R, HIGH);
            
            if((rPinLightIntensity - (defaultLightIntensity + 50)) < 255){
              analogWrite(PIN_PWM_R, rPinLightIntensity - (defaultLightIntensity + 50));
            }
            else{
              analogWrite(PIN_PWM_R, 255);
            }
          }
          else{
            analogWrite(PIN_PWM_R, 0);
          }
          
          if(lPinLightIntensity > defaultLightIntensity + 50){
            digitalWrite(PIN_DIR_L, HIGH);
            
            if((lPinLightIntensity - (defaultLightIntensity + 50)) < 255){
              analogWrite(PIN_PWM_L, lPinLightIntensity - (defaultLightIntensity + 50));
            }
            else{
              analogWrite(PIN_PWM_L, 255);
            }
          }
          else{
            analogWrite(PIN_PWM_L, 0);
          }

          //obracanie sie lazika w wybranym kierunku za pomoca CH5 z predkoscia CH6
          if(ch5Signal < 1300 || ch5Signal > 1700){
            if(ch5Signal < 1300){
              digitalWrite(PIN_DIR_R, HIGH);
              digitalWrite(PIN_DIR_L, LOW);
            }
            if(ch5Signal > 1700){
              digitalWrite(PIN_DIR_R, LOW);
              digitalWrite(PIN_DIR_L, HIGH);
            }
            
            if(ch6Signal >= MAX_CONTROLER_INPUT){
              analogWrite(PIN_PWM_R, 255);
              analogWrite(PIN_PWM_L, 255);
            }
            else if(ch6Signal <= MIN_CONTROLER_INPUT){
              analogWrite(PIN_PWM_R, 0);
              analogWrite(PIN_PWM_L, 0);
            }
            else{
              analogWrite(PIN_PWM_R, abs((ch6Signal - 1000)/4));
              analogWrite(PIN_PWM_L, abs((ch6Signal - 1000)/4));
            }
          }
        }
        else{
          if(speedR <= MIN_CONTROLER_INPUT){
            analogWrite(PIN_PWM_R, 255);
            digitalWrite(PIN_DIR_R, LOW);
          }
          else if(speedR >= MAX_CONTROLER_INPUT){
            analogWrite(PIN_PWM_R, 255);
            digitalWrite(PIN_DIR_R, HIGH);
          }
          else {
            if((speedR <= BASIC_CONTROLER_INPUT)){
              analogWrite(PIN_PWM_R, abs(speedR - BASIC_CONTROLER_INPUT)/2);
              digitalWrite(PIN_DIR_R, LOW);
            }
            else {
              analogWrite(PIN_PWM_R, (speedR - BASIC_CONTROLER_INPUT)/2);
              digitalWrite(PIN_DIR_R, HIGH);
            }
          }
          
          if(speedL <= MIN_CONTROLER_INPUT){
            analogWrite(PIN_PWM_L, 255);
            digitalWrite(PIN_DIR_L, LOW);
          }
          else if(speedL >= MAX_CONTROLER_INPUT){
            analogWrite(PIN_PWM_L, 255);
            digitalWrite(PIN_DIR_L, HIGH);
          }
          else {
            if((speedL <= BASIC_CONTROLER_INPUT)){
              analogWrite(PIN_PWM_L, abs(speedL - BASIC_CONTROLER_INPUT)/2);
              digitalWrite(PIN_DIR_L, LOW);
            }
            else {
              analogWrite(PIN_PWM_L, (speedL - BASIC_CONTROLER_INPUT)/2);
              digitalWrite(PIN_DIR_L, HIGH);
            }
          }
        }
      }
    }

    delay(10);
}
