#include <Arduino.h>
/*******************************************************************************
 * IO DEFINITION                                                                *
 *******************************************************************************/

// PWM is connected to pin 3 and 5.
#define PIN_PWM_R 3
#define PIN_PWM_L 5

// PWN controler
//#define PIN_CH1_CONTROLER 6 //prawy czujnik,
#define PIN_CH2_CONTROLER 7 //prawy joystick, predkosc prawych kol
#define PIN_CH3_CONTROLER 8 //lewy joystick, predkosc lewych kol
//#define PIN_CH4_CONTROLER 9 //lewy czujnik
#define PIN_CH5_CONTROLER 10 //obracanie sie lazika
//#define PIN_CH6_CONTROLER 11 //szybkosc autoatycznego obracania sie lazika odnoscie CH5 
#define PIN_CH7_CONTROLER 12 
//#define PIN_CH8_CONTROLER 13

// DIR is connected to pin 2 and 4.
#define PIN_DIR_R 2
#define PIN_DIR_L 4

//photoresistor
#define R_PIN A0
#define L_PIN A1

#define BASIC_CONTROLER_INPUT 1500
#define MAX_CONTROLER_INPUT 1900
#define MIN_CONTROLER_INPUT 1100

//Sensor connection
#define R_T_SENSOR 9
#define R_E_SENSOR 6
#define L_T_SENSOR 13
#define L_E_SENSOR 11

int defaultLightIntensity = 0;
int rPinLightIntensity = 0;
int lPinLightIntensity = 0;

int speedR = 0;
int speedL = 0;

int ch5Signal;
//int ch6Signal;
int ch7Signal;

bool autoModStatus = false;

//Ultrasonic distance sensor (0 - right sensor, 1 - left sensor)
int echo[2];
int trig[2];
int distanceReading[2];

long h_time, h_distance;

/*******************************************************************************
 * FUNCTIONS                                                                    *
 *******************************************************************************/

void hcsr_begin(int index, int echoPin, int trigPin);
float hcsr_getDistance(int index);

// The setup routine runs once when you press reset.
void setup() {
  Serial.begin(9600);
  // Initialize the PWM and DIR pins as digital outputs.
  pinMode(PIN_PWM_R, OUTPUT);
  pinMode(PIN_PWM_L, OUTPUT);
  pinMode(PIN_DIR_R, OUTPUT);
  pinMode(PIN_DIR_L, OUTPUT);

  //pinMode(PIN_CH1_CONTROLER, INPUT);
  pinMode(PIN_CH2_CONTROLER, INPUT);
  pinMode(PIN_CH3_CONTROLER, INPUT);
  //pinMode(PIN_CH4_CONTROLER, INPUT);
  pinMode(PIN_CH5_CONTROLER, INPUT);
  //pinMode(PIN_CH6_CONTROLER, INPUT);
  pinMode(PIN_CH7_CONTROLER, INPUT);
  //pinMode(PIN_CH8_CONTROLER, INPUT);

  digitalWrite(PIN_DIR_R, HIGH);
  digitalWrite(PIN_DIR_L, HIGH);

  pinMode(R_PIN, INPUT);
  pinMode(L_PIN, INPUT);
  defaultLightIntensity = (analogRead(R_PIN) + analogRead(L_PIN))/ 2;

  //Sensor activate
  hcsr_begin (0, R_E_SENSOR, R_T_SENSOR);
  hcsr_begin (1,L_E_SENSOR, L_T_SENSOR);
  
  //Sensor distance return
}

// The loop routine runs over and over again forever.
void loop() {
  //Serial.println(pulseIn(PIN_CH4_CONTROLER, HIGH));
  speedR = pulseIn(PIN_CH2_CONTROLER, HIGH);
  speedL = pulseIn(PIN_CH3_CONTROLER, HIGH);
  ch5Signal = pulseIn(PIN_CH5_CONTROLER, HIGH);
  //ch6Signal = pulseIn(PIN_CH6_CONTROLER, HIGH);
  ch7Signal = pulseIn(PIN_CH7_CONTROLER, HIGH);

  //Ustawienia automatycznego dzialania lazika
  if(ch7Signal < 1300)
    autoModStatus = true;
  else
    autoModStatus = false;

  //zatrzymanie lazika jesli kontroler jest wylaczony
  if(speedR < 930 || speedR > 2070 || speedL < 930 || speedL > 2070 || ((ch7Signal > 1300 && ch7Signal < 1700) || ch7Signal < 800)) {
    analogWrite(PIN_PWM_R, 0);
    analogWrite(PIN_PWM_L, 0);
  }
  else {
    if(speedR > (BASIC_CONTROLER_INPUT + 50) || speedR < (BASIC_CONTROLER_INPUT - 50) || speedL > (BASIC_CONTROLER_INPUT + 50) || speedL < (BASIC_CONTROLER_INPUT - 50)) {
      if(speedR <= MIN_CONTROLER_INPUT) {
        analogWrite(PIN_PWM_R, 255);
        digitalWrite(PIN_DIR_R, LOW);
      }
      else if(speedR >= MAX_CONTROLER_INPUT) {
        analogWrite(PIN_PWM_R, 255);
        digitalWrite(PIN_DIR_R, HIGH);
      }
      else {
        if((speedR <= BASIC_CONTROLER_INPUT)) {
          analogWrite(PIN_PWM_R, abs(speedR - BASIC_CONTROLER_INPUT)/2);
          digitalWrite(PIN_DIR_R, LOW);
        }
        else {
          analogWrite(PIN_PWM_R, (speedR - BASIC_CONTROLER_INPUT)/2);
          digitalWrite(PIN_DIR_R, HIGH);
        }
      }
      
      if(speedL <= MIN_CONTROLER_INPUT) {
        analogWrite(PIN_PWM_L, 255);
        digitalWrite(PIN_DIR_L, LOW);
      }
      else if(speedL >= MAX_CONTROLER_INPUT) {
        analogWrite(PIN_PWM_L, 255);
        digitalWrite(PIN_DIR_L, HIGH);
      }
      else {
        if((speedL <= BASIC_CONTROLER_INPUT)) {
          analogWrite(PIN_PWM_L, abs(speedL - BASIC_CONTROLER_INPUT)/2);
          digitalWrite(PIN_DIR_L, LOW);
        }
        else {
          analogWrite(PIN_PWM_L, (speedL - BASIC_CONTROLER_INPUT)/2);
          digitalWrite(PIN_DIR_L, HIGH);
        }
      }
    }
    else if(autoModStatus) {
      //obracanie sie lazika w wybranym kierunku za pomoca CH5 z predkoscia CH6
      if(ch5Signal < 1300 || ch5Signal > 1700){
        if(ch5Signal < 1300){
          digitalWrite(PIN_DIR_R, HIGH);
          digitalWrite(PIN_DIR_L, LOW);
        }
        else if(ch5Signal > 1700){
          digitalWrite(PIN_DIR_R, LOW);
          digitalWrite(PIN_DIR_L, HIGH);
        }

        analogWrite(PIN_PWM_R, 255);
        analogWrite(PIN_PWM_L, 255);
        
        // if(ch6Signal >= MAX_CONTROLER_INPUT){
        //   analogWrite(PIN_PWM_R, 255);
        //   analogWrite(PIN_PWM_L, 255);
        // }
        // else if(ch6Signal <= MIN_CONTROLER_INPUT){
        //   analogWrite(PIN_PWM_R, 0);
        //   analogWrite(PIN_PWM_L, 0);
        // }
        // else{
        //   analogWrite(PIN_PWM_R, abs((ch6Signal - 1000)/4));
        //   analogWrite(PIN_PWM_L, abs((ch6Signal - 1000)/4));
        // }
      }
      else {
        int speed[2];
        digitalWrite(PIN_DIR_R, HIGH);
        digitalWrite(PIN_DIR_L, HIGH);

        for(int i = 0; i < 2; i++)
        {
          distanceReading[i] = hcsr_getDistance(i);
          Serial.print(distanceReading[i]);
          Serial.print("\t");

          if(distanceReading[i] >= 255)
            speed[i] = 255;
          else if(distanceReading[i] < 20)
          {
            for(int j = 0; j < 2; j ++)
              speed[j] = 0;
            break;
          }
          else if(distanceReading[i] < 40)
            speed[i] = 0;
          else
            speed[i] = 255 - (255 - distanceReading[i]);
          
          delay(30);
        }
        Serial.print("\n");

        analogWrite(PIN_PWM_R, speed[1]);
        analogWrite(PIN_PWM_L, speed[0]);
        // rPinLightIntensity = analogRead(R_PIN);
        // lPinLightIntensity = analogRead(L_PIN);
        
        // if(rPinLightIntensity > defaultLightIntensity + 50){
        //   digitalWrite(PIN_DIR_R, HIGH);
          
        //   if((rPinLightIntensity - (defaultLightIntensity + 50)) < 255){
        //     analogWrite(PIN_PWM_R, rPinLightIntensity - (defaultLightIntensity + 50));
        //   }
        //   else{
        //     analogWrite(PIN_PWM_R, 255);
        //   }
        // }
        // else{
        //   analogWrite(PIN_PWM_R, 0);
        // }
        
        // if(lPinLightIntensity > defaultLightIntensity + 50){
        //   digitalWrite(PIN_DIR_L, HIGH);
          
        //   if((lPinLightIntensity - (defaultLightIntensity + 50)) < 255){
        //     analogWrite(PIN_PWM_L, lPinLightIntensity - (defaultLightIntensity + 50));
        //   }
        //   else{
        //     analogWrite(PIN_PWM_L, 255);
        //   }
        // }
        // else{
        //   analogWrite(PIN_PWM_L, 0);
        // }
      }
    }
    else{
      analogWrite(PIN_PWM_R, 0);
      analogWrite(PIN_PWM_L, 0);
    }
  }
  delay(10);

  // if(!hcsr_getDistance(0)) Serial.print("ERROR");
  // else Serial.print(hcsr_getDistance(0));

  // Serial.print("\t");
  // delay(30);
  // if(!hcsr_getDistance(1)) Serial.print("ERROR");
  // else Serial.print(hcsr_getDistance(1));
  // Serial.print("\n");
}

void hcsr_begin(int index, int echoPin, int trigPin)
{
  echo [index] = echoPin;
  trig [index] = trigPin;

  pinMode(echo[index], INPUT);
  pinMode(trig[index], OUTPUT);

}

float hcsr_getDistance(int index)
{
  digitalWrite(trig[index], LOW);
  delayMicroseconds(2);
  digitalWrite(trig[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(trig[index], LOW);
  
  h_time = pulseIn(echo[index], HIGH);
  if(h_time < 0 || h_time >= 23200) return 0;
  return h_time / 58.00;
}