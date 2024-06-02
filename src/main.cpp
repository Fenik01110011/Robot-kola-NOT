#include <Arduino.h>
//#include <NewPing.h>
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

//Line follower analog signals (LOW when detected black line or dont have return signal, if detected signal then HIGH), A0=S1, A1=S2, A2=S3, A3=S4, A4=S5, A5=Near-proximity sensor)
static const uint8_t lineFollowerPins[6] = {A0, A1, A2, A3, A4, A5};
bool lineFollowerPinsValue[6];

#define BASIC_CONTROLER_INPUT 1500
#define MAX_CONTROLER_INPUT 1900
#define MIN_CONTROLER_INPUT 1100
#define PULSEIN_TIMEOUT 25000

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
bool blackLineDetected = false;
int sensorResultant = 0;
int blackLaneTimeLost = 0;
int blackLaneTimeLostLimit = 100; //100pkt = 6 sec searching black lane in this configuration

//Ultrasonic distance sensor (0 - right sensor, 1 - left sensor)
int echo[2];
int trig[2];
int distanceReading[2];
int closeReading = 0;

long h_time, h_distance;

/*******************************************************************************
 * FUNCTIONS                                                                    *
 *******************************************************************************/

void hcsr_begin(int index, int echoPin, int trigPin);
float hcsr_getDistance(int index);
void manualControl();
void autoMode();

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
  

  //Sensor activate
  hcsr_begin (0, R_E_SENSOR, R_T_SENSOR);
  hcsr_begin (1,L_E_SENSOR, L_T_SENSOR);
  
  //Sensor distance return
}

// The loop routine runs over and over again forever.
void loop() {
  //Serial.println(pulseIn(PIN_CH4_CONTROLER, HIGH, PULSEIN_TIMEOUT));
  speedR = pulseIn(PIN_CH2_CONTROLER, HIGH, PULSEIN_TIMEOUT);
  speedL = pulseIn(PIN_CH3_CONTROLER, HIGH, PULSEIN_TIMEOUT);
  ch5Signal = pulseIn(PIN_CH5_CONTROLER, HIGH, PULSEIN_TIMEOUT);
  //ch6Signal = pulseIn(PIN_CH6_CONTROLER, HIGH, PULSEIN_TIMEOUT);
  ch7Signal = pulseIn(PIN_CH7_CONTROLER, HIGH, PULSEIN_TIMEOUT);

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
      manualControl();
    }
    else if(autoModStatus) {
      autoMode();
    }
    else{
      analogWrite(PIN_PWM_R, 0);
      analogWrite(PIN_PWM_L, 0);
    }
  }
  delay(10);
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
  noInterrupts();
  digitalWrite(trig[index], LOW);
  delayMicroseconds(2);
  digitalWrite(trig[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(trig[index], LOW);
  
  h_time = pulseIn(echo[index], HIGH, PULSEIN_TIMEOUT);
  interrupts();
  if(h_time <= 0 || h_time >= 23200) return 401;
  return h_time / 58.00;
}

void manualControl()
{
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

void autoMode()
{
  int speed[2]; //engin speed
  float speedPercentage = 1;
  if(speedPercentage < 0 || speedPercentage > 1)
  {
    speedPercentage = 1;
    Serial.print("Speed percentage error.");
  }

  for(int i = 0; i < 2; i++) 
  {
    distanceReading[i] = hcsr_getDistance(i);
    if(distanceReading[i] > 255)
      distanceReading[i] = 255;
    Serial.print(distanceReading[i]);
    Serial.print("\t");
  }
  Serial.print("\n");

  
  for(int i = 0; i <= 5; i++)
  {
    lineFollowerPinsValue[i] = digitalRead(lineFollowerPins[i]);
    //Serial.println(lineFollowerPinsValue[i]);
  }
  Serial.println(lineFollowerPinsValue[5]);

  blackLineDetected = false;
  for(int i = 0; i <= 4; i++)
    if(lineFollowerPinsValue[i] == false)
      blackLineDetected = true;

  if(ch5Signal < 1700 && blackLineDetected)
  {
    blackLaneTimeLost = 0;

    if(/*lineFollowerPinsValue[5] ||*/ distanceReading[0] < 20 || distanceReading[1] < 20)
    {
      speed[0] = 0;
      speed[1] = 0;
    }
    else
    {
      sensorResultant = lineFollowerPinsValue[0] + lineFollowerPinsValue[1] - lineFollowerPinsValue[3] - lineFollowerPinsValue[4];
      switch (sensorResultant)
      {
      case -2:
        {
          digitalWrite(PIN_DIR_R, HIGH);
          digitalWrite(PIN_DIR_L, LOW);
          speed[0] = 255;
          speed[1] = 255;
        }
        break;
      case -1:
        {
          digitalWrite(PIN_DIR_R, HIGH);
          digitalWrite(PIN_DIR_L, HIGH);
          speed[0] = 0;
          speed[1] = 255;
        }
        break;
      case 0:
        {
          digitalWrite(PIN_DIR_R, HIGH);
          digitalWrite(PIN_DIR_L, HIGH);
          speed[0] = 255;
          speed[1] = 255;
        }
        break;
      case 1:
        {
          digitalWrite(PIN_DIR_R, HIGH);
          digitalWrite(PIN_DIR_L, HIGH);
          speed[0] = 255;
          speed[1] = 0;
        }
        break;
      case 2:
        {
          digitalWrite(PIN_DIR_R, LOW);
          digitalWrite(PIN_DIR_L, HIGH);
          speed[0] = 255;
          speed[1] = 255;
        }
        break;
      default:
        {
          Serial.print("Line Follower switch error.");
        }
        break;
      }
    }
  }
  else
  {
    if(ch5Signal < 1700 && (ch5Signal < 1300 || blackLaneTimeLost < blackLaneTimeLostLimit))
    {
      if(sensorResultant >= 0)
      {
        digitalWrite(PIN_DIR_R, LOW);
        digitalWrite(PIN_DIR_L, HIGH);
        speed[0] = 255;
        speed[1] = 255;
      }
      else
      {
        digitalWrite(PIN_DIR_R, HIGH);
        digitalWrite(PIN_DIR_L, LOW);
        speed[0] = 255;
        speed[1] = 255;
      }
      blackLaneTimeLost++;
      //Serial.println(blackLaneTimeLost);
    }
    else
    {
      if(distanceReading[0] < 40 || distanceReading[1] < 40) //|| lineFollowerPinsValue[5])
      {
        digitalWrite(PIN_DIR_R, LOW);
        digitalWrite(PIN_DIR_L, LOW);
        speed[0] = 128;
        speed[1] = 128;
        analogWrite(PIN_PWM_R, speed[1]*speedPercentage);
        analogWrite(PIN_PWM_L, speed[0]*speedPercentage);
        delay(500);
        digitalWrite(PIN_DIR_R, LOW);
        digitalWrite(PIN_DIR_L, HIGH);
        speed[0] = 255;
        speed[1] = 255;
        analogWrite(PIN_PWM_R, speed[1]*speedPercentage);
        analogWrite(PIN_PWM_L, speed[0]*speedPercentage);
        delay(300);
      }
      else if(distanceReading[0] >= 255 && distanceReading[1] >= 255)
      {
        digitalWrite(PIN_DIR_R, HIGH);
        digitalWrite(PIN_DIR_L, HIGH);
        speed[0] = 255;
        speed[1] = 255;
      }
      else if(distanceReading[0] < 80 || distanceReading[1] < 80) 
      {
        
        digitalWrite(PIN_DIR_R, LOW);
        digitalWrite(PIN_DIR_L, HIGH);
        speed[0] = 255;
        speed[1] = 255;
        analogWrite(PIN_PWM_R, speed[1]*speedPercentage);
        analogWrite(PIN_PWM_L, speed[0]*speedPercentage);
        delay(300);
      }
      else
      {
        digitalWrite(PIN_DIR_R, HIGH);
        digitalWrite(PIN_DIR_L, HIGH);
        if(distanceReading[0] >= distanceReading[1])
        {
          speed[0] = 255;
          speed[1] = 255 - (255 - distanceReading[1]);
        }
        else
        {
          speed[0] = 255 - (255 - distanceReading[0]);
          speed[1] = 255;
        }
      }
    }
  }
  
  analogWrite(PIN_PWM_R, speed[1]*speedPercentage);
  analogWrite(PIN_PWM_L, speed[0]*speedPercentage);
}
        
      //   // if(ch6Signal >= MAX_CONTROLER_INPUT){
      //   //   analogWrite(PIN_PWM_R, 255);
      //   //   analogWrite(PIN_PWM_L, 255);
      //   // }
      //   // else if(ch6Signal <= MIN_CONTROLER_INPUT){
      //   //   analogWrite(PIN_PWM_R, 0);
      //   //   analogWrite(PIN_PWM_L, 0);
      //   // }
      //   // else{
      //   //   analogWrite(PIN_PWM_R, abs((ch6Signal - 1000)/4));
      //   //   analogWrite(PIN_PWM_L, abs((ch6Signal - 1000)/4));
      //   // }
      // }