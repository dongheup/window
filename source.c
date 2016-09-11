T/*****************************************************************************
*
* Copyright (C) 2016 Diwell Electronics Co.,Ltd.
* Project Name : PM1001 UART Code <softwareserial 이용>
* Version : 1.0 (2016.05.04)
* SYSTEM CLOCK : 16Mhz 
* BOARD : Arduino UNO. 5V operation 


 PORT Description

1. RX : 13           
2. TX : 11
  먼지센서 전원은 5V로 하셔야 하며 포트 연결 방법은 회로도를 참고하십시오.

 Revision history.

1. 2016.5.4  : First version is released.
****************************************************************************/

#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include "DRV8835MotorShield.h"

SoftwareSerial mySerial(13, 4);                        // RX 13, TX 11z
AltSoftSerial altSerial; 

unsigned char Send_data[4] = {0x11,0x01,0x01,0xED};       // 읽는명령
unsigned char Receive_Buff[16];                           // data buffer
unsigned long PCS;                                        // 수량 저장 변수 
float ug;                                                 // 농도 저장 변수 
unsigned char recv_cnt = 0;

int _M1DIR = 7;
int _M2DIR = 11;
int _M1PWM = 12;
int _M2PWM = 10;
boolean DRV8835MotorShield::_flipM1 = false;
boolean DRV8835MotorShield::_flipM2 = false;

//char window_mode = 2;


void Send_CMD(void)                                        // COMMAND
{
  unsigned char i;
  for(i=0; i<4; i++)
  {
    mySerial.write(Send_data[i]);
    delay(1);      // Don't delete this line !!
  }
}
unsigned char Checksum_cal(void)                          // CHECKSUM 
{
  unsigned char count, SUM=0;
  for(count=0; count<15; count++)
  {
     SUM += Receive_Buff[count];
  }
  return 256-SUM;
}

void setup() {
  pinMode(13,INPUT);
  pinMode(4,OUTPUT);

  //pinMode(0,INPUT);
  //pinMode(1,OUTPUT);

  pinMode(7, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  
  Serial.begin(9600);
  while (!Serial) ;
  altSerial.begin(9600);
  mySerial.begin(9600);
  while (!mySerial);
}






void DRV8835MotorShield::initPinsAndMaybeTimer()
{
  // Initialize the pin states used by the motor driver shield
  // digitalWrite is called before and after setting pinMode.
  // It called before pinMode to handle the case where the board
  // is using an ATmega AVR to avoid ever driving the pin high, 
  // even for a short time.
  // It is called after pinMode to handle the case where the board
  // is based on the Atmel SAM3X8E ARM Cortex-M3 CPU, like the Arduino
  // Due. This is necessary because when pinMode is called for the Due
  // it sets the output to high (or 3.3V) regardless of previous
  // digitalWrite calls.
  digitalWrite(_M1PWM, LOW);
  digitalWrite(_M1PWM, LOW);
  digitalWrite(_M2PWM, LOW);
  digitalWrite(_M2PWM, LOW);
  digitalWrite(_M1DIR, LOW);
  digitalWrite(_M1DIR, LOW);
  digitalWrite(_M2DIR, LOW);
  digitalWrite(_M2DIR, LOW);
#ifdef DRV8835MOTORSHIELD_USE_20KHZ_PWM
  // timer 1 configuration
  // prescaler: clockI/O / 1
  // outputs enabled
  // phase-correct PWM
  // top of 400
  //
  // PWM frequency calculation
  // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
  TCCR1A = 0b10100000;
  TCCR1B = 0b00010001;
  ICR1 = 400;
#endif
}

// speed should be a number between -400 and 400
void DRV8835MotorShield::setM1Speed(int speed)
{
  init(); // initialize if necessary
    
  boolean reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1;    // preserve the direction
  }
  if (speed > 400)  // max 
    speed = 400;
    
#ifdef DRV8835MOTORSHIELD_USE_20KHZ_PWM
  OCR1A = speed;
#else
  analogWrite(_M1PWM, speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
#endif 

  if (reverse ^ _flipM1) // flip if speed was negative or _flipM1 setting is active, but not both
    digitalWrite(_M1DIR, HIGH);
  else
    digitalWrite(_M1DIR, LOW);
}

// speed should be a number between -400 and 400
void DRV8835MotorShield::setM2Speed(int speed)
{
  init(); // initialize if necessary
    
  boolean reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // make speed a positive quantity
    reverse = 1;  // preserve the direction
  }
  if (speed > 400)  // max PWM duty cycle
    speed = 400;
    
#ifdef DRV8835MOTORSHIELD_USE_20KHZ_PWM
  OCR1B = speed;
#else
  analogWrite(_M2PWM, speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
#endif

  if (reverse ^ _flipM2) // flip if speed was negative or _flipM2 setting is active, but not both
    digitalWrite(_M2DIR, HIGH);
  else
    digitalWrite(_M2DIR, LOW);
}

// set speed for both motors
// speed should be a number between -400 and 400
void DRV8835MotorShield::setSpeeds(int m1Speed, int m2Speed){
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

void DRV8835MotorShield::flipM1(boolean flip)
{
  _flipM1 = flip;
}

void DRV8835MotorShield::flipM2(boolean flip)
{
  _flipM2 = flip;
}


uint32_t timer = millis();
uint32_t timer2 = millis();

/*
void loop()
{
        digitalWrite(_M2DIR, HIGH);
        digitalWrite(_M2PWM, 250);
}
*/

void loop() {


 int d_time = 1000;
  int o_time = 1000;
  int window_speed = 1023;

  

  if(altSerial.available())
   {
    char window_mode = altSerial.read();
    switch(window_mode){
      case '2' :
        
        digitalWrite(7, HIGH);
        digitalWrite(12, LOW);
        delay(o_time);
        digitalWrite(7, LOW);
        digitalWrite(12, LOW); 
      break;
      case '3' : 
        
        digitalWrite(7, LOW);
        digitalWrite(12, HIGH);
        delay(o_time);
        digitalWrite(7, LOW);
        digitalWrite(12, LOW);
      break;


      case '5' :
        Serial.write("5");
        digitalWrite(10, HIGH);
        digitalWrite(11, LOW);
        delay(o_time);
        digitalWrite(10, LOW);
        digitalWrite(11, LOW); 
      break;
      case '6' : 
        Serial.write("6");
        digitalWrite(10, LOW);
        digitalWrite(11, HIGH);
        delay(o_time);
        digitalWrite(10, LOW);
        digitalWrite(11, LOW);
      break;
    }
   
   }
   
    

  if (timer > millis())  timer = millis();
  if (timer2 > millis())  timer2 = millis();

  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer
    
    Send_CMD();  // Send Read Command

  while(1)
  {
    if(mySerial.available())
    { 
       Receive_Buff[recv_cnt++] = mySerial.read();
      if(recv_cnt ==16){recv_cnt = 0; break;}
    }
  }
      
      
  }

  
  
  
  if (millis() - timer2 > 3000) {
    timer2 = millis(); // reset the timer

    if(Checksum_cal() == Receive_Buff[15])  // CS 확인을 통해 통신 에러 없으면
  {
        PCS = (unsigned long)Receive_Buff[3]<<24 | (unsigned long)Receive_Buff[4]<<16 | (unsigned long)Receive_Buff[5]<<8| (unsigned long)Receive_Buff[6];  // 수량 
        ug = (float)PCS*3528/100000; // 농도 변환(이 식은 PM1001 모델만 적용됩니다.)
        Serial.write("PCS : ");
        Serial.print(PCS);
        
        Serial.write(",  ug : ");
        Serial.println(ug);

        //String ch = String(ug);
        altSerial.println(ug);
        
   }
   
      
      
  }
  
  
   else
   {
     //Serial.write("CHECKSUM Error");
   }
   delay(100);       //1000ms
}
