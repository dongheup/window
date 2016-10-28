#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
SoftwareSerial mySerial(13, 4);                        // RX 13, TX 11z
AltSoftSerial altSerial; 

unsigned char Send_data[4] = {0x11,0x01,0x01,0xED};       // 읽는명령
unsigned char Receive_Buff[16];                           // data buffer
unsigned long PCS;                                        // 수량 저장 변수 
float ug;       // 농도 저장 변수
float airug;    // 대기중 미세먼지농도
unsigned int ug_int;
unsigned char recv_cnt = 0;

int _M1DIR = 7;
int _M2DIR = 11;
int _M1PWM = 12;
int _M2PWM = 10;


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
  pinMode(5,OUTPUT);


  Serial.begin(9600);
  //while (!Serial) ;
  altSerial.begin(9600);
  mySerial.begin(9600);
  while (!mySerial);
}








uint32_t timer = millis();
uint32_t timer2 = millis();

void loop() {

  //digitalWrite(8, HIGH);
  char window_mode;
  char window_mode_cf;
   if(altSerial.available()) {   
    window_mode = altSerial.read();
   }
    //Serial.println(window_mode);
   switch(window_mode){
      case '2' :
       Serial.println("close"); 
        digitalWrite(5, LOW);    //닫기
      break;
      case '3' : 
        Serial.println("open");
        digitalWrite(5, HIGH);   //열기
     break;
      case '5' :
        Serial.print("airug: ");
        airug = altSerial.read();
        Serial.println(airug);
     break;
      case '6' : 
      while(window_mode == '6'){
        Serial.println("Automatic ventilation");
        if(ug>airug){ digitalWrite(5, HIGH); Serial.println("open");}   //열기
        else{digitalWrite(5, LOW); Serial.println("close");}             //닫기
        if(altSerial.available()) {   
         window_mode = altSerial.read();
   }
      }
      break;
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
        //ug_int = (int)ug; 
        //String ch = String(ug_int);
        //altSerial.println(ug_int);
        String ch = String(ug);
        altSerial.println(ug);
        //Serial.println(window_mode_cf);
   }
   
      
      
  }
  
  
   else
   {
     //Serial.write("CHECKSUM Error");
   }
   delay(1500);       //1000ms
}