

#include<MsTimer2.h>
//#include<math.h>
//#include "PinChangeInterrupt.h"

//encoder phase
#define EA_CHA_P1 31
#define EA_CHB_P1 18

#define EA_CHB_P4 2
#define EA_CHA_P4 A1

//Encoder
float en_P1_Pos = 0;  //Motor1 Port1
float en_P4_Pos = 0;  //Motor1 Port1

float current = 0;
float previous = 0;
long dt;
int rpm;

//PORT1
int PWM_DIR_P1 = 12;
int in_1_P1 = 34;
int in_2_P1 = 35;

//PORT4
int PWM_DIR_P4 = 5;
int in_1_P4 = A4;
int in_2_P4 = A5;


//ADC
long IR[4] = {0,};

int Max_IR = 0; // 최대값 for 노멀라이징
int Min_IR = 100; // 최소값 for 노멀라이징

long Nomal[4] = {0,};

int weight1 = 50; //가운데 weight
int weight2 = 150; //사이드 weight
float Pos = 0;

float rotate = 0;
float speedA = 0;
int target_speed = 20; // 1m/s
float errorA = 0;
float errorB = 0;
float pwmA = 0;
float pwmB = 0;

int target_Pos = 0;
int prev_Pos = 0; ///////////////////////////////////////////////////// previous Pos

float P_Gain_A = 2.5; //0.7 2.5
float P_Gain_B = 2.5; //0.7


void Position(void);
void Control(void);

void Sensing(void)
{
   int i =0;
   IR[0] = analogRead(A6);
   IR[1] = analogRead(A7);
   IR[2] = analogRead(A8);
   IR[3] = analogRead(A9);
  
   Max_IR = 570;
   Min_IR = 80;

   for(i=0; i<4;i++)
   {
      Nomal[i] = (IR[i] -  Min_IR) * 100 / (Max_IR - Min_IR);
   }
   /*
   Serial.print("nomal[0] : "); Serial.print(IR[0]); 
   Serial.print("   ");
   Serial.print("nomal[1] : ");
   Serial.print(IR[1]);Serial.print("   ");
   Serial.print("nomal[2] : ");
   Serial.print(IR[2]);Serial.print("   ");
   Serial.print("nomal[3] : ");
   Serial.print(IR[3]);Serial.println("   ");
   */
   /*
   Serial.print("nomal[0] : "); Serial.println(Nomal[0]); Serial.print("   ");
   Serial.print("nomal[1] : ");
   Serial.print(Nomal[1]);Serial.print("   ");
   Serial.print("nomal[2] : ");
   Serial.print(Nomal[2]);Serial.print("   ");
   Serial.print("nomal[3] : ");
   Serial.print(Nomal[3]);Serial.print("   ");
   */
   /*
   Serial.print("nomal[0] : "); Serial.println(IR[0] -  Min_IR); Serial.print("   ");
   Serial.print("nomal[1] : ");
   Serial.print(IR[1] -  Min_IR);Serial.print("   ");
   Serial.print("nomal[2] : ");
   Serial.print(IR[2] -  Min_IR);Serial.print("   ");
   Serial.print("nomal[3] : ");
   Serial.print(IR[3] -  Min_IR);Serial.print("   ");
   */
   /*
   Serial.print("IR_Max : "); Serial.println(Max_IR);
   Serial.print("IR_Min : "); Serial.println(Min_IR);
   */
   /*
   Serial.print("nomal[0]"); Serial.print("   ");
   Serial.print(Nomal[0]);Serial.print("   ");
   Serial.print("nomal0"); Serial.print("   ");
   Serial.print(IR[0] -  Min_IR);Serial.print("   ");
   Serial.print("IR[0]"); Serial.print("   ");
   Serial.print(Max_IR - Min_IR);Serial.println("   ");
   */
}

//void Control()
//{
//  
//  if(10+(IR[2]-IR[1])*0.1+(IR[3]-IR[2])*0.05>30) pwmA=20; 
//  
//  if(10+(IR[1]-IR[2])*0.1+(IR[0]-IR[1])*0.05>30) pwmB=20;
//  
//  else if(abs(prev_Pos-Pos)<40){
//    pwmA=10+(IR[2]-IR[1])*0.3+(IR[3]-IR[2])*0.2;
//    pwmB=10+(IR[1]-IR[2])*0.3+(IR[0]-IR[1])*0.2;
//    prev_Pos=Pos;
//  }
//  else if(IR[0]<50 && IR[3]<50){
//    pwmA=20;
//    pwmB=20;
//  }
//  else{
//    pwmA=20;
//    pwmB=20;
//  }
//  
//}








void Control()
{
  if(IR[0]<80 && IR[1]<80 && IR[2]<80 && IR[3]<80){
    pwmA= 80;
    pwmB= 80;
  }
  else if(IR[0]<80 && IR[1]<80){
    pwmA=80;
    pwmB=80;
  }
    
  else if(IR[2]<80 && IR[3]<80){
    pwmA= 80;
    pwmB= 80;
  }
//  
//    Serial.print(pwmA);
//      Serial.print("         ");
//      Serial.println(pwmB);
  else if(abs(prev_Pos-Pos)<50) ////////////////////////
  {
    prev_Pos = Pos;
    if((Pos > 0)&&(Pos<=70))
   {
      errorB = Pos - target_Pos;
      pwmA = 25;
      pwmB = errorB * P_Gain_B*2 + 25;
      
   }
   else if((Pos > 70)&&(Pos < 200))
   {
      errorB = 70 - target_Pos; //////////////////////////////////////////////////////
      //P_Gain_B = 3.5;
        pwmA =5;
        pwmB = errorB * P_Gain_B +15; //errorB
   }
   else if((Pos < -70)&&(Pos > -200))
   {
      errorA = 70 - target_Pos;
      //P_Gain_A = 3.5;
      pwmA = errorA * P_Gain_A +15; //errorA
      pwmB = 5;
   }
   else if((Pos < 0)&&(Pos >= -70))
   {
     errorA = -Pos + target_Pos;
     pwmA = errorA * P_Gain_A*2 + 25;
     pwmB = 25;
     } 

   else {
    pwmA = 80;
    pwmB = 80;
   }
    Serial.print(" Pos : "); Serial.print(Pos); Serial.println("   ");
  }
}
void TimerISR()
{
 // errorA = target_speed - en_P1_Pos;
 // errorB = target_speed - en_P4_Pos;
 // pwmA = errorA * P_Gain +  pwmA;
 // pwmB = errorB * P_Gain +  pwmB;
  //Serial.println(en_P1_Pos);
  
  Sensing();
  
  Position();
  
  Control();
  
  en_P1_Pos = 0; //무시해도 됨
  en_P4_Pos=0;   //무시해도 됨 
}


void setup() {
 
  pinMode(EA_CHA_P1,INPUT);
  pinMode(EA_CHB_P1,INPUT);
  
  pinMode(EA_CHA_P4,INPUT);
  pinMode(EA_CHB_P4,INPUT);

  
  pinMode(PWM_DIR_P1, OUTPUT);
  pinMode(in_1_P1, OUTPUT);
  pinMode(in_2_P1, OUTPUT);

  pinMode(PWM_DIR_P4, OUTPUT);
  pinMode(in_1_P4, OUTPUT);
  pinMode(in_2_P4, OUTPUT);
  
 // attachInterrupt(digitalPinToInterrupt(EA_CHB_P1),en_P1_chA_ISR,CHANGE);
 // attachInterrupt(digitalPinToInterrupt(EA_CHB_P4),en_P4_chA_ISR,CHANGE);
  MsTimer2::set(10, TimerISR);
  MsTimer2::start();
  Serial.begin(9600);
}

void loop() {
  
  digitalWrite(in_1_P1,LOW);   
  digitalWrite(in_2_P1,HIGH);
  analogWrite(PWM_DIR_P1, pwmA);

  digitalWrite(in_1_P4,HIGH);   
  digitalWrite(in_2_P4,LOW);
  analogWrite(PWM_DIR_P4, pwmB);
  
}
void en_P1_chA_ISR()
{
  if(digitalRead(EA_CHA_P1) == HIGH)
  {
    if(digitalRead(EA_CHB_P1) == LOW)
    {
      en_P1_Pos = en_P1_Pos - 1;
    }
    else
      en_P1_Pos = en_P1_Pos + 1;
  }
  else
  {
     if(digitalRead(EA_CHB_P1) == HIGH)
    {
      en_P1_Pos = en_P1_Pos - 1;
    }
    else
      en_P1_Pos = en_P1_Pos + 1;
  }
}
void en_P4_chA_ISR()
{
  if(digitalRead(EA_CHA_P4) == HIGH)
  {
    if(digitalRead(EA_CHB_P4) == LOW)
    {
      en_P4_Pos = en_P4_Pos - 1;
    }
    else
      en_P4_Pos = en_P4_Pos + 1;
  }
  else
  {
     if(digitalRead(EA_CHB_P4) == HIGH)
    {
      en_P4_Pos = en_P4_Pos - 1;
    }
    else
      en_P4_Pos = en_P4_Pos + 1;
  }
}

void Position()
{
  Pos =  ((Nomal[0] - Nomal[3]) * weight2 + (Nomal[1] - Nomal[2]) * weight1 )  / ( Nomal[0] + Nomal[1] + Nomal[2]+ Nomal[3] );    
  //Serial.print("Pos : "); Serial.println(Pos);
  
  
}


