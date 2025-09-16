//INTERRUPTS IN STM32F103C8
#include <Servo.h>
Servo PWM1;  //3.NM
Servo PWM2;  //6.9NM
Servo PWM3;  //9.7NM

//Motor 1 par 3.4 NM
const int AA1 = PA5; // Entrada de la señal A del encoder.
const int BB1 = PA6; // Entrada de la señal B del encoder.
const int CC1 = PA7; // Entrada de la señal C del encoder.

//Motor 1 par 6.9 NM
const int AA2 = PC13; // Entrada del encoder.
const int BB2 = PC14; 
const int CC2 = PC15; 

//Motor 1 par 9.7 NM
const int AA3 = PA1; // Entrada del encoder.
const int BB3 = PA2; 
const int CC3 = PA3; 

volatile int  n1 = 0, n2 = 0,n3 = 0 ;
volatile byte ant1 = 0, ant2 = 0, ant3 = 0;
volatile byte act1 = 0, act2 = 0, act3 = 0;

float grados1 = 0.0, grados1_ant=0.0, vel1=0.0, int_grados1=0.0;
float grados2 = 0.0, grados2_ant=0.0, vel2=0.0, int_grados2=0.0;
float grados3 = 0.0, grados3_ant=0.0, vel3=0.0, int_grados3=0.0;

float pos_des1=-90, pid1=0.0, e1=0.0, e1_p=0.0, e1_i=0.0;
float pos_des2=-90, pid2=0.0, e2=0.0, e2_p=0.0, e2_i=0.0;
float pos_des3=-90, pid3=0.0, e3=0.0, e3_p=0.0, e3_i=0.0;

float kp1 = 3.4, kd1 = 1.0, ki1 = 1.0;
float kp2 = 3.5, kd2 = 1.0, ki2 = 1.0;
float kp3 = 3.5, kd3 = 1.0, ki3 = 1.0;

//int pwm1 = 0; 

unsigned long lastTime = 0;  // Tiempo anterior
unsigned long sampleTime = 50;  // Tiempo de muestreo

float offset(float d);

void setup()
{
  Serial.begin(9600);

  pinMode(AA1, INPUT);
  pinMode(BB1, INPUT);
  pinMode(CC1, INPUT);
  pinMode(AA2, INPUT);
  pinMode(BB2, INPUT);
  pinMode(CC2, INPUT);
  pinMode(AA3, INPUT);
  pinMode(BB3, INPUT);
  pinMode(CC3, INPUT);

  PWM1.attach(PB7,1000,2000); // pin, min input ms, max input ms
  PWM2.attach(PB8,1000,2000);
  PWM3.attach(PB9,1000,2000);

  PWM1.write(90);
  PWM2.write(90);
  PWM3.write(90);

  //3.4NM
  attachInterrupt(digitalPinToInterrupt(AA1), encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BB1), encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CC1), encoder1, CHANGE);

  //6.9NM
  attachInterrupt(digitalPinToInterrupt(AA2), encoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BB2), encoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CC2), encoder2, CHANGE);

  //9.7NM
  attachInterrupt(digitalPinToInterrupt(AA3), encoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BB3), encoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CC3), encoder3, CHANGE);
  
  //Serial.println("Numero de conteos");
  delay(2000);
}

void loop() {
  grados1 = n1*0.1787;  //deg  2014
  grados2 = n2*0.0708;  //deg  5085
  grados3 = n3*0.0507;  //deg  7098

//Serial.print(" n2  6.9 ");Serial.print(grados2);  //debug
//Serial.print(" n3  9.7 ");Serial.println(grados3);  //debug
  
  if (millis() - lastTime >= sampleTime || lastTime==0)
  {  // Se actualiza cada sampleTime (milisegundos)
      lastTime = millis();
      
      vel1 = (grados1 - grados1_ant)/0.05; //deg-s
      int_grados1 = int_grados1+(grados1*0.05); //deg s
      e1 = pos_des1 - grados1;
      e1_p = 0 - vel1;
      e1_i = 0 - int_grados1;

      pid1 = -(kp1*e1 + kd1*e1_p);// + ki1*e1_i;
      PWM1.write(offset(pid1));

      vel2 = (grados2 - grados2_ant)/0.05; //deg-s
      int_grados2 = int_grados2+(grados2*0.05); //deg s
      e2 = pos_des2 - grados2;
      e2_p = 0 - vel2;
      e2_i = 0 - int_grados2;

      pid2 = -(kp2*e2 + kd2*e2_p);// + ki1*e1_i;
      PWM2.write(offset(pid2));

      vel3 = (grados3 - grados3_ant)/0.05; //deg-s
      int_grados3 = int_grados3+(grados3*0.05); //deg s
      e3 = pos_des3 - grados3;
      e3_p = 0 - vel3;
      e3_i = 0 - int_grados3;

      pid3 = (kp3*e3 + kd3*e3_p);// + ki1*e1_i;
      PWM3.write(offset(pid3));

      
//      Serial.print(" e: ");Serial.print(e3);  //debug
//      Serial.print(" pid: ");Serial.print(pid3);  //debug
//      Serial.print(" pid_offset: ");Serial.println(offset(pid3));  //debug
      
      
      //Actualiza variables
      grados1_ant = grados1;
      grados2_ant = grados2;
      grados3_ant = grados3;

      //Monitor 
//      Serial.print(" pose deg: ");Serial.print(grados1);
//      Serial.print(" derivate: ");Serial.print(vel1);
//      Serial.print(" integral: ");Serial.println(int_grados1);
   }
   
}

// Encoder 3 fases.
void encoder1(void)
{
  ant1=act1;
  
  if(digitalRead(AA1)) bitSet(act1,2); else bitClear(act1,2);            
  if(digitalRead(BB1)) bitSet(act1,1); else bitClear(act1,1);
  if(digitalRead(CC1)) bitSet(act1,0); else bitClear(act1,0);
      
  if(ant1 == 1 && act1 ==5) n1++;
  if(ant1 == 3 && act1 ==1) n1++;
  if(ant1 == 2 && act1 ==3) n1++;
  if(ant1 == 6 && act1 ==2) n1++;
  if(ant1 == 4 && act1 ==6) n1++;
  if(ant1 == 5 && act1 ==4) n1++;
  
  if(ant1 == 1 && act1 ==3) n1--;
  if(ant1 == 3 && act1 ==2) n1--;
  if(ant1 == 2 && act1 ==6) n1--;
  if(ant1 == 6 && act1 ==4) n1--;
  if(ant1 == 4 && act1 ==5) n1--;
  if(ant1 == 5 && act1 ==1) n1--;    

//  Serial.print(" A: ");Serial.print(digitalRead(AA1));  //DEBUG
//  Serial.print(" B: ");Serial.print(digitalRead(BB1));
//  Serial.print(" C: ");Serial.print(digitalRead(CC1));
//  Serial.print(" ESTADO: ");Serial.println(act1);
}

void encoder2(void)
{
  ant2=act2;
  
  if(digitalRead(AA2)) bitSet(act2,2); else bitClear(act2,2);            
  if(digitalRead(BB2)) bitSet(act2,1); else bitClear(act2,1);
  if(digitalRead(CC2)) bitSet(act2,0); else bitClear(act2,0);
    
  if(ant2 == 1 && act2 ==5) n2++;
  if(ant2 == 3 && act2 ==1) n2++;
  if(ant2 == 2 && act2 ==3) n2++;
  if(ant2 == 6 && act2 ==2) n2++;
  if(ant2 == 4 && act2 ==6) n2++;
  if(ant2 == 5 && act2 ==4) n2++;
  
  if(ant2 == 1 && act2 ==3) n2--;
  if(ant2 == 3 && act2 ==2) n2--;
  if(ant2 == 2 && act2 ==6) n2--;
  if(ant2 == 6 && act2 ==4) n2--;
  if(ant2 == 4 && act2 ==5) n2--;
  if(ant2 == 5 && act2 ==1) n2--;    
}

void encoder3(void)
{
  ant3=act3;
  
  if(digitalRead(AA3)) bitSet(act3,2); else bitClear(act3,2);            
  if(digitalRead(BB3)) bitSet(act3,1); else bitClear(act3,1);
  if(digitalRead(CC3)) bitSet(act3,0); else bitClear(act3,0);
    
  if(ant3 == 1 && act3 ==5) n3++;
  if(ant3 == 3 && act3 ==1) n3++;
  if(ant3 == 2 && act3 ==3) n3++;
  if(ant3 == 6 && act3 ==2) n3++;
  if(ant3 == 4 && act3 ==6) n3++;
  if(ant3 == 5 && act3 ==4) n3++;
  
  if(ant3 == 1 && act3 ==3) n3--;
  if(ant3 == 3 && act3 ==2) n3--;
  if(ant3 == 2 && act3 ==6) n3--;
  if(ant3 == 6 && act3 ==4) n3--;
  if(ant3 == 4 && act3 ==5) n3--;
  if(ant3 == 5 && act3 ==1) n3--;    
}


float offset(float d)  //pwm 0 - 65 535 middle  32768 
{
  float out;
  if(d >= 0)
  {
    out = (90+d);
    if (out > 180) out=180;
    return out;
  }
  else if(d  < 0)
  {    
    out = (90+d);
    if (out < 0) out=0;
    return out;
  }

  
}
  

////PRUEBA BIT SET
//int x = 0b00000111;
//int n = 0;  ///Posicion del bit que cambia
//  
//void setup() {
//  Serial.begin(9600);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }
//
//  
//}
//
//void loop() {
//  bitClear(x,0);
//  Serial.println(x); // print the output of bitSet(x,n)
//  delay(500);
//  bitClear(x,1);
//  Serial.println(x); // print the output of bitSet(x,n)
//  delay(500);
//}
