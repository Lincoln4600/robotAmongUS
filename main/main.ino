
#include "MotorControllerEncoder.h"

/*
 * Definicion de los pines que se utlizarán
 * 
 * Todos estos pines (9,5,6 y 10) se han conectado a un modulo PMW
 * PIN 9 -> direccion para adelante del motor izquierdo
 * PIN 5 -> direccion para atras del motor izquierdo
 * PIN 6 -> direccion para adelante del motor derecho
 * PIN 10 -> direccion para atras del motor derecho
 * 
 * PIN 3 -> pulsaciones del motor izquierdo en direccion de las agujas del reloj 
 * PIN 7 -> pulsaciones del motor izquierdo en direccion en contra de las agujas del reloj 
 * PIN 2 -> pulsaciones del motor derecho en direccion de las agujas del reloj 
 * PIN 4 -> pulsaciones del motor derecho en direccion en contra de las agujas del reloj 
 * 
 * Solo se utlizaran las pulsaciones que van en direccion a las agujas del reloj para medir la velocidad.
 */
 
int MotorLeft_dir2 = 9;
int MotorLeft_dir1 = 5;
int MotorRight_dir2 = 6;
int MotorRight_dir1 = 10;

#define encoderMotorLeft_CW  3
#define encoderMotorLeft_CCW  7

#define encoderMotorRight_CW  2
#define encoderMotorRight_CCW  4

#define pulsePerRevolution =  390

MotorControllerEncoder * MotorControllerEncoder::instances [2] = { NULL, NULL };
MotorControllerEncoder motorRight(encoderMotorRight_CW, encoderMotorRight_CCW, 
                                 MotorLeft_dir1, MotorLeft_dir2);


volatile long encoderAPos=0;
long newpositionA;
long oldpositionA = 0;
unsigned long newtimeA;
unsigned long oldtimeA = 0;
long velA;
int voltA = 0;

volatile long encoderBPos=0;
long newpositionB;
long oldpositionB = 0;
unsigned long newtimeB;
unsigned long oldtimeB = 0;
long velB;
int voltB = 0;

void setup() {

  
  
//  pinMode (MotorA_dir2, OUTPUT);
//  pinMode (MotorA_dir1, OUTPUT);
//  pinMode (MotorB_dir2, OUTPUT);
//  pinMode (MotorB_dir1, OUTPUT);
//
//  pinMode(encoderBPin_1, INPUT);
//  digitalWrite(encoderBPin_1, HIGH);       // turn on pullup resistor
//  pinMode(encoderBPin_2, INPUT);
//  digitalWrite(encoderBPin_2, HIGH);       // turn on pullup resistor
//  attachInterrupt(digitalPinToInterrupt(encoderBPin_1), doEncoderB, RISING);  // encoDER ON PIN 2
//
//  pinMode(encoderAPin_1, INPUT);
//  digitalWrite(encoderAPin_1, HIGH);       // turn on pullup resistor
//  pinMode(encoderAPin_2, INPUT);
//  digitalWrite(encoderAPin_2, HIGH);       // turn on pullup resistor
//  attachInterrupt(digitalPinToInterrupt(encoderAPin_1), doEncoderA, RISING);  // encoDER ON PIN 2
//
  Serial.begin (9600);
  Serial.println("start");       

}

void loop() {

  //motorRight.move(15, true);
//  newpositionA = encoderAPos;
//  newtimeA = millis();
//  velA = (newpositionA-oldpositionA) * 1000 /(newtimeA-oldtimeA);  
//  oldpositionA = newpositionA;
//  oldtimeA = newtimeA;
//
//  newpositionB = encoderBPos;
//  newtimeB = millis();
//  velB = (newpositionB-oldpositionB) * 1000 /(newtimeB-oldtimeB);  
//  oldpositionB = newpositionB;
//  oldtimeB = newtimeB;
//  
//  if(velA < 600)
//    voltA = voltA + 1;
//  else if (velA > 600)
//    voltA = voltA - 1;
//
//  if(velB < 600)
//    voltB = voltB + 5;
//  else if (velB > 625)
//    voltB = voltB - 5;
//
//  analogWrite(MotorA_dir1, 0);
//  analogWrite(MotorB_dir1, 0);
//  analogWrite(MotorA_dir2, 0);
//  analogWrite(MotorB_dir2, 0);
//  Serial.print ("encoder A = ");
//  Serial.print (encoderAPos);
//  Serial.print ("\t");
//  Serial.print ("encoder B = ");
//  Serial.println (encoderBPos);
//  //Serial.print ("speed B = ");
//  //Serial.println (velB);
//  delay(250);
}
//
//
//void doEncoderA()
//{
//  if (digitalRead(encoderAPin_1) == digitalRead(encoderAPin_2)) {
//    encoderAPos++;
//  } else {
//    encoderAPos--;
//  }
//}
//
//void doEncoderB()
//{
//  if (digitalRead(encoderBPin_1) == digitalRead(encoderBPin_2)) {
//    encoderBPos++;
//  } else {
//    encoderBPos--;
//  }
//}
