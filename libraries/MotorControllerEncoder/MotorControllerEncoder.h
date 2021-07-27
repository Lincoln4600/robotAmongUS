#ifndef MotorControllerEncoder_H
#define MotorControllerEncoder_H
#define LIBRARY_VERSION	1.0.0
#include "Arduino.h"

class MotorControllerEncoder
{
  public:
  
  #define Motor_K 800L //Numero de ranuras en el motor en una revolucion
/* Parametros de PID para controlar la velocidad de los motores  */
  #define Kp 1
  #define Ki 0.1
  #define Kd 0.1
  #define Ts 0.001
  
  //Funciones **************************************************************************
	MotorControllerEncoder::MotorControllerEncoder(int, int, int, int);
	
	//static void isrA();	
	void switchPressed ();
	void begin (const byte );
	static void switchPressedExt0 ();
	static void switchPressedExt1 ();
	
	
	
	static MotorControllerEncoder * instances [2];
	void move (double, bool);
	void stop();
	void PidSetTunings (); 
	double GetDispMotor();
	double GetRevMotor ();
	double GetErrorVel();
	void Initialize();
  
	private:

		double errorVel;
		volatile long count;
		
		int _encoderPinA;
		int _encoderPinB;

		int _dir1;
		int _dir2;
		
		long last_t;
		long last_count;
		long count_stop;
		long t_stop;
		
		bool start_moveF;
		float voltMotor;
		bool status_stop;
		
		unsigned long lastTime;
		double ITerm, lastInput;
		
		double kp;                  // * (P)roportional Tuning Parameter
		double ki;                  // * (I)ntegral Tuning Parameter
		double kd;                  // * (D)erivative Tuning Parameter
};
#endif