#include "MotorControllerEncoder.h"
#include "Arduino.h"

/*Constructor (...)*********************************************************
 Los parametros de entrada son:
	- encoderPinA 	-> Es el pin que se utilizara con el ISR. Las pulsaciones se contaran cada vez que este pin cambie de valor.
	- encoderPinB 	-> En el pin que se utilizara para saber la direccion de giro.
	- en 			-> Es el pin que se utilizara para habilitar el motor, estara conectado a un PMW.
	- dir1 y dir2 	-> Son los pines que habilitaran la direccion de motor en un sentido u otro.
 ***************************************************************************/

MotorControllerEncoder::MotorControllerEncoder(int encoderPinA, int encoderPinB, 
												int dir1, int dir2)

{   
	_encoderPinA  = encoderPinA;
	_encoderPinB = encoderPinB;
	_dir1 = dir1;
	_dir2 = dir2;


	pinMode(_dir1, OUTPUT);
	pinMode(_dir2, OUTPUT);

	begin(_encoderPinA);
	count = 0;
	last_t = 0;
		
	PidSetTunings ();
	start_moveF = false;
	status_stop = false;
	voltMotor = 0;
	errorVel = 0;
}

void MotorControllerEncoder::switchPressedExt0 ()
    {
		Serial.println("ENTRO");
		if (MotorControllerEncoder::instances [0] != NULL)
		MotorControllerEncoder::instances [0]->switchPressed ();
    }
  
static void MotorControllerEncoder::switchPressedExt1 ()
    {
		if (MotorControllerEncoder::instances [1] != NULL)
		MotorControllerEncoder::instances [1]->switchPressed ();
    } 

void MotorControllerEncoder::switchPressed ()
{
	if(digitalRead(_encoderPinA) != digitalRead(_encoderPinB))
	{
		count ++;
	}
	else 
	{
		count --;
	}
}
void MotorControllerEncoder::begin(const byte whichPin)
{
	pinMode (whichPin, INPUT_PULLUP);
    switch (whichPin)
      {
      case 2: 
        attachInterrupt (digitalPinToInterrupt(whichPin), switchPressedExt0, CHANGE);
        instances [0] = this;
        break;
        
      case 3: 
        attachInterrupt (digitalPinToInterrupt(whichPin), switchPressedExt1, CHANGE);
        instances [1] = this;
        break;
        
      } // end of switch	
}

void MotorControllerEncoder::PidSetTunings()
{
   if (Kp<0 || Ki<0 || Kd<0) return;
  
   double SampleTimeInSec = ((double)Ts)/1000;  
   
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;	
}

/*
Parametros de entrada:
vel = Velocidad de las ruedas en rev/s
MoveF = Si quiere que se mueva para adelante o no
*/
void MotorControllerEncoder::move(double vel, bool MoveF)
{
	if(MoveF)
	{
		if(!start_moveF){Initialize();}
		
		digitalWrite(_dir1, LOW);
		digitalWrite(_dir2, HIGH);
	}
	else
	{
		if(start_moveF){ Initialize();}
		
		digitalWrite(_dir1, HIGH);
		digitalWrite(_dir2, LOW);
	}
	
	unsigned long now = millis();
	unsigned long timeChange = (now - lastTime);
	
	if(timeChange >= Ts)
	{
		double freq = 1000/Ts;
		
		long input = 0;
		count > last_count ? input = count - last_count : input = last_count - count;
		
		
		double ObjVel = vel*(Motor_K/freq);
		errorVel = ObjVel - input;
		if(errorVel!=0)
		{
			ITerm+= (ki * errorVel);
			double dInput = (input - lastInput);
			voltMotor += kp * errorVel + ITerm - kd * dInput;
			voltMotor = abs(voltMotor);
		}
		
		
		if(voltMotor > 255) voltMotor = 255;
		
		lastInput = input;
		lastTime = now;
		last_count = count;	
	}
	//analogWrite(_en,  abs(voltMotor));
}

void MotorControllerEncoder::stop()
{
	
	if(!status_stop) 
    {
		count_stop = count;
		t_stop = millis();
    }
	
	if(count >= count_stop)
	{
		status_stop = true;
		digitalWrite(_dir1, HIGH);
		digitalWrite(_dir2, LOW);
		//analogWrite(_en,  voltMotor);
	}
	else{
		digitalWrite(_dir1, LOW);
		digitalWrite(_dir2, HIGH);
		//analogWrite(_en,  voltMotor);
	}

	if(status_stop && (millis() - t_stop > 100))
	{
		voltMotor = 0;
	}
}

void MotorControllerEncoder::Initialize()
{
	last_count = count;
	lastTime = millis();
	
	voltMotor = 0;
	ITerm = 0;
	lastInput = 0;
	start_moveF = !start_moveF;
}


double MotorControllerEncoder::GetDispMotor ()
{
	
	//double dist =  count*(2*PI*RW)/ Motor_K;
	double rev = count/Motor_K;
	//return dist;
	return count;
}
double MotorControllerEncoder::GetRevMotor()
{
	double rev = count/Motor_K;
	return rev;
}
double MotorControllerEncoder::GetErrorVel(){ return errorVel; }








