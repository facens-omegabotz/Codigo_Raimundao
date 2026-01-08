/*
  Itamotorino.ino - Library to control the motor shield the Itapêduino
  Created and developed by GELSON J.A. FILHO, July 26, 2014 (4:12 am).
  Released into the public domain

  Itamotorino.ino - Biblioteca para comandar el controlador de motor de la Itapêduino
  Creado y desarrollado por GELSON J.A. FILHO, Julio 26, 2014 (4:12)
  Publicada en el dominio público
  
  Itamotorino.ino - Biblioteca para comandar a controladora de motores da Itapêduino
  Criado e desenvolvido por GELSON J. A. FILHO, Julho 26, 2014 (4:12)
  Lançamentos para o domínio público
*/

/*
  Explanation of the operation of shield ITAMOTORINO. 
  
  The shield has the L298 chip it is the main component.
  L298 that controls the motors practically. 
  In addition to the direct cable entries,
  engine ground and vcc battery, have 8 outputs "jumper cables".
  Gnd, Vcc, M1A, M1B, M2A, M2B, m1enable, m2enable.
  In this example we are connecting sketch of the motors
  in their digital pins: 1,2,3,4,6,7.
  But it's just an example.
  You can call anywhere you want provided that PIN DIGITAL.
  The shield has protection diodes against reverse driving electro.
  Successfully controls two DC motors.
*/

//Inclusion of "library" Itamotorino
#include <Itamotorino.h>

/*
  Definition digital pins relating to 
  motors 1 an 2 and their "Enables".
  Adapt as the use.
*/
#define pin_Motor1A 1
#define pin_Motor1B 2
#define pin_Motor2A 3
#define pin_Motor2B 4
#define pin_EnableM1 6
#define pin_EnableM2 7

/*
  Instantiating the class object (it can have any name).
  The parameters (in parentheses) are digital pins concerning shield,
  the library automatically declares as "output".
  ATTENTION: the last two pins ALWAYS be the "enable"
  that define the PWM (speed) motors.
*/

Itamotorino itamotorino(pin_Motor1A, pin_Motor1B,
                        pin_Motor2A, pin_Motor2B,
                        pin_EnableM1, pin_EnableM2);

void setup()
{
}

void loop()
{
/* 
  The ".setSpeeds​​" command is very simple to use.
  The first parameter sets the speed motor1.
  The second parameter sets the speed of motor2.
  The range is the same as PWM (0 to 255).
  If the value is negative the motor spins BACKWARD.
  If the value is positive the motor rotates FORWARD.
  If the value is zero the motor had been STOPPED.
*/

  //test motors:
  
  itamotorino.setSpeeds(100,100);
  delay(500);
  itamotorino.setSpeeds(150,150);
  delay(500);
  itamotorino.setSpeeds(200,200);
  delay(500);
  itamotorino.setSpeeds(255,255);
  delay(500);
  
  itamotorino.setSpeeds(0,0);
  delay(1000);
  
  itamotorino.setSpeeds(-100,-100);
  delay(500);
  itamotorino.setSpeeds(-150,-150);
  delay(500);
  itamotorino.setSpeeds(-200,-200);
  delay(500);
  itamotorino.setSpeeds(-255,-255);
  
  itamotorino.setSpeeds(0,0);
  delay(1000);
  
  itamotorino.setSpeeds(-100,100);
  delay(500);
  itamotorino.setSpeeds(-150,150);
  delay(500);
  itamotorino.setSpeeds(-200,200);
  delay(500);
  itamotorino.setSpeeds(-255,255);
  
  itamotorino.setSpeeds(0,0);
  delay(1000);
  
  itamotorino.setSpeeds(100,-100);
  delay(500);
  itamotorino.setSpeeds(150,-150);
  delay(500);
  itamotorino.setSpeeds(200,-200);
  delay(500);
  itamotorino.setSpeeds(255,-255);
  
  itamotorino.setSpeeds(0,0);
  delay(1000);
  
  itamotorino.setSpeeds(100,0);
  delay(500);
  itamotorino.setSpeeds(150,0);
  delay(500);
  itamotorino.setSpeeds(200,0);
  delay(500);
  itamotorino.setSpeeds(255,0);
  
  itamotorino.setSpeeds(0,0);
  delay(1000);
  
  itamotorino.setSpeeds(-100,0);
  delay(500);
  itamotorino.setSpeeds(-150,0);
  delay(500);
  itamotorino.setSpeeds(-200,0);
  delay(500);
  itamotorino.setSpeeds(-255,0);
  
  itamotorino.setSpeeds(0,0);
  delay(1000);
  
  itamotorino.setSpeeds(0,100);
  delay(500);
  itamotorino.setSpeeds(0,150);
  delay(500);
  itamotorino.setSpeeds(0,200);
  delay(500);
  itamotorino.setSpeeds(0,255);
  
  itamotorino.setSpeeds(0,0);
  delay(1000);
  
  itamotorino.setSpeeds(0,-100);
  delay(500);
  itamotorino.setSpeeds(0,-150);
  delay(500);
  itamotorino.setSpeeds(0,-200);
  delay(500);
  itamotorino.setSpeeds(0,-255);
  
  itamotorino.setSpeeds(0,0);
  delay(1000);
}

