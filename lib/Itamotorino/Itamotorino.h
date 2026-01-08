/*
  Itamotorino.h - Library to control the motor shield the Itapêduino
  Created and developed by GELSON J.A. FILHO, July 26, 2014 (4:12 am).
  Released into the public domain

  Itamotorino.h - Biblioteca para comandar el controlador de motor de la Itapêduino
  Creado y desarrollado por GELSON J.A. FILHO, Julio 26, 2014 (4:12)
  Publicada en el dominio público
  
  Itamotorino.h - Biblioteca para comandar a controladora de motores da Itapêduino
  Criado e desenvolvido por GELSON J. A. FILHO, Julho 26, 2014 (4:12)
  Lançamentos para o domínio público
*/
  
#ifndef Itamotorino_h
#define Itamotorino_h

#include "Arduino.h"

class Itamotorino
{
  public:
    Itamotorino(int pinM1a, int pinM1b,
                int pinM2a, int pinM2b,
                int pinEnableM1, int pinEnableM2);
    void setSpeeds(int m1, int m2);
    void setupADC(int channelM1, int freqM1, int resolutionM1, int channelM2, int freqM2, int resolutionM2);

  private:
	int _pinM1a, _pinM1b;
	int _pinM2a, _pinM2b;
	int _pinEnableM1, _pinEnableM2;
  int _m1, _m2;
  int _channelM1, _freqM1, _resolutionM1;
  int _channelM2, _freqM2, _resolutionM2;
};

#endif