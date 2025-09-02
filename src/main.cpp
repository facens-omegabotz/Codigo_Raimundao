/**
 * Código Raimundão
 * 
 * Código completo do funcionamento do Raimundão - Inspirado em outros códigos
 * do Omegabotz, mas com uso de FreeRTOS para usar melhor os núcleos do ESP32.
 * Um dos núcleos deverá cuidar da detecção (tanto de inimigos quanto de linha)
 * e enviar mensagens adequadas para o outro núcleo.
 * 
 * @author Felipe Mastromauro Corrêa
 */


#include <EEPROM.h>
#include "headers/enumerators.h"
#include "headers/mem_functions.h"
#include <esp_ipc.h>
#include <Arduino.h>
#include <QTRSensors.h>
#include <IRremote.hpp>
#include <Itamotorino.h>
#include <freertos/task.h>
#include <freertos/FreeRTOS.h>

#define DECODE_SONY // Limita biblioteca de IR ao protocolo Sony.
#define NUM_SENSORS 2
#define NUM_SAMPLES_PER_SENSOR 4


constexpr uint8_t kSensor1 = 35;
constexpr uint8_t kSensor2 = 33;
constexpr uint8_t kSensor3 = 25;
constexpr uint8_t kSensor4 = 27;
constexpr uint8_t kIrPin = 4;
constexpr uint8_t kQtr1 = 36;
constexpr uint8_t kQtr2 = 39;

constexpr bool kDebug = true; // Se true, habilita Serial e mensagens.

// Globais

uint_fast32_t start_time = millis();
uint_fast32_t now_time;

RobotState state = kReady;
Strategy strat = kRadarEsq;
bool flashing = true;
bool use_eeprom_calibration = true;
bool is_calibrated = false;

uint32_t sensorValues[NUM_SENSORS];

// Declaração de funções

void calibrate();
void readEEPROMCalibration();
void writeEEPROMCalibration();
void receive_signal(void *arg);
void blinkLed(void *arg);
void radar(bool turnRight);
void sensingTask();
void motorControlTask();

void setup() {
  IrReceiver.begin(kIrPin, false);
  pinMode(LED_BUILTIN, OUTPUT);
  if (kDebug){
    Serial.begin(115200);
    while (!Serial){;}
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

// TODO: separar em duas tasks maiores: Sensoriamento e Motores.
void loop() {
  esp_ipc_call(PRO_CPU_NUM, receive_signal, NULL);
  if (state == kRunning){
    esp_ipc_call(APP_CPU_NUM, blinkLed, NULL);
  }
  if (state == kStop){
    if (kDebug)
      Serial.println("Recepcao e outras tarefas paradas.");
    while (true){
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
    }
  }
}

/**
 * Recebe e interpreta o sinal do controle para o funcionamento do robô, definindo
 * os estados e estratégias.
 */
void receive_signal(void *arg){
  (void)arg;
  if (IrReceiver.decode()){
    IrReceiver.resume();
    if (state != kStop && state != kRunning && IrReceiver.decodedIRData.command == kReady)
      state = kReady;

    if (state == kReady){
      switch(IrReceiver.decodedIRData.command){
        case kRunning:
          state = kRunning;
          break;
        case kRadarEsq:
          strat = kRadarEsq;
          break;
        case kRadarDir:
          strat = kRadarDir;
          break;
        case kCurvaAberta:
          strat = kCurvaAberta;
          break;
        case kFollowOnly:
          strat = kFollowOnly;
          break;
        case  kWoodPecker:
          strat = kWoodPecker;
        default:
          break;
      }
    }
    if (IrReceiver.decodedIRData.command == kStop)
      state = kStop;

    if (kDebug){
      Serial.print("Comando Sony SIRC protocol: 0x");
      Serial.println(IrReceiver.decodedIRData.command);
      Serial.print("RobotState apos comando: 0x");
      Serial.println(state);
      Serial.print("Estrategia apos comando: 0x");
      Serial.println(strat);
      Serial.print("Core ID de receive_signal(): ");
      Serial.println(xPortGetCoreID());
    }
  }
  IrReceiver.resume(); // might be unnecessary
}

/**
 * Funcão simples para piscar o LED onboard do ESP com um delay não 
 * blocante de 2 segundos.
 */
void blinkLed(void *arg){
  (void)arg;
  now_time = millis();
  if (now_time - start_time >= 2000L){
    start_time = millis();
    if (flashing)
      digitalWrite(LED_BUILTIN, LOW);
    else
      digitalWrite(LED_BUILTIN, HIGH);
    flashing = !flashing;
    if (kDebug){
      Serial.print("Core ID de blinkLed(): ");
      Serial.println(xPortGetCoreID());
    }
  }
}

// Parece simples. Só acelera se for centralizado.
void radar(bool turnRight){
  now_time = millis();
  while(!digitalRead(kSensor1) and !digitalRead(kSensor2) and !digitalRead(kSensor3) and !digitalRead(kSensor4)){
    if (turnRight){
      // Vira à direita
    }
    else{
      // Vira à esquerda
    }
  }
  if (now_time - start_time > 1000L){
    turnRight = !turnRight;
    start_time = millis();
  }
  while(digitalRead(kSensor2) or digitalRead(kSensor3)){
    // Acelera
  }
}

void sensingTask(){}
void motorControlTask(){}