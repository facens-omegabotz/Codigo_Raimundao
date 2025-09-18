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
#define EVENT_SENSOR1 (1<<0)
#define EVENT_SENSOR2 (1<<1)
#define EVENT_SENSOR3 (1<<2)
#define EVENT_SENSOR4 (1<<3)
#define EVENT_QRE (1<<4)
#define NUM_SENSORS 2
#define NUM_SAMPLES_PER_SENSOR 4

// Sensores

constexpr uint8_t kSensor1 = 32;
constexpr uint8_t kSensor2 = 33;
constexpr uint8_t kSensor3 = 25;
constexpr uint8_t kSensor4 = 27;

// QTRs

constexpr uint8_t kQtr1 = 36;
constexpr uint8_t kQtr2 = 39;

// IR

constexpr uint8_t kIrPin = 17;

// LEDs

constexpr uint8_t kLed1 = 18;
constexpr uint8_t kLed2 = 19;

// Motores

constexpr uint8_t kPwmA = 4;
constexpr uint8_t kPwmB = 21; 
constexpr uint8_t kAIn1 = 16;
constexpr uint8_t kAIn2 = 19;
constexpr uint8_t kBIn1 = 23;
constexpr uint8_t kBIn2 = 5;

constexpr bool kDebug = true; // Se true, habilita Serial e mensagens.

// Protótipos

// Tasks principais

void CoreTaskOne(void *pvParameters);
void CoreTaskZero(void *pvParameters);

// Recepção de Infravermelho

void ReceiveIrSignal();

// Calibração de sensores

void CalibrateSensors();

// Detecção de inimigos

void DetectEnemies();

// Estratégias

void RunStrategy();

void RadarEsquerdo();

void RadarDireito();

void CurvaAberta();

void Follow();

void Woodpecker();

// Globais

RobotState state = kReady;
Strategy strat = kRadarEsq;
constexpr bool kUseNVSCalibration = true;

QTRSensorsAnalog qtra((unsigned char[]){kQtr1, kQtr2}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR);
uint32_t sensor_values[NUM_SENSORS];
const char* kMinKeys[NUM_SENSORS] = {"kMinQtr1", "kMinQtr2"};
const char* kMaxKeys[NUM_SENSORS] = {"kMaxQtr1", "kMaxQtr2"};

Itamotorino motor_control = Itamotorino(kAIn1, kAIn2, kBIn1, kBIn2, kPwmA, kPwmB);

TaskHandle_t core_0;
TaskHandle_t core_1;

EventGroupHandle_t sensor_events;
void setup(){
  if (kDebug){
    Serial.begin(115200);
    while (!Serial){;}
    Serial.println("Serial initialized.");
    Serial.print("Starting state: ");
    Serial.println(state);
    Serial.print("Starting strat: ");
    Serial.println(strat);
  }
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(kSensor1, INPUT);
  pinMode(kSensor2, INPUT);
  pinMode(kSensor3, INPUT);
  pinMode(kSensor4, INPUT);
  pinMode(kLed1, OUTPUT);
  pinMode(kLed2, OUTPUT);
  IrReceiver.begin(kIrPin, true);
  CalibrateSensors();
  xTaskCreatePinnedToCore(
    CoreTaskOne,
    "CoreTaskOne",
    10000,
    NULL,
    1,
    &core_1,
    1
  );

  xTaskCreatePinnedToCore(
    CoreTaskZero,
    "CoreTaskZero",
    10000,
    NULL,
    1,
    &core_0,
    0
  );
}

void loop(){}

void CalibrateSensors(){
  qtra.calibrate();
  if (kUseNVSCalibration){
    initNVSStorage();
    for(int i = 0; i < NUM_SENSORS; i++){
      if (readUnsignedIntFromNVS(kMinKeys[i]) != -1){
        qtra.calibratedMinimumOn[i] = readUnsignedIntFromNVS(kMinKeys[i]);
      }
      if (readUnsignedIntFromNVS(kMaxKeys[i]) != -1){
        qtra.calibratedMaximumOn[i] = readUnsignedIntFromNVS(kMaxKeys[i]);
      }
      if (kDebug){
        Serial.println("== NVS read ==");
        Serial.print(kMinKeys[i]);
        Serial.print(" = ");
        Serial.println(qtra.calibratedMinimumOn[i]);
        Serial.print(kMaxKeys[i]);
        Serial.print(" = ");
        Serial.println(qtra.calibratedMaximumOn[i]);
      }
    }
    closeNVSStorage();
  }
  else{
    for(int i = 0; i < 1200; i++){
      qtra.calibrate();
    }
    initNVSStorage();
    for(int i = 0; i < NUM_SENSORS; i++){
      writeUnsignedIntToNVS(kMinKeys[i], qtra.calibratedMinimumOn[i]);
      writeUnsignedIntToNVS(kMaxKeys[i], qtra.calibratedMaximumOn[i]);
      if (kDebug){
        Serial.println("");
        Serial.println(qtra.calibratedMinimumOn[i]);
        Serial.println(qtra.calibratedMaximumOn[i]);
        Serial.println("== NVS written ==");
        Serial.print(kMinKeys[i]);
        Serial.print(" = ");
        Serial.println(readUnsignedIntFromNVS(kMinKeys[i]));
        Serial.print(kMaxKeys[i]);
        Serial.print(" = ");
        Serial.println(readUnsignedIntFromNVS(kMaxKeys[i]));
      }
    }
    closeNVSStorage();
  }
}

void ReceiveIrSignal(){
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
      Serial.print("Received the command: ");
      Serial.println(IrReceiver.decodedIRData.command);
      Serial.print("State after command: ");
      Serial.println(state);
      Serial.print("Strategy after command: ");
      Serial.println(strat);
    }
  }
  IrReceiver.resume(); // might be unnecessary
}

void RunStrategy(){
  switch (strat){
    case kRadarEsq:
      RadarEsquerdo();
      break;
    case kRadarDir:
      RadarDireito();
      break;
    case kCurvaAberta:
      CurvaAberta();
      break;
    case kFollowOnly:
      Follow();
      break;
    case kWoodPecker:
      Woodpecker();
      break;
    default:
      break;
  }
}

void DetectEnemies(){
  if (state == kRunning){
    if (digitalRead(kSensor1)){
      // enviar valor para algum lugar
    }
    if (digitalRead(kSensor2)){
      // enviar valor para algum lugar
    }
    if (digitalRead(kSensor3)){
      // enviar valor para algum lugar
    }
    if (digitalRead(kSensor4)){
      // enviar valor para algum lugar
    }
  }
}

void CoreTaskOne(void *pvParameters){
  for(;;){
    ReceiveIrSignal();
    DetectEnemies();
  }
}

void CoreTaskZero(void *pvParameters){
  for(;;){
    RunStrategy();
    
    // motor_control.setSpeeds(255, 255);
    // delay(1000);
    // motor_control.setSpeeds(0, 0);
    // delay(1000);
  }
}

void RadarEsquerdo(){
  if (state == kRunning){
    if (/* nenhum dado */){
      motor_control.setSpeeds(110, 95);
    }
    
    while (/* dado de sensor central*/){
      Follow();
    }
  }
}

void RadarDireito(){
  if (state == kRunning){
    if (/* nenhum dado */){
      motor_control.setSpeeds(110, 95);
    }
    
    while (/* dado de sensor central*/){
      Follow();
    }
  }
}