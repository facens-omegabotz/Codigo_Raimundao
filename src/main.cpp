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

// TODO: Biblioteca separada com as estratégias (provavelmente o código ficaria mais limpo).

#include <esp_ipc.h>
#include <Arduino.h>
#include <QTRSensors.h>
#include <IRremote.hpp>
#include <Itamotorino.h>
#include <freertos/task.h>
#include <freertos/FreeRTOS.h>

#define DECODE_SONY // Limita biblioteca de IR ao protocolo Sony.

enum RobotState {
  READY = 0x0, // Comando HEX recebido pelo número 1.
  RUNNING,
  STOP
};

enum Strategy {
  RADAR_ESQUERDA = 0x3, // Comando HEX recebido pelo número 4.
  RADAR_DIREITA,
  CURVA_ABERTA,
  FOLLOW_ONLY,
  WOODPECKER
};

constexpr uint8_t SENSOR_1 = 35;
constexpr uint8_t SENSOR_2 = 33;
constexpr uint8_t SENSOR_3 = 25;
constexpr uint8_t SENSOR_4 = 27;
constexpr uint8_t IR_SIGNAL = 4;
constexpr uint8_t QTR_1 = 36;
constexpr uint8_t QTR_2 = 39;

constexpr bool DEBUG = true; // Se true, habilita Serial e mensagens.

// Globais

uint_fast32_t startTime = millis();
uint_fast32_t nowTime;

RobotState robotState = READY;
Strategy strategy = RADAR_ESQUERDA;
bool flashing = true;

QTRSensors qtr;
constexpr uint8_t sensorCount = 2;
uint16_t sensorValues[sensorCount];

// Declaração de funções

void receiveSignal(void *arg);
void blinkLed(void *arg);

void setup() {
  IrReceiver.begin(IR_SIGNAL, false);
  pinMode(LED_BUILTIN, OUTPUT);
  if (DEBUG){
    Serial.begin(115200);
    while (!Serial){;}
  }
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){QTR_1, QTR_2}, sensorCount);
  qtr.calibrate();
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  esp_ipc_call(PRO_CPU_NUM, receiveSignal, NULL);
  if (robotState == RUNNING){
    esp_ipc_call(APP_CPU_NUM, blinkLed, NULL);
  }
  if (robotState == STOP){
    if (DEBUG)
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
void receiveSignal(void *arg){
  (void)arg;
  if (IrReceiver.decode()){
    IrReceiver.resume();
    if (robotState != STOP && robotState != RUNNING && IrReceiver.decodedIRData.command == READY)
      robotState = READY;

    if (robotState == READY){
      switch(IrReceiver.decodedIRData.command){
        case RUNNING:
          robotState = RUNNING;
          break;
        case RADAR_ESQUERDA:
          strategy = RADAR_ESQUERDA;
          break;
        case RADAR_DIREITA:
          strategy = RADAR_DIREITA;
          break;
        case CURVA_ABERTA:
          strategy = CURVA_ABERTA;
          break;
        case FOLLOW_ONLY:
          strategy = FOLLOW_ONLY;
          break;
        case WOODPECKER:
          strategy = WOODPECKER;
        default:
          break;
      }
    }
    if (IrReceiver.decodedIRData.command == STOP)
      robotState = STOP;

    if (DEBUG){
      Serial.print("Comando Sony SIRC protocol: 0x");
      Serial.println(IrReceiver.decodedIRData.command);
      Serial.print("RobotState apos comando: 0x");
      Serial.println(robotState);
      Serial.print("Estrategia apos comando: 0x");
      Serial.println(strategy);
      Serial.print("Core ID de receiveSignal(): ");
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
  nowTime = millis();
  if (nowTime - startTime >= 2000L){
    startTime = millis();
    if (flashing)
      digitalWrite(LED_BUILTIN, LOW);
    else
      digitalWrite(LED_BUILTIN, HIGH);
    flashing = !flashing;
    if (DEBUG){
      Serial.print("Core ID de blinkLed(): ");
      Serial.println(xPortGetCoreID());
    }
  }
}