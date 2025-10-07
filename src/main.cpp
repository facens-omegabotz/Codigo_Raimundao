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

#define DECODE_SONY // Limita biblioteca de IR ao protocolo Sony.

#include "enumerators.h"
#include "mem_functions.h"
#include <esp_ipc.h>
#include <Arduino.h>
#include <QTRSensors.h>
#include <IRremote.hpp>
#include <Itamotorino.h>
#include <freertos/task.h>
#include <freertos/FreeRTOS.h>

// Bits de eventos

#define EVENT_SENSOR1 (1<<0)
#define EVENT_SENSOR2 (1<<1)
#define EVENT_SENSOR3 (1<<2)
#define EVENT_SENSOR4 (1<<3)
#define EVENT_QRE (1<<4)

// Configurações para sensores QTR

#define NUM_SENSORS 2
#define NUM_SAMPLES_PER_SENSOR 4

// Pinos dos sensores IR

constexpr uint8_t kSensor1 = 32;
constexpr uint8_t kSensor2 = 33;
constexpr uint8_t kSensor3 = 25;
constexpr uint8_t kSensor4 = 27;

// Pinos dos sensores de linha

constexpr uint8_t kQtr1 = 36;
constexpr uint8_t kQtr2 = 39;

// Pino do receptor IR

constexpr uint8_t kIrPin = 17;

// Pinos dos LEDs

constexpr uint8_t kLed1 = 18;
constexpr uint8_t kLed2 = 19;

// Pinos da ponte H (TB6612FNG)

constexpr uint8_t kPwmA = 4;
constexpr uint8_t kPwmB = 21; 
constexpr uint8_t kAIn1 = 16;
constexpr uint8_t kAIn2 = 22;
constexpr uint8_t kBIn1 = 23;
constexpr uint8_t kBIn2 = 5;

// Valores para configuração de PWM

constexpr int kPwmChannelM1 = 1;
constexpr int kPwmFreqM1 = 1000;
constexpr int kPwmResolutionM1 = 8;

constexpr int kPwmChannelM2 = 2;
constexpr int kPwmFreqM2 = 1000;
constexpr int kPwmResolutionM2 = 8;

// Constantes de configuração

constexpr bool kDebug = false;            // Se true, habilita Serial e mensagens.
constexpr bool kWhiteLine = true;         // Se true, envia valores quando a refletância for alta.
constexpr bool kUseNVSCalibration = true; // Se true, usa a calibração na memória não volátil.

// Protótipos de função

void CoreTaskOne(void *pvParameters);
void CoreTaskZero(void *pvParameters);

void ReceiveIrSignal();
void CalibrateSensors();
void DetectEnemies();
void DetectLine();        
void RunStrategy();
void RadarEsquerdo();
void RadarDireito();
void CurvaAberta();
void Woodpecker();
void Follow();
void LineDetectedProtocol(Direction direction);
void KillMotors();
EventBits_t WaitForSensorEvents();

// Globais

RobotState state = kReady;
Strategy strat = kRadarEsq;

QTRSensorsAnalog qtra((unsigned char[]){kQtr1, kQtr2}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR);
uint32_t sensor_values[NUM_SENSORS];

const char* kMinKeys[NUM_SENSORS] = {"kMinQtr1", "kMinQtr2"};
const char* kMaxKeys[NUM_SENSORS] = {"kMaxQtr1", "kMaxQtr2"};

Itamotorino motor_control = Itamotorino(kAIn1, kAIn2, kBIn1, kBIn2, kPwmA, kPwmB);

unsigned long time_1, time_2;

TaskHandle_t core_0;
TaskHandle_t core_1;

EventGroupHandle_t sensor_events;
EventBits_t x;

void setup(){
  analogReadResolution(10); // Necessário para o bom funcionamento do QTR.
  // Desligamento de watchdogs para funcionamento das tasks em loop.
  disableCore1WDT();
  disableCore0WDT();
  if (kDebug){
    Serial.begin(115200);
    while (!Serial){;}
    Serial.println("Serial initialized.");
    Serial.print("Starting state: ");
    Serial.println(state);
    Serial.print("Starting strat: ");
    Serial.println(strat);
  }
  motor_control.setupADC(kPwmChannelM1, kPwmFreqM1, kPwmResolutionM1, 
                         kPwmChannelM2, kPwmFreqM2, kPwmResolutionM2);
  sensor_events = xEventGroupCreate();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(kSensor1, INPUT);
  pinMode(kSensor2, INPUT);
  pinMode(kSensor3, INPUT);
  pinMode(kSensor4, INPUT);
  pinMode(kLed1, OUTPUT);
  pinMode(kLed2, OUTPUT);
  IrReceiver.begin(kIrPin, true); // LED_BUILTIN como feedback de resposta.
  IrReceiver.enableIRIn();        // Linha possivelmente desnecessária.
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

void loop(){} // Loop vazio para funcionamento adequado das tasks.

/// @brief Task para a execução da estratégia selecionada pelo operador.
/// @param pvParameters Ponteiro para void que pode conter dados importantes à tarefa. Inutilizado neste caso.
void CoreTaskOne(void *pvParameters){
  for(;;){
    if (state != kStop)
      RunStrategy();
    else{
      KillMotors();
      vTaskDelete(core_1);
    }
  }
}

/// @brief Task para a detecção de todos os sinais necessários para o robô (receptor IR, sensores IR e sensores QTR).
/// @param pvParameters Ponteiro para void que pode conter dados importantes à tarefa. Inutilizado neste caso.
void CoreTaskZero(void *pvParameters){
  for(;;){
    if (IrReceiver.decode())
      ReceiveIrSignal();
    DetectEnemies();
    DetectLine();
  }
}

/// @brief Função para alterar os valores importantes ao robô de acordo com o sinal recebido via infravermelho.
void ReceiveIrSignal(){
  if (kDebug){
    Serial.println("Entered Receive Signal");
    Serial.println(IrReceiver.decodedIRData.command);
  }
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

/// @brief Função para calibrar os sensores QTR. Obtém a calibração da memória não volátil ou calibra os sensores.
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
    digitalWrite(kLed2, HIGH);
    for(int i = 0; i < 1200; i++){
      qtra.calibrate();
    }
    digitalWrite(kLed2, LOW);
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

/// @brief Função para esperar a obtenção de eventos de todos os sensores. Possui um timeout de 30ms.
/// @return Os bits escritos no EventGroup global.
EventBits_t WaitForSensorEvents(){
  return xEventGroupWaitBits(
    sensor_events, 
    EVENT_SENSOR1 | EVENT_SENSOR2 | EVENT_SENSOR3 | EVENT_SENSOR4 | EVENT_QRE, 
    true, 
    false, 
    pdMS_TO_TICKS(30)
  );
}

/// @brief Função para executar a estratégia selecionada pelo operador.
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

/// @brief Função que lê os valores de cada um dos sensores infravermelhos e escreve ou limpa os bits no EventGroup global.
void DetectEnemies(){
  if (state == kRunning){
    if (digitalRead(kSensor1)){
      xEventGroupSetBits(sensor_events, EVENT_SENSOR1);
    }
    else{
      xEventGroupClearBits(sensor_events, EVENT_SENSOR1);
    }

    if (digitalRead(kSensor2)){
      xEventGroupSetBits(sensor_events, EVENT_SENSOR2);
    }
    else{
      xEventGroupClearBits(sensor_events, EVENT_SENSOR2);
    }

    if (digitalRead(kSensor3)){
      xEventGroupSetBits(sensor_events, EVENT_SENSOR3);
    }
    else{
      xEventGroupClearBits(sensor_events, EVENT_SENSOR3);
    }
    if (digitalRead(kSensor4)){
      xEventGroupSetBits(sensor_events, EVENT_SENSOR4);
    }
    else{
      xEventGroupClearBits(sensor_events, EVENT_SENSOR4);
    }
  }
}

/**
 * Função para detectar a linha e escrever no EventGroup.
 */
void DetectLine(){
  if (state == kRunning){
    int line_info = qtra.readLine(sensor_values, QTR_EMITTERS_ON, kWhiteLine);
    if (line_info > 500)
      xEventGroupSetBits(sensor_events, EVENT_QRE);
    else
      xEventGroupClearBits(sensor_events, EVENT_QRE);
  }
}

/**
 * @brief Estratégia para procurar o inimigo virando para a esquerda se não houver nenhuma detecção. 
 * Ao detectar, muda a estratégia para Follow();
 */
void RadarEsquerdo(){
  if (state == kRunning){
    x = WaitForSensorEvents();
    if (!(x & EVENT_SENSOR1) && !(x & EVENT_SENSOR2) && !(x & EVENT_SENSOR3) && !(x & EVENT_SENSOR4)){
      motor_control.setSpeeds(191, -191);
    }
    else{
      while(state == kRunning)
        Follow();
    }
  }
}

/**
 * @brief Estratégia para procurar o inimigo virando para a direita se não houver nenhuma detecção. 
 * Ao detectar, muda a estratégia para Follow();
 */
void RadarDireito(){
  if (state == kRunning){
    if (kDebug)
      Serial.println(x, BIN);
    x = WaitForSensorEvents();
    if (!(x | EVENT_SENSOR1) && !(x | EVENT_SENSOR2) && !(x | EVENT_SENSOR3) && !(x | EVENT_SENSOR4)){
      motor_control.setSpeeds(-191, 191);
    }
    else{
      while(state == kRunning)
        Follow();
    }
  }
}

/// @brief Estratégia que faz uma curva aberta antes de entrar em Follow();
void CurvaAberta(){
  time_1 = millis();
  if (state == kRunning){
    Direction direction;
    x = WaitForSensorEvents();
    if (x & EVENT_SENSOR1){
      direction = kLeft;
      motor_control.setSpeeds(-191, 191);
    }
    else if (x & EVENT_SENSOR4){
      direction = kRight;
      motor_control.setSpeeds(191, 191);
    }
    if (x & EVENT_QRE){
      if (direction == kLeft)
        LineDetectedProtocol(kRight);
      else
        LineDetectedProtocol(kLeft);
    }
    if (millis() - time_1 >= 2000){
      if (direction == kLeft)
        motor_control.setSpeeds(191, 191); 
      else
        motor_control.setSpeeds(-191, 191);
        vTaskDelay(300);
    }
    Follow();
  }
}

/**
 * @todo Implementar a estratégia Woodpecker, que consiste em pequenas acelerações ("bicadinhas")
 * antes de uma eventual aceleração.
 */ 
void Woodpecker(){
  if (state == kRunning){
    for (;;){}
  }
}

/// @brief Estratégia que segue o inimigo tentando centralizá-lo para acelerar em sua direção.
void Follow(){
  if (state == kRunning){
    x = WaitForSensorEvents();
    if (x & EVENT_QRE){
      if (x & EVENT_SENSOR1)
        LineDetectedProtocol(kLeft);
      else if (x & EVENT_SENSOR4)
        LineDetectedProtocol(kRight);
      else
        LineDetectedProtocol(kLeft);
    }
    if (x & EVENT_SENSOR1 ||
       ((x & EVENT_SENSOR1) && (x & EVENT_SENSOR2)) ||
       ((x & EVENT_SENSOR1) && (x & EVENT_SENSOR2) && (x & EVENT_SENSOR3))){
      motor_control.setSpeeds(191, -191);
    }
    else if (x & EVENT_SENSOR4 ||
            ((x & EVENT_SENSOR4) && (x & EVENT_SENSOR3)) || 
            ((x & EVENT_SENSOR4) && (x & EVENT_SENSOR2) && (x & EVENT_SENSOR3))){
      motor_control.setSpeeds(-191, 191);
    }
    else if ((x & EVENT_SENSOR2) || (x & EVENT_SENSOR3) || ((x & EVENT_SENSOR2) && (x & EVENT_SENSOR3))){
      motor_control.setSpeeds(-255, -255);
    }
  }
}

/// @brief Protocolo do que deve ser feito de imediato ao detectar uma linha.
/// @param direction A direção para a qual o robô deve ir ao haver uma detecção.
void LineDetectedProtocol(Direction direction){
  motor_control.setSpeeds(-255, 255);
  vTaskDelay(pdMS_TO_TICKS(300));
  if (direction == kLeft)
    motor_control.setSpeeds(191, -191);
  else
    motor_control.setSpeeds(-191, 191);
  vTaskDelay(pdMS_TO_TICKS(300));
}

/// @brief Para completamente os motores.
void KillMotors(){
  motor_control.setSpeeds(0, 0);
}