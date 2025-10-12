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

// Bits de eventos

#define EVENT_SENSOR1 (1<<0)
#define EVENT_SENSOR2 (1<<1)
#define EVENT_SENSOR3 (1<<2)
#define EVENT_SENSOR4 (1<<3)
#define EVENT_QRE (1<<4)

#include <enumerators.h>
#include <nvs_control.hpp>
#include <motor_control.hpp>

#include <esp_ipc.h>
#include <Arduino.h>
#include <QTRSensors.h>
#include <Itamotorino.h>
#include <freertos/task.h>
#include <freertos/FreeRTOS.h>
#include <ir_control.hpp>
#include <detection_control.hpp>

// Configurações para sensores QTR

#define NUM_SENSORS 2
#define NUM_SAMPLES_PER_SENSOR 4

// Pino do receptor IR

constexpr uint8_t kIrPin = 17;

// Pinos dos LEDs

constexpr uint8_t kLed1 = 18;
constexpr uint8_t kLed2 = 19;

// Constantes de configuração

constexpr bool kDebug = false;            // Se true, habilita Serial e mensagens.
// constexpr bool kWhiteLine = true;         // Se true, envia valores quando a refletância for alta.
constexpr bool kUseNVSCalibration = true; // Se true, usa a calibração na memória não volátil.

// Protótipos de função

void CoreTaskOne(void *pvParameters);
void CoreTaskZero(void *pvParameters);

void CalibrateSensors();    
void RunStrategy();
void Radar(Direction d);
void CurvaAberta();
void Woodpecker();
void Follow();
void LineDetectedProtocol(Direction direction);

// Globais

RobotState state = RobotState::kReady;
Strategy strat = Strategy::kRadarEsq;

QTRSensorsAnalog qtra((unsigned char[]){kQtr1, kQtr2}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR);
uint32_t sensor_values[NUM_SENSORS];

const char* kMinKeys[NUM_SENSORS] = {"kMinQtr1", "kMinQtr2"};
const char* kMaxKeys[NUM_SENSORS] = {"kMaxQtr1", "kMaxQtr2"};

EventGroupHandle_t sensor_events;
EventBits_t x;

MotorControl motor_control = MotorControl();
NVSControl nvs_control = NVSControl("Valores QTR");
IRControl ir_control = IRControl(kIrPin, &state, &strat);
DetectionControl detection_control = DetectionControl(&state, &sensor_events, &qtra, sensor_values);

unsigned long time_1, time_2;

TaskHandle_t core_0;
TaskHandle_t core_1;

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
    Serial.println(static_cast<uint16_t>(state));
    Serial.print("Starting strat: ");
    Serial.println(static_cast<uint16_t>(strat));
  }
  sensor_events = xEventGroupCreate();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(kSensor1, INPUT);
  pinMode(kSensor2, INPUT);
  pinMode(kSensor3, INPUT);
  pinMode(kSensor4, INPUT);
  pinMode(kLed1, OUTPUT);
  pinMode(kLed2, OUTPUT);
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
    if (state != RobotState::kStop)
      RunStrategy();
    else{
      motor_control.StopMotors();
      vTaskDelete(core_1);
    }
  }
}

/// @brief Task para a detecção de todos os sinais necessários para o robô (receptor IR, sensores IR e sensores QTR).
/// @param pvParameters Ponteiro para void que pode conter dados importantes à tarefa. Inutilizado neste caso.
void CoreTaskZero(void *pvParameters){
  for(;;){
    ir_control.ExpectIRSignal();
    detection_control.DetectEnemies();
    detection_control.DetectLine();
  }
}

/// @brief Função para calibrar os sensores QTR. Obtém a calibração da memória não volátil ou calibra os sensores.
void CalibrateSensors(){
  qtra.calibrate();
  if (kUseNVSCalibration && nvs_control.getNVSOk()){
    for(int i = 0; i < NUM_SENSORS; i++){
      if (nvs_control.ReadUnsignedIntFromNVS(kMinKeys[i]) != -1){
        qtra.calibratedMinimumOn[i] = nvs_control.ReadUnsignedIntFromNVS(kMinKeys[i]);
      }
      if (nvs_control.ReadUnsignedIntFromNVS(kMaxKeys[i]) != -1){
        qtra.calibratedMaximumOn[i] = nvs_control.ReadUnsignedIntFromNVS(kMaxKeys[i]);
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
  }
  else{
    digitalWrite(kLed2, HIGH);
    for(int i = 0; i < 1200; i++){
      qtra.calibrate();
    }
    digitalWrite(kLed2, LOW);
    for(int i = 0; i < NUM_SENSORS; i++){
      nvs_control.WriteUnsignedIntToNVS(kMinKeys[i], qtra.calibratedMinimumOn[i]);
      nvs_control.WriteUnsignedIntToNVS(kMaxKeys[i], qtra.calibratedMaximumOn[i]);
      if (kDebug){
        Serial.println("");
        Serial.println(qtra.calibratedMinimumOn[i]);
        Serial.println(qtra.calibratedMaximumOn[i]);
        Serial.println("== NVS written ==");
        Serial.print(kMinKeys[i]);
        Serial.print(" = ");
        Serial.println(nvs_control.ReadUnsignedIntFromNVS(kMinKeys[i]));
        Serial.print(kMaxKeys[i]);
        Serial.print(" = ");
        Serial.println(nvs_control.ReadUnsignedIntFromNVS(kMaxKeys[i]));
      }
    }
  }
  nvs_control.CloseStorage();
}

/// @brief Função para executar a estratégia selecionada pelo operador.
void RunStrategy(){
  switch (strat){
    case Strategy::kRadarEsq:
      Radar(Direction::kLeft);
      break;
    case Strategy::kRadarDir:
      Radar(Direction::kRight);
      break;
    case Strategy::kCurvaAberta:
      CurvaAberta();
      break;
    case Strategy::kFollowOnly:
      Follow();
      break;
    case Strategy::kWoodPecker:
      Woodpecker();
      break;
    default:
      break;
  }
}

/**
 * @brief Estratégia para procurar o inimigo virando para uma direção determinada se não houver 
 * nenhuma detecção. Ao detectar, muda a estratégia para Follow();
 */
void Radar(Direction d){
  if (state == RobotState::kRunning){
    x = detection_control.GetSensorEvents(30);
    if (!(x & EVENT_SENSOR1) && !(x & EVENT_SENSOR2) && !(x & EVENT_SENSOR3) && !(x & EVENT_SENSOR4)){
      motor_control.Turn(d);
    }
    else{
      while(state == RobotState::kRunning)
        Follow();
    }
  }
}

/// @brief Estratégia que faz uma curva aberta antes de entrar em Follow();
void CurvaAberta(){
  time_1 = millis();
  if (state == RobotState::kRunning){
    Direction direction;
    x = detection_control.GetSensorEvents(30);
    if (x & EVENT_SENSOR1){
      direction = Direction::kLeft;
      motor_control.Turn(direction);
    }
    else if (x & EVENT_SENSOR4){
      direction = Direction::kRight;
      motor_control.Turn(direction);
    }
    if (x & EVENT_QRE){
      if (direction == Direction::kLeft)
        LineDetectedProtocol(Direction::kRight);
      else
        LineDetectedProtocol(Direction::kLeft);
    }
    if (millis() - time_1 >= 2000){
      if (direction == Direction::kLeft)
        motor_control.Turn(direction); 
      else
        motor_control.Turn(Direction::kRight);
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
  if (state == RobotState::kRunning){
    for (;;){}
  }
}

/// @brief Estratégia que segue o inimigo tentando centralizá-lo para acelerar em sua direção.
void Follow(){
  if (state == RobotState::kRunning){
    x = detection_control.GetSensorEvents(30);
    if (x & EVENT_QRE){
      if (x & EVENT_SENSOR1)
        LineDetectedProtocol(Direction::kLeft);
      else if (x & EVENT_SENSOR4)
        LineDetectedProtocol(Direction::kRight);
      else
        LineDetectedProtocol(Direction::kLeft);
    }
    if (x & EVENT_SENSOR1 ||
       ((x & EVENT_SENSOR1) && (x & EVENT_SENSOR2)) ||
       ((x & EVENT_SENSOR1) && (x & EVENT_SENSOR2) && (x & EVENT_SENSOR3))){
      motor_control.Turn(Direction::kLeft);
    }
    else if (x & EVENT_SENSOR4 ||
            ((x & EVENT_SENSOR4) && (x & EVENT_SENSOR3)) || 
            ((x & EVENT_SENSOR4) && (x & EVENT_SENSOR2) && (x & EVENT_SENSOR3))){
      motor_control.Turn(Direction::kRight);
    }
    else if ((x & EVENT_SENSOR2) || (x & EVENT_SENSOR3) || ((x & EVENT_SENSOR2) && (x & EVENT_SENSOR3))){
      motor_control.Accelerate(Direction::kForward);
    }
  }
}

/// @brief Protocolo do que deve ser feito de imediato ao detectar uma linha.
/// @param direction A direção para a qual o robô deve ir ao haver uma detecção.
void LineDetectedProtocol(Direction direction){
  motor_control.Accelerate(Direction::kBackward);
  vTaskDelay(pdMS_TO_TICKS(300));
  motor_control.Turn(direction);
  vTaskDelay(pdMS_TO_TICKS(300));
}