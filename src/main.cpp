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

#include <enumerators.h>
#include <nvs_operator.hpp>
#include <motor_control.hpp>

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

RobotState state = RobotState::kReady;
Strategy strat = Strategy::kRadarEsq;

QTRSensorsAnalog qtra((unsigned char[]){kQtr1, kQtr2}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR);
uint32_t sensor_values[NUM_SENSORS];

const char* kMinKeys[NUM_SENSORS] = {"kMinQtr1", "kMinQtr2"};
const char* kMaxKeys[NUM_SENSORS] = {"kMaxQtr1", "kMaxQtr2"};

MotorControl motor_control = MotorControl();
NVSOperator nvs_operator = NVSOperator("Valores QTR");

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
    Serial.println(static_cast<uint16_t>(state));
    Serial.print("Starting strat: ");
    Serial.println(static_cast<uint16_t>(strat));
  }
  // motor_control.setupADC(kPwmChannelM1, kPwmFreqM1, kPwmResolutionM1, 
  //                        kPwmChannelM2, kPwmFreqM2, kPwmResolutionM2);
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
    if (state != RobotState::kStop)
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
  if (state != RobotState::kStop && state != RobotState::kRunning 
      && IrReceiver.decodedIRData.command == static_cast<uint16_t>(RobotState::kReady))
    state = RobotState::kReady;

  if (state == RobotState::kReady){
    switch(IrReceiver.decodedIRData.command){
      case static_cast<uint16_t>(RobotState::kRunning):
        state = RobotState::kRunning;
        break;

      case static_cast<uint16_t>(Strategy::kRadarEsq):
        strat = Strategy::kRadarEsq;
        break;

      case static_cast<uint16_t>(Strategy::kRadarDir):
        strat = Strategy::kRadarDir;
        break;

      case static_cast<uint16_t>(Strategy::kCurvaAberta):
        strat = Strategy::kCurvaAberta;
        break;

      case static_cast<uint16_t>(Strategy::kFollowOnly):
        strat = Strategy::kFollowOnly;
        break;

      case static_cast<uint16_t>(Strategy::kWoodPecker):
        strat = Strategy::kWoodPecker;
      default:
        break;
    }
  }
  if (IrReceiver.decodedIRData.command == static_cast<uint16_t>(RobotState::kStop))
    state = RobotState::kStop;
  
  if (kDebug){
    Serial.print("Received the command: ");
    Serial.println(IrReceiver.decodedIRData.command);
    Serial.print("State after command: ");
    Serial.println(static_cast<uint16_t>(state));
    Serial.print("Strategy after command: ");
    Serial.println(static_cast<uint16_t>(strat));
  }
}

/// @brief Função para calibrar os sensores QTR. Obtém a calibração da memória não volátil ou calibra os sensores.
void CalibrateSensors(){
  qtra.calibrate();
  if (kUseNVSCalibration && nvs_operator.getNVSOk()){
    for(int i = 0; i < NUM_SENSORS; i++){
      if (nvs_operator.ReadUnsignedIntFromNVS(kMinKeys[i]) != -1){
        qtra.calibratedMinimumOn[i] = nvs_operator.ReadUnsignedIntFromNVS(kMinKeys[i]);
      }
      if (nvs_operator.ReadUnsignedIntFromNVS(kMaxKeys[i]) != -1){
        qtra.calibratedMaximumOn[i] = nvs_operator.ReadUnsignedIntFromNVS(kMaxKeys[i]);
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
      nvs_operator.WriteUnsignedIntToNVS(kMinKeys[i], qtra.calibratedMinimumOn[i]);
      nvs_operator.WriteUnsignedIntToNVS(kMaxKeys[i], qtra.calibratedMaximumOn[i]);
      if (kDebug){
        Serial.println("");
        Serial.println(qtra.calibratedMinimumOn[i]);
        Serial.println(qtra.calibratedMaximumOn[i]);
        Serial.println("== NVS written ==");
        Serial.print(kMinKeys[i]);
        Serial.print(" = ");
        Serial.println(nvs_operator.ReadUnsignedIntFromNVS(kMinKeys[i]));
        Serial.print(kMaxKeys[i]);
        Serial.print(" = ");
        Serial.println(nvs_operator.ReadUnsignedIntFromNVS(kMaxKeys[i]));
      }
    }
  }
  nvs_operator.CloseStorage();
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
    case Strategy::kRadarEsq:
      RadarEsquerdo();
      break;
    case Strategy::kRadarDir:
      RadarDireito();
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

/// @brief Função que lê os valores de cada um dos sensores infravermelhos e escreve ou limpa os bits no EventGroup global.
void DetectEnemies(){
  if (state == RobotState::kRunning){
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
  if (state == RobotState::kRunning){
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
  if (state == RobotState::kRunning){
    x = WaitForSensorEvents();
    if (!(x & EVENT_SENSOR1) && !(x & EVENT_SENSOR2) && !(x & EVENT_SENSOR3) && !(x & EVENT_SENSOR4)){
      motor_control.Turn(Direction::kLeft);
    }
    else{
      while(state == RobotState::kRunning)
        Follow();
    }
  }
}

/**
 * @brief Estratégia para procurar o inimigo virando para a direita se não houver nenhuma detecção. 
 * Ao detectar, muda a estratégia para Follow();
 */
void RadarDireito(){
  if (state == RobotState::kRunning){
    if (kDebug)
      Serial.println(x, BIN);
    x = WaitForSensorEvents();
    if (!(x | EVENT_SENSOR1) && !(x | EVENT_SENSOR2) && !(x | EVENT_SENSOR3) && !(x | EVENT_SENSOR4)){
      motor_control.Turn(Direction::kRight);
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
    x = WaitForSensorEvents();
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
    x = WaitForSensorEvents();
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