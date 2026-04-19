#define DECODE_SONY // Limita biblioteca de IR ao protocolo Sony.

#include <map>
#include <functional>
#include <Arduino.h>
#include <globals.h>
#include <QTRSensors.h>
#include <IRremote.hpp>
#include <Itamotorino.h>
#include <enumerators.hpp>
#include <NVSHandler.hpp>
#include <detection_functions.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// protótipos de função

void CalibrateQTRSensors(QTRCalibration qtr_calibration);
void BlinkNTimes(uint8_t n);
void DecodeIrSignal();

void DetectLine();

void RunStrategy();
void RadarEsquerdo();
void RadarDireito();
void Woodpecker();
void Follow();
void LineDetectedProtocol(Direction direction);
void KillMotors();
void PulseMotors(uint8_t qty);
void ControlMotors(Direction d);

void MovementTask(void *pvParameters);
void SensingTask(void *pvParameters);

// globais e constantes

unsigned long time_1, time_2;

FightState state = FightState::kReady;
Strategy strat = Strategy::kRadarEsq;

QTRSensors qtr_handler;
uint16_t sensor_values[NUM_SENSORS];

const char* kMinKeys[NUM_SENSORS] = {"kMinQtr1", "kMinQtr2"};
const char* kMaxKeys[NUM_SENSORS] = {"kMaxQtr1", "kMaxQtr2"};

const std::map<uint16_t, FightState> states = {
  {static_cast<uint16_t>(FightState::kReady), FightState::kReady}, 
  {static_cast<uint16_t>(FightState::kRunning), FightState::kRunning},
  {static_cast<uint16_t>(FightState::kStop), FightState::kStop},
};

const std::map<uint16_t, Strategy> strats = {
  {static_cast<uint16_t>(Strategy::kFollowOnly), Strategy::kFollowOnly},
  {static_cast<uint16_t>(Strategy::kRadarDir), Strategy::kRadarDir},
  {static_cast<uint16_t>(Strategy::kRadarEsq), Strategy::kRadarEsq},
  {static_cast<uint16_t>(Strategy::kWoodPecker), Strategy::kWoodPecker},
};

/* Tem repetição de dados aqui. Não tem um problema maior por ser ESP32 devkit, mas não deveria estar aqui. */
const std::map<int, int> sensor_pins_and_bits = {
  {IR_SENSOR_1, EVENT_BIT_SENSOR_1},
  {IR_SENSOR_2, EVENT_BIT_SENSOR_2},
  {IR_SENSOR_3, EVENT_BIT_SENSOR_3},
  {IR_SENSOR_4, EVENT_BIT_SENSOR_4},
};

const int input_pins[4] = {IR_SENSOR_1, IR_SENSOR_2, IR_SENSOR_3, IR_SENSOR_4};

Itamotorino itamotorino = Itamotorino(AIN1, AIN2, BIN1, BIN2, PWMA, PWMB);

TaskHandle_t movement_task;
TaskHandle_t sensing_task;

EventGroupHandle_t sensor_events;
EventBits_t x;

NVSHandler nvs_handler("QTR Values");

void setup(){
  if (DEBUG_MODE){
    Serial.begin(115200);
    while (!Serial){;}
  }
  analogReadResolution(10);
  disableCore0WDT();
  disableCore1WDT();
  itamotorino.setupADC(PWM_CH1, PWM_FREQ, PWM_RES, PWM_CH2, PWM_FREQ, PWM_RES);
  sensor_events = xEventGroupCreate();

  for (auto& pin : input_pins) pinMode(pin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  IrReceiver.begin(IR, true, LED_BUILTIN);
  IrReceiver.enableIRIn();

  CalibrateQTRSensors(QTRCalibration::kUseCalibration); // mudar quando já calibrado
  xTaskCreatePinnedToCore(MovementTask, "MovementTask", STACK_DEPTH,
                          NULL, TASK_PRIORITY, &movement_task, 1);

  xTaskCreatePinnedToCore(SensingTask, "SensingTask", STACK_DEPTH,
                          NULL, TASK_PRIORITY, &sensing_task, 0);
}

void loop(){}

void CalibrateQTRSensors(QTRCalibration qtr_calibration){
  qtr_handler.calibrate();
  if (nvs_handler.StartStorage(NVS_READWRITE) == ESP_OK){
    if (qtr_calibration == QTRCalibration::kUseCalibration){
      digitalWrite(LED_BUILTIN, HIGH);
      for (unsigned int i = 0; i < 1200; ++i) qtr_handler.calibrate();
      for (unsigned int i = 0; i < NUM_SENSORS; i++){
        ESP_ERROR_CHECK(nvs_handler.WriteUInt16(kMinKeys[i], &qtr_handler.calibrationOn.minimum[i]));
        ESP_ERROR_CHECK(nvs_handler.WriteUInt16(kMaxKeys[i], &qtr_handler.calibrationOn.maximum[i]));
      }
    }
    else {
      for (unsigned int i = 0; i < NUM_SENSORS; i++){
        ESP_ERROR_CHECK(nvs_handler.ReadUInt16(kMinKeys[i], &qtr_handler.calibrationOn.minimum[i]));
        ESP_ERROR_CHECK(nvs_handler.ReadUInt16(kMaxKeys[i], &qtr_handler.calibrationOn.maximum[i]));
      }
    }
    nvs_handler.CloseStorage();    
  }
  else{
    if (DEBUG_MODE) Serial.println("Erro na leitura/escrita de NVS.");
  }
}

void BlinkNTimes(uint8_t n){
  while (n > 0){
    --n;
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(200);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(200);
  }
}

void DecodeIrSignal(){
  IrReceiver.resume();

  if (states.find(IrReceiver.decodedIRData.command) != states.end()){
    switch (states.find(IrReceiver.decodedIRData.command)->second){
      case FightState::kStop:
        state = FightState::kStop;
        break;
      default:
        if (state == FightState::kReady){
          state = states.find(IrReceiver.decodedIRData.command)->second;
        }
        break;
    }
  }

  if (strats.find(IrReceiver.decodedIRData.command) != strats.end()){
    if (state == FightState::kReady){
      strat = strats.find(IrReceiver.decodedIRData.command)->second;
    }
  }

  if (DEBUG_MODE){
    Serial.print("Current state: ");
    Serial.println(static_cast<uint16_t>(state));
    Serial.print("Current strat: ");
    Serial.println(static_cast<uint16_t>(strat));
  }
}

void DetectLine(){
  if (state == FightState::kRunning){
    uint16_t line_info = qtr_handler.readLineWhite(sensor_values, QTRReadMode::On); // este valor é entre zero ou mil
    if (DEBUG_MODE) Serial.println(line_info);
    
    if (sensor_values[0] < 500 && sensor_values[1] > 500){
      xEventGroupSetBits(sensor_events, EVENT_QRE_RIGHT);
      xEventGroupClearBits(sensor_events, EVENT_QRE_LEFT);
    }
    else if (sensor_values[0] > 500 && sensor_values[1] < 500){
      xEventGroupSetBits(sensor_events, EVENT_QRE_LEFT);
      xEventGroupClearBits(sensor_events, EVENT_QRE_RIGHT);
    }
    else if (sensor_values[0] < 500 && sensor_values[1] < 500){
      xEventGroupSetBits(sensor_events, EVENT_QRE_LEFT);
      xEventGroupSetBits(sensor_events, EVENT_QRE_RIGHT);
    }
    else if (sensor_values[0] > 500 && sensor_values[1] > 500){
      xEventGroupClearBits(sensor_events, EVENT_QRE_LEFT);
      xEventGroupClearBits(sensor_events, EVENT_QRE_RIGHT);
    }
  }
}

void RunStrategy(){ // isso podia ser um map<Strategy, void (*function)()> ?
  switch (strat){
    case Strategy::kRadarEsq:
      RadarEsquerdo();
      break;
    case Strategy::kRadarDir:
      RadarDireito();
      break;
    case Strategy::kFollowOnly:
      Follow();
      break;
    case Strategy::kWoodPecker:
      Woodpecker();
      break;
    default: // Afinal, default é um estado "impossível" e desnecessário para o nosso caso, não?
      break;
  }
}

void RadarEsquerdo(){
  if (state == FightState::kRunning){
    x = WaitForSensorEvents(sensor_events);
    if (!(x & EVENT_BIT_SENSOR_1) && !(x & EVENT_BIT_SENSOR_2) && !(x & EVENT_BIT_SENSOR_3) && !(x & EVENT_BIT_SENSOR_4)){
      ControlMotors(Direction::kLeft);
    }
    else{
      while(state == FightState::kRunning)
        Follow();
    }
  }
}

void RadarDireito(){
  if (state == FightState::kRunning){
    x = WaitForSensorEvents(sensor_events);
    if (!(x | EVENT_BIT_SENSOR_1) && !(x | EVENT_BIT_SENSOR_2) && !(x | EVENT_BIT_SENSOR_3) && !(x | EVENT_BIT_SENSOR_4)){
      ControlMotors(Direction::kLeft);
    }
    else{
      while(state == FightState::kRunning)
        Follow();
    }
  }
}

void Woodpecker(){
  if (state == FightState::kRunning){
    PulseMotors(WOODPECKER_PULSES);
    vTaskDelay(pdMS_TO_TICKS(1000));
    while(state == FightState::kRunning) Follow();
  }
}

void Follow(){
  if (state == FightState::kRunning){
    x = WaitForSensorEvents(sensor_events);

    if (x & EVENT_BIT_SENSOR_1 ||
       ((x & EVENT_BIT_SENSOR_1) && (x & EVENT_BIT_SENSOR_2)) ||
       ((x & EVENT_BIT_SENSOR_1) && (x & EVENT_BIT_SENSOR_2) && (x & EVENT_BIT_SENSOR_3))){
      ControlMotors(Direction::kLeft);
    }
    else if (x & EVENT_BIT_SENSOR_4 ||
            ((x & EVENT_BIT_SENSOR_4) && (x & EVENT_BIT_SENSOR_3)) || 
            ((x & EVENT_BIT_SENSOR_4) && (x & EVENT_BIT_SENSOR_2) && (x & EVENT_BIT_SENSOR_3))){
      ControlMotors(Direction::kRight);          
    }
    else if ((x & EVENT_BIT_SENSOR_2) || (x & EVENT_BIT_SENSOR_3) || ((x & EVENT_BIT_SENSOR_2) && (x & EVENT_BIT_SENSOR_3))){
      ControlMotors(Direction::kForward);
    }
  }
}

void LineDetectedProtocol(Direction direction){
  ControlMotors(Direction::kBackward);
  vTaskDelay(pdMS_TO_TICKS(300));
  ControlMotors(direction);
  vTaskDelay(pdMS_TO_TICKS(300));
}

void KillMotors(){
  itamotorino.setSpeeds(0, 0);
}

void PulseMotors(uint8_t qty){
  while (qty){
    ControlMotors(Direction::kForward);
    vTaskDelay(pdMS_TO_TICKS(100));
    KillMotors();
    vTaskDelay(pdMS_TO_TICKS(1000));
    --qty;
  }
}

void MovementTask(void *pvParameters){
  for(;;){
    if (state != FightState::kStop)
      RunStrategy();
    else{
      KillMotors();
      vTaskDelete(movement_task);
    }
  }
}

void SensingTask(void *pvParameters){
  for(;;){
    if (IrReceiver.decode())
      DecodeIrSignal();
    DetectEnemies(sensor_events, sensor_pins_and_bits);
    DetectLine();
  }
}

void ControlMotors(Direction d){
  switch (d){
    case Direction::kBackward:
      itamotorino.setSpeeds(255, 255);
      break;
    case Direction::kForward:
      itamotorino.setSpeeds(-255, -255);
      break;
    case Direction::kLeft:
      itamotorino.setSpeeds(191, -191);
      break;
    case Direction::kRight:
      itamotorino.setSpeeds(-191, 191);
      break;
    default:
      break;
  }
}