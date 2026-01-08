#define DECODE_SONY // Limita biblioteca de IR ao protocolo Sony.

#include <Arduino.h>
#include <globals.h>
#include <IRremote.hpp>
#include <enumerators.hpp>
#include <Itamotorino.h>
#include <nvs_handler.hpp>
#include <QTRSensors.h>
#include <esp_log.h>
#include <detection_functions.hpp>
#include <map>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// protótipos de função

void CalibrateSensors(const bool use_nvs_calibration);
void DecodeIrSignal();

void DetectLine();

void RunStrategy();
void RadarEsquerdo();
void RadarDireito();
void CurvaAberta();
void Woodpecker();
void Follow();
void LineDetectedProtocol(Direction direction);
void KillMotors();

void CoreTaskOne(void *pvParameters);
void CoreTaskZero(void *pvParameters);

// globais e constantes

unsigned long time_1, time_2;

RobotState state = RobotState::kReady;
Strategy strat = Strategy::kRadarEsq;

QTRSensorsAnalog qtra((unsigned char[]){QTR1, QTR2}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR);
uint32_t sensor_values[NUM_SENSORS];

const char* kMinKeys[NUM_SENSORS] = {"kMinQtr1", "kMinQtr2"};
const char* kMaxKeys[NUM_SENSORS] = {"kMaxQtr1", "kMaxQtr2"};

const std::map<uint16_t, RobotState> states = {
  {static_cast<uint16_t>(RobotState::kReady), RobotState::kReady}, 
  {static_cast<uint16_t>(RobotState::kRunning), RobotState::kRunning},
  {static_cast<uint16_t>(RobotState::kStop), RobotState::kStop},
};

const std::map<uint16_t, Strategy> strats = {
  {static_cast<uint16_t>(Strategy::kCurvaAberta), Strategy::kCurvaAberta}, 
  {static_cast<uint16_t>(Strategy::kFollowOnly), Strategy::kFollowOnly},
  {static_cast<uint16_t>(Strategy::kRadarDir), Strategy::kRadarDir},
  {static_cast<uint16_t>(Strategy::kRadarEsq), Strategy::kRadarEsq},
};

const std::map<int, int> sensor_pins_and_bits = {
  {SENSOR1, EVENT_SENSOR1},
  {SENSOR2, EVENT_SENSOR2},
  {SENSOR3, EVENT_SENSOR3},
  {SENSOR4, EVENT_SENSOR4},
};

const int input_pins[4] = {SENSOR1, SENSOR2, SENSOR3, SENSOR4};
const int output_pins[3] = {LED_BUILTIN, LED1, LED2};

Itamotorino itamotorino = Itamotorino(AIN1, AIN2, BIN1, BIN2, PWMA, PWMB);

TaskHandle_t core_0;
TaskHandle_t core_1;

EventGroupHandle_t sensor_events;
EventBits_t x;

NVSHandler nvs_handler("QTR Min Max Values");

const char* const TAG = "Main program";

void setup(){
  if (DEBUG_MODE){
    Serial.begin(115200);
    while (!Serial){;}
    ESP_LOGI(TAG, "Initialized with state: %d and strat: %d\n", state, strat);
  }
  analogReadResolution(10);
  disableCore0WDT();
  disableCore1WDT();
  itamotorino.setupADC(PWM_CH1, PWM_FREQ, PWM_RES, PWM_CH2, PWM_FREQ, PWM_RES);
  sensor_events = xEventGroupCreate();

  for (auto& pin : input_pins)
    pinMode(pin, INPUT);
  for (auto& pin : output_pins)
    pinMode(pin, OUTPUT);
  
  IrReceiver.begin(IR, true, LED_BUILTIN);  // LED_BUILTIN como feedback de resposta.
  IrReceiver.enableIRIn();                  // Linha possivelmente desnecessária.

  CalibrateSensors(false); // mudar para true quando já calibrado
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

void CalibrateSensors(const bool use_nvs_calibration){
  qtra.calibrate(); // chamada única para inicializar os valores
  uint32_t min_value, max_value;
  if (nvs_handler.StartStorage()){
    if (use_nvs_calibration){
      for(int i = 0; i < NUM_SENSORS; i++){
        min_value = nvs_handler.ReadUnsignedIntFromNVS(kMinKeys[i]);
        max_value = nvs_handler.ReadUnsignedIntFromNVS(kMaxKeys[i]);
        if(min_value){
          qtra.calibratedMinimumOn[i] = min_value;
        }
        if(max_value){
          qtra.calibratedMinimumOn[i] = max_value;
        }

        if (DEBUG_MODE){
          ESP_LOGI(TAG, "Read %s = %d, %s = %d from NVS memory\n", kMinKeys[i], min_value, kMaxKeys[i], max_value);
        }
      }
    }
    else{
      digitalWrite(LED1, HIGH);
      for(int i = 0; i < 1200; i++){
        qtra.calibrate();
      }
      digitalWrite(LED2, LOW);
      if (nvs_handler.StartStorage()){
        for(int i = 0; i < NUM_SENSORS; i++){
          // talvez nem precise retornar bool e possa só ser uma checagem de nvs_ok
          nvs_handler.WriteUnsignedIntToNVS(kMinKeys[i], qtra.calibratedMinimumOn[i]);
          nvs_handler.WriteUnsignedIntToNVS(kMaxKeys[i], qtra.calibratedMaximumOn[i]);
          if (DEBUG_MODE){
            ESP_LOGI(TAG, "Wrote %s = %d, %s = %d from NVS memory\n", kMinKeys[i], qtra.calibratedMinimumOn[i], kMaxKeys[i], qtra.calibratedMaximumOn[i]);
          }
        }
      }
    }
    nvs_handler.CloseStorage();
  }
  else{
    // blink_error();
  }
}

void DecodeIrSignal(){
  IrReceiver.resume();
  if (DEBUG_MODE)
    ESP_LOGI(TAG, "IR module received command %d\n", IrReceiver.decodedIRData.command);

  if (states.find(IrReceiver.decodedIRData.command) != states.end()){
    switch (states.find(IrReceiver.decodedIRData.command)->second){
      case RobotState::kStop:
        state = RobotState::kStop;
        break;
      default:
        if (state == RobotState::kReady){
          state = states.find(IrReceiver.decodedIRData.command)->second;
        }
        break;
    }
  }

  if (strats.find(IrReceiver.decodedIRData.command) != strats.end()){
    if (state == RobotState::kReady){
      strat = strats.find(IrReceiver.decodedIRData.command)->second;
    }
  }

  if (DEBUG_MODE)
    ESP_LOGI(TAG, "Current state: %d - Currentn strat: %d\n", static_cast<uint16_t>(state), static_cast<uint16_t>(strat));
}

void DetectLine(){
  if (state == RobotState::kRunning){
    int line_info = qtra.readLine(sensor_values, QTR_EMITTERS_ON, true); // este valor é entre zero ou mil
    if (line_info <= 500){
      xEventGroupSetBits(sensor_events, EVENT_QRE_LEFT);
      xEventGroupClearBits(sensor_events, EVENT_QRE_RIGHT);
    }
    else{
      xEventGroupSetBits(sensor_events, EVENT_QRE_RIGHT);
      xEventGroupClearBits(sensor_events, EVENT_QRE_LEFT);
    }
  }
}

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

void RadarEsquerdo(){
  if (state == RobotState::kRunning){
    x = WaitForSensorEvents(sensor_events);
    if (!(x & EVENT_SENSOR1) && !(x & EVENT_SENSOR2) && !(x & EVENT_SENSOR3) && !(x & EVENT_SENSOR4)){
      itamotorino.setSpeeds(191, -191);
    }
    else{
      while(state == RobotState::kRunning)
        Follow();
    }
  }
}

void RadarDireito(){
  if (state == RobotState::kRunning){
    x = WaitForSensorEvents(sensor_events);
    if (!(x | EVENT_SENSOR1) && !(x | EVENT_SENSOR2) && !(x | EVENT_SENSOR3) && !(x | EVENT_SENSOR4)){
      itamotorino.setSpeeds(-191, 191);
    }
    else{
      while(state == RobotState::kRunning)
        Follow();
    }
  }
}

void CurvaAberta(){
  time_1 = millis();
  if (state == RobotState::kRunning){
    Direction direction;
    x = WaitForSensorEvents(sensor_events);
    if (x & EVENT_SENSOR1){
      direction = Direction::kLeft;
      itamotorino.setSpeeds(-191, 191);
    }
    else if (x & EVENT_SENSOR4){
      direction = Direction::kRight;
      itamotorino.setSpeeds(191, 191);
    }
    if (x & EVENT_QRE_LEFT || x & EVENT_QRE_RIGHT){
      if (direction == Direction::kLeft)
        LineDetectedProtocol(Direction::kRight);
      else
        LineDetectedProtocol(Direction::kLeft);
    }
    if (millis() - time_1 >= 2000){
      if (direction == Direction::kLeft)
        itamotorino.setSpeeds(191, 191); 
      else
        itamotorino.setSpeeds(-191, 191);
        vTaskDelay(300);
    }
    Follow();
  }
}

void Woodpecker(){
  if (state == RobotState::kRunning){
    for (;;){}
  }
}

void Follow(){
  if (state == RobotState::kRunning){
    x = WaitForSensorEvents(sensor_events);
    if (x & EVENT_QRE_LEFT || x & EVENT_QRE_RIGHT){
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
      itamotorino.setSpeeds(191, -191);
    }
    else if (x & EVENT_SENSOR4 ||
            ((x & EVENT_SENSOR4) && (x & EVENT_SENSOR3)) || 
            ((x & EVENT_SENSOR4) && (x & EVENT_SENSOR2) && (x & EVENT_SENSOR3))){
      itamotorino.setSpeeds(-191, 191);
    }
    else if ((x & EVENT_SENSOR2) || (x & EVENT_SENSOR3) || ((x & EVENT_SENSOR2) && (x & EVENT_SENSOR3))){
      itamotorino.setSpeeds(-255, -255);
    }
  }
}

void LineDetectedProtocol(Direction direction){
  itamotorino.setSpeeds(-255, 255);
  vTaskDelay(pdMS_TO_TICKS(300));
  if (direction == Direction::kLeft)
    itamotorino.setSpeeds(191, -191);
  else
    itamotorino.setSpeeds(-191, 191);
  vTaskDelay(pdMS_TO_TICKS(300));
}

void KillMotors(){
  itamotorino.setSpeeds(0, 0);
}

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

void CoreTaskZero(void *pvParameters){
  for(;;){
    if (IrReceiver.decode())
      DecodeIrSignal();
    DetectEnemies(sensor_events, sensor_pins_and_bits);
    DetectLine();
  }
}