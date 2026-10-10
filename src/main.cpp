#define DECODE_SONY

#include <Arduino.h>
#include <QTRSensors.h>
#include <IRremote.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "raimundao_macros.h"
#include "raimundao_pins.h"
#include "raimundao_types.hpp"
#include "StateMachine.hpp"
#include "MotorHandler.hpp"
#include "NVSHandler.hpp"
#include "StrategyExecutor.hpp"
#include "SensorHandlers.hpp"

Motor left_motor = {
  .pins = {
    .a_pin = AIN_1,
    .b_pin = AIN_2
  },
  .timer = MCPWM_TIMER_0,
  .duty_cycle_a = 0.0,
  .duty_cycle_b = 0.0,
  .dir = Direction::kNone,
};

Motor right_motor = {
  .pins = {
    .a_pin = BIN_1,
    .b_pin = BIN_2
  },
  .timer = MCPWM_TIMER_1,
  .duty_cycle_a = 0.0,
  .duty_cycle_b = 0.0,
  .dir = Direction::kNone,
};

mcpwm_config_t cfg = {
  .frequency = PWM_FREQ,
  .cmpr_a = CMPR_A,
  .cmpr_b = CMPR_B,
  .duty_mode = MCPWM_DUTY_MODE_0,
  .counter_mode = MCPWM_UP_COUNTER,
};

EnemySensorHandler enemy_sensor_handler;
LineSensorHandler line_sensor_handler;
StateMachine state_machine;
StrategyExecutor strategy_executor = StrategyExecutor(&cfg, &left_motor, &right_motor);
NVSHandler qtr_info = NVSHandler("QTR");

TaskHandle_t motor_task_handle, sensing_task_handle;

void SensorsTask(void* pvParameters);
void MotorsTask(void* pvParameters);

void setup(){
  Serial.begin(115200);
  while (!Serial){;}
  Serial.println("iniciou serial");
  disableCore0WDT();
  disableCore1WDT();

  state_machine = StateMachine(enemy_sensor_handler.event_handle, line_sensor_handler.event_handle);
  ESP_ERROR_CHECK(qtr_info.StartStorage(NVS_READWRITE));
  pinMode(LED_BUILTIN, OUTPUT);
  IrReceiver.begin(IR_RECEIVER, true, LED_BUILTIN);
  IrReceiver.enableIRIn();

  line_sensor_handler.Calibrate(QTRCalibration::kUseNVSValues, &qtr_info);

  Serial.println("calibracao concluida");

  xTaskCreatePinnedToCore(
    SensorsTask, 
    "sensors_task",
    TASK_STACK_DEPTH,
    NULL,
    1, 
    &sensing_task_handle,
    0  
  );

  xTaskCreatePinnedToCore(
    MotorsTask, 
    "motors_task",
    TASK_STACK_DEPTH,
    NULL,
    1, 
    &motor_task_handle,
    1
  );
}

void loop(){}

void SensorsTask(void* pvParameters){
  for (;;){
    if (IrReceiver.decode()){
      IrReceiver.resume();
      state_machine.UpdateIRReceiverState(IrReceiver.decodedIRData.command);
      Serial.print("Estado da luta: ");
      Serial.println((int)state_machine.states.fight_state);
      Serial.print("Estado da estrategia: ");
      Serial.println((int)state_machine.states.strategy);
    }
    if (state_machine.states.fight_state == FightState::kFighting){
      
      enemy_sensor_handler.Detect();
      // line_sensor_handler.Detect();
      state_machine.UpdateState();
    }

    if (state_machine.states.fight_state == FightState::kStop){
      vTaskDelete(sensing_task_handle);
    }
  }
}

void MotorsTask(void* pvParameters){
  for (;;){
    if (state_machine.states.fight_state == FightState::kFighting){
      strategy_executor.RunStrategy(state_machine);
    }
    if (state_machine.states.fight_state == FightState::kStop){
      strategy_executor.KillMotors();
      vTaskDelete(motor_task_handle);
    }
  }
}