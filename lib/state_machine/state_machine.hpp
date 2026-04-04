#include <unordered_map>
#include <unordered_set>
#include <globals.h>
#include <enumerators.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <Arduino.h>

constexpr uint8_t kDefaultWaitTicks = 30;

const std::unordered_map<uint16_t, FightState> fight_states = {
  {static_cast<uint16_t>(FightState::kReady), FightState::kReady}, 
  {static_cast<uint16_t>(FightState::kRunning), FightState::kRunning},
  {static_cast<uint16_t>(FightState::kStop), FightState::kStop},
};

const std::unordered_map<uint16_t, Strategy> strats = {
  {static_cast<uint16_t>(Strategy::kCurvaAberta), Strategy::kCurvaAberta}, 
  {static_cast<uint16_t>(Strategy::kFollowOnly), Strategy::kFollowOnly},
  {static_cast<uint16_t>(Strategy::kRadarDir), Strategy::kRadarDir},
  {static_cast<uint16_t>(Strategy::kRadarEsq), Strategy::kRadarEsq},
  {static_cast<uint16_t>(Strategy::kWoodPecker), Strategy::kWoodPecker},
};

const std::unordered_map<uint8_t, uint8_t> events_per_ir_sensor = {
  {IR_SENSOR_1, EVENT_BIT_SENSOR_1},
  {IR_SENSOR_2, EVENT_BIT_SENSOR_2},
  {IR_SENSOR_3, EVENT_BIT_SENSOR_3},
  {IR_SENSOR_4, EVENT_BIT_SENSOR_4},
};

struct States {
  FightState fight_state {FightState::kReady};
  Strategy strategy {Strategy::kFollowOnly};
  Direction direction {Direction::kLeft};
  IrDetectionState ir_detection_state {IrDetectionState::kNone};
};

class StateMachine {
  private:
    States states;
    EventGroupHandle_t ir_sensor_events_handle;
    EventBits_t ir_sensor_event_bits;

  public:
    StateMachine();
    void WaitForIrSensorEvents(uint8_t wait_ticks); // is this acoplamento da silva?
    void ReadIrSensorEvents();
    void UpdateIrDetectionState();
    void SetFightState(uint16_t command);
};