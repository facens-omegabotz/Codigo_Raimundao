#ifndef HEADERS_STATE_MACHINE_H_
#define HEADERS_STATE_MACHINE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>

#include <globals.h>
#include <enumerators.hpp>

class StateMachine {
 public:
  StateMachine() = default;

  RobotState GetRobotState() const { return state_; }
  void SetRobotState(const RobotState state) { state_ = state; }

  Strategy GetSelectedStrategy() const { return selected_strategy_; }
  void SetSelectedStrategy(const Strategy strategy) { selected_strategy_ = strategy; }

  Direction GetDirection() const { return direction_; }
  void SetDirection(const Direction direction) { direction_ = direction; }

  int16_t GetLeftMotorSpeed() const { return left_motor_speed_; }
  int16_t GetRightMotorSpeed() const { return right_motor_speed_; }
  void SetMotorSpeeds(const int16_t left_speed, const int16_t right_speed) {
    left_motor_speed_ = left_speed;
    right_motor_speed_ = right_speed;
  }

  uint16_t GetLastIrCommand() const { return last_ir_command_; }

  void ApplyIrCommand(const uint16_t command,
                      const std::map<uint16_t, RobotState>& states,
                      const std::map<uint16_t, Strategy>& strategies) {
    last_ir_command_ = command;

    const auto state_it = states.find(command);
    if (state_it != states.end()) {
      if (state_it->second == RobotState::kStop) {
        state_ = RobotState::kStop;
      } else if (state_ == RobotState::kReady) {
        state_ = state_it->second;
      }
    }

    const auto strategy_it = strategies.find(command);
    if (strategy_it != strategies.end() && state_ == RobotState::kReady) {
      selected_strategy_ = strategy_it->second;
    }
  }

  void SetInfraredSensorBits(const EventBits_t bits) {
    infrared_sensor_bits_ = bits & (EVENT_SENSOR1 | EVENT_SENSOR2 | EVENT_SENSOR3 | EVENT_SENSOR4);
  }
  EventBits_t GetInfraredSensorBits() const { return infrared_sensor_bits_; }

  void SetQtrData(const int line_info, const uint32_t* values, const std::size_t count) {
    qtr_line_info_ = line_info;
    const std::size_t limit = count > qtr_values_.size() ? qtr_values_.size() : count;
    for (std::size_t i = 0; i < limit; ++i) {
      qtr_values_[i] = values[i];
    }
  }

  int GetQtrLineInfo() const { return qtr_line_info_; }
  const std::array<uint32_t, NUM_SENSORS>& GetQtrSensorValues() const { return qtr_values_; }

 private:
  RobotState state_ = RobotState::kReady;
  Strategy selected_strategy_ = Strategy::kRadarEsq;
  Direction direction_ = Direction::kFront;
  int16_t left_motor_speed_ = 0;
  int16_t right_motor_speed_ = 0;
  uint16_t last_ir_command_ = 0;
  EventBits_t infrared_sensor_bits_ = 0;
  int qtr_line_info_ = 0;
  std::array<uint32_t, NUM_SENSORS> qtr_values_ = {};
};

#endif
