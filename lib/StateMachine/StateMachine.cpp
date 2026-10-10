#include "StateMachine.hpp"

StateMachine::StateMachine(EventGroupHandle_t enemy_handle, EventGroupHandle_t qtr_handle){
  this->enemy_handle = enemy_handle;
  this->qtr_handle = qtr_handle;
}

bool StateMachine::ParseRobotState(const uint16_t command){
  Serial.println(command);
  /* if (states.fight_state == FightState::kStop) {
    states.fight_state = FightState::kStop;

    return true;
  } else */
  switch (command){
    case static_cast<uint16_t>(FightState::kReady):
      states.fight_state = FightState::kReady;
      return true;
    case static_cast<uint16_t>(FightState::kFighting):
      states.fight_state = FightState::kFighting;
      return true;
    case static_cast<uint16_t>(FightState::kStop):
      states.fight_state = FightState::kStop;
      return true;
    default:
      return false;
  }
}

bool StateMachine::ParseStrategy(const uint16_t command){
  switch (command){
    case static_cast<uint16_t>(Strategy::kFollowEnemy):
      states.strategy = Strategy::kFollowEnemy;
      return true;
    case static_cast<uint16_t>(Strategy::kSearchLeft):
      states.strategy = Strategy::kSearchLeft;
      return true;
    case static_cast<uint16_t>(Strategy::kSearchRight):
      states.strategy = Strategy::kSearchRight;
      return true;
    default:
      return false;
  }
}

void StateMachine::UpdateIRReceiverState(const uint16_t command){
  if (!ParseRobotState(command)) ParseStrategy(command);
}

void StateMachine::UpdateSensorState(){
  if (enemy_handle == nullptr) return;
  enemy_bits = xEventGroupWaitBits(
    enemy_handle, 
    LEFT_SENSOR_BIT | FRONT_LEFT_SENSOR_BIT | CENTER_SENSOR_BIT | FRONT_RIGHT_SENSOR_BIT | RIGHT_SENSOR_BIT,
    pdTRUE,
    pdFALSE,
    pdMS_TO_TICKS(5));
  
  Serial.println(enemy_bits, BIN);
  switch (enemy_bits){
    case 0b00100:
    case 0b01110:
      states.sensor_state = states.last_state = SensorState::kFront;
      break;
    case 0b00001:
      states.sensor_state = states.last_state = SensorState::kLeft;
      break;
    case 0b10000:
      states.sensor_state = states.last_state = SensorState::kRight;
      break;
    case 0b00010:
    case 0b00011:
    case 0b00111:
      states.sensor_state = states.last_state = SensorState::kFrontLeft;
      break;
    case 0b01000:
    case 0b11000:
    case 0b11100:
      states.sensor_state = states.last_state = SensorState::kFrontRight;
      break;
    case 0b00000:
      states.sensor_state = SensorState::kNone;
      break;
    default:
      break;
  }
}

void StateMachine::UpdateQTRState(){
  if (qtr_handle == nullptr) return;
  qtr_bits = xEventGroupWaitBits(
    qtr_handle, 
    LEFT_QTR_BIT | RIGHT_QTR_BIT,
    pdFALSE,
    pdTRUE,
    pdMS_TO_TICKS(5));

  switch (qtr_bits){
    case 0b01:
      states.qtr_state = QTRState::kLeft;
      break;
    case 0b10:
      states.qtr_state = QTRState::kRight;
      break;
    case 0b11:
      states.qtr_state = QTRState::kBoth;
      break;
    case 0b00:
    default:
      states.qtr_state = QTRState::kNone;
      break;
  }
}

void StateMachine::UpdateState(){
  UpdateQTRState();
  UpdateSensorState();
}