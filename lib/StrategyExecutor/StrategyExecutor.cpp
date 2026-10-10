#include "StrategyExecutor.hpp"

StrategyExecutor::StrategyExecutor(mcpwm_config_t* cfg, Motor* motor_a, Motor* motor_b){
  motor_handle = MotorHandler(cfg, motor_a, motor_b);
  motor_handle.Init();
}

void StrategyExecutor::RunStrategy(StateMachine& state_machine){
  switch(state_machine.states.strategy){
    case Strategy::kFollowEnemy:
      FollowEnemy(state_machine);
      break;
    case Strategy::kSearchLeft:
      SearchLeft(state_machine);
      break;
    case Strategy::kSearchRight:
      SearchRight(state_machine);
      break;
    default:
      break;
  }
}

void StrategyExecutor::SearchLeft(StateMachine& state_machine){
  state_machine.UpdateState();
  if(state_machine.states.sensor_state == SensorState::kNone && state_machine.states.fight_state == FightState::kFighting){
    // VIRAR ESQUERDA
    motor_handle.SetDutyCycles(20.0, Direction::kForward, 20.0, Direction::kForward);
  }
  else{
    if (state_machine.states.fight_state == FightState::kFighting){
      FollowEnemy(state_machine);
    }
  }
}

void StrategyExecutor::SearchRight(StateMachine& state_machine){
  state_machine.UpdateState();
  if(state_machine.states.sensor_state == SensorState::kNone && state_machine.states.fight_state == FightState::kFighting){
    // VIRAR DIREITA
    motor_handle.SetDutyCycles(20.0, Direction::kBackward, 20.0, Direction::kBackward);
  }
  else{
    if (state_machine.states.fight_state == FightState::kFighting){
      FollowEnemy(state_machine);
    }
  }
}

void StrategyExecutor::FollowEnemy(StateMachine &state_machine){
  if (state_machine.states.fight_state == FightState::kFighting){
    state_machine.UpdateState();
    if (state_machine.states.qtr_state != QTRState::kNone){
      switch (state_machine.states.qtr_state){
        case QTRState::kLeft:
          motor_handle.SetDutyCycles(40.0, Direction::kBackward, 40.0, Direction::kBackward);
          vTaskDelay(pdMS_TO_TICKS(400));
          motor_handle.SetDutyCycles(40.0, Direction::kBackward, 40.0, Direction::kForward);
          break;
        case QTRState::kRight:
          motor_handle.SetDutyCycles(40.0, Direction::kBackward, 40.0, Direction::kBackward);
          vTaskDelay(pdMS_TO_TICKS(400));
          motor_handle.SetDutyCycles(40.0, Direction::kForward, 40.0, Direction::kBackward);
          break;
        case QTRState::kNone:
          motor_handle.SetDutyCycles(40.0, Direction::kBackward, 40.0, Direction::kBackward);
          break;
      }
    }
    else{
      switch (state_machine.states.sensor_state){
        case SensorState::kFront:
          motor_handle.SetDutyCycles(100.0, Direction::kBackward, 100.0, Direction::kForward);
          break;
        case SensorState::kLeft:
          motor_handle.SetDutyCycles(60.0, Direction::kForward, 60.0, Direction::kForward);
          break;
        case SensorState::kRight:
          motor_handle.SetDutyCycles(60.0, Direction::kBackward, 60.0, Direction::kBackward);
          break;
        case SensorState::kFrontLeft:
          motor_handle.SetDutyCycles(40.0, Direction::kForward, 40.0, Direction::kForward);
          break;
        case SensorState::kFrontRight:
          motor_handle.SetDutyCycles(40.0, Direction::kBackward, 40.0, Direction::kBackward);
          break;
        case SensorState::kNone:
          break;
        default:
          break;
      }
    }
  }
}

void StrategyExecutor::KillMotors(){
  motor_handle.SetDutyCycles(0.0, Direction::kForward, 0.0, Direction::kBackward);
}