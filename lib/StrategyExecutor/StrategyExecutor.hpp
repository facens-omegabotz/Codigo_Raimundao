#ifndef LIB_STRATEGY_EXECUTOR_HPP_
#define LIB_STRATEGY_EXECUTOR_HPP_

#include "raimundao_macros.h"
#include "raimundao_pins.h"
#include "raimundao_types.hpp"
#include "MotorHandler.hpp"
#include "StateMachine.hpp"

class StrategyExecutor {
  private:
    MotorHandler motor_handle;
    void FollowEnemy(StateMachine& state_machine);
    void SearchLeft(StateMachine& state_machine);
    void SearchRight(StateMachine& state_machine);
  public:
    StrategyExecutor(mcpwm_config_t* cfg, Motor* motor_a, Motor* motor_b);
    void RunStrategy(StateMachine& state_machine);
    void KillMotors();
};

#endif