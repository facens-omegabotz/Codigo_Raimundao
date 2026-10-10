#include "MotorHandler.hpp"

// rever valores de ramp up com teste

MotorHandler::MotorHandler(mcpwm_config_t* cfg, Motor* motor_a, Motor* motor_b){
  this->cfg = cfg;
  this->motor_a = motor_a;
  this->motor_b = motor_b;
}

esp_err_t MotorHandler::Init(){
  mcpwm_init(MCPWM_UNIT_0, motor_a->timer, cfg);
  mcpwm_init(MCPWM_UNIT_0, motor_b->timer, cfg);
  pin_cfg = {
    .mcpwm0a_out_num = motor_a->pins.a_pin,
    .mcpwm0b_out_num = motor_a->pins.b_pin,
    .mcpwm1a_out_num = motor_b->pins.a_pin,
    .mcpwm1b_out_num = motor_b->pins.b_pin,
  };
  mcpwm_set_pin(MCPWM_UNIT_0, &pin_cfg);
  return ESP_OK;
}

void MotorHandler::SetDutyCycle(Motor* motor, Direction dir, float duty_cycle){
  if (motor->dir == dir){
    if (dir == Direction::kForward){
      if (motor->duty_cycle_a > duty_cycle){
        for (float i = motor->duty_cycle_a; i >= duty_cycle; i -= 5.0){
          mcpwm_set_duty(MCPWM_UNIT_0, motor->timer, MCPWM_OPR_A, i);
          motor->duty_cycle_a = i;
          vTaskDelay(pdMS_TO_TICKS(5));
        }
      }
      else if (motor->duty_cycle_a < duty_cycle){
        for (float i = motor->duty_cycle_a; i <= duty_cycle; i += 5.0){
          mcpwm_set_duty(MCPWM_UNIT_0, motor->timer, MCPWM_OPR_A, i);
          motor->duty_cycle_a = i;
          vTaskDelay(pdMS_TO_TICKS(5));
        }
      }
    }
    else if (dir == Direction::kBackward){
      if (motor->duty_cycle_b > duty_cycle){
        for (float i = motor->duty_cycle_b; i >= duty_cycle; i -= 5.0){
          mcpwm_set_duty(MCPWM_UNIT_0, motor->timer, MCPWM_OPR_B, i);
          motor->duty_cycle_b = i;
          vTaskDelay(pdMS_TO_TICKS(5));
        }
      }
      else if (motor->duty_cycle_b < duty_cycle){
        for (float i = motor->duty_cycle_b; i <= duty_cycle; i += 5.0){
          mcpwm_set_duty(MCPWM_UNIT_0, motor->timer, MCPWM_OPR_B, i);
          motor->duty_cycle_b = i;
          vTaskDelay(pdMS_TO_TICKS(5));
        }
      }
    }
  }
  else{
    if (dir == Direction::kBackward){
      for (float i = motor->duty_cycle_a; i >= 0; i -= 5.0){
        mcpwm_set_duty(MCPWM_UNIT_0, motor->timer, MCPWM_OPR_A, i);
        motor->duty_cycle_a = i;
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      for (float i = motor->duty_cycle_b; i <= duty_cycle; i += 5.0){
        mcpwm_set_duty(MCPWM_UNIT_0, motor->timer, MCPWM_OPR_B, i);
        motor->duty_cycle_b = i;
        vTaskDelay(pdMS_TO_TICKS(5));
      }
    }
    else{
      for (float i = motor->duty_cycle_b; i >= 0; i -= 5.0){
        mcpwm_set_duty(MCPWM_UNIT_0, motor->timer, MCPWM_OPR_B, i);
        motor->duty_cycle_b = i;
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      for (float i = motor->duty_cycle_a; i <= duty_cycle; i += 5.0){
        mcpwm_set_duty(MCPWM_UNIT_0, motor->timer, MCPWM_OPR_A, i);
        motor->duty_cycle_a = i;
        vTaskDelay(pdMS_TO_TICKS(5));
      }
    }
    motor->dir = dir;
  }
}

void MotorHandler::SetDutyCycles(float duty_left, Direction dir_left, float duty_right, Direction dir_right){
  SetDutyCycle(motor_a, dir_left, duty_left);
  SetDutyCycle(motor_b, dir_right, duty_right);
}

