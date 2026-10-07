#ifndef LIB_PWM_HANDLERS_MOTOR_HANDLER_HPP_
#define LIB_PWM_HANDLERS_MOTOR_HANDLER_HPP_

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>

enum class Direction : unsigned char {
  kForward,
  kBackward,
  kNone
};

typedef struct MotorPins {
  uint8_t a_pin;
  uint8_t b_pin;
} MotorPins;

typedef struct Motor {
  MotorPins pins;
  mcpwm_timer_t timer;
  float duty_cycle_a;
  float duty_cycle_b;
  Direction dir;
} Motor;

class MotorHandler {
  private:
    mcpwm_config_t* cfg;
    mcpwm_pin_config_t pin_cfg;
    Motor* motor_a; 
    Motor* motor_b;
    void SetDutyCycle(Motor* motor, Direction d, float duty_cycle);

  public:
    MotorHandler(mcpwm_config_t* cfg, Motor* motor_a, Motor* motor_b); // config explícita, talvez mudar
    esp_err_t Init();
    void SetDutyCycles(float duty_left, Direction dir_left, float duty_right, Direction dir_right);
};

#endif
