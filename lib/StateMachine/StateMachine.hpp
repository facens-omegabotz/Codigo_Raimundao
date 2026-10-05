#ifndef LIB_STATE_MACHINE_HPP_
#define LIB_STATE_MACHINE_HPP_

#include <map>
#include <array>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include "raimundao_macros.h"
#include "raimundao_pins.h"
#include "raimundao_types.hpp"

class StateMachine {
  private:
    EventBits_t enemy_bits;
    EventBits_t qtr_bits;
    EventGroupHandle_t enemy_handle = nullptr;
    EventGroupHandle_t qtr_handle = nullptr;
    void UpdateSensorState();
    void UpdateQTRState();
    bool ParseRobotState(const uint16_t command);
    bool ParseStrategy(const uint16_t command);

  public:
    States states;
    unsigned long start_time;
    
    StateMachine() = default;
    StateMachine(EventGroupHandle_t enemy_handle, EventGroupHandle_t line_handle);
    void UpdateIRReceiverState(const uint16_t command);
    void UpdateState();
};

#endif