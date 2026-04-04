#include <state_machine.hpp>

StateMachine::StateMachine(){
  ir_sensor_events_handle = xEventGroupCreate();
  for (const auto& [sensor, _] : events_per_ir_sensor){
    gpio_set_intr_type(static_cast<gpio_num_t>(sensor), GPIO_INTR_POSEDGE);
  }
}

// Função com nome ruim; espera E tem colateral, e o nome deve refletir isso
void StateMachine::WaitForIrSensorEvents(uint8_t wait_ticks){
  if (states.fight_state == FightState::kRunning){
    ir_sensor_event_bits = xEventGroupWaitBits(
      ir_sensor_events_handle, 
      ALL_IR_EVENT_BITS,
      pdFALSE, // xClearOnExit
      pdTRUE, // xWaitForAllBits
      pdMS_TO_TICKS(wait_ticks)
    ); 
  } 
}

void StateMachine::ReadIrSensorEvents(){
  if (states.fight_state == FightState::kRunning){
    for (const auto& [sensor, event] : events_per_ir_sensor){
      if (digitalRead(sensor)){
        xEventGroupSetBits(ir_sensor_events_handle, event);
      }
      else{
        xEventGroupClearBits(ir_sensor_events_handle, event);
      }
    }
  }
}

// Talvez simplista demais?
void StateMachine::UpdateIrDetectionState(){
  WaitForIrSensorEvents(kDefaultWaitTicks);
  
  if (states.fight_state == FightState::kRunning){
    if ((ir_sensor_event_bits & LEFT_SENSOR_BITS) && !(ir_sensor_event_bits & RIGHT_SENSOR_BITS)){
      states.ir_detection_state = IrDetectionState::kLeft;
      return;
    }
    else if ((ir_sensor_event_bits & RIGHT_SENSOR_BITS) && !(ir_sensor_event_bits & LEFT_SENSOR_BITS)){
      states.ir_detection_state = IrDetectionState::kRight;
      return;
    }
    else if ((ir_sensor_event_bits & FORWARD_BITS) && !(ir_sensor_event_bits & (EVENT_BIT_SENSOR_1 | EVENT_BIT_SENSOR_4))){
      states.ir_detection_state = IrDetectionState::kFront;
      return;
    }
    else{
      return;
    }
  } 
  else {
    states.ir_detection_state = IrDetectionState::kNone;
  } 
}

// Muitos acessos?
void StateMachine::SetFightState(uint16_t command){
  try{
    switch (fight_states.at(command)){
      case FightState::kStop:
        states.fight_state = FightState::kStop;
        break;
      default:
        if (states.fight_state == FightState::kRunning){
          states.fight_state = fight_states.at(command);
        }
        break;
    }
  }
  catch (std::out_of_range){
    try{
      if (states.fight_state == FightState::kReady){
        states.strategy = strats.at(command);
      }
    }
    catch (std::out_of_range){
      return;
    }
  }   
}