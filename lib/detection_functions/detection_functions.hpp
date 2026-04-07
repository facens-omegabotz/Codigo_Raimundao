#include <globals.h>
#include <Arduino.h>
#include <map>

// passado assim pois EventGroupHandle_t é void*
EventBits_t WaitForSensorEvents(EventGroupHandle_t event_group){
  return xEventGroupWaitBits(
    event_group, 
    ALL_IR_EVENT_BITS | EVENT_QRE_LEFT | EVENT_QRE_RIGHT, 
    true, 
    false, 
    pdMS_TO_TICKS(30)
  );
}

void DetectEnemies(EventGroupHandle_t event_group, const std::map<int, int>& sensor_pins_and_bits){
  for (const auto& s: sensor_pins_and_bits){
    if (digitalRead(s.first)){
      xEventGroupSetBits(event_group, s.second);
    }
    else{
      xEventGroupClearBits(event_group, s.second);
    }
  }
}

// i should put detectlines here...
