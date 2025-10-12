constexpr bool kWhiteLine = true;         // Se true, envia valores quando a refletância for alta.
// Pinos dos sensores IR

constexpr uint8_t kSensor1 = 32;
constexpr uint8_t kSensor2 = 33;
constexpr uint8_t kSensor3 = 25;
constexpr uint8_t kSensor4 = 27;

// Pinos dos sensores de linha

constexpr uint8_t kQtr1 = 36;
constexpr uint8_t kQtr2 = 39;

class DetectionControl{
  private:
    RobotState *state;
    EventGroupHandle_t *sensor_events;
    QTRSensorsAnalog *qtra;
    uint32_t *sensor_values;
  
  public:
    DetectionControl(RobotState *p_state, EventGroupHandle_t *p_sensor_events, QTRSensorsAnalog *p_qtra, uint32_t *p_sensor_values);
    EventBits_t GetSensorEvents(uint16_t ms);
    void DetectEnemies();
    void DetectLine();
};

DetectionControl::DetectionControl(RobotState *p_state, EventGroupHandle_t *p_sensor_events, QTRSensorsAnalog *p_qtra, uint32_t *p_sensor_values){
  state = p_state;
  sensor_events = p_sensor_events;
  qtra = p_qtra;
  sensor_values = p_sensor_values;
}

EventBits_t DetectionControl::GetSensorEvents(uint16_t ms){
  return xEventGroupWaitBits(
    *sensor_events, 
    EVENT_SENSOR1 | EVENT_SENSOR2 | EVENT_SENSOR3 | EVENT_SENSOR4 | EVENT_QRE, 
    true, 
    false, 
    pdMS_TO_TICKS(ms)
  );
}

/// @brief Função que lê os valores de cada um dos sensores infravermelhos e escreve ou limpa os bits no EventGroup global.
void DetectionControl::DetectEnemies(){
  if (*state == RobotState::kRunning){
    if (digitalRead(kSensor1)){
      xEventGroupSetBits(*sensor_events, EVENT_SENSOR1);
    }
    else{
      xEventGroupClearBits(*sensor_events, EVENT_SENSOR1);
    }

    if (digitalRead(kSensor2)){
      xEventGroupSetBits(*sensor_events, EVENT_SENSOR2);
    }
    else{
      xEventGroupClearBits(*sensor_events, EVENT_SENSOR2);
    }

    if (digitalRead(kSensor3)){
      xEventGroupSetBits(*sensor_events, EVENT_SENSOR3);
    }
    else{
      xEventGroupClearBits(*sensor_events, EVENT_SENSOR3);
    }
    if (digitalRead(kSensor4)){
      xEventGroupSetBits(*sensor_events, EVENT_SENSOR4);
    }
    else{
      xEventGroupClearBits(*sensor_events, EVENT_SENSOR4);
    }
  }
}

/// @brief Função para detectar a linha e escrever no EventGroup.
void DetectionControl::DetectLine(){
  if (*state == RobotState::kRunning){
    int line_info = qtra->readLine(sensor_values, QTR_EMITTERS_ON, kWhiteLine);
    if (line_info > 500)
      xEventGroupSetBits(sensor_events, EVENT_QRE);
    else
      xEventGroupClearBits(sensor_events, EVENT_QRE);
  }
}