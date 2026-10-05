#include "SensorHandlers.hpp"

SensorHandler::SensorHandler(){
  event_handle = xEventGroupCreate();
}

EnemySensorHandler::EnemySensorHandler(){
  const std::map<uint8_t, uint8_t> sensor_bit = {
    {LEFT_SENSOR, LEFT_SENSOR_BIT}, 
    {FRONT_LEFT_SENSOR, FRONT_LEFT_SENSOR_BIT},
    {CENTER_SENSOR, CENTER_SENSOR_BIT},
    {FRONT_RIGHT_SENSOR, FRONT_RIGHT_SENSOR_BIT},
    {RIGHT_SENSOR, RIGHT_SENSOR_BIT}
  };

  for (const auto &[s, _]: sensor_bit) pinMode(s, INPUT);
}

void EnemySensorHandler::Detect(){
  for (const auto &[s, bit]: sensor_bit){
    if (digitalRead(s)) xEventGroupSetBits(event_handle, bit);
    else xEventGroupClearBits(event_handle, bit);
  }
}

LineSensorHandler::LineSensorHandler(){
  const uint16_t qtr_bits[QTR_COUNT] = {LEFT_QTR_BIT, RIGHT_QTR_BIT};
  const char* kMinOnKeys[QTR_COUNT] = {"min_on_1", "min_on_2"};
  const char* kMaxOnKeys[QTR_COUNT] = {"max_on_1", "max_on_2"};
  qtr.setTypeAnalog();
}

esp_err_t LineSensorHandler::Calibrate(const QTRCalibration calib_mode, NVSHandler *const nvs){
  if (calib_mode == QTRCalibration::kUseNVSValues){
    for (uint8_t i = 0; i < QTR_COUNT; ++i){
      nvs->ReadUInt16(kMinOnKeys[i], &qtr.calibrationOn.minimum[i]);
      nvs->ReadUInt16(kMaxOnKeys[i], &qtr.calibrationOn.maximum[i]);
    }
  }
  else{
    for (uint8_t i = 0; i < 800; ++i) qtr.calibrate();
    for (uint8_t i = 0; i < QTR_COUNT; ++i) {
      nvs->WriteUInt16(kMinOnKeys[i], &qtr.calibrationOn.minimum[i]);
      nvs->WriteUInt16(kMaxOnKeys[i], &qtr.calibrationOn.maximum[i]);
    }
  }
}

void LineSensorHandler::Detect(){
  qtr.readCalibrated(qtr_values);
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    if (qtr_values[i] >= 750) xEventGroupSetBits(event_handle, qtr_bits[i]);
    else xEventGroupClearBits(event_handle, qtr_bits[i]);
  }
}