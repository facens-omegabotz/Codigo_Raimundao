#ifndef LIB_DETECTORS_HPP_
#define LIB_DETECTORS_HPP_

#include <map>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <QTRSensors.h>
#include "raimundao_macros.h"
#include "raimundao_pins.h"
#include "raimundao_types.hpp"
#include "NVSHandler.hpp"

enum class QTRCalibration : unsigned char {
  kUseNVSValues,
  kUseQTRCalibration
};

class SensorHandler {
  public:
    EventGroupHandle_t event_handle; // acessível pela máquina de estados
    SensorHandler();
    ~SensorHandler() = default;
    virtual void Detect() = 0;
};

class EnemySensorHandler : public SensorHandler {
  private:
    std::map<uint8_t, uint8_t> sensor_bit;
  public:
    EnemySensorHandler();
    void Detect() override;  
};

class LineSensorHandler : public SensorHandler {
  private:
    uint16_t qtr_bits[QTR_COUNT] = {LEFT_QTR_BIT, RIGHT_QTR_BIT};
    char* kMinOnKeys[QTR_COUNT] = {"min_on_1", "min_on_2"};
    char* kMaxOnKeys[QTR_COUNT] = {"max_on_1", "max_on_2"};
    QTRSensors qtr;
    uint16_t qtr_values[QTR_COUNT];

  public:
    LineSensorHandler();
    void Detect() override;
    esp_err_t Calibrate(const QTRCalibration calib_mode, NVSHandler *const nvs);
};

#endif