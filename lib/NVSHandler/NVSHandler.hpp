#ifndef HEADERS_NVS_HANDLER_H_
#define HEADERS_NVS_HANDLER_H_

#include "nvs_flash.h"
#include "nvs.h"
#include <Arduino.h>

class NVSHandler{
  private:
    nvs_handle_t mem_handle;
    const char* mem_name;
    esp_err_t nvs_err;

  public:
    NVSHandler(const char* name);
    esp_err_t StartStorage(nvs_open_mode_t open_mode);
    void CloseStorage();
    esp_err_t WriteUInt16(const char* k, const uint16_t* v);
    esp_err_t ReadUInt16(const char* k, uint16_t* const v);
};

#endif