#ifndef HEADERS_NVS_HANDLER_H_
#define HEADERS_NVS_HANDLER_H_

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

class NVSHandler{
  private:
    nvs_handle_t mem_handle;
    bool nvs_ok;
    const char* name;
    esp_err_t err;

  public:
    NVSHandler(const char* name);
    bool StartStorage();
    bool get_nvs_ok();
    void CloseStorage();
    bool WriteUnsignedIntToNVS(const char *key, uint32_t value);
    uint32_t ReadUnsignedIntFromNVS(const char *key);
};

#endif