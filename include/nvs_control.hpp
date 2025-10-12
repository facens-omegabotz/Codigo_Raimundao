#ifndef INCLUDE_NVS_CONTROL_H_
#define INCLUDE_NVS_CONTROL_H_
#endif

#include "nvs_flash.h"
#include "nvs.h"

class NVSControl{
  private:
    const char* name;
    nvs_handle_t mem_handle;
    esp_err_t err;
    bool nvs_ok;

  public:
    NVSControl(const char* name);
    bool getNVSOk();
    bool InitStorage();
    void CloseStorage();
    bool WriteUnsignedIntToNVS(const char *key, uint32_t value);
    uint32_t ReadUnsignedIntFromNVS(const char *key);
};

NVSControl::NVSControl(const char *name){
  this->name = name;
  nvs_ok = InitStorage();
}

bool NVSControl::getNVSOk(){
  return nvs_ok;
}

bool NVSControl::InitStorage(){
  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  err = nvs_open(name, NVS_READWRITE, &mem_handle);
  if (err != ESP_OK)
    return false;
  return true;
}

void NVSControl::CloseStorage(){
  nvs_close(mem_handle);
}

bool NVSControl::WriteUnsignedIntToNVS(const char *key, uint32_t value){
  err = nvs_set_u32(mem_handle, key, value);
  if (err != ESP_OK)
    return false;
  return true;
}

uint32_t NVSControl::ReadUnsignedIntFromNVS(const char *key){
  uint32_t value;
  err = nvs_get_u32(mem_handle, key, &value);
  if (err != ESP_OK)
    return -1;
  return value;
}


