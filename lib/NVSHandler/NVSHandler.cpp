#include "NVSHandler.hpp"

NVSHandler::NVSHandler(const char* name){
  mem_name = name;
}

esp_err_t NVSHandler::StartStorage(nvs_open_mode_t open_mode = NVS_READWRITE){
  nvs_err = nvs_flash_init();
  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvs_err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvs_err);
  nvs_err = nvs_open(mem_name, open_mode, &mem_handle);
  return nvs_err;
}

inline void NVSHandler::CloseStorage(){ nvs_close(mem_handle); }

esp_err_t NVSHandler::WriteUInt16(const char* k, const uint16_t* v){
  nvs_err = nvs_set_u16(mem_handle, k, *v);
  ESP_ERROR_CHECK(nvs_err);
  nvs_err = nvs_commit(mem_handle);
  return nvs_err;
}

inline esp_err_t NVSHandler::ReadUInt16(const char* k, uint16_t* const v){ return nvs_get_u16(mem_handle, k, v); }