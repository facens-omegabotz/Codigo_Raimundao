#include <nvs_handler.hpp>

static constexpr const char* TAG = "NVSControl";

NVSHandler::NVSHandler(const char* name){
  this->name = name;
}

bool NVSHandler::get_nvs_ok(){ return nvs_ok; }

bool NVSHandler::StartStorage(){
  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    if (Serial) Serial.println("Init storage unsuccessful. Erasing and restarting...");
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  err = nvs_open(name, NVS_READWRITE, &mem_handle);
  if (err != ESP_OK){
    if (Serial) Serial.println("Storage init did not work.");
    if (Serial) Serial.println(esp_err_to_name(err));
    nvs_ok = false;
    return false;
  }
  if (Serial) Serial.println("Storage init worked.");
  nvs_ok = true;
  return true;
}

void NVSHandler::CloseStorage(){
  if (Serial) Serial.println("Closing storage.");
  nvs_close(mem_handle);
}

bool NVSHandler::WriteUnsignedIntToNVS(const char *key, uint32_t value){
  err = nvs_set_u32(mem_handle, key, value);
  if (err != ESP_OK){
    if (Serial) Serial.println("Write unsigned int returned error.");
    return false;
  }
  if (Serial) Serial.println("Write unsigned int successful.");

  err = nvs_commit(mem_handle);
  if (err != ESP_OK){
    if (Serial) Serial.println("Commit to nvs failed.");
    return false;
  }
  if (Serial) Serial.println("Commit to nvs successful.");
  return true;
}


uint32_t NVSHandler::ReadUnsignedIntFromNVS(const char *key){ // isso poderia dar throw em uma exceção?
  uint32_t value;
  err = nvs_get_u32(mem_handle, key, &value);
  if (err != ESP_OK){
    if (Serial) Serial.println("Read unsigned int failed.");
    return -1; // -1 é um valor impossível. Força checagem na chamada mas melhor que confusão com valores.
  }
  if (Serial) Serial.println("Read unsigned int successful."); 
  return value;
}