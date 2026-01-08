#include <nvs_handler.hpp>

static constexpr const char* TAG = "NVSControl";

NVSHandler::NVSHandler(const char* name){
  this->name = name;
  nvs_ok = StartStorage();
  // ESP_LOGI(TAG, "Started NVSControl module with status %s and name '%s'\n", nvs_ok ? "true" : "false", *name);
}

bool NVSHandler::get_nvs_ok(){
  return nvs_ok;
}

bool NVSHandler::StartStorage(){
  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // ESP_LOGW(TAG, "Init unsuccessful with status %d. Erasing and restarting...\n", err);
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  err = nvs_open(name, NVS_READWRITE, &mem_handle);
  if (err != ESP_OK){
    // ESP_LOGE(TAG, "Storage init returned status %d\n", err);
    return false;
  }
  // ESP_LOGI(TAG, "Storage init successful with status %d\n", err);
  return true;
}

void NVSHandler::CloseStorage(){
  // ESP_LOGI(TAG, "Closing storage at %d\n", mem_handle);
  nvs_close(mem_handle);
}

bool NVSHandler::WriteUnsignedIntToNVS(const char *key, uint32_t value){
  err = nvs_set_u32(mem_handle, key, value);
  if (err != ESP_OK){
    // ESP_LOGE(TAG, "Write unsigned int returned status %d\n", err);
    return false;
  }
  // ESP_LOGI(TAG, "Write unsigned int successful with status %d\n", err);
  
  // nvs_commit é necessário para funcionar - Olha a Bomba que você fez na versão anterior
  err = nvs_commit(mem_handle);
  if (err != ESP_OK){
    // ESP_LOGE(TAG, "Write commit returned status %d\n", err);
    return false;
  }
  // ESP_LOGI(TAG, "Write commit successful with status %d\n", err);
  return true;
}


uint32_t NVSHandler::ReadUnsignedIntFromNVS(const char *key){ // isso poderia dar throw em uma exceção?
  uint32_t value;
  err = nvs_get_u32(mem_handle, key, &value);
  if (err != ESP_OK){
    // ESP_LOGE(TAG, "Read unsigned int returned status %d\n", err);
    return 0; // 0 é falsy em cpp
  }
    
  // ESP_LOGI(TAG, "Read unsigned int successful (read %d) with status %d\n", value, err);
  return value;
}