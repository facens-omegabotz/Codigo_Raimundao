#ifndef HEADERS_MEM_FUNCTIONS_H_
#define HEADERS_MEM_FUNCTIONS_H_
#endif

#include "nvs_flash.h"
#include "nvs.h"

bool initNVSStorage();
bool closeNVSStorage();
bool writeUnsignedIntToNVS(char* key, uint32_t value);
uint32_t readUnsignedIntFromNVS(char* key);

nvs_handle_t memHandle;
esp_err_t err;

bool initNVSStorage(){
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    err = nvs_open("QTR values", NVS_READWRITE, &memHandle);
    if (err != ESP_OK)
        return false;
    return true;
}

bool closeNVSStorage(){
    nvs_close(memHandle);
}

bool writeUnsignedIntToNVS(char *key, uint32_t value){
    err = nvs_set_u32(memHandle, key, value);
    if (err != ESP_OK)
        return false;
    return true;
}

uint32_t readUnsignedIntFromNVS(char *key){
    uint32_t value;
    err = nvs_get_u32(memHandle, key, &value);
    if (err != ESP_OK)
        return -1;
    return value;
}
