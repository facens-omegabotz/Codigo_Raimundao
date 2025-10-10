#ifndef INCLUDE_NVS_OPERATOR_H_
#define INCLUDE_NVS_OPERATOR_H_
#endif

#include "nvs_flash.h"
#include "nvs.h"

class NVSOperator{
    private:
        char* name;
        nvs_handle_t mem_handle;
        esp_err_t err;
        bool nvs_ok;

    public:
        NVSOperator(char* name);
        bool getNVSOk();
        bool InitStorage();
        void CloseStorage();
        bool WriteUnsignedIntToNVS(const char *key, uint32_t value);
        uint32_t ReadUnsignedIntFromNVS(const char *key);
};

NVSOperator::NVSOperator(char *name){
    this->name = name;
    this->nvs_ok = InitStorage();
}

bool NVSOperator::getNVSOk(){
    return this->nvs_ok;
}

bool NVSOperator::InitStorage(){
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    err = nvs_open(this->name, NVS_READWRITE, &this->mem_handle);
    if (err != ESP_OK)
        return false;
    return true;
}

void NVSOperator::CloseStorage(){
    nvs_close(this->mem_handle);
}

bool NVSOperator::WriteUnsignedIntToNVS(const char *key, uint32_t value){
    err = nvs_set_u32(this->mem_handle, key, value);
    if (err != ESP_OK)
        return false;
    return true;
}

uint32_t NVSOperator::ReadUnsignedIntFromNVS(const char *key){
    uint32_t value;
    err = nvs_get_u32(this->mem_handle, key, &value);
    if (err != ESP_OK)
        return -1;
    return value;
}


