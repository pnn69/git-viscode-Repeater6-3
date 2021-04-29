/*
 * struckt.c
 *
 *  Created on: 26 Aug 2020
 *      Author: Peter
 */
#include "struckt.h"
#include "esp_system.h"
#include "global.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "timer.h"
#include <esp_log.h>
#define TAG "Struck"

#define STORAGE_NAMESPACE "storage"

esp_err_t restore_struckt_name(char *nameStore, void *name, size_t strucktSize) {
    nvs_handle my_handle;
    // open
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Could not open storage space\n");
        return err;
    }

    size_t required_size = 0;
    err = nvs_get_blob(my_handle, nameStore, NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "Blob read went wrong!\n");
        return err;
    }
    err = nvs_get_blob(my_handle, nameStore, name, &strucktSize);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Blob read error\n");
        ESP_LOGI(TAG, "Error (%s) reading data to NVS", esp_err_to_name(err));
        return err;
    }
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t store_struckt_name(char *nameStore, void *name, size_t strucktSize) {
    enableOutput = false;
    nvs_handle my_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
        return err;
    err = nvs_set_blob(my_handle, nameStore, name, strucktSize);
    if (err != ESP_OK)
        return err;
    err = nvs_commit(my_handle);
    enableOutput = true;
    if (err != ESP_OK)
        return err;
    nvs_close(my_handle);
    ESP_LOGI(TAG, "Struck stored");
    return ESP_OK;
}

int32_t read_nvmint32(char *nameStore) {
    int32_t b;
    nvs_handle my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        nvs_close(my_handle);
        return err;
    }
    err = nvs_get_i32(my_handle, nameStore, &b);
    nvs_close(my_handle);
    return b;
}

esp_err_t write_nvmint32(char *nameStore, int32_t b) {
    nvs_handle my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        nvs_close(my_handle);
        return err;
    }
    err = nvs_set_i32(my_handle, nameStore, b);
    nvs_close(my_handle);
    ESP_LOGI(TAG, "NVM Data written:%d", b);
    return err;
}

void initKabel(void) {
    Kabel400.nvm_version = NVM_VERSION;
    Kabel400.fanspeedminimal = 0;
    Kabel400.fanspeedmaximal = 100;
    Kabel400.purifyvalveOn = false;
    Kabel400.fanOn = false;
    Kabel400.fanManual = false;
    Kabel400.drainOn = false;
    Kabel400.floatsensorError = 60; // 60 seconds timeout
    Kabel400.tout = 30;
    Kabel400.onTime = 0;
    Kabel400.pumpTime = 0;
    Kabel400.circulateTimer = 30;
    Kabel400.circulateSpeed = 0;
    Kabel400.circulateTime = 0;
    Kabel400.RH_day = 50;
    Kabel400.RH_night = 50;
    Kabel400.RHmaximal = 95;
    Kabel400.RHmimimal = 10;
    Kabel400.cleantimeset = 3;
    Kabel400.cleanManual = false;
    Kabel400.P = 30;
    Kabel400.I = 15;
    Kabel400.D = 0;
}

void initPump(void) {
    Pump.nvm_version = NVM_VERSION;
    Pump.speedminimalpump = 50;
    Pump.speedmaximalpump = 100;
    Pump.TempDelta = 5;
    Pump.iErrorfanmax = 1;
    Pump.dEerrorfanmax = 100;
    Pump.Thigh = 30;
    Pump.Ppump = 15;
    Pump.Ipump = 6;
    Pump.Dpump = 0;
    Pump.Tout = 30;
}
void initFan(void) {
    Fan.nvm_version = NVM_VERSION;
    Fan.speedminimalfan = 0;
    Fan.speedmaximalfan = 100;
    Fan.iErrorfanmax = 1;
    Fan.dEerrorfanmax = 100;
    Fan.Tout = 15;
    Fan.Pfan = 6;
    Fan.Ifan = 15;
    Fan.Dfan = 0;
}
