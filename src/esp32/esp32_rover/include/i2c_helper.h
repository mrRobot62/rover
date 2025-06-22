#ifndef _I2C_HELPER_H_
#define _I2C_HELPER_H_

#include <Wire.h>
#include <ArduinoJson.h>
#include "driver/i2c.h"

void printHex(const char* label, const uint8_t* data, size_t len) {
    Serial.printf("%s (%d bytes): ", label, len);
    for (size_t i=0; i < len; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();
}

bool initI2CSlave(i2c_port_t port,
                  uint8_t sda,
                  uint8_t scl,
                  uint8_t addr,
                  size_t bufferSize,
                  void (*onReceive)(int),
                  void (*onRequest)())
{
    // ggf. alten Treiber löschen
    esp_err_t err = i2c_driver_delete(port);
    delay(50);
    if (err != ESP_OK) {
        Serial.printf("[I2C] ggf. war kein I2C-Driver aktiv %d\n", err);
    }

    //-----------------------------------
    //- unter Arduino-Framework funktioniert
    //- nachfolgende manuelle Konfiguration von I2C nicht
    // - daher kann der buffer von 128Bytes für Arduino auch nicht vergrößert werden :-(
    //-----------------------------------

    // // Konfiguration
    // i2c_config_t conf = {};
    // conf.mode = I2C_MODE_SLAVE;
    // conf.sda_io_num = (gpio_num_t)sda;
    // conf.scl_io_num = (gpio_num_t)scl;
    // conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    // conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    // conf.slave.addr_10bit_en = 0;
    // conf.slave.slave_addr = addr;
    // conf.slave.maximum_speed = 400000;

    // err = i2c_param_config(port, &conf);
    // if (err != ESP_OK)
    // {
    //     Serial.printf("[I2C] Param config failed: %d\n", err);
    //     return false;
    // }

    // err = i2c_driver_install(port, I2C_MODE_SLAVE, bufferSize, bufferSize, 0);
    // if (err != ESP_OK)
    // {
    //     Serial.printf("[I2C] Driver install failed: %d\n", err);
    //     return false;
    // }

    // Arduino-kompatible Wire-Initialisierung
    Wire.setPins(sda, scl);
    Wire.begin(addr);

    if (onReceive) {
        Serial.printf("[I2C] onReceiver func {%d}\n", onReceive);
        Wire.onReceive(onReceive);
    }
    if (onRequest) {
        Serial.printf("[I2C] onRequest func {%d}\n", onRequest);
        Wire.onRequest(onRequest);
    }
    Serial.printf("[I2C] Slave bereit auf Port %d, Adresse 0x%02X, Buffer %d\n",
                  port, addr, bufferSize);

    return true;
}

#endif