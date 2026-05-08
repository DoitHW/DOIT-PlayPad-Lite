#pragma once

#ifndef ELEMENT_DMS_H
#define ELEMENT_DMS_H

#include <defines_DMS/defines_DMS.h>
#include <stdint.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Frame_DMS/Frame_DMS.h>
#include <FastLED.h>
#include <vector>
#include <Colors_DMS/Color_DMS.h>
#include <map>
#include <algorithm>
#include <Colors_DMS/Color_DMS.h>

class ELEMENT_ {

    public: 
        void   begin();
        void   configurar_RF(uint32_t uartBaud);

        void      set_type(byte typein);
        //byte      get_type();

        //void      set_mode(uint8_t mode);

        void activarAP_OTA();
        void desactivarAP_OTA();

        const char* AP_SSID =    AP_SSID_NAME;
        const char* AP_PASSWORD= AP_SSID_PASS;
        const char* OTA_PASSWORD= OTA_PASS;

    protected:
        virtual void  RX_main_handler(LAST_ENTRY_FRAME_T LEF){} 
        byte     type;
};

extern volatile bool ap_ota_activo;

#endif