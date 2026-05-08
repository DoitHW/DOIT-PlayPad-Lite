#pragma once
#ifndef BOTONERA_DMS_H
#define BOTONERA_DMS_H

#include <Colors_DMS/Color_DMS.h>
#include <SPIFFS_handler/SPIFFS_handler.h>
#include <defines_DMS/defines_DMS.h>
#include <Element_DMS/Element_DMS.h>
#include <FastLED.h>
#include <Arduino.h>
#include <Frame_DMS/Frame_DMS.h>
#include <vector>
#include <play_DMS/play_DMS.h>

struct SectorMsg {
    uint8_t sector{0xFF};            // data[0]
    std::vector<uint8_t> payload;    // data[1..N] (puede iniciar con NS o no)
    uint32_t tsMs{0};
};

// Protocolo (botón 1..9) → índice físico de LED (0..8)
// static constexpr uint8_t kBtnIdToLedIdx[10] = {
//   0xFF, // [0] no usado
//   8,    // 1 → LED8 (AZUL)
//   6,    // 2 → LED6 (VERDE)
//   4,    // 3 → LED4 (AMARILLO)
//   2,    // 4 → LED2 (ROJO)
//   0,    // 5 → LED0 (RELE)
//   7,    // 6 → LED7 (VIOLETA)
//   5,    // 7 → LED5 (NARANJA)
//   3,    // 8 → LED3 (CELESTE)
//   1     // 9 → LED1 (BLANCO)
// };

class BOTONERA_ : public ELEMENT_{

    public:
        BOTONERA_();
        //void botonera_begin();
        void RX_main_handler(LAST_ENTRY_FRAME_T LEF)override;
        void sectorIn_handler(const std::vector<byte>& data, const TARGETNS& originNS, uint8_t originType);
        bool serialExistsInSPIFFS(byte serialNum[5]);
        void dibujarMarco(uint16_t color);
        void mostrarMensajeTemporal(int respuesta, int dTime);
        bool guardar_elemento(INFO_PACK_T* infoPack);
        bool esperar_respuesta(uint8_t expectedSector, const uint8_t* expectedNS, std::vector<uint8_t>& outPayload, unsigned long timeoutMs);
        void printFrameInfo(LAST_ENTRY_FRAME_T LEF);
        void activateCognitiveMode();
        void deactivateCognitiveMode();
        //String getFilePathBySerial(const TARGETNS& ns);
        void escanearSala();
        bool procesar_sector_NS(int sector, INFO_PACK_T* info, const TARGETNS& ns, const uint8_t* data, size_t len);
        //void iniciarEscaneoElemento(const char* mensajeInicial);
        void actualizarBarraProgreso2(int pasoActual, int pasosTotales, const char* etiqueta);
};

extern BOTONERA_ *element;
extern DOITSOUNDS_ doitPlayer;
extern byte currentCognitiveCommand;
extern bool inCognitiveMenu;



#endif