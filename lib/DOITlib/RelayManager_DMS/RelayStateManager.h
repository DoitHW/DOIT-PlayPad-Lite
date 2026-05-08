#pragma once
#include <array>
#include <Arduino.h>
#include <vector>
#include <WString.h>   // String
#include <defines_DMS/defines_DMS.h> // TARGETNS, etc.
#include <Frame_DMS/Frame_DMS.h> // TARGETNS, etc.

namespace RelayStateManager {

// Estado ON/OFF por NS
void set(const TARGETNS& ns, bool on);
bool get(const TARGETNS& ns);

// ¿Algún modo del elemento con ese NS tiene relé?
bool hasRelay(const TARGETNS& ns);

// (opcional) limpiar mapas
void clear();

// Recalcular capacidades (leer ficheros y RAM) y mapear por NS
// void initCapabilitiesNS(const std::vector<String>& elementFiles);

// Utilidades (si las necesitas en otras partes)
bool getModeConfigForNS(const TARGETNS& ns, uint8_t modeCfg[2]);

}
