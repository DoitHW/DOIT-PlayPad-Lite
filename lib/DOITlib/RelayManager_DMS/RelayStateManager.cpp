#include "RelayStateManager.h"
#include <unordered_map>
#include <SPIFFS_handler/SPIFFS_handler.h>
#include <defines_DMS/defines_DMS.h>
#include <botonera_DMS/botonera_DMS.h>
#include <encoder_handler/encoder_handler.h>
#include <cstring>
#include <array>

// --- hash para array<5> ---
struct Array5Hash {
    std::size_t operator()(const std::array<uint8_t,5>& k) const noexcept {
        // hash simple (mezcla bytes)
        std::size_t h = 0;
        for (uint8_t b : k) { h = (h * 131) ^ b; }
        return h;
    }
};
struct Array5Eq {
    bool operator()(const std::array<uint8_t,5>& a, const std::array<uint8_t,5>& b) const noexcept {
        return std::memcmp(a.data(), b.data(), 5) == 0;
    }
};

// Helpers para convertir
static inline std::array<uint8_t,5> toKey(const TARGETNS& ns) {
    return { ns.mac01, ns.mac02, ns.mac03, ns.mac04, ns.mac05 };
}

// Mapas por NS
static std::unordered_map<std::array<uint8_t,5>, bool, Array5Hash, Array5Eq> relayStateByNS;
static std::unordered_map<std::array<uint8_t,5>, bool, Array5Hash, Array5Eq> relayCapByNS;

void RelayStateManager::set(const TARGETNS& ns, bool on) {
    relayStateByNS[toKey(ns)] = on;
}

bool RelayStateManager::get(const TARGETNS& ns) {
    auto it = relayStateByNS.find(toKey(ns));
    return (it != relayStateByNS.end()) ? it->second : false;
}

void RelayStateManager::clear() {
    relayStateByNS.clear();
    relayCapByNS.clear();
}

// Lee NS de un "file" (SPIFFS) o de opciones RAM
static bool readNSFromFileOrRAM(const String& file, TARGETNS &out) {
    // RAM
    if (file == "Ambientes" || file == "Fichas" || file == "Comunicador" ||
        file == "Apagar" || file == "Dado")
    {
        const INFO_PACK_T* opt = nullptr;
        if      (file == "Ambientes")   opt = &ambientesOption;
        else if (file == "Fichas")      opt = &fichasOption;
        else if (file == "Comunicador") opt = &comunicadorOption;
        else if (file == "Apagar")      opt = &apagarSala;
        else /* Dado */                 opt = &dadoOption;

        if (!opt) return false;
        out.mac01 = opt->serialNum[0];
        out.mac02 = opt->serialNum[1];
        out.mac03 = opt->serialNum[2];
        out.mac04 = opt->serialNum[3];
        out.mac05 = opt->serialNum[4];
        return true;
    }

    // SPIFFS
    String path = file.startsWith("/") ? file : "/" + file;
    fs::File f = SPIFFS.open(path, "r");
    if (!f) return false;

    // Asumimos OFFSET_SERIALNUM apunta al array de 5 bytes en el INFO_PACK
    TARGETNS tmp{};
    if (f.seek(OFFSET_SERIALNUM, SeekSet)) {
        uint8_t sn[5] = {0};
        f.read(sn, 5);
        tmp.mac01 = sn[0]; tmp.mac02 = sn[1]; tmp.mac03 = sn[2]; tmp.mac04 = sn[3]; tmp.mac05 = sn[4];
        out = tmp;
        f.close();
        return true;
    }
    f.close();
    return false;
}

// void RelayStateManager::initCapabilitiesNS(const std::vector<String>& files) {
//     relayCapByNS.clear();

//     for (const auto &file : files) {
//         TARGETNS ns{};
//         if (!readNSFromFileOrRAM(file, ns)) {
//             continue;
//         }

//         // No analizar especiales "no-elemento" (según tu lógica original)
//         if (file == "Ambientes" || file == "Fichas" || file == "Apagar") {
//             relayCapByNS[toKey(ns)] = false;
//             continue;
//         }

//         bool anyRelay = false;
//         byte cfg[2] = {0};
//         for (int m = 0; m < 16; ++m) {
//             if (getModeConfig(file, m, cfg) && getModeFlag(cfg, HAS_RELAY)) {
//                 anyRelay = true; break;
//             }
//         }
//         relayCapByNS[toKey(ns)] = anyRelay;
//     }
// }

bool RelayStateManager::hasRelay(const TARGETNS& ns) {
    auto it = relayCapByNS.find(toKey(ns));
    return (it != relayCapByNS.end()) && it->second;
}

bool RelayStateManager::getModeConfigForNS(const TARGETNS& ns, uint8_t modeCfg[2])
{
    std::memset(modeCfg, 0, 2);

    // RAM
    const INFO_PACK_T* statics[] = { &ambientesOption, &fichasOption, &comunicadorOption, &apagarSala, &dadoOption };
    for (const INFO_PACK_T* opt : statics) {
        if (std::memcmp(opt->serialNum, &ns, 5) == 0) {
            std::memcpy(modeCfg, opt->mode[opt->currentMode].config, 2);
            return true;
        }
    }

    // SPIFFS: buscar por NS
    for (const String& f : elementFiles) {
        if (f == "Ambientes" || f == "Fichas" || f == "Comunicador" || f == "Apagar" || f == "Dado") continue;

        TARGETNS fileNS{};
        if (!readNSFromFileOrRAM(f, fileNS)) continue;
        if (std::memcmp(&fileNS, &ns, 5) != 0) continue;

        String path = f.startsWith("/") ? f : "/" + f;
        fs::File file = SPIFFS.open(path, "r");
        if (!file) continue;

        uint8_t curMode = 0;
        file.seek(OFFSET_CURRENTMODE, SeekSet);
        file.read(&curMode, 1);

        file.seek(OFFSET_MODES + curMode * SIZE_MODE + 216, SeekSet);
        file.read(modeCfg, 2);
        file.close();
        return true;
    }
    return false;
}
