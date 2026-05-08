#include <defines_DMS/defines_DMS.h>
#include <Arduino.h>
#include <SPIFFS.h>
#include <FS.h>
#include <Frame_DMS/Frame_DMS.h>
#include <Colors_DMS/Color_DMS.h>
#include <icons_64x64_DMS/icons_64x64_DMS.h>
#include <WiFi.h>
#include <map>
#include <algorithm>
#include <ArduinoOTA.h>

extern COLORHANDLER_ colorHandler;
volatile bool ap_ota_activo = false;


void ELEMENT_::begin() {

    Serial.begin(115200);

    configurar_RF(RF_BAUD_RATE);
    delay(100);
    #ifdef PLAYER
        Serial2.begin(9600, SERIAL_8N1, 8, 9);
    #endif
    pinMode (0, INPUT_PULLUP);
    delay(100);
    Serial1.onReceive(onUartInterrupt);
    //digitalWrite(RF_CONFIG_PIN, HIGH);
    delay(100);
    if(!SPIFFS.begin(true)){
        #ifdef DEBUG
            DEBUG__________ln("Error al montar SPIFFS");
        #endif
    } else {
        #ifdef DEBUG
            //DEBUG__________ln("SPIFFS montado correctamente");
        #endif
    }
    delay(100);    
}


void ELEMENT_::set_type(byte typein){
    type= typein;
}

// byte ELEMENT_::get_type(){
//     return type;
// }

// void ELEMENT_::configurar_RF(int baudRate) {
//     pinMode(RF_CONFIG_PIN, OUTPUT);
//     digitalWrite(RF_CONFIG_PIN, LOW);  // Entrar en modo configuración
//     delay(50);

//     // Intentar comunicarse con ambas velocidades por seguridad
//     Serial1.begin(115200, SERIAL_8N1, RF_RX_PIN, RF_TX_PIN);
//     byte comandoReset[] = {0xAA, 0xFA, 0xF0};  // Comando para resetear a valores de fábrica
//     Serial1.write(comandoReset, sizeof(comandoReset));
//     delay(200); // Dar tiempo al módulo para reiniciarse

//     // Verificar si responde a 115200
//     bool resetConfirmado = false;
//     if (Serial1.available()) {
//         #ifdef DEBUG
//         DEBUG__________ln("Respuesta a 115200 detectada");
//         #endif
//         resetConfirmado = true;
//     } else {
//         // Si no responde, intentar con 9600
//         Serial1.end();
//         Serial1.begin(9600, SERIAL_8N1, RF_RX_PIN, RF_TX_PIN);
//         Serial1.write(comandoReset, sizeof(comandoReset));
//         delay(200);
//         if (Serial1.available()) {
//             #ifdef DEBUG
//                 //DEBUG__________ln("Respuesta a 9600 detectada");
//             #endif
//             resetConfirmado = true;
//         }
//     }

//     if (!resetConfirmado) {
//                                                                                                 #ifdef DEBUG
//                                                                                                 DEBUG__________ln("Error: No se detectó respuesta del módulo RF");
//                                                                                                 #endif
//         digitalWrite(RF_CONFIG_PIN, HIGH);
//         return;
//     }

//     // Configurar comandos según la velocidad deseada
//     byte comandoUART[7];
//     byte comandoWirelessDataRate[7];

//     if (baudRate == 115200) {
//                                                                                                 #ifdef DEBUG
//                                                                                                 //DEBUG__________ln("Config UART y Wireless a 115200");
//                                                                                                 #endif
//         comandoUART[0] = 0xAA; comandoUART[1] = 0xFA; comandoUART[2] = 0x1E;
//         comandoUART[3] = 0x00; comandoUART[4] = 0x01; comandoUART[5] = 0xC2; comandoUART[6] = 0x00;

//         comandoWirelessDataRate[0] = 0xAA; comandoWirelessDataRate[1] = 0xFA; comandoWirelessDataRate[2] = 0xC3;
//         comandoWirelessDataRate[3] = 0x00; comandoWirelessDataRate[4] = 0x01; comandoWirelessDataRate[5] = 0xC2; comandoWirelessDataRate[6] = 0x00;
//     } else {
//                                                                                                 #ifdef DEBUG
//                                                                                                 //DEBUG__________ln("Config UART y Wireless a 9600");
//                                                                                                 #endif
//         comandoUART[0] = 0xAA; comandoUART[1] = 0xFA; comandoUART[2] = 0x1E;
//         comandoUART[3] = 0x00; comandoUART[4] = 0x00; comandoUART[5] = 0x25; comandoUART[6] = 0x80;

//         comandoWirelessDataRate[0] = 0xAA; comandoWirelessDataRate[1] = 0xFA; comandoWirelessDataRate[2] = 0xC3;
//         comandoWirelessDataRate[3] = 0x00; comandoWirelessDataRate[4] = 0x00; comandoWirelessDataRate[5] = 0x25; comandoWirelessDataRate[6] = 0x80;
//     }

//     // Configurar UART y velocidad inalámbrica
//     Serial1.write(comandoUART, sizeof(comandoUART));
//     delay(200);
//     Serial1.write(comandoWirelessDataRate, sizeof(comandoWirelessDataRate));
//     delay(200);

//     // Confirmar configuración
//     byte comandoLeerConfiguracion[] = {0xAA, 0xFA, 0xE1};
//     Serial1.write(comandoLeerConfiguracion, sizeof(comandoLeerConfiguracion));
//     delay(200);
//                                                                                                         #ifdef DEBUG
//                                                                                                         //DEBUG__________ln(" =[Desglosando configuración recibida]=");
//                                                                                                         #endif

//     // Filtrar valores no relevantes
//     while (Serial1.available()) {
//         if (Serial1.peek() == 0x4F) {  // ASCII 'O'
//             Serial1.read(); // Ignorar 'O'
//             if (Serial1.peek() == 0x4B) {  // ASCII 'K'
//                 Serial1.read(); // Ignorar 'K'
//                 Serial1.read(); // Ignorar '\r'
//                 Serial1.read(); // Ignorar '\n'
//             }
//         } else {
//             break;
//         }
//     }

//     if (Serial1.available() >= 13) {  // Verificar que haya suficientes datos para desglosar
//         byte frecuencia[4];
//         byte velocidad[4];
//         byte anchoBanda[2];
//         byte desviacionFrecuencia;
//         byte potencia;

//         for (int i = 0; i < 4; i++) frecuencia[i] = Serial1.read();
//         for (int i = 0; i < 4; i++)  velocidad[i] = Serial1.read();
//         for (int i = 0; i < 2; i++) anchoBanda[i] = Serial1.read();
//         desviacionFrecuencia = Serial1.read();
//         potencia = Serial1.read();

//         uint32_t freq = (frecuencia[0] << 24) | (frecuencia[1] << 16) | (frecuencia[2] << 8) | frecuencia[3];
//         uint32_t baudrate = (velocidad[0] << 24) | (velocidad[1] << 16) | (velocidad[2] << 8) | velocidad[3];
//         uint16_t bw = (anchoBanda[0] << 8) | anchoBanda[1];
//         #ifdef DEBUG
//         // DEBUG__________("📡 Frecuencia: ");
//         // DEBUG__________(freq);
//         // DEBUG__________ln(" Hz");

//         // DEBUG__________("⚡ Velocidad inalámbrica: ");
//         // DEBUG__________(baudrate);
//         // DEBUG__________ln(" bps");

//         // DEBUG__________("📶 Ancho de banda: ");
//         // DEBUG__________(bw);
//         // DEBUG__________ln(" kHz");

//         // DEBUG__________("🎛️  Desviación de frecuencia: ");
//         // DEBUG__________(desviacionFrecuencia);
//         // DEBUG__________ln(" kHz");

//         // DEBUG__________("🔋 Potencia de transmisión: ");
//         // DEBUG__________(potencia);
//         // DEBUG__________ln(" dBm");
//         // DEBUG__________ln();
//         #endif
//     } else {
//         #ifdef DEBUG
//            // DEBUG__________ln("Error: Datos insuficientes para interpretar la configuración.");
//         #endif
//     }

//     // Salir del modo configuración
//     digitalWrite(RF_CONFIG_PIN, HIGH);
//     delay(200);

//     // Reconfigurar la velocidad UART en el ESP32
//     Serial1.end();
//     if (baudRate == 115200) {
//         #ifdef DEBUG
//             //DEBUG__________ln("Config velocidad UART en ESP32 a 115200");
//         #endif
//         Serial1.begin(115200, SERIAL_8N1, RF_RX_PIN, RF_TX_PIN);
//     } else {
//         #ifdef DEBUG
//             //DEBUG__________ln("Config velocidad UART en ESP32 a 9600");
//         #endif
//         Serial1.begin(9600, SERIAL_8N1, RF_RX_PIN, RF_TX_PIN);
//     }
//         #ifdef DEBUG
//         //DEBUG__________ln("Configuración completa y módulo reiniciado correctamente.");
//         #endif
//     delay(10);
// }

void ELEMENT_::configurar_RF(uint32_t uartBaud)
{
    // Normaliza por seguridad a los valores que realmente usas
    const uint32_t br = (uartBaud == 115200u) ? 115200u : 9600u;
    Serial.println("Configurando a: " + String(br) + " bps");
    Serial1.end();
    delay(10);
    Serial1.begin(br, SERIAL_8N1, RF_RX_PIN, RF_TX_PIN);

    // Limpia bytes residuales por si el driver arrastra basura
    while (Serial1.available()) (void)Serial1.read();

    #ifdef DEBUG
      DEBUG__________("RF UART begin @ ");
      DEBUG__________ln(br);
    #endif
}


void ELEMENT_::activarAP_OTA() {
    delay(100);

    if (ap_ota_activo) {
        DEBUG__________("OTA YA ESTA ACTIVO");
        return;
    }

    DEBUG__________("Activando Access Point...");

    // 0) Saneado fuerte del estado WiFi (evita hostap_init en estado roto)
    WiFi.disconnect(true, true);
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    WiFi.mode(WIFI_AP);
    delay(100);

    // 1) Config IP del AP
    IPAddress ap_ip(AP_ELEM_IP);
    IPAddress gw(AP_ELEM_IP);
    IPAddress mask(255, 255, 255, 0);

    if (!WiFi.softAPConfig(ap_ip, gw, mask)) {
        DEBUG__________("ERROR: softAPConfig fallo");
        WiFi.mode(WIFI_OFF);
        return;
    }

    // 2) Levantar AP
    // Nota: password debe ser "" (abierto) o >= 8 chars. Si no, softAP falla.
    if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) {
        DEBUG__________("ERROR: softAP fallo (SSID/PASS o estado WiFi)");
        WiFi.mode(WIFI_OFF);
        return;
    }

    DEBUG__________("IP ADDR: " + String(ap_ip));

    // 3) Preparar OTA (protegido por password)
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        DEBUG__________("Iniciando actualización OTA de " + type);
    });
    ArduinoOTA.onEnd([]() {
        DEBUG__________("\nFinalizando OTA");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        DEBUG__________("Progreso OTA: " + String(progress * 100 / total) + "%");
    });
    ArduinoOTA.onError([](ota_error_t error) {
        DEBUG__________("Error OTA: " + String(error));
    });

    ArduinoOTA.begin();
    ap_ota_activo = true;
    DEBUG__________("Access Point y OTA ACTIVADOS.");
}

void ELEMENT_::desactivarAP_OTA() {
    if (!ap_ota_activo) {
        DEBUG__________("AP y OTA ya están desactivados.");
        return;
    }
    DEBUG__________("Desactivando OTA...");
    ArduinoOTA.end();              // Cierra el servidor OTA
    WiFi.softAPdisconnect(true);   // Apaga el AP
    // Opcional: WiFi.mode(WIFI_OFF);
    ap_ota_activo = false;
    DEBUG__________("AP y OTA DESACTIVADOS.");
}
