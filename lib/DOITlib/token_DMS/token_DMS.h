#ifndef TOKEN_DMS_H
#define TOKEN_DMS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PN532.h>
#include <Frame_DMS/Frame_DMS.h>
#include <play_DMS/play_DMS.h>
#include <Translations_handler/translations.h>
#include <vector>

#define PN532_IRQ 46 //botonera antiga 42
// Enumeración de modos de operación del token
enum TOKEN_MODE_ {
    TOKEN_BASIC_MODE = 0,
    TOKEN_PARTNER_MODE,
    TOKEN_GUESS_MODE
};

class TOKEN_ {
public:
    // Estructuras para almacenar la información del token
    struct TOKEN_ADDR {
        byte bank;
        byte file;
    };

    struct TOKEN_COLOR {
        byte r;
        byte g;
        byte b;
    };

    struct TOKEN_DATA {
        byte cmd;
        byte cmd2;
        TOKEN_ADDR addr;
        TOKEN_COLOR color;
        TOKEN_ADDR partner[8];
        char familyName[25]; 
    };

    // Constructor
    TOKEN_();

    // Inicializa el bus I2C y configura el PN532 (Adafruit)
    void begin();

    // Lee el UID de la tarjeta. Retorna true si se detecta tarjeta y asigna el UID a 'uid'
    bool readCard(String &uid);

    // Lee y procesa el mensaje NDEF de la tarjeta y retorna el token (cadena delimitada por '#')
    // Si no se encuentra un mensaje válido, retorna cadena vacía.
    bool leerMensajeNFC(String& mensaje);

    // Función de compatibilidad: lectura de página (no utilizada en la nueva lógica)
    //bool leerPagina(uint8_t pagina, uint8_t *buffer);

    // Funciones para la lógica de la aplicación (se adaptan según se requiera)
    void set_mode(TOKEN_MODE_ modein) { tokenCurrentMode = modein; }
    //void set_reset_partner(bool state) { reset_partner = state; }
    void token_handler(const TOKEN_DATA& token,
                           uint8_t          lang_in,
                           bool             genre_in,
                           uint8_t          origin,
                           const std::vector<TARGETNS>& targetsNS);
    void proponer_token(byte guessbank);
    void printFicha(const TOKEN_DATA &f);

    // Agrega estas líneas en la sección pública de TOKEN_ en token_DMS.h:
    void startListeningToNFC();
    void handleCardDetected();
    bool isCardPresent();
    void resetReader();

    // Función auxiliar para convertir dos caracteres ASCII hex a un byte
    byte asciiHexToByte(char high, char low);
    //byte hexToByte(const String &hex);
    // Función auxiliar para parsear la cadena del token y convertirla en TOKEN_DATA
    //TOKEN_DATA parseTokenString(const String &tokenStr);
    // Variables públicas para facilitar el control en el loop
    String currentUID;
    String lastProcessedUID;

    // Parámetros de audio u otros (a adaptar según tu lógica)
    byte genre;
    byte lang;

    // Datos del token actual y el token propuesto (para modos de pareja/guess)
    TOKEN_DATA currentToken;
    TOKEN_DATA propossedToken;
    TOKEN_MODE_ tokenCurrentMode = TOKEN_BASIC_MODE;

    bool waitingForPartner = false;

    bool uidDetected = false;
    bool mensajeLeido = false;
    unsigned long uidDetectionTime = 0;


private:
    // Variables internas para controlar el estado
    
    //bool reset_partner = false;
    // Función auxiliar para decodificar el texto de un registro NDEF (tipo Text)
    String decodeNdefText(const byte* payload, int payloadLength);
};

extern DOITSOUNDS_ doitPlayer;

extern std::vector<byte> bankList;
extern std::vector<bool> selectedBanks;


#endif // TOKEN_DMS_H
