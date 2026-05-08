#ifndef TRANSLATIONS_H
#define TRANSLATIONS_H

#include <Arduino.h>
#include <string.h>

// Enumerado de idiomas disponibles

enum class Language {
  ES = 1,  // Español
  EN,      // English
  DE,      // Deutch
  FR,      // Français
  ES_MX,   // Mex
  CA,      // Català
  EU,      // Euskera
  IT,       // X
  X1       // X1
};
// Variable global para almacenar el idioma actual (por defecto ES)
extern Language currentLanguage;

// Estructura que contiene la cadena en cada idioma
struct Translation {
    const char* key;  // Identificador único de la cadena
    const char* es;
    const char* es_mx;
    const char* ca;
    const char* eu;
    const char* fr;
    const char* de;
    const char* en;
    const char* it;
  };

// Declaración del array de traducciones
extern const Translation translations[];

// Función que dado un identificador retorna la cadena traducida según el idioma actual
const char* getTranslation(const char* key);

#endif // TRANSLATIONS_H
