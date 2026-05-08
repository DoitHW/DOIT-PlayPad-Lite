    #pragma once

#ifndef COLOR_DMS_H
#define COLOR_DMS_H

#include <defines_DMS/defines_DMS.h>
#include <Element_DMS/Element_DMS.h>
#include <DynamicLEDManager_DMS/DynamicLEDManager_DMS.h>
#include <FastLED.h>

/*
 ██████╗ ██████╗ ██╗      ██████╗ ██████╗ ███████╗
██╔════╝██╔═══██╗██║     ██╔═══██╗██╔══██╗██╔════╝
██║     ██║   ██║██║     ██║   ██║██████╔╝███████╗
██║     ██║   ██║██║     ██║   ██║██╔══██╗╚════██║
╚██████╗╚██████╔╝███████╗╚██████╔╝██║  ██║███████║
 ╚═════╝ ╚═════╝ ╚══════╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝
*/





// const unsigned int listaColoresPasivos[]=
// {

//     0xFF2300, // rojo_anaranjado_t
//     0xFF2500, // marron_oscuro_t
//     0xFF4005, // marron_t
//     0xFF4500, // naranja_t
//     0xFF7E00, // amarillo_anaranjado_t
//     0xFFDF00, // verde_amarillo_t
//     0x806600, // crema_t
//     0x66FF00, // verde_claro_t
//     0x7FA300, // verde_oliva_claro_t
//     0x758A00, // verde_oliva_t
//     0x7FA364, // verde_grisaceo_t
//     0x7FFF64, // verde_lima_t
//     0x98FF98, // verde_menta_t
//     0x50C878, // esmeralda_t
//     0x0DBA98, // verde_azulado_t
//     0x1F3438, // azul_verdoso_t
//     0x40E0D0, // turquesa_t
//     0x00FFFF, // cyan_t
//     0x5080FF, // celeste_claro_t
//     0x330099, // indigo_t
//     0x9400D3, // morado_t
//     0x8F00FF, // lila_t
//     0xFF00D2, // rosa_t
//     0xFF1493, // rosa_oscuro_t
//     0xE4007C, // rosa_fuerte_t
//     0xD9017A, // magenta_t
//     0xE73410  // salmon_t
// };


// const unsigned int listaColoresPasivos_COPYRIGHT[]={

//     0x05F016, // DOIT
//     0x3A007C, // CADSBURY
//     0xDA1884, // ROSA BARBIE
//     0x00843D, // CORTE INGLES
//     0xF40009, // COCA-COLA
//     0x0033A0, // YVES KLEIN
//     0x582C83, // MILKA
//     0x81D8D0, // TIFFANYS BLUE
//     0xF2CA00  // CAT
// };

// const unsigned int listaColoresPasivos_3[]=
// {

// 0xFF0000, // pure_red
// 0xFF1E1E, // bright_red
// 0xFF4242, // coral_red

// 0xFF6B00, // vivid_orange
// 0xFF8C1A, // amber

// 0xFFD700, // golden_yellow
// 0xFFEB3B, // bright_yellow

// 0xB4D335, // lime_yellow
// 0x7FBA00, // chartreuse

// 0x00FF00, // pure_green
// 0x00D364, // spring_green
// 0x009B77, // forest_green

// 0x00CED1, // turquoise
// 0x20B2AA, // sea_green

// 0x0099FF, // sky_blue
// 0x0066CC, // ocean_blue
// 0x0033FF, // royal_blue

// 0x6A0DAD, // purple
// 0x8B00FF, // violet

// 0xFF00FF  // magenta
// };


const unsigned int listaColores[36] = 
{
    0xFFFFAA, // white_t 0
    0xFF9B00, // yellow_t 1 
    0xFF5900, // orange_t 2
    0xFF0000, // red_t 3
    0xFF00D2, // violet_t 4
    0x0000FF, // blue_t 5
    0x00FFC8, // light_blue_t 6
    0x00FF00, // green_t 7
    0x000000, // black_t 8

    0xFF2300, // 9 rojo_anaranjado_t (mezcla entre red y orange)
    0xFF2500, // 10 marron_oscuro_t   (mezcla entre )
    0xFF4005, // 11 marron_t

    0xFF4500, // 12 naranja_t (mezcla entre yellow y red)
    0xFF7E00, // 13 amarillo_anaranjado_t

    0xFFDF00, // 14 verde_amarillo_t (mezcla entre yellow y green)
    0x806600, // 15 crema_t (mezcla entre white y yellow)

    0x66FF00, // 16 verde_claro_t (mezcla entre white y green)
    0x7FA300, // 17 verde_oliva_claro_t
    0x758A00, // 18 verde_oliva_t (mezcla entre yellow y blue)
    0x7FA364, // 19 verde_grisaceo_t
    0x7FFF64, // 20 verde_lima_t (mezcla entre yellow y light_blue)
    0x98FF98, // 21 verde_menta_t 
    0x50C878, // 22 esmeralda_t
    0x0DBA98, // 23 verde_azulado_t

    0x1F3438, // 24 azul_verdoso_t
    0x40E0D0, // 25 turquesa_t (mezcla entre white y light_blue)
    0x00FFFF, // 26 cyan_t
    0x5080FF, // 27 celeste_claro_t (mezcla entre white y blue)

    0x330099, // 28 indigo_t

    0x9400D3, // 29 morado_t
    0x8F00FF, // 30 lila_t (mezcla entre white y violet)
    0xFF00D2, // 31 rosa_t (mezcla entre white y red)
    0xFF1493, // 32 rosa_oscuro_t  (mezcla entre yellow y violet)
    0xE4007C, // 33 rosa_fuerte_t
    0xD9017A, // 34 magenta_t (mezcla entre red y violet)
    0xE73410  // 35 salmon_t (blanco y naranja)
};



class COLORHANDLER_ {

    public:
        COLORHANDLER_();
        void begin(int numLeds);
        void setPatternBotonera(byte mode, DynamicLEDManager& ledManager);
        void welcomeEffect();
        CRGB* leds;
        int numLeds;
        void setCurrentFile(const String& file) { currentFile = file; } 
        bool color_mix_handler(int color1, int color2, byte *resultado);
        void mapCognitiveLEDs();
        

    private:
    String currentFile;
    static inline bool isBlack(const CRGB& c) { return c.r==0 && c.g==0 && c.b==0; }
    void setAllButtonsActive(bool val);
    void syncButtonsWithLEDs();  

};

extern COLORHANDLER_ colorHandler;


#endif