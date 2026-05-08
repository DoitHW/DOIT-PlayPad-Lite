#ifndef MICROPHONE_DMS_H
#define MICROPHONE_DMS_H


#include <defines_DMS/defines_DMS.h>
#include <Element_DMS/Element_DMS.h>
#include <FastLED.h>
#include <Arduino.h>
#include <Frame_DMS/Frame_DMS.h>
#include <vector>
#include <driver/i2s.h>
#include <arduinoFFT.h>


#ifdef PLAYER
    #include <play_DMS/play_DMS.h>
    extern DOITSOUNDS_ doitPlayer;
#endif



class MICROPHONE_{

    private:
        arduinoFFT FFT = arduinoFFT();
        // Frecuencias centrales de las notas (en Hz)
        const float noteFrequencies[7] = {261.63, 293.66, 329.63, 349.23, 392.00, 440.00, 493.88};
        const char* noteNames[7] = {"DO", "RE", "MI", "FA", "SOL", "LA", "SI"};
        unsigned long lastSoundTime = 0;
;

    public:

        MICROPHONE_(){}
        void begin();

        byte get_mic_value_BYTE(int sens);
        byte get_mic_value_BYTE_voice();
        bool detect_sound_threshold();
        byte detect_musical_note();
        void end();
        int readMicRaw();
        void calibracionInicial(unsigned long duracionCalibracion);
        void updateSoundTime() { lastSoundTime = millis(); }
        unsigned long getLastSoundTime() const { return lastSoundTime; }
        bool isActive() const;
        
};

extern bool useMic;

#endif