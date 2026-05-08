#pragma once
#ifndef PLAY_DMS_H
#define PLAY_DMS_H

#include <defines_DMS/defines_DMS.h>
#include <Element_DMS/Element_DMS.h>
#include <Arduino.h>
#include <Frame_DMS/Frame_DMS.h>
#include <vector>
#include "DFRobotDFPlayerMini.h"

//DFRobotDFPlayerMini player;
// test de commit
class DOITSOUNDS_ : public ELEMENT_{

    public:
        DOITSOUNDS_(byte voiceType= WOMAN_VOICE) : VOICE_TYPE(voiceType) {}
        DFRobotDFPlayerMini player;
        byte VOICE_TYPE;
        byte availableFolders[99];
        
        void begin();
        void play_file(byte bankin, byte filein);
        void stop_file();
        bool is_playing();
        void get_available_folders();
        bool wait_end(uint32_t startTimeoutMs = 300, uint32_t endTimeoutMs = 20000, uint32_t pollMs = 20);


};



#endif