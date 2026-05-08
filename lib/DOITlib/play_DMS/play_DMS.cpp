#include <play_DMS/play_DMS.h>




void DOITSOUNDS_::begin(){
    
  if (player.begin(Serial2, false, true)) {
    delay(800);
    player.volume(30);
    player.playFolder(1, 9);
                                                                        #ifdef DEBUG
                                                                        //DEBUG__________ln("DFPlayer Mini inicializado");
                                                                        #endif
  } else {
                                                                        #ifdef DEBUG
                                                                        DEBUG__________ln("Error al inicializar DFPlayer Mini");
                                                                        #endif
  }
  delay(10);

}

void DOITSOUNDS_::play_file(byte bankin, byte filein){
  player.stop();
  delay(30);              // 20–50 ms suele bastar
  player.playFolder(bankin, filein);
}

void DOITSOUNDS_::stop_file(){
  player.stop();
}

bool DOITSOUNDS_::is_playing(){   // devuelve true mientras este sonando.
 
  return (player.readState() == 513);
}

void DOITSOUNDS_::get_available_folders(){
  int j= 0;
  for(int i= 1; i < 100; i++){

    if(player.readFileCountsInFolder(i)) availableFolders[j++]= i;
  }
}

bool DOITSOUNDS_::wait_end(uint32_t startTimeoutMs, uint32_t endTimeoutMs, uint32_t pollMs)
{
    if (pollMs < 5) pollMs = 5;

    // 1) Espera a que el DFPlayer “arranque” realmente el estado PLAYING
    const uint32_t t0 = millis();
    while ((millis() - t0) < startTimeoutMs) {
        if (is_playing()) break;
        delay(pollMs);
    }

    // Si nunca llegó a marcar PLAYING, no bloquees indefinidamente
    if (!is_playing()) return false;

    // 2) Espera a que termine
    const uint32_t t1 = millis();
    while ((millis() - t1) < endTimeoutMs) {
        if (!is_playing()) return true;
        delay(pollMs);
    }

    // Recovery: si se quedó colgado, corta para no bloquear el sistema
    player.stop();
    return false;
}
