#include <ADXL345_handler/ADXL345_handler.h>
#include <Arduino.h>
#include <Frame_DMS/Frame_DMS.h>
#include <SPIFFS_handler/SPIFFS_handler.h>
#include <encoder_handler/encoder_handler.h>


ADXL345Handler adxl345Handler;

// Constructor
ADXL345Handler::ADXL345Handler() 
    : accel(Adafruit_ADXL345_Unified(12345)), 
      threshold(1.0), 
      thresholdBinary(1.0), 
      initialized(false),
      lastSentValX(-99999), 
      lastSentValY(-99999), 
      lastFrameTime(0) {}

// Inicialización del ADXL345
void ADXL345Handler::init()
{
    // Si ya estaba activo, lo cerramos para reiniciar limpio
    if (initialized) {
        Wire.end(); 
    }

    // Iniciamos Wire forzando los pines explícitamente
    Wire.begin(SDA_ADXL_PIN, SCL_ADXL_PIN);
    Wire.setClock(100000); // 100kHz es más estable para cables largos/compartidos

    const int maxAttempts = 3;
    int attempt = 0;
    bool detected = false;
    
    while (attempt < maxAttempts)
    {
        if (accel.begin())
        {
            detected = true;
            break;
        }
        else
        {
            DEBUG__________ln("ADXL345 no detectado, reintentando...");
            
            // Intento de recuperación leve dentro del bucle
            Wire.end();
            delay(50);
            Wire.begin(SDA_ADXL_PIN, SCL_ADXL_PIN);
            delay(100); 
        }
        attempt++;
    }

    if (!detected)
    {
        DEBUG__________ln("ADXL345 no detectado tras varios intentos.");
        initialized = false;
        return;
    }

    accel.setRange(ADXL345_RANGE_2_G);
    initialized = true;
    DEBUG__________ln("ADXL345 inicializado correctamente");
}
// Función para convertir la inclinación en un valor escalado
long ADXL345Handler::convertInclinationToValue(float inclination) {
    long scaledInclination = inclination * 100;
    return map(scaledInclination, -1000, 1000, 0, 2000);
}

// Función para empaquetar el valor en SENSOR_VALUE_T
// SENSOR_VALUE_T ADXL345Handler::createSensorValue(long finalValue) {
//     SENSOR_VALUE_T sensorin;
//     sensorin.lsb_min = 0x00;
//     sensorin.msb_min = 0x00;
//     sensorin.msb_max = 0x07;
//     sensorin.lsb_max = 0xD0;
//     sensorin.msb_val = (byte)(finalValue >> 8);
//     sensorin.lsb_val = (byte)(finalValue & 0xFF);
//     return sensorin;
// }


// Verificar si el ADXL345 está inicializado
// bool ADXL345Handler::isInitialized() const {
//     return initialized;
// }

// Establecer un nuevo umbral
// void ADXL345Handler::setThreshold(float newThreshold) {
//     threshold = newThreshold;
// }

void ADXL345Handler::end()
{
    if (!initialized)
    {
                                                                    #ifdef DEBUG
                                                                    DEBUG__________ln("ADXL345Handler::end() llamado, pero ya estaba desactivado.");
                                                                    #endif 
        return;
    }

    Wire.end();
    initialized = false;
                                                                    #ifdef DEBUG 
                                                                    DEBUG__________ln("I2C desactivado (acelerómetro end).");
                                                                    #endif
}

void ADXL345Handler::readInclinations(bool isBinary, TARGETNS ns) {
    if (!initialized || !adxl) return;

    // 1) Control de tiempo (50ms)
    unsigned long now = millis();
    if (now - lastSampleTime < movementSampleInterval) return;
    lastSampleTime = now;

    sensors_event_t event;
    if (!accel.getEvent(&event)) return;

    // --- NUEVO: FILTRO PASO BAJO (Suavizado) ---
    // Usamos variables estáticas para guardar el estado del filtro entre llamadas
    static float smoothX = 0;
    static float smoothY = 0;
    static bool firstRun = true;

    if (firstRun) {
        // Inicializamos el filtro con el primer valor real para evitar saltos al inicio
        smoothX = event.acceleration.x;
        smoothY = event.acceleration.y;
        
        // Inicializamos la referencia de movimiento para evitar disparo al arrancar
        lastSampleX = smoothX;
        lastSampleY = smoothY;
        
        firstRun = false;
        return; 
    }

    // Factor de suavizado (Alpha). 
    // 0.2 = Muy suave (elimina mucho ruido, un pelín de lag). 
    // 0.5 = Respuesta rápida (menos filtrado).
    // Con tus picos de ruido, 0.2 es ideal.
    float alpha = 0.6f; // originalfloat alpha = 0.2; 

    // Aplicamos el filtro:
    smoothX = (event.acceleration.x * alpha) + (smoothX * (1.0 - alpha));
    smoothY = (event.acceleration.y * alpha) + (smoothY * (1.0 - alpha));

    // Usamos los valores SUAVIZADOS para toda la lógica
    float currX = smoothX;
    float currY = smoothY;
    // ---------------------------------------------

    if (isBinary) {
        float diffX = abs(currX - lastSampleX);
        float diffY = abs(currY - lastSampleY);
        bool movementDetected = (diffX >= thresholdBinary || diffY >= thresholdBinary);

        if (movementDetected && !movementDetectedLast) {
            SENSOR_DOUBLE_T binVal = {0,0, 0,1, 0,1, 0,0, 0,0, 0,0};
            send_frame(frameMaker_SEND_SENSOR_VALUE(DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, binVal));
            movementDetectedLast = true;
        }
        if (movementDetected) lastMovementTime = now;

        if (movementDetectedLast && (now - lastMovementTime >= inactivityTimeout)) {
            SENSOR_DOUBLE_T binVal = {0,0, 0,1, 0,0, 0,0, 0,0, 0,0};
            send_frame(frameMaker_SEND_SENSOR_VALUE(DEFAULT_BOTONERA, DEFAULT_DEVICE, ns, binVal));
            movementDetectedLast = false;
        }
        lastSampleX = currX;
        lastSampleY = currY;
    } 
    else {
        // --- LÓGICA ANALÓGICA ACUMULATIVA (Usando valores suavizados) ---
        
        float diffX = abs(currX - lastSampleX);
        float diffY = abs(currY - lastSampleY);

        // Mantenemos tu threshold de 1.2 (es correcto para filtrar, el filtro de software ayuda extra)
        bool triggerX = (diffX >= threshold);
        bool triggerY = (diffY >= threshold);
        
        if (triggerX || triggerY) {
            
            // long valXraw = constrain(currX, -10, 10);
            // long valYraw = constrain(currY, -10, 10);
            // long valX = convertInclinationToValue(valXraw);
            // long valY = convertInclinationToValue(valYraw);

            const float valXraw = constrain(currX, -10.0f, 10.0f);
            const float valYraw = constrain(currY, -10.0f, 10.0f);
            const long  valX    = convertInclinationToValue(valXraw);
            const long  valY    = convertInclinationToValue(valYraw);

            unsigned long nowFrame = millis();
            
            if (nowFrame - lastFrameTime >= 70) {//if (nowFrame - lastFrameTime >= 20) { //originalmente 70ms
                
                if (valX != lastSentValX || valY != lastSentValY) {
                    
                    String axisCause = "";
                    if (triggerX) axisCause += "[X🟢]";
                    if (triggerY) axisCause += "[Y🛑]";

                    // DEBUG__________printf("🚀 TX | Causa: %s | DiffAcum: [X=%.2f Y=%.2f] | Send: [X=%ld Y=%ld]\n", 
                    //                       axisCause.c_str(), diffX, diffY, valX, valY);

                    SENSOR_DOUBLE_T sensorVal = createSensorDoubleValue(valX, valY);
                    sendSensorValueDouble(sensorVal, ns);

                    lastSentValX = valX;
                    lastSentValY = valY;
                    lastFrameTime = nowFrame; 

                    lastMovementTime = nowFrame;
                    
                    // Actualizamos la referencia con el valor SUAVIZADO actual
                    lastSampleX = currX;
                    lastSampleY = currY;
                }
            }
        }
    }
}

SENSOR_DOUBLE_T ADXL345Handler::createSensorDoubleValue(long finalValueX, long finalValueY) {
    SENSOR_DOUBLE_T s;
    // Primer eje
    s.msb_min  = 0x00; s.lsb_min  = 0x00;
    s.msb_max  = 0x07; s.lsb_max  = 0xD0;
    s.msb_val  = (byte)(finalValueX >> 8);
    s.lsb_val  = (byte)(finalValueX & 0xFF);
    // Segundo eje
    s.msb_min2 = 0x00;   s.lsb_min2 = 0x00;
    s.msb_max2 = 0x07;   s.lsb_max2 = 0xD0;
    s.msb_val2 = (byte)(finalValueY >> 8);
    s.lsb_val2 = (byte)(finalValueY & 0xFF);
    return s;
}

void ADXL345Handler::sendSensorValueDouble(const SENSOR_DOUBLE_T &sensorValue, TARGETNS ns) {

    // Verificar que el número de serie no sea nulo (ej. {0,0,0,0,0})
    bool isValid = false;
    for (int i = 0; i < 5; i++) {
        if (((uint8_t*)&ns)[i] != 0) { 
            isValid = true; 
            break; 
        }
    }

    if (!isValid) {
        #ifdef DEBUG
                DEBUG__________ln("⚠️ No se pudo obtener el número de serie del elemento. Trama no enviada.");
        #endif
        return;
    }

    send_frame(frameMaker_SEND_SENSOR_VALUE(
        DEFAULT_BOTONERA,   // origen = botonera
        DEFAULT_DEVICE,     // destino = un dispositivo normal
        ns,                // número de serie del elemento
        sensorValue
    ));
}

// ====== Configuración de INT de ACTIVIDAD para wake (INT1) ======
static inline void adxl_write_u8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x53); // ADXL345 por defecto
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static inline uint8_t adxl_read_u8(uint8_t reg) {
  Wire.beginTransmission(0x53);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x53, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

// Registros
#define ADXL_REG_THRESH_ACT     0x24
#define ADXL_REG_ACT_INACT_CTL  0x27
#define ADXL_REG_BW_RATE        0x2C
#define ADXL_REG_POWER_CTL      0x2D
#define ADXL_REG_INT_ENABLE     0x2E
#define ADXL_REG_INT_MAP        0x2F
#define ADXL_REG_INT_SOURCE     0x30
#define ADXL_REG_DATA_FORMAT    0x31

void ADXL345Handler::enableActivityInterrupt(uint16_t threshold_mg, bool enX, bool enY, bool enZ) {
  if (!initialized) init();  // asegura I2C y detección

  // --- Mantener DATA_FORMAT de init y forzar solo el rango a ±16 g ---
  // DATA_FORMAT: [D3=FULL_RES][D2=JUSTIFY][D1:D0=RANGE]
  uint8_t fmt = adxl_read_u8(ADXL_REG_DATA_FORMAT);
  fmt = (fmt & ~0x03) | 0x03;              // D1:D0 = 11b => ±16 g; no toca FULL_RES/JUSTIFY
  //adxl_write_u8(ADXL_REG_DATA_FORMAT, fmt);
  adxl_write_u8(ADXL_REG_DATA_FORMAT, 0x00);

  // --- Tasa de datos / ancho de banda ---
  // 0x08 = 25 Hz (OK para bajo consumo y menos ruido). Cambia si necesitas más rapidez.
  adxl_write_u8(ADXL_REG_BW_RATE, 0x08);

  // --- Umbral de actividad (62,5 mg/LSB), saturado a 1..255 LSB ---
  uint16_t lsbs = (uint16_t)lroundf(threshold_mg / 62.5f);
  if (lsbs < 1)   lsbs = 1;
  if (lsbs > 255) lsbs = 255;
  adxl_write_u8(ADXL_REG_THRESH_ACT, (uint8_t)lsbs);

  // --- ACT_INACT_CTL: AC-coupled en actividad; ejes habilitados según flags ---
  uint8_t act = 0;
  if (enX) act |= (1 << 6);
  if (enY) act |= (1 << 5);
  if (enZ) act |= (1 << 4);
  act |= (1 << 7); // ACT_AC = 1
  adxl_write_u8(ADXL_REG_ACT_INACT_CTL, act);

  // --- Mapear / habilitar la interrupción de ACTIVIDAD en INT1 ---
  uint8_t int_map = adxl_read_u8(ADXL_REG_INT_MAP);
  int_map &= ~(1 << 4);                    // bit4=0 -> INT1
  adxl_write_u8(ADXL_REG_INT_MAP, int_map);

  uint8_t int_en = adxl_read_u8(ADXL_REG_INT_ENABLE);
  int_en |= (1 << 4);                      // habilita ACTIVIDAD
  adxl_write_u8(ADXL_REG_INT_ENABLE, int_en);

  // --- Entrar en modo Measure ---
  adxl_write_u8(ADXL_REG_POWER_CTL, 0x08);
}


void ADXL345Handler::clearInterrupts() {
  (void)adxl_read_u8(ADXL_REG_INT_SOURCE);  // leer limpia el latch
}




