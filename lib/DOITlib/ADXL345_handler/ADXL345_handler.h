#pragma once
#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Frame_DMS/Frame_DMS.h>

// Definición de pines I2C
#if defined(DOIT_LITE)
  #if defined(DEVKIT)
    #define SDA_ADXL_PIN 42
    #define SCL_ADXL_PIN 2
    #define INT1_ADXL_PIN 39
    #define INT2_ADXL_PIN 40
  #else
    #define SDA_ADXL_PIN 47
    #define SCL_ADXL_PIN 48
    #define INT1_ADXL_PIN 14
    #define INT2_ADXL_PIN 15
  #endif
#else
  #define SDA_ADXL_PIN 47
  #define SCL_ADXL_PIN 48
  #define INT1_ADXL_PIN 14
  #define INT2_ADXL_PIN 15
#endif

// Clase para manejar el acelerómetro ADXL345
class ADXL345Handler {
public:
    ADXL345Handler();                  // Constructor
    void init();                      // Inicializar el acelerómetro
    long convertInclinationToValue(float inclination);
    //SENSOR_VALUE_T createSensorValue(long finalValue);
    //bool isInitialized() const;       // Verificar si está inicializado
    //void setThreshold(float newThreshold);  // Configurar un nuevo umbral
    void end();
    void readInclinations(bool isBinary, TARGETNS ns);
    SENSOR_DOUBLE_T createSensorDoubleValue(long finalValueX, long finalValueY);
    void sendSensorValueDouble(const SENSOR_DOUBLE_T &sensorValue, TARGETNS ns);
    void enableActivityInterrupt(uint16_t threshold_mg, bool enX, bool enY, bool enZ);
    void clearInterrupts();
    bool isInitialized() const { return initialized; }
    unsigned long getLastMovementTime() const { return lastMovementTime; }

private:
    Adafruit_ADXL345_Unified accel;
    //float lastInclination;  // Última inclinación leída
    //float lastInclinationX;
    //float lastInclinationY;
    float threshold;        // Umbral de cambio significativo
    float thresholdBinary;  // Umbral para sensor binario
    bool initialized;       // Estado de inicialización
    //int errorCount;
    static const unsigned long movementSampleInterval = 20;  // muestreo rápido (50 originalmente)
    static const unsigned long inactivityTimeout     = 1000; // 1 s sin movimiento

    unsigned long lastSampleTime    = 0;
    unsigned long lastMovementTime  = 0;
    bool         movementDetectedLast = false;
    float        lastSampleX = 0, lastSampleY = 0;

    // --- NUEVO: Control de envío duplicado y throttling ---
    long          lastSentValX = -99999; // Valor imposible inicial
    long          lastSentValY = -99999; // Valor imposible inicial
    unsigned long lastFrameTime = 0;     // Para controlar los 100ms
};

// Declaración de variable global para el control del ADXL345
extern ADXL345Handler adxl345Handler;
extern bool adxl;  // Variable global para habilitar/deshabilitar lecturas
