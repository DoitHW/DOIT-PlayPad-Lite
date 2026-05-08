# DOIT - Protocolo ROOM+MAC

## 1. Vision general del sistema
- La botonera (origin 0xDB) dirige la red y coordina elementos (origin 0xDD), consola (0xDC) y otros roles opcionales.
- Cada mensaje transporta la sala (`room`) y las MAC de emisor y receptor (`originNS`, `targetNS`), lo que permite segmentar instalaciones y rastrear dispositivos moviles.
- Las MAC son la unica identidad persistente; no se usan IDs locales ni listas de targets.

## 2. Direccionamiento y roles
- `room` identifica la sala fisica o logica y se propaga en todas las tramas; `setLocalRoom()` actualiza el campo local.
- `origin` indica el tipo de emisor (0xDB botonera, 0xDD elemento, 0xDC consola, 0xDA dado, 0xDE error, 0xDF tech tool).
- `originNS` son los 5 bytes de la MAC del emisor, siempre presente para que el receptor valide el interlocutor real.
- `targetType` define a quien va dirigida la trama (0xDD elemento, 0xDB botonera, 0xFF broadcast, etc.).
- `targetNS` es la MAC del receptor. `NS_ZERO` (00:00:00:00:00) se usa para broadcast y discovery.
- Las respuestas invierten `origin`/`targetType`, copian `targetNS <- originNS` y conservan `room` para correlacion inmediata.

## 3. Formato de trama (FRAME_T)
- [0] `start` = 0xE1
- [1..2] `frameLengthMsb`, `frameLengthLsb`: longitud sin start/end
- [3] `room`
- [4] `origin`
- [5..9] `originNS`
- [10] `targetType`
- [11..15] `targetNS`
- [16] `function`
- [17..18] `dataLengthMsb`, `dataLengthLsb`
- [19..] `data` (payload)
- Ultimo byte antes del fin: `checksum`
- Byte final: `end` = 0xBB
- `checksum_calc()` protege la integrity; cualquier receptor debe recalcularlo antes de aceptar la trama.

### Ejemplo unicast (request de sector)
```
e1 00 0f db 01 db f2 00 1f 1a 94 dd 00 0a a0 00 02 01 05 ce bb
```
`room=0x01`, `origin=0xDB`, `originNS=f2:00:1f:1a:94`, `targetType=0xDD`, `targetNS` (MAC del elemento), `function=0xA0` (F_REQ_ELEM_SECTOR), `data=[idioma, sector]`.

### Ejemplo broadcast discovery
```
e1 00 0e db 01 db 01 02 03 04 05 ff 00 00 00 00 00 cf 00 01 02 59 bb
```
`targetType=0xFF`, `targetNS=NS_ZERO`, `function=0xCF` (F_SEND_COMMAND), `data=[wake_cmd]`. Todos los elementos despiertos responden en la misma sala.

## 4. Codigos de funcion efectivos
- 0xA0 `F_REQ_ELEM_SECTOR` (DLEN 0x0002) -> `[idioma, sector]`
- 0xD0 `F_RETURN_ELEM_SECTOR` (DLEN variable) -> `[sector, originNS, datos...]`
- 0xB2 `F_SET_ELEM_MODE` -> `[modeIdx]`
- 0xB3 `F_SET_ELEM_DEAF` -> `[0/1]`
- 0xC0 `F_SEND_RESPONSE` -> `[resp]`
- 0xC1 `F_SEND_COLOR` -> `[colorIndex]`
- 0xC2 `F_SEND_RGB` -> `[R,G,B]`
- 0xC3 `F_SEND_BRIGHTNESS` -> `[MSB,LSB]`
- 0xCA `F_SEND_SENSOR_VALUE_1` -> `SENSOR_DOUBLE_T`
- 0xCB `F_SEND_SENSOR_VALUE_2` -> `SENSOR_VALUE_T`
- 0xCC `F_SEND_FILE_NUM` -> `[bank,file]`
- 0xCD `F_SEND_PATTERN_NUM` -> `[pattern]`
- 0xCE `F_SEND_FLAG_BYTE` -> `[bitmask]`
- 0xCF `F_SEND_COMMAND` -> `[cmd,...]`
- `frameMaker_SET_ELEM_ID` solo se mantiene por compatibilidad, pero la botonera no lo usa ni lo persiste.

## 5. Flujos operativos
### 5.1 Escaneo y agregamiento
- Activar `scanInProgress` y mostrar modal de escaneo sin flicker.
- Enviar discovery broadcast (`F_SEND_COMMAND`) respetando una ventana (p. ej. 60 s).
- RX en modo scan solo encola `F_RETURN_ELEM_SECTOR` dentro de `rxSectorInbox`; no procesa hasta que se cierra la ventana.
- Registrar MAC nuevas leyendo `ELEM_SERIAL_SECTOR`.
- Iterar cada MAC detectada: solicitar sectores minimos (nombre, descripcion, cmode, flags, icono 0..63) usando `frameMaker_REQ_ELEM_SECTOR` y respetando `kInterCmdDelayMs >= 300 ms`.
- `esperar_respuesta(expectedSector, expectedNS, ...)` consume la cola, valida `originNS` y reintenta con timeout finito.
- `procesar_sector_NS()` quita los 5 bytes de MAC del payload (excepto en `ELEM_SERIAL_SECTOR`) tras validar contra `expectedNS`.

### 5.2 Control y sensores en foco
- Al enfocar un elemento solicitar `ELEM_CMODE_SECTOR` y flags del modo activo.
- Mapear LEDs de la botonera mediante `COLORPAD_BTNMAP` segun los bits (`HAS_PULSE`, `HAS_RELAY_1`, `HAS_ADVANCED_COLOR`, etc.).
- Enviar acciones unicas por combinacion: `F_SEND_COLOR` o `F_SEND_RGB` segun flags; `F_SEND_FLAG_BYTE` para relays; evitar rafagas multiples.
- Sensores: usar `F_SEND_SENSOR_VALUE[_2]` para datos analogicos/binarios; ADXL345 y microfono generan animaciones y payloads adecuados.

### 5.3 Persistencia SPIFFS
- Consolidar datos minimos del elemento y guardar en `/element_*.bin`.
- `serialNum` almacena la MAC; `room` permite detectar cambios de sala.
- Resolver nombres duplicados agregando sufijo `(n)`.
- No guardar archivos hasta completar todos los sectores requeridos.

## 6. Convenciones de UI y antiflicker
- Centralizar el render de modal, barra de progreso y mensajes temporales; dibujar solo cuando cambia el estado.
- Evitar llamadas encadenadas a rutinas de pintado dentro del mismo ciclo.

## 7. Checklist rapido
- [ ] Nunca leer ni persistir IDs locales.
- [ ] Toda trama debe fijar `room`, `targetType` y `targetNS` (o `NS_ZERO` si es broadcast).
- [ ] Validar `originNS` antes de procesar `F_RETURN_ELEM_SECTOR` y descartar los 5 bytes embebidos tras la validacion.
- [ ] Reintentar por sector/NS ante timeout respetando la ventana de delay.
- [ ] Guardar en SPIFFS solo al completar sectores minimos y asegurar nombres unicos.

## 8. Referencias rapidas
- `FRAME_T` y `TARGETNS` definen la estructura base (ver `lib/DOITlib/Frame_DMS/Frame_DMS.h`).
- Utilizar `setLocalNS()` y `setLocalRoom()` al arrancar para garantizar que las tramas salientes esten normalizadas.
- `NS_ZERO` = `{0,0,0,0,0}` esta disponible para broadcast y pruebas.
