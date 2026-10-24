#pragma once

// Mega 2560 + circuito externo: D5 HIGH arranca; LOW para.
// Potencia habilitada segun prueba confirmada por el usuario.
const bool POTENCIA_VERIFICADA = true;
const bool SERVOS_VERIFICADOS = false;
const bool CALIBRACION_LISTA = false;

// true: tabla literal del PDF. false: altura fisica usual.
const bool LOGICA_LITERAL_DOCUMENTO = true;
const uint8_t LCD_ADDR = 0x27;
const uint32_t BT_BAUD = 9600;

const uint8_t LDR_BAJO = A0, LDR_ALTO = A1, LDR_PRESENCIA = A2;
const uint8_t S1_PIN = 6, S2_PIN = 7;
const uint8_t PIN_SENAL = 5;
const uint8_t AZUL = 22, VERDE = 23, ROJO = 24, BUZZER = 25;

// EJEMPLOS en milisegundos de motor activado, desde deteccion A2.
// Medir/calibrar con SU banda a velocidad fija de su motor. No equivalen a posicion medida.
const uint32_t S1_DESDE_MS = 1500, S1_HASTA_MS = 2400;
const uint32_t S2_DESDE_MS = 3000, S2_HASTA_MS = 3900;
const uint32_t SALIDA_MS = 5000;
const uint16_t VENTANA_MS = 100;
const uint16_t UMBRAL_BAJO = 500, UMBRAL_ALTO = 500, UMBRAL_PRESENCIA = 500;
const uint16_t HISTERESIS = 35;
const uint16_t FILTRO_PRESENCIA_MS = 25;
const uint16_t PORTAL_LIBRE_MS = 200;
const uint8_t S1_NEUTRO = 90, S1_ACTIVO = 120;
const uint8_t S2_NEUTRO = 90, S2_ACTIVO = 60;

static_assert(S1_DESDE_MS > VENTANA_MS, "S1 debe quedar despues de lectura");
static_assert(S1_HASTA_MS > S1_DESDE_MS, "Ventana S1 incorrecta");
static_assert(S2_HASTA_MS > S2_DESDE_MS, "Ventana S2 incorrecta");
static_assert(S2_DESDE_MS > VENTANA_MS, "S2 debe quedar despues de lectura");
static_assert(SALIDA_MS > S1_HASTA_MS && SALIDA_MS > S2_HASTA_MS, "Salida demasiado cercana");
static_assert(UMBRAL_BAJO > HISTERESIS && UMBRAL_BAJO + HISTERESIS < 1023, "Umbral A0 invalido");
static_assert(UMBRAL_ALTO > HISTERESIS && UMBRAL_ALTO + HISTERESIS < 1023, "Umbral A1 invalido");
static_assert(UMBRAL_PRESENCIA > HISTERESIS && UMBRAL_PRESENCIA + HISTERESIS < 1023, "Umbral A2 invalido");
