/*
 * SmartSort GT - Clasificadora de paquetes
 * Organizacion Computacional - FIUSAC - 2S 2026 - Arduino Mega 2560
 */
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// ---- Configuracion ----
#if defined(__AVR_ATmega2560__)
const uint8_t PIN_BT_RX     = 19;
const uint8_t PIN_BT_TX     = 18;
#else
const uint8_t PIN_BT_RX     = 2;
const uint8_t PIN_BT_TX     = 3;
#endif
const uint8_t PIN_MOTOR     = 5;
const uint8_t PIN_LED_AZUL  = 6;
const uint8_t PIN_LED_VERDE = 7;
const uint8_t PIN_LED_ROJO  = 8;
const uint8_t PIN_SERVO_S1  = 9;
const uint8_t PIN_SERVO_S2  = 10;
const uint8_t PIN_BUZZER    = 12;
const uint8_t PIN_LDR_L1    = A0;
const uint8_t PIN_LDR_L2    = A1;
const uint8_t PIN_LDR_L3    = A2;   // laser al final de la banda (rechazos)

// 0: solo L1 = PEQUENO, L1+L2 = GRANDE (seccion 4.4). 1: al reves (seccion 4.1)
#define INVERTIR_GRANDE_PEQUENO 0
#define BUZZER_PASIVO           0   // 1 si el buzzer es pasivo

const int UMBRAL_L1  = 600;          // lectura < umbral = laser cortado
const int UMBRAL_L2  = 600;
const int UMBRAL_L3  = 600;
const int HISTERESIS = 30;
const unsigned long T_FILTRO_MS      = 15;
const unsigned long T_SALIDA_MS      = 150;
const unsigned long T_MIN_PAQUETE_MS = 30;

const uint8_t S1_NEUTRAL = 0;
const uint8_t S1_ACTIVO  = 60;
const uint8_t S2_NEUTRAL = 0;
const uint8_t S2_ACTIVO  = 60;
const unsigned long RETARDO_S1_MS = 600;
const unsigned long RETARDO_S2_MS = 1200;
const unsigned long T_DESVIO_MS   = 1500;

const uint8_t VELOCIDAD_BANDA = 255;
const unsigned long T_VACIADO_MS = 3000;
const uint8_t  LOTE_MAX           = 99;
const unsigned long T_FIN_COMANDO_MS = 2000;
const unsigned long T_MENSAJE_MS  = 2500;
const unsigned long T_REPORTE_MS  = 4000;
const uint16_t TONO_GRANDE  = 1500;
const uint16_t TONO_PEQUENO = 2500;
const uint16_t TONO_RECHAZO = 600;
const uint16_t TONO_LOTE    = 1000;

// ---- Mapa de la EEPROM ----
const int EE_FIRMA    = 0;
const int EE_GRANDES  = 2;
const int EE_PEQUENOS = 4;
const int EE_RECHAZOS = 6;
const int EE_TOTAL    = 8;
const int EE_CHECKSUM = 12;
const uint16_t FIRMA_EEPROM = 0x5347;
const uint32_t MAX_TURNO    = 9999UL;
const uint32_t MAX_TOTAL    = 99999999UL;

enum Estado    : uint8_t { EST_LISTO, EST_ACTIVO, EST_PAUSADO };
enum Modo      : uint8_t { MODO_NINGUNO, MODO_NORMAL, MODO_LOTE, MODO_MANTENIMIENTO };
enum Categoria : uint8_t { CAT_NINGUNA, CAT_GRANDE, CAT_PEQUENO, CAT_RECHAZO };
enum Pantalla  : uint8_t { PANT_BASE, PANT_REPORTE };

struct SensorLaser {
  uint8_t pin;
  int umbral;
  bool cortado;
  bool candidato;
  unsigned long tCandidato;
  int lectura;
};

struct Portal {
  bool ocupado;
  bool vioL1, vioL2;
  unsigned long tInicio, tUltimo;
};

struct Desviador {
  Servo *servo;
  uint8_t angNeutral, angActivo;
  unsigned long retardo;
  bool desviando;
  unsigned long tActivo;
};

struct EventoServo {
  uint8_t desviador;
  unsigned long tRegistro;
};

struct Indicador {
  uint8_t pin;
  bool esBuzzer;
  uint8_t pendientes;
  uint16_t tOn, tOff, frecuencia;
  bool encendido;
  unsigned long tCambio;
};

struct Entrada {
  char texto[24];
  uint8_t largo;
  bool desbordado;
  unsigned long tUltimo;
};

// ---- LCD 16x2 I2C (PCF8574) ----
class LcdI2C {
public:
  bool iniciar();
  void linea(uint8_t fila, const char *texto);
  uint8_t direccion() { return dir; }
private:
  uint8_t dir = 0;
  char sombra[2][16];
  void expander(uint8_t b);
  void nibble(uint8_t alto, uint8_t rs);
  void enviar(uint8_t valor, uint8_t rs);
};

void LcdI2C::expander(uint8_t b) {
  Wire.beginTransmission(dir);
  Wire.write((uint8_t)(b | 0x08));
  Wire.endTransmission();
}

void LcdI2C::nibble(uint8_t alto, uint8_t rs) {
  uint8_t b = (alto & 0xF0) | rs;
  expander(b);
  expander(b | 0x04);
  delayMicroseconds(1);
  expander(b);
  delayMicroseconds(50);
}

void LcdI2C::enviar(uint8_t valor, uint8_t rs) {
  nibble(valor & 0xF0, rs);
  nibble((uint8_t)(valor << 4), rs);
}

bool LcdI2C::iniciar() {
  static const uint8_t candidatas[] = {0x27, 0x3F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
                                       0x26, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E};
  dir = 0;
  for (uint8_t i = 0; i < sizeof(candidatas); i++) {
    Wire.beginTransmission(candidatas[i]);
    if (Wire.endTransmission() == 0) { dir = candidatas[i]; break; }
  }
  if (dir == 0) return false;

  delay(50);
  expander(0);
  delay(5);
  nibble(0x30, 0); delayMicroseconds(4500);
  nibble(0x30, 0); delayMicroseconds(4500);
  nibble(0x30, 0); delayMicroseconds(150);
  nibble(0x20, 0);
  enviar(0x28, 0);
  enviar(0x0C, 0);
  enviar(0x01, 0);
  delay(2);
  enviar(0x06, 0);
  memset(sombra, ' ', sizeof(sombra));
  return true;
}

void LcdI2C::linea(uint8_t fila, const char *texto) {
  if (dir == 0 || fila > 1) return;
  bool fin = false;
  int8_t cursor = -1;
  for (uint8_t col = 0; col < 16; col++) {
    char c = ' ';
    if (!fin) {
      if (texto[col] == '\0') fin = true; else c = texto[col];
    }
    if (sombra[fila][col] == c) continue;
    if (cursor != (int8_t)col) enviar(0x80 | (fila ? 0x40 : 0x00) | col, 0);
    enviar((uint8_t)c, 0x01);
    sombra[fila][col] = c;
    cursor = col + 1;
  }
}

#if defined(__AVR_ATmega2560__)
#define bt Serial1
#else
SoftwareSerial bt(PIN_BT_RX, PIN_BT_TX);
#endif
Servo servoS1, servoS2;
LcdI2C lcd;

const uint8_t NUM_SENSORES = 3;
SensorLaser sensores[NUM_SENSORES] = {
  {PIN_LDR_L1, UMBRAL_L1, false, false, 0, 0},
  {PIN_LDR_L2, UMBRAL_L2, false, false, 0, 0},
  {PIN_LDR_L3, UMBRAL_L3, false, false, 0, 0},
};
Portal portal;
Portal salida;

const uint8_t D_S1 = 0, D_S2 = 1;
Desviador desviadores[2] = {
  {&servoS1, S1_NEUTRAL, S1_ACTIVO, RETARDO_S1_MS, false, 0},
  {&servoS2, S2_NEUTRAL, S2_ACTIVO, RETARDO_S2_MS, false, 0},
};
const uint8_t TAM_COLA = 6;
EventoServo cola[TAM_COLA];
uint8_t colaCantidad = 0;

Indicador ledVerde = {PIN_LED_VERDE, false, 0, 0, 0, 0, false, 0};
Indicador ledRojo  = {PIN_LED_ROJO,  false, 0, 0, 0, 0, false, 0};
Indicador buzzer   = {PIN_BUZZER,    true,  0, 0, 0, 0, false, 0};

Entrada entradaBT, entradaUSB;

unsigned long ahora = 0;
Estado estado = EST_LISTO;
Modo modo = MODO_NINGUNO;
Categoria ultimoPaquete = CAT_NINGUNA;
bool bandaEncendida = false;
bool falloEEPROM = false;

uint8_t loteObjetivo = 0;
uint8_t loteProcesados = 0;
bool loteCerrando = false;
unsigned long tCierreLote = 0;
unsigned long tPausa = 0;

unsigned long tMensaje = 0, duracionMensaje = 0;
Pantalla pantallaDespues = PANT_BASE;
unsigned long tMonitor = 0;

uint16_t eeLeer16(int dir);
uint32_t eeLeer32(int dir);
uint8_t eeChecksum();
bool eeEscribir16(int dir, uint32_t valor);
bool eeEscribir32(int dir, uint32_t valor);
bool eeSellar();
bool eeValida();
void iniciarEEPROM();
bool guardarEEPROM(Categoria c);
bool resetearTurnoEEPROM();

void controlarBanda(bool encender);
void moverServo(Desviador &d, bool activar);
void programarDesvio(uint8_t idx);
void vaciarDesvios();
void compensarPausa();
void actualizarServos();
void indicadorSalida(Indicador &x, bool on);
void indicadorPulsos(Indicador &x, uint8_t n, uint16_t tOn, uint16_t tOff, uint16_t freq);
void indicadorActualizar(Indicador &x);
void actualizarLedRojo();

void leerSensor(SensorLaser &s);
void actualizarPortal();
void actualizarSalida();
void objetoDetectado(bool l1, bool l2);
void rechazoDetectado();
Categoria clasificarPaquete(bool l1, bool l2);
void procesarPaquete(Categoria c);

void lcdMostrar(const char *l1, const char *l2, unsigned long duracion, Pantalla despues);
void lcdMostrarP(const char *l1P, const char *l2P, unsigned long duracion, Pantalla despues);
void mostrarBase();
void mostrarReporte(unsigned long duracion);
void mostrarEstado();
void actualizarLCD();
void textoContadores(char *dst, uint8_t n);
const char *nombreCategoria(Categoria c);
const char *nombreModo(Modo m);

void leerEntrada(Stream &s, Entrada &e);
void ejecutarComando(char *cmd, bool desbordado);
bool leerNumeroLote(const char *s, uint8_t &n);
void comandoInvalido();
void ponerEnMarcha();
void cmdIniciar();
void cmdPausa();
void cmdDetener();
void cmdReporte();
void cmdResetear();
void cmdModoLote(uint8_t n);
void cmdModoNormal();
void cmdModoMantenimiento();
void cmdEstado();
void revisarLote();
void finalizarLote();
void avisarSensoresBloqueados();
void enviarTexto(const char *t);
void enviarTextoF(const __FlashStringHelper *t);
void enviarReporte();
void monitorSerial();

// ---- Setup y loop ----
void setup() {
  Serial.begin(9600);
  bt.begin(9600);

  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_LED_AZUL, OUTPUT);
  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  controlarBanda(false);

  servoS1.attach(PIN_SERVO_S1);
  servoS2.attach(PIN_SERVO_S2);
  vaciarDesvios();

  Wire.begin();
  ahora = millis();
  bool lcdOk = lcd.iniciar();

  enviarTextoF(F("== SmartSort GT =="));
  if (lcdOk) {
    char t[32];
    snprintf_P(t, sizeof t, PSTR("LCD I2C en 0x%02X"), lcd.direccion());
    enviarTexto(t);
  } else {
    enviarTextoF(F("AVISO: no se encontro el LCD I2C (revise SDA, SCL, VCC y GND)"));
  }

  iniciarEEPROM();
  digitalWrite(PIN_LED_AZUL, HIGH);

  ahora = millis();
  if (falloEEPROM) lcdMostrarP(PSTR("ERROR EEPROM"), PSTR("Use: resetear"), 3000, PANT_BASE);
  else             lcdMostrarP(PSTR("SmartSort GT"), PSTR("Iniciando..."), 1500, PANT_BASE);

  enviarTextoF(F("Estadisticas guardadas en EEPROM:"));
  enviarReporte();
  enviarTextoF(F("Comandos: iniciar, pausa, detener, reporte, resetear, modo_lote:N, modo_normal, modo_mantenimiento, estado"));
}

void loop() {
  ahora = millis();

  leerEntrada(bt, entradaBT);
  leerEntrada(Serial, entradaUSB);

  for (uint8_t i = 0; i < NUM_SENSORES; i++) leerSensor(sensores[i]);
  actualizarPortal();
  actualizarSalida();
  actualizarServos();
  revisarLote();

  indicadorActualizar(ledVerde);
  indicadorActualizar(ledRojo);
  indicadorActualizar(buzzer);
  actualizarLedRojo();

  actualizarLCD();
  monitorSerial();
}

// ---- EEPROM ----
uint16_t eeLeer16(int dir) { uint16_t v; EEPROM.get(dir, v); return v; }
uint32_t eeLeer32(int dir) { uint32_t v; EEPROM.get(dir, v); return v; }

uint8_t eeChecksum() {
  uint8_t c = 0xA5;
  for (int i = EE_GRANDES; i < EE_CHECKSUM; i++) c ^= EEPROM.read(i);
  return c;
}

bool eeEscribir16(int dir, uint32_t valor) {
  if (valor > MAX_TURNO) return false;
  EEPROM.put(dir, (uint16_t)valor);
  return eeLeer16(dir) == valor;
}

bool eeEscribir32(int dir, uint32_t valor) {
  if (valor > MAX_TOTAL) return false;
  EEPROM.put(dir, valor);
  return eeLeer32(dir) == valor;
}

bool eeSellar() {
  uint8_t c = eeChecksum();
  EEPROM.update(EE_CHECKSUM, c);
  return EEPROM.read(EE_CHECKSUM) == c;
}

bool eeValida() {
  return eeLeer16(EE_FIRMA) == FIRMA_EEPROM &&
         EEPROM.read(EE_CHECKSUM) == eeChecksum() &&
         eeLeer16(EE_GRANDES) <= MAX_TURNO &&
         eeLeer16(EE_PEQUENOS) <= MAX_TURNO &&
         eeLeer16(EE_RECHAZOS) <= MAX_TURNO &&
         eeLeer32(EE_TOTAL) <= MAX_TOTAL;
}

void iniciarEEPROM() {
  if (eeLeer16(EE_FIRMA) != FIRMA_EEPROM) {
    bool ok = eeEscribir16(EE_GRANDES, 0) && eeEscribir16(EE_PEQUENOS, 0) &&
              eeEscribir16(EE_RECHAZOS, 0) && eeEscribir32(EE_TOTAL, 0);
    EEPROM.put(EE_FIRMA, FIRMA_EEPROM);
    ok = ok && eeSellar() && eeValida();
    falloEEPROM = !ok;
    enviarTextoF(F("EEPROM inicializada (primer uso)"));
  } else {
    falloEEPROM = !eeValida();
  }
  if (falloEEPROM) enviarTextoF(F("FALLO EEPROM: datos invalidos. Envie 'resetear'."));
}

// se llama al clasificar cada paquete: los datos quedan guardados al instante
bool guardarEEPROM(Categoria c) {
  int dir = (c == CAT_GRANDE) ? EE_GRANDES : (c == CAT_PEQUENO) ? EE_PEQUENOS : EE_RECHAZOS;
  uint32_t turno = eeLeer16(dir);
  uint32_t total = eeLeer32(EE_TOTAL);
  if (turno >= MAX_TURNO || total >= MAX_TOTAL) {
    falloEEPROM = true;
    return false;
  }
  bool ok = eeEscribir16(dir, turno + 1) && eeEscribir32(EE_TOTAL, total + 1) && eeSellar();
  if (!ok) falloEEPROM = true;
  return ok;
}

bool resetearTurnoEEPROM() {
  uint32_t total = eeLeer32(EE_TOTAL);
  if (total > MAX_TOTAL) total = 0;
  bool ok = eeEscribir16(EE_GRANDES, 0) && eeEscribir16(EE_PEQUENOS, 0) &&
            eeEscribir16(EE_RECHAZOS, 0) && eeEscribir32(EE_TOTAL, total);
  if (eeLeer16(EE_FIRMA) != FIRMA_EEPROM) EEPROM.put(EE_FIRMA, FIRMA_EEPROM);
  ok = ok && eeSellar() && eeValida();
  falloEEPROM = !ok;
  return ok;
}

// ---- Banda, servos, LEDs y buzzer ----
void controlarBanda(bool encender) {
  analogWrite(PIN_MOTOR, encender ? VELOCIDAD_BANDA : 0);
  bandaEncendida = encender;
}

void moverServo(Desviador &d, bool activar) {
  d.servo->write(activar ? d.angActivo : d.angNeutral);
  d.desviando = activar;
  if (activar) d.tActivo = ahora;
}

void programarDesvio(uint8_t idx) {
  if (colaCantidad >= TAM_COLA) {
    enviarTextoF(F("AVISO: demasiados paquetes seguidos"));
    return;
  }
  cola[colaCantidad].desviador = idx;
  cola[colaCantidad].tRegistro = ahora;
  colaCantidad++;
}

void vaciarDesvios() {
  colaCantidad = 0;
  moverServo(desviadores[D_S1], false);
  moverServo(desviadores[D_S2], false);
}

void compensarPausa() {
  unsigned long dt = ahora - tPausa;
  for (uint8_t i = 0; i < colaCantidad; i++) cola[i].tRegistro += dt;
  for (uint8_t k = 0; k < 2; k++) {
    if (desviadores[k].desviando) desviadores[k].tActivo += dt;
  }
  tCierreLote += dt;
}

void actualizarServos() {
  if (estado == EST_PAUSADO) return;
  uint8_t i = 0;
  while (i < colaCantidad) {
    Desviador &d = desviadores[cola[i].desviador];
    if (ahora - cola[i].tRegistro >= d.retardo) {
      moverServo(d, true);
      for (uint8_t j = i + 1; j < colaCantidad; j++) cola[j - 1] = cola[j];
      colaCantidad--;
    } else {
      i++;
    }
  }
  for (uint8_t k = 0; k < 2; k++) {
    Desviador &d = desviadores[k];
    if (d.desviando && ahora - d.tActivo >= T_DESVIO_MS) moverServo(d, false);
  }
}

void indicadorSalida(Indicador &x, bool on) {
  x.encendido = on;
  if (x.esBuzzer) {
#if BUZZER_PASIVO
    if (on) tone(x.pin, x.frecuencia); else noTone(x.pin);
#else
    digitalWrite(x.pin, on ? HIGH : LOW);
#endif
  } else {
    digitalWrite(x.pin, on ? HIGH : LOW);
  }
}

void indicadorPulsos(Indicador &x, uint8_t n, uint16_t tOn, uint16_t tOff, uint16_t freq) {
  x.pendientes = n;
  x.tOn = tOn;
  x.tOff = tOff;
  x.frecuencia = freq;
  x.tCambio = ahora;
  indicadorSalida(x, true);
}

void indicadorActualizar(Indicador &x) {
  if (x.pendientes == 0) return;
  if (x.encendido) {
    if (ahora - x.tCambio >= x.tOn) {
      indicadorSalida(x, false);
      x.tCambio = ahora;
      x.pendientes--;
    }
  } else if (ahora - x.tCambio >= x.tOff) {
    indicadorSalida(x, true);
    x.tCambio = ahora;
  }
}

void actualizarLedRojo() {
  if (ledRojo.pendientes == 0) digitalWrite(PIN_LED_ROJO, falloEEPROM ? HIGH : LOW);
}

// ---- Sensores y clasificacion ----
void leerSensor(SensorLaser &s) {
  s.lectura = analogRead(s.pin);
  bool crudo = s.cortado ? (s.lectura < s.umbral + HISTERESIS)
                         : (s.lectura < s.umbral - HISTERESIS);
  if (crudo == s.cortado) { s.candidato = crudo; return; }
  if (crudo != s.candidato) { s.candidato = crudo; s.tCandidato = ahora; return; }
  if (ahora - s.tCandidato >= T_FILTRO_MS) s.cortado = crudo;
}

void actualizarPortal() {
  bool c1 = sensores[0].cortado;
  bool c2 = sensores[1].cortado;
  bool alguno = c1 || c2;

  if (!portal.ocupado) {
    if (!alguno) return;
    portal.ocupado = true;
    portal.vioL1 = c1;
    portal.vioL2 = c2;
    portal.tInicio = portal.tUltimo = ahora;
    return;
  }
  if (c1) portal.vioL1 = true;
  if (c2) portal.vioL2 = true;
  if (alguno) { portal.tUltimo = ahora; return; }
  if (ahora - portal.tUltimo < T_SALIDA_MS) return;

  portal.ocupado = false;
  if (portal.tUltimo - portal.tInicio < T_MIN_PAQUETE_MS) return;
  objetoDetectado(portal.vioL1, portal.vioL2);
}

void objetoDetectado(bool l1, bool l2) {
  if (estado == EST_LISTO) return;
  if (modo == MODO_MANTENIMIENTO) {
    enviarTextoF(F("Mantenimiento: objeto detectado (no se clasifica)"));
    return;
  }
  if (modo == MODO_LOTE && loteCerrando) {
    enviarTextoF(F("Lote completo: objeto extra ignorado"));
    return;
  }
  Categoria c = clasificarPaquete(l1, l2);
  if (c == CAT_RECHAZO) {
    enviarTextoF(F("Objeto fuera de rango: sigue recto a rechazo"));
    return;
  }
  procesarPaquete(c);
}

// todo lo que llega al final de la banda (L3) no fue desviado: es RECHAZO
void actualizarSalida() {
  bool c3 = sensores[2].cortado;
  if (!salida.ocupado) {
    if (!c3) return;
    salida.ocupado = true;
    salida.tInicio = salida.tUltimo = ahora;
    return;
  }
  if (c3) { salida.tUltimo = ahora; return; }
  if (ahora - salida.tUltimo < T_SALIDA_MS) return;
  salida.ocupado = false;
  if (salida.tUltimo - salida.tInicio < T_MIN_PAQUETE_MS) return;
  rechazoDetectado();
}

void rechazoDetectado() {
  if (estado == EST_LISTO) return;
  if (modo == MODO_MANTENIMIENTO) {
    enviarTextoF(F("Mantenimiento: objeto en L3 (no se cuenta)"));
    return;
  }
  if (modo == MODO_LOTE && loteCerrando) {
    enviarTextoF(F("Lote completo: objeto extra ignorado"));
    return;
  }
  procesarPaquete(CAT_RECHAZO);
}

Categoria clasificarPaquete(bool l1, bool l2) {
  if (l1 && l2) return INVERTIR_GRANDE_PEQUENO ? CAT_PEQUENO : CAT_GRANDE;
  if (l1)       return INVERTIR_GRANDE_PEQUENO ? CAT_GRANDE : CAT_PEQUENO;
  return CAT_RECHAZO;
}

void procesarPaquete(Categoria c) {
  bool ok = guardarEEPROM(c);
  ultimoPaquete = c;

  if (c == CAT_GRANDE)       programarDesvio(D_S1);
  else if (c == CAT_PEQUENO) programarDesvio(D_S2);

  if (c == CAT_RECHAZO) {
    indicadorPulsos(ledRojo, 3, 100, 100, 0);
    indicadorPulsos(buzzer, 3, 100, 100, TONO_RECHAZO);
  } else {
    indicadorPulsos(ledVerde, 1, 300, 0, 0);
    indicadorPulsos(buzzer, 1, 150, 0, c == CAT_GRANDE ? TONO_GRANDE : TONO_PEQUENO);
  }

  if (modo == MODO_LOTE) {
    loteProcesados++;
    if (loteProcesados >= loteObjetivo) {
      loteCerrando = true;
      tCierreLote = ahora;
    }
  }

  char l1[17], l2[24];
  snprintf_P(l1, sizeof l1, PSTR("Paq: %s"), nombreCategoria(c));
  if (!ok)                    strcpy_P(l2, PSTR("ERROR EEPROM"));
  else if (modo == MODO_LOTE) snprintf_P(l2, sizeof l2, PSTR("Procesados:%u/%u"), loteProcesados, loteObjetivo);
  else                        textoContadores(l2, sizeof l2);
  lcdMostrar(l1, l2, T_MENSAJE_MS, PANT_BASE);

  char t[24];
  snprintf_P(t, sizeof t, PSTR("Paq: %s"), nombreCategoria(c));
  enviarTexto(t);
  if (!ok) enviarTextoF(F("FALLO EEPROM al guardar"));
}

// ---- Pantallas del LCD ----
void lcdMostrar(const char *l1, const char *l2, unsigned long duracion, Pantalla despues) {
  lcd.linea(0, l1);
  lcd.linea(1, l2);
  tMensaje = ahora;
  duracionMensaje = duracion;
  pantallaDespues = despues;
}

void lcdMostrarP(const char *l1P, const char *l2P, unsigned long duracion, Pantalla despues) {
  char l1[17], l2[17];
  strncpy_P(l1, l1P, 16); l1[16] = '\0';
  strncpy_P(l2, l2P, 16); l2[16] = '\0';
  lcdMostrar(l1, l2, duracion, despues);
}

void actualizarLCD() {
  if (duracionMensaje == 0 || ahora - tMensaje < duracionMensaje) return;
  duracionMensaje = 0;
  if (pantallaDespues == PANT_REPORTE) mostrarReporte(0);
  else mostrarBase();
}

void mostrarBase() {
  if (estado == EST_LISTO)   { lcdMostrarP(PSTR("SmartSort GT"), PSTR("Esperando cmd.."), 0, PANT_BASE); return; }
  if (estado == EST_PAUSADO) { lcdMostrarP(PSTR("PAUSADO"), PSTR("Banda: OFF"), 0, PANT_BASE); return; }
  if (modo == MODO_LOTE) {
    char l1[17], l2[17];
    if (loteObjetivo < 10) snprintf_P(l1, sizeof l1, PSTR("Lote: %u paquetes"), loteObjetivo);
    else                   snprintf_P(l1, sizeof l1, PSTR("Lote:%u paquetes"), loteObjetivo);
    snprintf_P(l2, sizeof l2, PSTR("Procesados:%u/%u"), loteProcesados, loteObjetivo);
    lcdMostrar(l1, l2, 0, PANT_BASE);
  } else if (modo == MODO_MANTENIMIENTO) {
    lcdMostrarP(PSTR("Modo: MANTENIM."), PSTR("Banda: ON"), 0, PANT_BASE);
  } else {
    lcdMostrarP(PSTR("Modo: NORMAL"), PSTR("Banda: ON"), 0, PANT_BASE);
  }
}

void textoContadores(char *dst, uint8_t n) {
  unsigned int g = eeLeer16(EE_GRANDES), p = eeLeer16(EE_PEQUENOS), r = eeLeer16(EE_RECHAZOS);
  snprintf_P(dst, n, PSTR("G:%u P:%u R:%u"), g, p, r);
  if (strlen(dst) > 16) snprintf_P(dst, n, PSTR("G:%uP:%uR:%u"), g, p, r);
}

void mostrarReporte(unsigned long duracion) {
  char l1[24], l2[17];
  textoContadores(l1, sizeof l1);
  snprintf_P(l2, sizeof l2, PSTR("Total:%lu"), (unsigned long)eeLeer32(EE_TOTAL));
  lcdMostrar(l1, l2, duracion, PANT_BASE);
}

void mostrarEstado() {
  char l1[17], l2[17];
  snprintf_P(l1, sizeof l1, PSTR("%-9s B:%s"), nombreModo(modo), bandaEncendida ? "ON" : "OFF");
  snprintf_P(l2, sizeof l2, PSTR("Ultimo: %s"), nombreCategoria(ultimoPaquete));
  lcdMostrar(l1, l2, T_REPORTE_MS, PANT_BASE);
}

const char *nombreCategoria(Categoria c) {
  switch (c) {
    case CAT_GRANDE:  return "GRANDE";
    case CAT_PEQUENO: return "PEQUENO";
    case CAT_RECHAZO: return "RECHAZO";
    default:          return "NINGUNO";
  }
}

const char *nombreModo(Modo m) {
  switch (m) {
    case MODO_NORMAL:        return "NORMAL";
    case MODO_LOTE:          return "LOTE";
    case MODO_MANTENIMIENTO: return "MANTENIM.";
    default:                 return "INACTIVO";
  }
}

// ---- Comandos (Bluetooth y USB) ----
void leerEntrada(Stream &s, Entrada &e) {
  while (s.available() > 0) {
    char c = (char)s.read();
    e.tUltimo = ahora;
    if (c == '\r' || c == '\n') {
      if (e.largo > 0 || e.desbordado) {
        e.texto[e.largo] = '\0';
        ejecutarComando(e.texto, e.desbordado);
      }
      e.largo = 0;
      e.desbordado = false;
    } else if (e.largo < sizeof(e.texto) - 1) {
      e.texto[e.largo++] = c;
    } else {
      e.desbordado = true;
    }
  }
  if ((e.largo > 0 || e.desbordado) && ahora - e.tUltimo >= T_FIN_COMANDO_MS) {
    e.texto[e.largo] = '\0';
    ejecutarComando(e.texto, e.desbordado);
    e.largo = 0;
    e.desbordado = false;
  }
}

void ejecutarComando(char *cmd, bool desbordado) {
  while (*cmd == ' ' || *cmd == '\t') cmd++;
  char *fin = cmd + strlen(cmd);
  while (fin > cmd && (fin[-1] == ' ' || fin[-1] == '\t')) *--fin = '\0';
  for (char *q = cmd; *q; q++) *q = tolower(*q);
  if (*cmd == '\0' && !desbordado) return;

  Serial.print(F("> "));
  Serial.println(cmd);

  uint8_t n;
  if (desbordado)                                   comandoInvalido();
  else if (!strcmp(cmd, "iniciar"))                 cmdIniciar();
  else if (!strcmp(cmd, "pausa") || !strcmp(cmd, "pausar")) cmdPausa();
  else if (!strcmp(cmd, "detener"))                 cmdDetener();
  else if (!strcmp(cmd, "reporte"))                 cmdReporte();
  else if (!strcmp(cmd, "resetear"))                cmdResetear();
  else if (!strcmp(cmd, "modo_normal"))             cmdModoNormal();
  else if (!strcmp(cmd, "modo_mantenimiento"))      cmdModoMantenimiento();
  else if (!strcmp(cmd, "estado"))                  cmdEstado();
  else if (!strncmp(cmd, "modo_lote:", 10)) {
    if (leerNumeroLote(cmd + 10, n)) cmdModoLote(n);
    else comandoInvalido();
  }
  else comandoInvalido();
}

bool leerNumeroLote(const char *s, uint8_t &n) {
  while (*s == ' ') s++;
  uint16_t v = 0;
  uint8_t digitos = 0;
  while (*s >= '0' && *s <= '9') {
    if (++digitos > 3) return false;
    v = v * 10 + (*s - '0');
    s++;
  }
  while (*s == ' ') s++;
  if (digitos == 0 || *s != '\0' || v < 1 || v > LOTE_MAX) return false;
  n = (uint8_t)v;
  return true;
}

void comandoInvalido() {
  lcdMostrarP(PSTR("ERROR:"), PSTR("Cmd invalido"), T_MENSAJE_MS, PANT_BASE);
  enviarTextoF(F("ERROR: Cmd invalido"));
  enviarTextoF(F("Use: iniciar, pausa, detener, reporte, resetear, modo_lote:N (1-99), modo_normal, modo_mantenimiento, estado"));
}

void ponerEnMarcha() {
  if (estado == EST_PAUSADO) compensarPausa();
  estado = EST_ACTIVO;
  controlarBanda(true);
  if (modo == MODO_MANTENIMIENTO) vaciarDesvios();
  avisarSensoresBloqueados();
}

void cmdIniciar() {
  if (estado == EST_LISTO) {
    modo = MODO_NORMAL;
    loteCerrando = false;
  }
  ponerEnMarcha();
  mostrarBase();
  enviarTextoF(F("OK: banda en marcha"));
}

void cmdPausa() {
  if (estado == EST_ACTIVO) {
    estado = EST_PAUSADO;
    tPausa = ahora;
    controlarBanda(false);
    mostrarBase();
    enviarTextoF(F("OK: pausado (sensores siguen activos)"));
  } else if (estado == EST_PAUSADO) {
    mostrarBase();
    enviarTextoF(F("Ya estaba en pausa"));
  } else {
    lcdMostrarP(PSTR("PAUSADO"), PSTR("Banda: OFF"), T_MENSAJE_MS, PANT_BASE);
    enviarTextoF(F("La banda ya esta detenida"));
  }
}

void cmdDetener() {
  controlarBanda(false);
  vaciarDesvios();
  estado = EST_LISTO;
  modo = MODO_NINGUNO;
  loteCerrando = false;
  loteObjetivo = 0;
  loteProcesados = 0;
  if (!falloEEPROM) falloEEPROM = !(eeSellar() && eeValida());
  mostrarReporte(0);
  enviarTextoF(F("OK: operacion finalizada. Reporte final:"));
  enviarReporte();
}

void cmdReporte() {
  mostrarReporte(T_REPORTE_MS);
  enviarReporte();
}

void cmdResetear() {
  if (resetearTurnoEEPROM()) {
    lcdMostrarP(PSTR("Reset OK"), PSTR("Nvo turno listo"), T_MENSAJE_MS, PANT_BASE);
    enviarTextoF(F("OK: turno en 0 (el total historico se conserva)"));
    enviarReporte();
  } else {
    lcdMostrarP(PSTR("ERROR EEPROM"), PSTR("Reset fallido"), T_MENSAJE_MS, PANT_BASE);
    enviarTextoF(F("FALLO EEPROM: no se pudo resetear"));
  }
}

void cmdModoLote(uint8_t n) {
  modo = MODO_LOTE;
  loteObjetivo = n;
  loteProcesados = 0;
  loteCerrando = false;
  ponerEnMarcha();
  mostrarBase();
  char t[32];
  snprintf_P(t, sizeof t, PSTR("OK: lote de %u paquetes"), n);
  enviarTexto(t);
}

void cmdModoNormal() {
  modo = MODO_NORMAL;
  loteCerrando = false;
  ponerEnMarcha();
  mostrarBase();
  enviarTextoF(F("OK: modo NORMAL"));
}

void cmdModoMantenimiento() {
  modo = MODO_MANTENIMIENTO;
  loteCerrando = false;
  ponerEnMarcha();
  mostrarBase();
  enviarTextoF(F("OK: modo MANTENIMIENTO (servos no clasifican)"));
}

void cmdEstado() {
  mostrarEstado();
  char t[64];
  snprintf_P(t, sizeof t, PSTR("Modo:%s Banda:%s Ultimo:%s"), nombreModo(modo),
           bandaEncendida ? "ON" : "OFF", nombreCategoria(ultimoPaquete));
  enviarTexto(t);
  if (modo == MODO_LOTE) {
    snprintf_P(t, sizeof t, PSTR("Lote: %u/%u"), loteProcesados, loteObjetivo);
    enviarTexto(t);
  }
  snprintf_P(t, sizeof t, PSTR("LDR: L1=%d L2=%d L3=%d"), sensores[0].lectura, sensores[1].lectura,
             sensores[2].lectura);
  enviarTexto(t);
}

void revisarLote() {
  if (estado != EST_ACTIVO || modo != MODO_LOTE || !loteCerrando) return;
  if (colaCantidad > 0 || desviadores[D_S1].desviando || desviadores[D_S2].desviando) return;
  if (ahora - tCierreLote < T_VACIADO_MS) return;
  finalizarLote();
}

void finalizarLote() {
  controlarBanda(false);
  estado = EST_LISTO;
  modo = MODO_NINGUNO;
  loteCerrando = false;
  indicadorPulsos(buzzer, 1, 1500, 0, TONO_LOTE);
  char l1[17], l2[17];
  strcpy_P(l1, PSTR("Lote completo"));
  snprintf_P(l2, sizeof l2, PSTR("%u paquetes"), loteObjetivo);
  lcdMostrar(l1, l2, T_MENSAJE_MS, PANT_REPORTE);
  enviarTextoF(F("Lote completo. Reporte:"));
  enviarReporte();
}

void avisarSensoresBloqueados() {
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    if (sensores[i].cortado) {
      char t[48];
      snprintf_P(t, sizeof t, PSTR("AVISO: L%u cortado (lectura %d)"), i + 1, sensores[i].lectura);
      enviarTexto(t);
    }
  }
}

void enviarTexto(const char *t) {
  Serial.println(t);
  bt.println(t);
}

void enviarTextoF(const __FlashStringHelper *t) {
  Serial.println(t);
  bt.println(t);
}

void enviarReporte() {
  char t[48];
  snprintf_P(t, sizeof t, PSTR("G:%u P:%u R:%u / Total:%lu"),
           (unsigned int)eeLeer16(EE_GRANDES), (unsigned int)eeLeer16(EE_PEQUENOS),
           (unsigned int)eeLeer16(EE_RECHAZOS), (unsigned long)eeLeer32(EE_TOTAL));
  enviarTexto(t);
}

void monitorSerial() {
  if (estado != EST_ACTIVO || modo != MODO_MANTENIMIENTO) return;
  if (ahora - tMonitor < 300) return;
  tMonitor = ahora;
  Serial.print(F("L1="));
  Serial.print(sensores[0].lectura);
  Serial.print(sensores[0].cortado ? F(" [CORTADO]") : F(" [libre]"));
  Serial.print(F("  L2="));
  Serial.print(sensores[1].lectura);
  Serial.print(sensores[1].cortado ? F(" [CORTADO]") : F(" [libre]"));
  Serial.print(F("  L3="));
  Serial.print(sensores[2].lectura);
  Serial.print(sensores[2].cortado ? F(" [CORTADO]") : F(" [libre]"));
  Serial.println();
}
