/*
 * SmartSort GT - Prueba de modulos
 * Monitor Serie a 9600 con "Nueva linea": 1=LDR 2=Servos 3=Motor 4=LEDs/Buzzer 5=LCD 6=Bluetooth
 */
#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>

const uint8_t PIN_MOTOR = 5;
const uint8_t PIN_LED_AZUL = 6, PIN_LED_VERDE = 7, PIN_LED_ROJO = 8;
const uint8_t PIN_SERVO_S1 = 9, PIN_SERVO_S2 = 10, PIN_BUZZER = 12;
const uint8_t PIN_LDR_L1 = A0, PIN_LDR_L2 = A1, PIN_LDR_L3 = A2;
const uint8_t S1_NEUTRAL = 0, S1_ACTIVO = 60, S2_NEUTRAL = 0, S2_ACTIVO = 60;

#if defined(__AVR_ATmega2560__)
#define bt Serial1
#else
SoftwareSerial bt(2, 3);
#endif
Servo servoS1, servoS2;
uint8_t dirLcd = 0;

void lcdByte(uint8_t b) { Wire.beginTransmission(dirLcd); Wire.write((uint8_t)(b | 0x08)); Wire.endTransmission(); }
void lcdNibble(uint8_t alto, uint8_t rs) {
  uint8_t b = (alto & 0xF0) | rs;
  lcdByte(b); lcdByte(b | 0x04); delayMicroseconds(1); lcdByte(b); delayMicroseconds(50);
}
void lcdEnviar(uint8_t v, uint8_t rs) { lcdNibble(v & 0xF0, rs); lcdNibble((uint8_t)(v << 4), rs); }
void lcdIniciar() {
  delay(50); lcdByte(0); delay(5);
  lcdNibble(0x30, 0); delayMicroseconds(4500);
  lcdNibble(0x30, 0); delayMicroseconds(4500);
  lcdNibble(0x30, 0); delayMicroseconds(150);
  lcdNibble(0x20, 0);
  lcdEnviar(0x28, 0); lcdEnviar(0x0C, 0); lcdEnviar(0x01, 0); delay(2); lcdEnviar(0x06, 0);
}
void lcdTexto(uint8_t fila, const char *t) {
  lcdEnviar(0x80 | (fila ? 0x40 : 0x00), 0);
  for (uint8_t i = 0; i < 16; i++) lcdEnviar(*t ? *t++ : ' ', 1);
}

void esperarTecla() { while (Serial.available()) Serial.read(); }
void menu();

void setup() {
  Serial.begin(9600);
  bt.begin(9600);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_LED_AZUL, OUTPUT);
  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  servoS1.attach(PIN_SERVO_S1);
  servoS2.attach(PIN_SERVO_S2);
  servoS1.write(S1_NEUTRAL);
  servoS2.write(S2_NEUTRAL);
  Wire.begin();
  menu();
}

void menu() {
  Serial.println();
  Serial.println(F("=== PRUEBA DE MODULOS SmartSort GT ==="));
  Serial.println(F("1=LDR  2=Servos  3=Motor  4=LEDs+Buzzer  5=LCD/I2C  6=Bluetooth"));
}

void pruebaLDR() {
  Serial.println(F("Lecturas LDR (cualquier tecla para salir). Laser sobre LDR = valor ALTO."));
  Serial.println(F("Anota el valor LIBRE y el valor CORTADO (con una caja). Umbral = promedio."));
  esperarTecla();
  while (!Serial.available()) {
    Serial.print(F("L1(A0)=")); Serial.print(analogRead(PIN_LDR_L1));
    Serial.print(F("   L2(A1)=")); Serial.print(analogRead(PIN_LDR_L2));
    Serial.print(F("   L3(A2)=")); Serial.println(analogRead(PIN_LDR_L3));
    delay(250);
  }
  esperarTecla();
}

void pruebaServos() {
  Serial.println(F("S1 (D9): activo..."));
  servoS1.write(S1_ACTIVO); delay(1500);
  servoS1.write(S1_NEUTRAL); delay(1000);
  Serial.println(F("S2 (D10): activo..."));
  servoS2.write(S2_ACTIVO); delay(1500);
  servoS2.write(S2_NEUTRAL); delay(1000);
  Serial.println(F("Listo. Si el brazo no cruza bien la banda, ajusta S1/S2_ACTIVO."));
}

void pruebaMotor() {
  Serial.println(F("Motor ON 3 s (D5 -> TIP120)"));
  analogWrite(PIN_MOTOR, 255); delay(3000);
  analogWrite(PIN_MOTOR, 0);
  Serial.println(F("Motor OFF"));
}

void pruebaLeds() {
  const uint8_t pines[] = {PIN_LED_AZUL, PIN_LED_VERDE, PIN_LED_ROJO, PIN_BUZZER};
  const char *nombres[] = {"Azul (D6)", "Verde (D7)", "Rojo (D8)", "Buzzer (D12)"};
  for (uint8_t i = 0; i < 4; i++) {
    Serial.println(nombres[i]);
    digitalWrite(pines[i], HIGH); delay(700);
    digitalWrite(pines[i], LOW);  delay(300);
  }
  Serial.println(F("Si el buzzer solo hace 'clic', es PASIVO: usa BUZZER_PASIVO 1."));
}

void pruebaLCD() {
  dirLcd = 0;
  Serial.println(F("Buscando dispositivos I2C..."));
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  Encontrado en 0x")); Serial.println(a, HEX);
      if (dirLcd == 0) dirLcd = a;
    }
  }
  if (dirLcd == 0) { Serial.println(F("Nada en I2C: revisa SDA (Mega D20 / UNO A4), SCL (Mega D21 / UNO A5), VCC y GND.")); return; }
  lcdIniciar();
  lcdTexto(0, "SmartSort GT");
  lcdTexto(1, "Prueba LCD OK");
  Serial.println(F("Texto enviado. Si no se ve, gira el potenciometro azul del modulo."));
}

void pruebaBluetooth() {
  Serial.println(F("Puente Bluetooth activo ('#' para salir)."));
  Serial.println(F("Conecta la app Serial Bluetooth Terminal al HC-06 y escribe algo."));
  bt.println(F("Hola desde SmartSort GT"));
  esperarTecla();
  while (true) {
    if (bt.available()) Serial.write(bt.read());
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '#') break;
      bt.write(c);
    }
  }
}

void loop() {
  if (!Serial.available()) return;
  char op = Serial.read();
  switch (op) {
    case '1': pruebaLDR(); break;
    case '2': pruebaServos(); break;
    case '3': pruebaMotor(); break;
    case '4': pruebaLeds(); break;
    case '5': pruebaLCD(); break;
    case '6': pruebaBluetooth(); break;
    default: return;
  }
  menu();
}
