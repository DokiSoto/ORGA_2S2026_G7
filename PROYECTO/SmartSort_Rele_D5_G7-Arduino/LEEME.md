# SmartSort: rele con senal D5

Version adaptada del codigo adjunto. Sustituye el control L298N por una unica salida digital D5: HIGH enciende y LOW apaga el circuito externo ya probado. No conectar el motor ni una bobina directamente a D5. Tierra del circuito a GND. D8/D9 ya no son salidas del motor. Alimentacion del motor: 5 V como confirmo el usuario.

## Abrir y cargar
Descomprimir. Abrir SmartSort_Rele_D5/SmartSort_Rele_D5.ino. Configuracion.h debe estar en la misma carpeta. Seleccionar Mega 2560 y puerto. Instalar LiquidCrystal I2C de Frank de Brabander y Servo si faltan. Cargar con alimentacion del motor desconectada. USB a 9600 baud (se conserva el codigo adjunto); Bluetooth Serial1 a BT_BAUD=9600. Ambos requieren LF o CRLF al final del comando.

## Configuracion pendiente
POTENCIA_VERIFICADA=true: el usuario ya probo la senal HIGH/LOW del motor. SERVOS_VERIFICADOS=false y CALIBRACION_LISTA=false se conservan hasta probar alimentacion externa/angulos de servos y calibrar sensores/tiempos. No ponerlos en true solo para eliminar mensajes. Configuracion.h contiene valores de ejemplo, no medidas de la maqueta. La pantalla debe responder en LCD_ADDR (ejemplo 0x27).

Con servos verificados, modo_mantenimiento permite marcha continua sin clasificar y sin exigir CALIBRACION_LISTA. Requiere LCD operativo y sin fallo EEPROM. Para clasificar tambien se requiere CALIBRACION_LISTA=true y los tres haces despejados.

## Comandos (terminal Bluetooth o USB)
- estado, reporte, sensores, monitor:on, monitor:off: diagnostico.
- iniciar: inicia o reanuda. En normal mantiene D5 HIGH incluso entre paquetes.
- pausa o pausar: D5 LOW; conserva paquete y tiempo de transporte. iniciar reanuda.
- detener: D5 LOW, servos neutros, cancela paquete en curso sin contarlo; retirar todas las cajas y enviar confirmar_vacia antes de reiniciar.
- modo_normal: inicia clasificacion normal tras comprobar condiciones.
- modo_lote:3: inicia lote de tres paquetes, incluidos rechazos, y para al completar.
- modo_mantenimiento: motor continuo, servos neutros, sin conteos. Salir con detener, retirar cajas, confirmar_vacia.
- resetear: con equipo detenido, reinicia conteos de turno y conserva historico.
No se usan 1/0 ni esperar Enter entre ciclos. El motor arranca detenido al reiniciar; necesita comando. No se apaga automaticamente si se pierde Bluetooth: esta version no tiene deteccion de desconexion. Mantener un corte fisico de alimentacion accesible.

## Pines restantes
Bluetooth TXD -> Mega19 RX1. Mega18 TX1 -> 1kohm -> nodo RXD Bluetooth; nodo -> 2kohm -> GND. VCC5V solo para el breakout mostrado con alimentacion3.6-6V; GND comun.
LCD SDA20, SCL21, VCC5V, GND. Servos senales6/7, fuente externa adecuada, GND comun. LDR A0 bajo, A1 alto, A2 presencia: cada uno 5V->LDR->nodo analogico->10kohm->GND. Cada detector necesita iluminacion; dos lasers no forman tres haces independientes. LED azul22, verde23, rojo24 cada uno con resistencia330ohm. Buzzer25 por circuito adecuado al buzzer.

## Limites y validacion
Un paquete a la vez. No insertar el siguiente hasta aviso de listo. Se conservan LCD, servos, sensores, avisos, EEPROM y comandos del archivo adjunto. LOGICA_LITERAL_DOCUMENTO=true conserva la tabla del documento; confirmar con docente si corresponde invertir grande/pequeno. Los tiempos estiman posicion: calibrarlos con la banda real y comprobar inercia al pausar. La escritura EEPROM sigue siendo sincrona; durante ella se mantiene HIGH y la recepcion de comandos puede demorarse decenas de milisegundos. Lote completo y fallos si paran el motor. No hay frenado activo ni ajuste PWM.

Validacion mediante simulacion C++ en host; no equivale a compilacion AVR ni a prueba del montaje real. Compilar en Arduino IDE y hacer pruebas fisicas antes de usar paquetes.
