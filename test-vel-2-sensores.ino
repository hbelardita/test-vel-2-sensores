#include <Arduino.h>

// Modo de telemetria por puerto serie:
// 1 = Habilitada para calibracion y diagnostico en banco.
// 0 = Modo competicion (desactiva el UART, ahorra RAM y optimiza el tiempo de ciclo del bucle de control).
#define HABILITAR_TELEMETRIA 0

// Definición de pines para Motor Izquierdo
const uint8_t PIN_ENA = 5;
const uint8_t PIN_IN1 = 4;
const uint8_t PIN_IN2 = 8;

// Definición de pines para Motor Derecho
const uint8_t PIN_ENB = 6;
const uint8_t PIN_IN3 = 10;
const uint8_t PIN_IN4 = 9;

// Definición de pines para Sensores Ópticos
const uint8_t PIN_SENSOR_IZQ = 2;
const uint8_t PIN_SENSOR_DER = 3;

// Pin del LED integrado para diagnóstico visual en placa
const uint8_t PIN_LED_TEST = 13;

// Pin del pulsador de arranque en chasis
const uint8_t PIN_PULSADOR = 7;
const uint8_t PULSADOR_PRESIONADO = LOW;

// Lógica de detección óptica (LOW sobre blanco para comparador estándar LM393)
const uint8_t DETECTA_BLANCO = LOW;

// Parámetros de velocidad PWM para alta velocidad en recta y curva rápida
const int VELOCIDAD_BASE        = 180; // Aumento de velocidad de crucero en recta
const int VELOCIDAD_MAX         = 255; // 100% ciclo de trabajo en motor exterior
const int VELOCIDAD_REVERSA_MAX = 170; // Contra-rotación de rescate en línea perdida

// Ganancias del controlador
const float KP = 135.0f;
const float KD = 50.0f;

// Intervalo de muestreo del bucle de control en milisegundos
const unsigned long INTERVALO_MS = 5;

// Ventana temporal para distinguir roce transitorio en recta de curva sostenida (en ms)
const unsigned long TIEMPO_FILTRO_CURVA_MS = 25;

// Modelado de estados de seguimiento de línea
enum EstadoSeguimiento {
  ESTADO_CENTRO,
  ESTADO_CORRECCION_IZQ,
  ESTADO_CORRECCION_DER,
  ESTADO_LINEA_PERDIDA,
  ESTADO_INTERSECCION
};

struct LecturaSensores {
  bool izqDetectaBlanco;
  bool derDetectaBlanco;
};

// Modelado del modo operativo del robot
enum ModoRobot {
  MODO_STANDBY,
  MODO_CARRERA
};

// Variables de estado interno
ModoRobot modoActual = MODO_STANDBY;
float ultimoError = 0.0f;
float ultimoGiroRecuperacion = 0.0f;
unsigned long tiempoAnterior = 0;
unsigned long tiempoInicioDeteccion = 0;

#if HABILITAR_TELEMETRIA
unsigned long tiempoTelemetria = 0;
#endif

LecturaSensores leerSensores() {
  LecturaSensores lectura;
  lectura.izqDetectaBlanco = (digitalRead(PIN_SENSOR_IZQ) == DETECTA_BLANCO);
  lectura.derDetectaBlanco = (digitalRead(PIN_SENSOR_DER) == DETECTA_BLANCO);
  return lectura;
}

EstadoSeguimiento clasificarEstado(LecturaSensores lectura) {
  if (!lectura.izqDetectaBlanco && !lectura.derDetectaBlanco) {
    if (ultimoError == 0.0f) {
      return ESTADO_CENTRO;
    }
    return ESTADO_LINEA_PERDIDA;
  }
  if (lectura.izqDetectaBlanco && !lectura.derDetectaBlanco) {
    return ESTADO_CORRECCION_IZQ;
  }
  if (!lectura.izqDetectaBlanco && lectura.derDetectaBlanco) {
    return ESTADO_CORRECCION_DER;
  }
  return ESTADO_INTERSECCION;
}

float calcularError(EstadoSeguimiento estado, unsigned long tiempoActual) {
  switch (estado) {
    case ESTADO_CENTRO:
      ultimoGiroRecuperacion = 0.0f;
      tiempoInicioDeteccion = 0;
      return 0.0f;

    case ESTADO_CORRECCION_IZQ:
      if (tiempoInicioDeteccion == 0) {
        tiempoInicioDeteccion = tiempoActual;
      }
      ultimoGiroRecuperacion = -2.0f;
      // Roce leve en recta: corrección sutil para mantener avance recto
      if (tiempoActual - tiempoInicioDeteccion < TIEMPO_FILTRO_CURVA_MS) {
        return -0.45f;
      }
      // Curva rápida: sostiene rueda interna en avance positivo (~93 PWM) y exterior al 100% (255 PWM)
      return -0.68f;

    case ESTADO_CORRECCION_DER:
      if (tiempoInicioDeteccion == 0) {
        tiempoInicioDeteccion = tiempoActual;
      }
      ultimoGiroRecuperacion = 2.0f;
      if (tiempoActual - tiempoInicioDeteccion < TIEMPO_FILTRO_CURVA_MS) {
        return 0.45f;
      }
      return 0.68f;

    // Rescate agresivo con contra-rotación inmediata si pierde la línea por inercia
    case ESTADO_LINEA_PERDIDA:
      tiempoInicioDeteccion = 0;
      return ultimoGiroRecuperacion;

    case ESTADO_INTERSECCION:
      tiempoInicioDeteccion = 0;
      return 0.0f;
  }
  return 0.0f;
}

void fijarMotores(int velocidadIzq, int velocidadDer) {
  velocidadIzq = constrain(velocidadIzq, -VELOCIDAD_REVERSA_MAX, VELOCIDAD_MAX);
  velocidadDer = constrain(velocidadDer, -VELOCIDAD_REVERSA_MAX, VELOCIDAD_MAX);

  // Control bidireccional Motor Izquierdo
  if (velocidadIzq >= 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, velocidadIzq);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, -velocidadIzq);
  }

  // Control bidireccional Motor Derecho
  if (velocidadDer >= 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, velocidadDer);
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    analogWrite(PIN_ENB, -velocidadDer);
  }
}

void detenerMotores() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
}

void setup() {
#if HABILITAR_TELEMETRIA
  Serial.begin(115200);
#endif

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  pinMode(PIN_LED_TEST, OUTPUT);
  digitalWrite(PIN_LED_TEST, LOW);

  pinMode(PIN_PULSADOR, INPUT_PULLUP);

  pinMode(PIN_SENSOR_IZQ, INPUT_PULLUP);
  pinMode(PIN_SENSOR_DER, INPUT_PULLUP);

  detenerMotores();
}

void loop() {
  if (modoActual == MODO_STANDBY) {
    detenerMotores();
    digitalWrite(PIN_LED_TEST, LOW);

    if (digitalRead(PIN_PULSADOR) == PULSADOR_PRESIONADO) {
      delay(50);
      if (digitalRead(PIN_PULSADOR) == PULSADOR_PRESIONADO) {
        while (digitalRead(PIN_PULSADOR) == PULSADOR_PRESIONADO) {
          delay(10);
        }
        delay(20);
        modoActual = MODO_CARRERA;
        digitalWrite(PIN_LED_TEST, HIGH);
        tiempoAnterior = millis();
        ultimoError = 0.0f;
      }
    }
    return;
  }

  unsigned long tiempoActual = millis();
  if (tiempoActual - tiempoAnterior < INTERVALO_MS) {
    return;
  }
  tiempoAnterior = tiempoActual;

  LecturaSensores lectura = leerSensores();

  EstadoSeguimiento estado = clasificarEstado(lectura);
  float error = calcularError(estado, tiempoActual);

  float derivada = error - ultimoError;
  ultimoError = error;

  float correccion = (KP * error) + (KD * derivada);

  int velocidadIzq = VELOCIDAD_BASE + (int)correccion;
  int velocidadDer = VELOCIDAD_BASE - (int)correccion;

  fijarMotores(velocidadIzq, velocidadDer);

#if HABILITAR_TELEMETRIA
  if (tiempoActual - tiempoTelemetria >= 100) {
    tiempoTelemetria = tiempoActual;
    Serial.print(F("Izq: "));
    Serial.print(digitalRead(PIN_SENSOR_IZQ));
    Serial.print(F(" | Der: "));
    Serial.print(digitalRead(PIN_SENSOR_DER));
    Serial.print(F(" | Estado: "));
    Serial.print(estado);
    Serial.print(F(" | Error: "));
    Serial.print(error);
    Serial.print(F(" | PWM_I: "));
    Serial.print(velocidadIzq);
    Serial.print(F(" | PWM_D: "));
    Serial.println(velocidadDer);
  }
#endif
}
