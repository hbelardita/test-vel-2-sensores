#include <Arduino.h>

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

// Lógica de detección óptica (LOW sobre blanco para comparador estándar LM393)
const uint8_t DETECTA_BLANCO = LOW;

// Parámetros de velocidad PWM
const int VELOCIDAD_BASE        = 135;
const int VELOCIDAD_MAX         = 220;
const int VELOCIDAD_REVERSA_MAX = 150; // Potencia máxima de contra-rotación en curva

// Ganancias del controlador para giro sobre su eje
const float KP = 160.0f;
const float KD = 60.0f;

// Intervalo de muestreo del bucle de control en milisegundos
const unsigned long INTERVALO_MS = 5;

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

// Variables de estado interno
float ultimoError = 0.0f;
float ultimoGiroRecuperacion = 0.0f;
unsigned long tiempoAnterior = 0;
unsigned long tiempoTelemetria = 0;

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

float calcularError(EstadoSeguimiento estado) {
  switch (estado) {
    case ESTADO_CENTRO:
      ultimoGiroRecuperacion = 0.0f;
      return 0.0f;

    case ESTADO_CORRECCION_IZQ:
      ultimoGiroRecuperacion = -1.5f;
      return -1.0f;

    case ESTADO_CORRECCION_DER:
      ultimoGiroRecuperacion = 1.5f;
      return 1.0f;

    case ESTADO_LINEA_PERDIDA:
      return ultimoGiroRecuperacion;

    case ESTADO_INTERSECCION:
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
  Serial.begin(115200);

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  pinMode(PIN_LED_TEST, OUTPUT);

  pinMode(PIN_SENSOR_IZQ, INPUT_PULLUP);
  pinMode(PIN_SENSOR_DER, INPUT_PULLUP);

  detenerMotores();
  delay(1000);
}

void loop() {
  unsigned long tiempoActual = millis();
  if (tiempoActual - tiempoAnterior < INTERVALO_MS) {
    return;
  }
  tiempoAnterior = tiempoActual;

  LecturaSensores lectura = leerSensores();
  digitalWrite(PIN_LED_TEST, (lectura.izqDetectaBlanco || lectura.derDetectaBlanco) ? HIGH : LOW);

  EstadoSeguimiento estado = clasificarEstado(lectura);
  float error = calcularError(estado);

  float derivada = error - ultimoError;
  ultimoError = error;

  float correccion = (KP * error) + (KD * derivada);

  int velocidadIzq = VELOCIDAD_BASE + (int)correccion;
  int velocidadDer = VELOCIDAD_BASE - (int)correccion;

  fijarMotores(velocidadIzq, velocidadDer);

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
}
