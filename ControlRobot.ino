#include <AccelStepper.h>
#include <ServoEasing.hpp>

// Pines Stepper TB6600
const int STEP_PIN = 19;      // PUL+
const int DIR_PIN  = 18;      // DIR+

// Pines Servos
const int SERVO1_PIN = 2;    // θ1
const int SERVO2_PIN = 4;    // θ2

ServoEasing servo1;
ServoEasing servo2;

// Posición actual (en grados "de servo" ya mapeados)
float servo1_pos = 90.0f;   // misma que usas al attach
float servo2_pos = 90.0f;

// Stepper eje Z
const int   STEPS_PER_REV = 500;
int Zans = 1;   // posición "anterior" en unidad lógica

// Velocidad máxima de servos (deg/s)
float servo_dps = 60.0f;

// Parser
String line;

// Utilidades
template<typename T> T clampT(T v, T lo, T hi){
  return v < lo ? lo : (v > hi ? hi : v);
}

// Prototipo
void moveServosSyncLogical(float t1_logical, float t2_logical);

void setup() {
  Serial.begin(115200);
  Serial.println(F("READY"));

  // Configurar easing
  servo1.setEasingType(EASE_LINEAR);
  servo2.setEasingType(EASE_LINEAR);

  // Adjuntar servos y posición inicial
  servo1.attach(SERVO1_PIN, servo1_pos);  // 45°
  servo2.attach(SERVO2_PIN, servo2_pos);  // 90°

  servo1.setSpeed(servo_dps);   // velocidad máxima por defecto
  servo2.setSpeed(servo_dps);

  // Configurar Z
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
}

void loop() {
  // Proceso de lectura serial
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length() > 0) {
        processLine(line);
        line = "";
      }
    } else {
      line += c;
    }
  }
}

// Procesar Comandos Recibidos
void processLine(const String& s_in){
  String s = s_in;
  s.trim();
  if (!s.length()) return;

  char cmd = s[0];

  // ----- Comando T: mover servos + Z -----
  if (cmd == 'T'){
    // Esperamos: T,theta1,theta2,zcm
    String p[4];
    int n = splitCSV(s, p, 4);
    if (n >= 4){
      float t1 = clampT(p[1].toFloat(), 0.0f, 180.0f);
      float t2 = clampT(p[2].toFloat(), 0.0f, 180.0f);
      int   zcm = (int)p[3].toFloat();

      // Mover servos de forma sincronizada en tiempo
      moveServosSyncLogical(t1, t2);

      // ------ Movimiento Z (solo si cambió) ------
      if (zcm != Zans){
        if (zcm > Zans) {
          digitalWrite(DIR_PIN, LOW);   // p.ej. LOW = subir
        } else {
          digitalWrite(DIR_PIN, HIGH);  // HIGH = bajar
        }

        moverPasos(STEPS_PER_REV, 400); // 1 vuelta (ajusta a gusto)

        Zans = zcm;
      }

      Serial.print(F("OK,T,"));
      Serial.print(t1); Serial.print(",");
      Serial.print(t2); Serial.print(",");
      Serial.println(zcm);
    } else {
      Serial.println(F("ERR,BAD_T_FORMAT"));
    }
    return;
  }

  // ----- Comando I: subir Z y poner servos en 90° -----
  if (cmd == 'I'){
    // Mover servos a 90° lógicos ambos
    moveServosSyncLogical(90.0f, 90.0f);

    // Movimiento Z fijo hacia "adentro"/subir
    digitalWrite(DIR_PIN, HIGH);   // ajusta según tu lógica
    moverPasos(2550, 400);        // pasos fijos

    Serial.println(F("OK,I"));
    return;
  }

  // ----- Comando F: bajar Z y poner servos en 90° -----
  if (cmd == 'F'){
    // Mover servos a 90° lógicos ambos
    moveServosSyncLogical(90.0f, 90.0f);T, 90.00, 90.00, 1.00

    // Movimiento Z fijo hacia "afuera"/bajar
    digitalWrite(DIR_PIN, LOW);
    moverPasos(2550, 400);

    Serial.println(F("OK,F"));
    return;
  }

  // Si no es T, I ni F:
  Serial.println(F("ERR,UNKNOWN_CMD"));
}

// ---- Mover servos sincronizados en tiempo, usando ángulos lógicos (0–180) ----
void moveServosSyncLogical(float t1_logical, float t2_logical) {
  // Clamp por seguridad
  t1_logical = clampT(t1_logical, 0.0f, 180.0f);
  t2_logical = clampT(t2_logical, 0.0f, 180.0f);

  // 1) Mapear de tus ángulos "lógicos" a la escala mecánica real del servo
  float t11 = map(t1_logical, 0, 180, 12, 177);   // servo 1
  float t22 = map(t2_logical, 0, 180, 0, 176);   // servo 2

  // 2) Calcular cuánto se tiene que mover cada uno (en grados)
  float delta1 = fabs(t11 - servo1_pos);
  float delta2 = fabs(t22 - servo2_pos);

  float maxDelta = max(delta1, delta2);

  if (maxDelta <= 0.1f) {
    // Casi no hay movimiento, no hacemos nada
    return;
  }

  // 3) Tiempo de movimiento usando la velocidad máxima deseada
  float Tmove = maxDelta / servo_dps;   // segundos

  // 4) Velocidad individual para que ambos terminen en Tmove
  float speed1 = (delta1 > 0.0f) ? (delta1 / Tmove) : 0.0f;
  float speed2 = (delta2 > 0.0f) ? (delta2 / Tmove) : 0.0f;

  if (speed1 < 1.0f && delta1 > 0.5f) speed1 = 1.0f;
  if (speed2 < 1.0f && delta2 > 0.5f) speed2 = 1.0f;

  servo1.startEaseTo(t11, speed1, START_UPDATE_BY_INTERRUPT);
  servo2.startEaseTo(t22, speed2, START_UPDATE_BY_INTERRUPT);

  // Actualizar posición actual
  servo1_pos = t11;
  servo2_pos = t22;
}

// ==== Función para mover el motor 'numPasos' pasos ====
void moverPasos(long numPasos, int delayMicros) {
  for (long i = 0; i < numPasos; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delayMicros);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(delayMicros);
  }
}

// CSV
int splitCSV(const String& s, String parts[], int maxParts){
  int n = 0, start = 0;
  for (int i = 0; i <= s.length(); ++i){
    if (i == s.length() || s[i] == ',' || s[i] == ';'){
      if (n < maxParts){
        parts[n++] = s.substring(start, i);
      }
      start = i + 1;
    }
  }
  return n;
}
