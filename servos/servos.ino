/*
 * Pan-tilt con 2 servos SG90 controlado por el PC (seguidor.py).
 * Placa: Arduino UNO / Nano (ATmega328P).
 *
 * Protocolo serie, 115200 baudios, una orden por línea ('\n'):
 *   PC -> Arduino
 *     P,<pan>,<tilt>   objetivo en DÉCIMAS de grado (ej. P,900,900 = centro)
 *     M0 | M1 | M2     modo: 0 libre, 1 fijado, 2 perdido (LEDs)
 *     C                centrar
 *   Arduino -> PC
 *     READY,<panMin>,<panMax>,<tiltMin>,<tiltMax>   al arrancar (grados)
 *     A,<pan>,<tilt>   posición real actual, décimas de grado, cada 50 ms
 *     IR,<0|1>         cambio en el sensor PIR
 */
#include <Servo.h>

// ---------- Pines ----------
const uint8_t PIN_PAN  = 9;    // servo eje X (horizontal)
const uint8_t PIN_TILT = 10;   // servo eje Y (vertical)
const uint8_t LED_FREE = 4;    // verde: modo libre
const uint8_t LED_LOCK = 5;    // rojo: objetivo fijado
const uint8_t LED_LOST = 6;    // amarillo: objetivo perdido (parpadea)
const uint8_t PIN_PIR  = 2;    // PIR opcional (HC-SR501)

// ---------- Calibración (ajústala a tu soporte) ----------
const int PAN_MIN = 10,  PAN_MAX = 170;    // grados, límites mecánicos
const int TILT_MIN = 40, TILT_MAX = 140;
const int US_0   = 544;    // pulso a 0°   (si el SG90 no llega a 0°, súbelo)
const int US_180 = 2400;   // pulso a 180° (si zumba en el extremo, bájalo)

const float VEL_MAX = 240.0;   // °/s  velocidad máxima
const float ACC_MAX = 1200.0;  // °/s² aceleración: arranques/frenadas suaves, sin tirones
const uint16_t PERIODO_MS = 20;          // 50 Hz, igual que la trama PWM del servo
const uint16_t PERIODO_REPORTE_MS = 50;
const unsigned long TIMEOUT_PC = 3000;   // sin datos del PC -> modo libre

struct Eje {
  Servo servo;
  float pos, vel, tgt;
  int lo, hi;

  void init(uint8_t pin, int minGr, int maxGr) {
    lo = minGr; hi = maxGr;
    pos = tgt = 90; vel = 0;
    servo.attach(pin, US_0, US_180);
    escribe();
  }
  void objetivo(float g) { tgt = constrain(g, lo, hi); }

  // Perfil trapezoidal: nunca más rápido de lo que permite frenar justo en el objetivo.
  void paso(float dt) {
    float err = tgt - pos;
    if (fabs(err) < 0.05 && fabs(vel) < 1.0) { pos = tgt; vel = 0; escribe(); return; }
    float vDes = min(VEL_MAX, sqrt(2.0 * ACC_MAX * fabs(err)));
    if (err < 0) vDes = -vDes;
    vel += constrain(vDes - vel, -ACC_MAX * dt, ACC_MAX * dt);
    pos += vel * dt;
    if ((tgt - pos) * err < 0) { pos = tgt; vel = 0; }  // se pasó: fija en el objetivo
    escribe();
  }
  // writeMicroseconds da ~0.1° de resolución frente a 1° de write().
  void escribe() { servo.writeMicroseconds(US_0 + (long)(pos * 10 + 0.5) * (US_180 - US_0) / 1800); }
};

Eje pan, tilt;
uint8_t modo = 0;
unsigned long tPaso = 0, tCmd = 0, tPir = 0, tRep = 0;
int8_t pirPrev = -1;

char buf[24];
uint8_t n = 0;
bool desborde = false;

void setModo(uint8_t m) {
  modo = m;
  digitalWrite(LED_FREE, m == 0);
  digitalWrite(LED_LOCK, m == 1);
  if (m != 2) digitalWrite(LED_LOST, LOW);
}

void procesaLinea() {
  tCmd = millis();
  if (buf[0] == 'P') {
    int a, b;
    if (sscanf(buf, "P,%d,%d", &a, &b) == 2) {
      pan.objetivo(a / 10.0);
      tilt.objetivo(b / 10.0);
    }
  } else if (buf[0] == 'M' && buf[1] >= '0' && buf[1] <= '2') {
    setModo(buf[1] - '0');
  } else if (buf[0] == 'C') {
    pan.objetivo(90);
    tilt.objetivo(90);
  }
}

void leeSerie() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[n] = 0;
      if (n > 0 && !desborde) procesaLinea();  // línea demasiado larga = corrupta, se descarta
      n = 0; desborde = false;
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    } else {
      desborde = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_FREE, OUTPUT);
  pinMode(LED_LOCK, OUTPUT);
  pinMode(LED_LOST, OUTPUT);
  pinMode(PIN_PIR, INPUT);
  pan.init(PIN_PAN, PAN_MIN, PAN_MAX);
  tilt.init(PIN_TILT, TILT_MIN, TILT_MAX);
  setModo(0);
  Serial.print(F("READY,"));
  Serial.print(PAN_MIN);  Serial.print(',');
  Serial.print(PAN_MAX);  Serial.print(',');
  Serial.print(TILT_MIN); Serial.print(',');
  Serial.println(TILT_MAX);
  tCmd = tPaso = millis();
}

void loop() {
  leeSerie();
  unsigned long ahora = millis();

  if (ahora - tPaso >= PERIODO_MS) {
    float dt = (ahora - tPaso) / 1000.0;
    tPaso = ahora;
    pan.paso(dt);
    tilt.paso(dt);
    if (modo == 2) digitalWrite(LED_LOST, (ahora / 125) % 2);
  }

  if (ahora - tRep >= PERIODO_REPORTE_MS) {
    tRep = ahora;
    Serial.print(F("A,"));
    Serial.print((int)(pan.pos * 10));
    Serial.print(',');
    Serial.println((int)(tilt.pos * 10));
  }

  if (modo != 0 && ahora - tCmd > TIMEOUT_PC) setModo(0);  // PC caído: mantiene posición

  if (ahora - tPir >= 50) {
    tPir = ahora;
    int8_t p = digitalRead(PIN_PIR);
    if (p != pirPrev) {
      pirPrev = p;
      Serial.print(F("IR,"));
      Serial.println(p);
    }
  }
}
