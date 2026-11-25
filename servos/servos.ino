/*
 * Pan-tilt con 2 servos SG90 controlado por el PC (seguidor.py / calibrar.py).
 * Placa: Arduino UNO / Nano (ATmega328P).
 *
 * Protocolo serie, 115200 baudios, una orden por línea ('\n'):
 *   PC -> Arduino
 *     P,<pan>,<tilt>   objetivo en DÉCIMAS de grado, dentro de los límites (ej. P,900,900 = centro)
 *     R,<pan>,<tilt>   igual pero ignorando los límites (-20° a 200°): solo para calibrar
 *     L,<panMin>,<panMax>,<tiltMin>,<tiltMax>   guarda los límites (grados) en EEPROM
 *     M0 | M1 | M2     modo: 0 libre, 1 fijado, 2 perdido (LEDs)
 *     C                centrar
 *     ?                vuelve a enviar READY
 *   Arduino -> PC
 *     READY,<panMin>,<panMax>,<tiltMin>,<tiltMax>   al arrancar y al cambiar límites (grados)
 *     A,<pan>,<tilt>   posición real actual, décimas de grado, cada 50 ms
 *     IR,<0|1>         cambio en el sensor PIR
 *     ERR,L            límites rechazados (mín < máx, -20..200, al menos 20° de recorrido)
 */
#include <Servo.h>
#include <EEPROM.h>

// ---------- Pines ----------
const uint8_t PIN_PAN  = 9;    // servo eje X (horizontal)
const uint8_t PIN_TILT = 10;   // servo eje Y (vertical)
const uint8_t LED_FREE = 4;    // verde: modo libre
const uint8_t LED_LOCK = 5;    // rojo: objetivo fijado
const uint8_t LED_LOST = 6;    // amarillo: objetivo perdido (parpadea)
const uint8_t PIN_PIR  = 2;    // PIR opcional (HC-SR501)

// ---------- Calibración ----------
// Los límites reales se miden con calibrar.py y se guardan en EEPROM; estos son solo los de fábrica.
const int16_t PAN_MIN_DEF = 10,  PAN_MAX_DEF = 170;
const int16_t TILT_MIN_DEF = 40, TILT_MAX_DEF = 140;
const int US_0   = 544;    // pulso a 0°
const int US_180 = 2400;   // pulso a 180°
// Rango eléctrico máximo que se deja usar (≈ -20° a 200°): muchos SG90 llegan algo más allá de 0-180.
const int16_t GR_MIN = -20, GR_MAX = 200;
const int US_MIN = 330, US_MAX = 2610;

// Velocidad y aceleración por eje: el pan mueve todo el soporte y la cámara, por eso va más suave.
const float PAN_VEL = 90.0,  PAN_ACC = 250.0;    // °/s y °/s²
const float TILT_VEL = 120.0, TILT_ACC = 400.0;

const uint16_t PERIODO_MS = 20;          // 50 Hz, igual que la trama PWM del servo
const uint16_t PERIODO_REPORTE_MS = 50;
const unsigned long TIMEOUT_PC = 3000;   // sin datos del PC -> modo libre

struct Limites {
  uint8_t magia;
  int16_t panMin, panMax, tiltMin, tiltMax;
};
const uint8_t MAGIA = 0xA7;   // marca de "EEPROM ya calibrada"
Limites lim = {MAGIA, PAN_MIN_DEF, PAN_MAX_DEF, TILT_MIN_DEF, TILT_MAX_DEF};

bool validos(const Limites &l) {
  return l.panMin >= GR_MIN && l.panMax <= GR_MAX && l.panMax - l.panMin >= 20 &&
         l.tiltMin >= GR_MIN && l.tiltMax <= GR_MAX && l.tiltMax - l.tiltMin >= 20;
}

struct Eje {
  Servo servo;
  float pos, vel, tgt;
  int lo, hi;
  float vMax, aMax;

  void init(uint8_t pin, int minGr, int maxGr, float v, float a) {
    lo = minGr; hi = maxGr; vMax = v; aMax = a;
    pos = tgt = constrain(90, lo, hi); vel = 0;
    servo.attach(pin, US_MIN, US_MAX);
    escribe();
  }
  void limites(int minGr, int maxGr) { lo = minGr; hi = maxGr; tgt = constrain(tgt, lo, hi); }
  void objetivo(float g) { tgt = constrain(g, lo, hi); }
  void objetivoLibre(float g) { tgt = constrain(g, GR_MIN, GR_MAX); }

  // Perfil trapezoidal: nunca más rápido de lo que permite frenar justo en el objetivo.
  void paso(float dt) {
    float err = tgt - pos;
    if (fabs(err) < 0.05 && fabs(vel) < 1.0) { pos = tgt; vel = 0; escribe(); return; }
    float vDes = min(vMax, sqrt(2.0 * aMax * fabs(err)));
    if (err < 0) vDes = -vDes;
    vel += constrain(vDes - vel, -aMax * dt, aMax * dt);
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

char buf[32];
uint8_t n = 0;
bool desborde = false;

void setModo(uint8_t m) {
  modo = m;
  digitalWrite(LED_FREE, m == 0);
  digitalWrite(LED_LOCK, m == 1);
  if (m != 2) digitalWrite(LED_LOST, LOW);
}

void enviaReady() {
  Serial.print(F("READY,"));
  Serial.print(lim.panMin);  Serial.print(',');
  Serial.print(lim.panMax);  Serial.print(',');
  Serial.print(lim.tiltMin); Serial.print(',');
  Serial.println(lim.tiltMax);
}

void procesaLinea() {
  tCmd = millis();
  int a, b, c, d;
  if (buf[0] == 'P' && sscanf(buf, "P,%d,%d", &a, &b) == 2) {
    pan.objetivo(a / 10.0);
    tilt.objetivo(b / 10.0);
  } else if (buf[0] == 'R' && sscanf(buf, "R,%d,%d", &a, &b) == 2) {
    pan.objetivoLibre(a / 10.0);
    tilt.objetivoLibre(b / 10.0);
  } else if (buf[0] == 'L' && sscanf(buf, "L,%d,%d,%d,%d", &a, &b, &c, &d) == 4) {
    Limites nuevo = {MAGIA, (int16_t)a, (int16_t)b, (int16_t)c, (int16_t)d};
    if (validos(nuevo)) {
      lim = nuevo;
      EEPROM.put(0, lim);  // put solo reescribe los bytes que cambian
      pan.limites(lim.panMin, lim.panMax);
      tilt.limites(lim.tiltMin, lim.tiltMax);
      enviaReady();
    } else {
      Serial.println(F("ERR,L"));
    }
  } else if (buf[0] == 'M' && buf[1] >= '0' && buf[1] <= '2') {
    setModo(buf[1] - '0');
  } else if (buf[0] == 'C') {
    pan.objetivo((lim.panMin + lim.panMax) / 2.0);
    tilt.objetivo((lim.tiltMin + lim.tiltMax) / 2.0);
  } else if (buf[0] == '?') {
    enviaReady();
  }
}

void leeSerie() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      buf[n] = 0;
      if (n > 0 && !desborde) procesaLinea();  // línea demasiado larga = corrupta, se descarta
      n = 0; desborde = false;
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = ch;
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

  Limites e;
  EEPROM.get(0, e);
  if (e.magia == MAGIA && validos(e)) lim = e;

  pan.init(PIN_PAN, lim.panMin, lim.panMax, PAN_VEL, PAN_ACC);
  tilt.init(PIN_TILT, lim.tiltMin, lim.tiltMax, TILT_VEL, TILT_ACC);
  setModo(0);
  enviaReady();
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
