/************************************************************
 * ESP32 + 2 ESC 30A (50 Hz) - UDP (CRC)  [SIN IMU]
 * Solo-adelante (v1,v2 en 0..1).
 * Lados IGUALES: mismos rangos y topes para ambos ESC.
 * Stop duro + log: [DRIVE] tL=.. tR=.. -> usL=.. usR=..
 ************************************************************/
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ================== WiFi / UDP ==================
const char* WIFI_SSID = "Johan";
const char* WIFI_PASS = "12345678";
const uint16_t UDP_LISTEN_PORT = 9000;
const uint16_t PC_LISTEN_PORT  = 9001;
WiFiUDP udp;
IPAddress lastRemoteIP;

// ================== Pines ESC ==================
const int ESC_LEFT_PIN  = 25;
const int ESC_RIGHT_PIN = 26;
const int PIN_BOOT      = 0;   // BOOT -> calibración ESC (opcional)

// ================== PWM ESC ==================
const int PWM_HZ = 50;   // 50 Hz

// ======== AJUSTES SIMÉTRICOS (AMBOS LADOS IGUALES) ========
const int ATTACH_MIN   = 900;   // rango del attach
const int QUIET_MIN    = 990;   // µs en STOP real
const int US_MIN       = 1000;  // referencia (no se usa directo en stop)
const int START_US     = 990;   // primer µs con empuje (ajústalo según ESC)
const int US_MAX       = 2000;  // techo HARD

// —— Tope SUAVE (ajusta para “qué tan lento” va) ——
int       SOFT_MAX     = 1080;  // p.ej. 1060–1100 para muy lento

// ===== Offsets finos en µs (si hiciera falta igualar) =====
int   TRIM_US_L = 0;
int   TRIM_US_R = 0;

// ===== Curva / modo global =====
float EXPO = 0.20f;         // curva general (0 lineal)
float SPEED_SCALE  = 0.20f; // LIMITE GLOBAL (0..1) para ir MUY lento
float MIN_RUN_T    = 0.05f; // piso cuando hay gas (apenas movimiento)

// ===== Entrada esperada =====
bool  EXPECT_RANGE_01 = true;   // v en 0..1
float MAX_CMD         = 10.0f;  // si EXPECT_RANGE_01=false

// ===== Seguridad / transición =====
const float DEADBAND_T        = 0.02f;
const uint32_t LINK_TIMEOUT_MS= 800;
const int SLEW_US_PER_SEC     = 12000;  // rampa µs/s (solo en movimiento)

// ===== Suavizado del “base” (gas común) =====
float BASE_LPF_FC_HZ      = 2.0f;
float BASE_SLEW_PER_SEC   = 0.80f;
float BASE_EXPO           = 0.20f;
float BASE_DEADBAND       = 0.02f;

// ===== Mezcla de giro simple =====
float TURN_GAIN = 0.60f;

// ===== Estado ESC =====
Servo escL, escR;
uint32_t last_rx_ms = 0;
int last_us_L = US_MIN, last_us_R = US_MIN;
bool estop_latched = false;

// Estado de suavizado del base
float base_lpf  = 0.0f;
float base_slew = 0.0f;
bool  base_init = false;

// ===== Comandos UDP =====
enum CmdCode { CMD_IDLE=0, CMD_ROLL=1, CMD_WALK=2, CMD_ESTOP=3, CMD_ERES=4, CMD_WALK_START=6 };

// ===== CRC Dallas/Maxim =====
uint8_t crc8_maxim(const uint8_t* d, size_t n){
  uint8_t c=0x00; for(size_t i=0;i<n;++i){ c^=d[i]; for(uint8_t b=0;b<8;++b) c=(c&1)?((c>>1)^0x8C):(c>>1); } return c & 0xFF;
}
String buildFrame(float v1, float v2, float v3, int cmd, const char* id_hex){
  char payload[128]; snprintf(payload, sizeof(payload), "%.3f,%.3f,%.3f,%d,%s", v1, v2, v3, cmd, id_hex);
  uint8_t crc = crc8_maxim((const uint8_t*)payload, strlen(payload));
  char frame[160]; snprintf(frame, sizeof(frame), "<%s,%02X>\n", payload, crc); return String(frame);
}
void sendUDPStatus(float v1, float v2, float v3, int cmd){
  if (!lastRemoteIP) return; String f = buildFrame(v1, v2, v3, cmd, "0xSTA");
  udp.beginPacket(lastRemoteIP, PC_LISTEN_PORT); udp.print(f); udp.endPacket();
}

// ================== Helpers ==================
static inline float clampf(float x, float a, float b){ return x<a?a:(x>b?b:x); }
float clamp01(float x){ if(x<0) x=0; if(x>1) x=1; return x; }

static inline float expo_ease(float x, float expo){
  if (x <= 0.0f) return 0.0f; if (x >= 1.0f) return 1.0f;
  float p = 1.0f + (expo < 0 ? 0 : expo);
  return 1.0f - powf(1.0f - x, p);
}

int slewStepUs(int target, int current, uint32_t dt_ms){
  int max_step = (int)((SLEW_US_PER_SEC * dt_ms) / 1000);
  int d = target - current;
  if (d >  max_step) d =  max_step;
  if (d < -max_step) d = -max_step;
  return current + d;
}
void writeBoth(int usL, int usR){ escL.writeMicroseconds(usL); escR.writeMicroseconds(usR); last_us_L=usL; last_us_R=usR; }
void setStop(){ escL.writeMicroseconds(QUIET_MIN); escR.writeMicroseconds(QUIET_MIN); last_us_L=QUIET_MIN; last_us_R=QUIET_MIN; }

// Mapa a µs con tope global + tope suave (SIMÉTRICO)
int mapThrottleToUsFast(float t_in,
                        int quiet_min, int attach_min,
                        int start_us,  int soft_max,
                        int trim_us){
  float t = clamp01(t_in);

  // STOP real (sin suavizado)
  if (t <= 0.0f) {
    int us0 = quiet_min + trim_us;
    if (us0 < attach_min) us0 = attach_min;
    return us0;
  }

  // Piso de movimiento
  if (t > 0.0f && t < MIN_RUN_T) t = MIN_RUN_T;

  // Límite global muy bajo + curva
  t *= clampf(SPEED_SCALE, 0.0f, 1.0f);
  t = expo_ease(t, EXPO);

  // Respeta el soft-max
  int smax = soft_max;
  if (smax < start_us) smax = start_us;

  int us = start_us + (int)((smax - start_us) * t) + trim_us;
  if (us < attach_min) us = attach_min;
  if (us > smax)       us = smax;
  return us;
}

// ================== WiFi ==================
void wifiConnect(){
  WiFi.mode(WIFI_STA); WiFi.persistent(false); WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Conectando a "); Serial.println(WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status()!=WL_CONNECTED && (millis()-t0)<15000){ delay(250); Serial.print("."); }
  Serial.println();
  if (WiFi.status()==WL_CONNECTED) Serial.print("[WiFi] IP: "), Serial.println(WiFi.localIP());
  else Serial.println("[WiFi] No se pudo conectar (arranque).");
}

// ================== Calibración ESC al boot (opcional) ==================
void maybeCalibrateESC(){
  pinMode(PIN_BOOT, INPUT_PULLUP);
  delay(20);
  if (digitalRead(PIN_BOOT)==LOW){
    Serial.println("[CAL] Calibración ESC: MAX 3s -> MIN 3s");
    escL.writeMicroseconds(US_MAX);
    escR.writeMicroseconds(US_MAX);
    delay(3000);
    escL.writeMicroseconds(QUIET_MIN);
    escR.writeMicroseconds(QUIET_MIN);
    delay(3000);
    Serial.println("[CAL] Listo. Continuando...");
  }
}

// ================== Setup ==================
void setup(){
  Serial.begin(115200);
  Serial.println("\n[ESP32] 2xESC UDP (SIN IMU) - LADOS IGUALES (MUY LENTO)");

  // ESC
  escL.setPeriodHertz(PWM_HZ); escR.setPeriodHertz(PWM_HZ);
  escL.attach(ESC_LEFT_PIN,  ATTACH_MIN, US_MAX);
  escR.attach(ESC_RIGHT_PIN, ATTACH_MIN, US_MAX);
  maybeCalibrateESC();

  // Arming
  uint32_t t0 = millis();
  while (millis()-t0 < 2000) { setStop(); delay(10); }

  // WiFi/UDP
  wifiConnect();
  udp.begin(UDP_LISTEN_PORT);
  Serial.printf("[UDP] Escuchando en %u\n", UDP_LISTEN_PORT);
  last_rx_ms = millis();
}

// ================== Loop ==================
void loop(){
  static uint32_t last_ms = millis();
  uint32_t now_ms = millis();
  uint32_t dt_ms  = now_ms - last_ms;
  if (dt_ms > 200) dt_ms = 200;

  // ---- UDP RX ----
  int sz = udp.parsePacket();
  static float vL_cmd=0, vR_cmd=0;
  static int   last_cmd = CMD_IDLE;

  if (sz > 0){
    char buf[512]; int n = udp.read(buf, sizeof(buf)-1); if(n<0) n=0; buf[n]='\0';
    lastRemoteIP = udp.remoteIP(); last_rx_ms = now_ms;

    String s(buf); s.trim();
    if(s.startsWith("<") && s.endsWith(">")){
      s.remove(0,1); s.remove(s.length()-1,1);
      int lc = s.lastIndexOf(','); if(lc>0){
        String payload = s.substring(0,lc);
        String crc_hex = s.substring(lc+1); crc_hex.trim();
        uint8_t calc = crc8_maxim((const uint8_t*)payload.c_str(), payload.length());
        uint8_t recv = (uint8_t) strtoul(crc_hex.c_str(), nullptr, 16);
        if(calc == recv){
          int p1=payload.indexOf(','), p2=payload.indexOf(',',p1+1);
          int p3=payload.indexOf(',',p2+1), p4=payload.indexOf(',',p3+1);
          if(p1>=0 && p2>=0 && p3>=0 && p4>=0){
            float v1f = payload.substring(0,p1).toFloat();    // 0..1
            float v2f = payload.substring(p1+1,p2).toFloat(); // 0..1
            /*float v3f =*/ payload.substring(p2+1,p3).toFloat();
            int   cmd  = payload.substring(p3+1,p4).toInt();
            last_cmd = cmd;

            if(cmd==CMD_ESTOP){ estop_latched=true; setStop(); sendUDPStatus(0,0,0,CMD_ESTOP); }
            else if(cmd==CMD_ERES){ estop_latched=false; }
            else if(cmd==CMD_IDLE){ setStop(); sendUDPStatus(0,0,0,CMD_IDLE); }
            else {
              auto norm = [&](float v){
                float t = EXPECT_RANGE_01 ? v : (MAX_CMD>0 ? v/MAX_CMD : 0.0f);
                if (fabsf(t) < DEADBAND_T) t = 0.0f;
                return clamp01(t);
              };
              vL_cmd = norm(v1f);
              vR_cmd = norm(v2f);
            }
          }
        }
      }
    }
  }

  // ---- Failsafe ----
  if (!estop_latched && (now_ms - last_rx_ms) > LINK_TIMEOUT_MS){
    setStop(); last_rx_ms = now_ms;
  }

  // ---- Suavizado del base + mezcla de giro (simétrico) ----
  if(!estop_latched && last_cmd!=CMD_IDLE){
    float base_raw = clamp01( 0.5f * (vL_cmd + vR_cmd) );
    if (base_raw < BASE_DEADBAND) base_raw = 0.0f;

    if (base_raw <= 0.0f) {
      setStop();
      Serial.printf("[DRIVE] tL=%.2f tR=%.2f -> usL=%d usR=%d\n", 0.0f, 0.0f, QUIET_MIN, QUIET_MIN);
      last_ms = now_ms;
      return;
    }

    // Suavizado del base solo si hay gas
    float base_exp = (BASE_EXPO > 0.0f) ? expo_ease(base_raw, BASE_EXPO) : base_raw;
    float dt_s = dt_ms * 0.001f;
    float alpha_b = (2.0f * M_PI * BASE_LPF_FC_HZ * dt_s);
    alpha_b = alpha_b / (1.0f + alpha_b);
    base_lpf = base_lpf + alpha_b * (base_exp - base_lpf);
    if (!base_init) { base_slew = base_lpf; base_init = true; }
    float max_step = BASE_SLEW_PER_SEC * dt_s;
    float step = base_lpf - base_slew;
    if (step >  max_step) step =  max_step;
    if (step < -max_step) step = -max_step;
    base_slew += step;

    float base = clamp01(base_slew);

    // Mezcla de giro a partir de vL/vR recibidos
    float turn = clampf( 0.5f * (vR_cmd - vL_cmd), -1.0f, 1.0f );
    float diff = clampf(TURN_GAIN * turn, -0.5f, 0.5f);

    float tL_raw = clamp01(base - diff);
    float tR_raw = clamp01(base + diff);

    // Piso mínimo si hay gas
    if (tL_raw > 0.0f && tL_raw < MIN_RUN_T) tL_raw = MIN_RUN_T;
    if (tR_raw > 0.0f && tR_raw < MIN_RUN_T) tR_raw = MIN_RUN_T;

    // —— mapping simétrico por lado ——
    int usL_target = mapThrottleToUsFast(tL_raw,
                                         QUIET_MIN, ATTACH_MIN,
                                         START_US, SOFT_MAX,
                                         TRIM_US_L);
    int usR_target = mapThrottleToUsFast(tR_raw,
                                         QUIET_MIN, ATTACH_MIN,
                                         START_US, SOFT_MAX,
                                         TRIM_US_R);

    int usL = slewStepUs(usL_target, last_us_L, dt_ms);
    int usR = slewStepUs(usR_target, last_us_R, dt_ms);

    Serial.printf("[DRIVE] tL=%.2f tR=%.2f -> usL=%d usR=%d\n", tL_raw, tR_raw, usL, usR);
    writeBoth(usL, usR);
  } else {
    setStop();
  }

  last_ms = now_ms;
}
