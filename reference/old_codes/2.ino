#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

// ============ WiFi + servidor asincrono (todo corre en Core 0) ============
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// -------------------- PPM CONFIG --------------------
#define PPM_PIN            4
#define CHANNELS           6
#define SYNC_PULSE_US      2100
#define BAUD_RATE          115200

#define PPM_DEADBAND_LOW   1480
#define PPM_DEADBAND_HIGH  1520
#define PPM_MIN_US         1000
#define PPM_MAX_US         2000

volatile uint32_t ppmValues[CHANNELS] = {0};
volatile uint8_t  currentChannel = 0;
volatile uint32_t lastPulse = 0;
volatile bool     frameReady = false;
volatile uint32_t lastFrameMicros = 0;

void IRAM_ATTR ppmISR()
{
  uint32_t now = micros();
  uint32_t pulseWidth = now - lastPulse;
  lastPulse = now;

  if (pulseWidth > SYNC_PULSE_US)
  {
    currentChannel = 0;
    frameReady = true;
    lastFrameMicros = now;
  }
  else
  {
    if (currentChannel < CHANNELS)
    {
      ppmValues[currentChannel] = pulseWidth;
      currentChannel++;
    }
  }
}

// -------------------- CAN / VESC CONFIG --------------------
static const uint8_t PIN_CS = 5;

// Drive
static const uint8_t VESC_LEFT_TRACK  = 60;
static const uint8_t VESC_RIGHT_TRACK = 50;

// Flippers
static const uint8_t VESC_FRONT_LEFT  = 20;
static const uint8_t VESC_FRONT_RIGHT = 10;
static const uint8_t VESC_REAR_LEFT   = 40;
static const uint8_t VESC_REAR_RIGHT  = 30;

static const uint8_t CMD_SET_RPM = 3;

// ---- NUEVO: VESC CAN status command IDs ----
static const uint8_t VESC_CMD_STATUS_1 = 9;   // ERPM | I_motor | duty
static const uint8_t VESC_CMD_STATUS_4 = 16;  // T_fet | T_motor | I_in | pid_pos
static const uint8_t VESC_CMD_STATUS_5 = 27;  // tach | V_in

MCP2515 mcp2515(PIN_CS);
struct can_frame tx;

static inline uint32_t vescEID(uint8_t unit_id, uint8_t cmd) {
  return ((uint32_t)cmd << 8) | unit_id;
}

static inline void putInt32BE(uint8_t *d, int32_t v) {
  d[0] = (v >> 24) & 0xFF;
  d[1] = (v >> 16) & 0xFF;
  d[2] = (v >>  8) & 0xFF;
  d[3] = (v >>  0) & 0xFF;
}

// ---- NUEVO: lectores big-endian para parsear los status frames ----
static inline int16_t rdI16BE(const uint8_t *d) {
  return (int16_t)(((uint16_t)d[0] << 8) | d[1]);
}
static inline int32_t rdI32BE(const uint8_t *d) {
  return (int32_t)(((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
                   ((uint32_t)d[2] <<  8) |  (uint32_t)d[3]);
}

void sendVescRPM(uint8_t vescID, int32_t rpm) {
  tx.can_id  = vescEID(vescID, CMD_SET_RPM) | CAN_EFF_FLAG;
  tx.can_dlc = 4;
  memset(tx.data, 0, sizeof(tx.data));
  putInt32BE(tx.data, rpm);

  auto err = mcp2515.sendMessage(&tx);
  if (err != MCP2515::ERROR_OK)
  {
    Serial.print("CAN send err to ID ");
    Serial.print(vescID);
    Serial.print(" = ");
    Serial.println((int)err);
  }
}

// =============================================================
// =============== NUEVO: TELEMETRIA VESC ======================
// =============================================================
// Tabla con los datos mas recientes de cada VESC.
// Solo Core 1 ESCRIBE (en loop, drainCanRx).
// Solo Core 0 LEE (en sensorsTask, broadcastVescTelemetry).
// Cada campo es volatile para evitar caching del compilador.
// 32-bit aligned reads/writes son atomicos en el ESP32, asi
// que no hay torn reads.
struct VescTelemetry {
  volatile float    erpm;
  volatile float    duty;            // -1.0 .. 1.0
  volatile float    current_motor;   // A
  volatile float    current_in;      // A (batería)
  volatile float    voltage_in;      // V (batería)
  volatile float    temp_fet;        // °C
  volatile float    temp_motor;      // °C
  volatile uint32_t lastUpdateMs;    // 0 = nunca
};

// Indices fijos para los 6 motores
// 0=left track 1=right track 2=front left 3=front right 4=rear left 5=rear right
static VescTelemetry tele[6];

// Nombres legibles (los usa el broadcast)
static const char* vescNames[6] = {
  "Left Track", "Right Track",
  "Front Left", "Front Right",
  "Rear Left",  "Rear Right"
};
// IDs CAN de cada indice (para el JSON)
static const uint8_t vescIds[6] = {
  60, 50, 20, 10, 40, 30
};

// Mapeo VESC ID -> indice interno
static inline int vescIdToIdx(uint8_t id) {
  switch (id) {
    case 60: return 0;
    case 50: return 1;
    case 20: return 2;
    case 10: return 3;
    case 40: return 4;
    case 30: return 5;
    default: return -1;
  }
}

// Parsear UN frame entrante de VESC. Lo llamamos desde Core 1.
static void processVescStatus(const struct can_frame &rx)
{
  // Solo nos interesan extended IDs (los VESC los mandan asi)
  if (!(rx.can_id & CAN_EFF_FLAG)) return;

  uint32_t eid    = rx.can_id & CAN_EFF_MASK;
  uint8_t  vescID = (uint8_t)(eid & 0xFF);
  uint8_t  cmd    = (uint8_t)((eid >> 8) & 0xFF);

  int idx = vescIdToIdx(vescID);
  if (idx < 0) return;

  uint32_t nowMs = millis();

  switch (cmd) {
    case VESC_CMD_STATUS_1: {
      if (rx.can_dlc < 8) return;
      int32_t erpm   = rdI32BE(&rx.data[0]);
      int16_t curMot = rdI16BE(&rx.data[4]);  // /10
      int16_t duty1k = rdI16BE(&rx.data[6]);  // /1000
      tele[idx].erpm          = (float)erpm;
      tele[idx].current_motor = curMot / 10.0f;
      tele[idx].duty          = duty1k / 1000.0f;
      tele[idx].lastUpdateMs  = nowMs;
    } break;

    case VESC_CMD_STATUS_4: {
      if (rx.can_dlc < 8) return;
      int16_t tFet = rdI16BE(&rx.data[0]); // /10
      int16_t tMot = rdI16BE(&rx.data[2]); // /10
      int16_t cIn  = rdI16BE(&rx.data[4]); // /10
      tele[idx].temp_fet     = tFet / 10.0f;
      tele[idx].temp_motor   = tMot / 10.0f;
      tele[idx].current_in   = cIn  / 10.0f;
      tele[idx].lastUpdateMs = nowMs;
    } break;

    case VESC_CMD_STATUS_5: {
      if (rx.can_dlc < 6) return;
      // tach (int32 BE) en [0..3], voltaje*10 (int16 BE) en [4..5]
      int16_t vIn = rdI16BE(&rx.data[4]);
      tele[idx].voltage_in   = vIn / 10.0f;
      tele[idx].lastUpdateMs = nowMs;
    } break;

    default:
      // Ignoramos cualquier otro mensaje (incluye ecos de SET_RPM si los hubiera)
      break;
  }
}

// Vacia el buffer RX del MCP2515. Llamado al final de cada loop (Core 1).
// Procesa hasta MAX_FRAMES por llamada para no extender demasiado el ciclo.
static void drainCanRx()
{
  static const int MAX_FRAMES = 16;
  struct can_frame rx;
  for (int i = 0; i < MAX_FRAMES; i++) {
    if (mcp2515.readMessage(&rx) != MCP2515::ERROR_OK) break;
    processVescStatus(rx);
  }
}
// ====================== FIN BLOQUE TELEMETRIA ======================


// -------------------- CONTROL TUNING --------------------
static const int32_t RPM_MAX_DRIVE   = 14000;
static const int32_t RPM_MAX_FLIPPER = 14000;
static const uint32_t CAN_PERIOD_MS  = 20;
static const uint32_t FAILSAFE_MS    = 200;
static const int32_t RPM_SLEW_PER_TICK = 200;

// -------------------- HELPERS --------------------
static inline int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float ppmToNormalized(int ppm)
{
  ppm = clampInt(ppm, PPM_MIN_US, PPM_MAX_US);

  if (ppm >= PPM_DEADBAND_LOW && ppm <= PPM_DEADBAND_HIGH) return 0.0f;

  if (ppm > PPM_DEADBAND_HIGH) {
    return (float)(ppm - PPM_DEADBAND_HIGH) /
           (float)(PPM_MAX_US - PPM_DEADBAND_HIGH);
  } else {
    return -(float)(PPM_DEADBAND_LOW - ppm) /
             (float)(PPM_DEADBAND_LOW - PPM_MIN_US);
  }
}

int32_t rampRPM(int32_t current, int32_t target) {
  int32_t diff = target - current;
  if (diff >  RPM_SLEW_PER_TICK) diff =  RPM_SLEW_PER_TICK;
  if (diff < -RPM_SLEW_PER_TICK) diff = -RPM_SLEW_PER_TICK;
  return current + diff;
}

// =========================
// SENSORES (corren en Core 0)
// =========================
#define SDA_PIN 21
#define SCL_PIN 22

// ---- NUEVO: MQ-2 (sensor de gas analogico) ----
// IMPORTANTE: en ESP32 el GPIO 12 esta en ADC2, y ADC2 NO funciona
// mientras WiFi este activo. Por eso usamos un pin del ADC1.
// Recomendados: 34, 35 (input-only, sin pull-ups), o 32/33.
// Si quieres usar el GPIO 12 a la fuerza, cambialo aqui pero las
// lecturas no van a ser confiables con el AP encendido.
#define MQ2_PIN 32

// Calibracion lineal (igual que tu codigo de referencia)
static const int   MQ2_CLEAN_ADC = 3750;   // lectura en aire limpio
static const int   MQ2_MAX_ADC   = 4095;   // tope del ADC de 12 bits
static const float MQ2_MAX_PPM   = 2000.0f;

Adafruit_LIS3MDL  lis3mdl;
Adafruit_MLX90640 mlx;

float frame[32 * 24];
float mag_x0 = 0, mag_y0 = 0, mag_z0 = 0;

// Variables compartidas entre cores
volatile float g_mag_dx = 0, g_mag_dy = 0, g_mag_dz = 0;
volatile float g_thermMin = 0, g_thermMax = 0, g_thermCenter = 0;
volatile bool  g_sensorsReady = false;

// NUEVO: ultima lectura del sensor de gas MQ-2 (PPM aproximadas).
// La escribe Core 0 (sensorsTask), la lee Core 0 mismo (broadcast).
volatile float g_mq2_ppm = 0;

// NUEVO: estado del modo invertido (canal 6). Lo escribe Core 1 (loop),
// lo lee Core 0 (broadcast) para avisar a la interfaz.
volatile bool  g_invertFront = false;

// =====================================================================
// ===================== WIFI ACCESS POINT + WEB =======================
// =====================================================================
const char* AP_SSID = "RescueRobot";
const char* AP_PASS = "rescue1234";   // minimo 8 chars para WPA2

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ---- Pagina web embebida (HTML + CSS + JS, sin dependencias externas) ----
const char index_html[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rescue Robot Monitor</title>
<style>
  :root{
    --bg:#0b0f14; --panel:#141a22; --line:#1f2937;
    --txt:#e5e7eb; --muted:#94a3b8; --ok:#22c55e; --warn:#f59e0b;
    --bad:#ef4444; --acc:#38bdf8;
  }
  *{box-sizing:border-box}
  html,body{margin:0;padding:0;background:var(--bg);color:var(--txt);
    font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
  header{display:flex;align-items:center;justify-content:space-between;
    padding:12px 20px;border-bottom:1px solid var(--line);background:#0e141b}
  header h1{font-size:18px;margin:0;letter-spacing:.5px}
  #status{font-size:13px;padding:4px 10px;border-radius:999px;
    background:#1f2937;color:var(--muted)}
  #status.ok{background:rgba(34,197,94,.15);color:var(--ok)}
  #status.bad{background:rgba(239,68,68,.15);color:var(--bad)}
  main{display:grid;grid-template-columns:1fr 1fr;gap:16px;padding:16px;
    max-width:1400px;margin:0 auto}
  @media(max-width:900px){main{grid-template-columns:1fr}}
  section{background:var(--panel);border:1px solid var(--line);
    border-radius:12px;padding:14px}
  section h2{margin:0 0 10px 0;font-size:14px;color:var(--acc);
    text-transform:uppercase;letter-spacing:1px}
  .canvasWrap{position:relative;width:100%;display:flex;justify-content:center}
  canvas{background:#000;border-radius:8px;display:block;max-width:100%}
  .stats{margin-top:12px;display:grid;grid-template-columns:repeat(3,1fr);
    gap:8px;font-size:13px}
  .stat{background:#0e141b;padding:8px 10px;border-radius:6px;
    border:1px solid var(--line)}
  .stat .k{color:var(--muted);font-size:11px;text-transform:uppercase}
  .stat .v{color:var(--txt);font-size:16px;margin-top:2px}
  .legend{margin-top:8px;height:14px;border-radius:4px;
    background:linear-gradient(to right,
      #000010 0%, #200060 15%, #6010a0 35%, #d03060 55%,
      #ff8020 75%, #ffd000 90%, #ffffff 100%)}
  .legend-lab{display:flex;justify-content:space-between;font-size:11px;
    color:var(--muted);margin-top:4px}
  footer{padding:10px 20px;color:var(--muted);font-size:12px;text-align:center;
    border-top:1px solid var(--line);margin-top:8px}

  /* ---- Sección Motores ---- */
  .full{grid-column:1 / -1}
  .vesc-head{display:flex;justify-content:space-between;align-items:center;
    margin-bottom:10px;flex-wrap:wrap;gap:8px}
  .vesc-bus{font-size:13px;color:var(--muted)}
  .vesc-bus b{color:var(--txt)}

  /* ---- Cámara FPV (procesada en el navegador, no toca ESP32) ---- */
  .fpv-head{display:flex;justify-content:space-between;align-items:center;
    flex-wrap:wrap;gap:10px;margin-bottom:10px}
  .fpv-ctrls{display:flex;gap:8px;align-items:center;flex-wrap:wrap;font-size:13px}
  .fpv-ctrls select,.fpv-ctrls button{
    background:#0e141b;color:var(--txt);border:1px solid var(--line);
    border-radius:6px;padding:6px 10px;font-family:inherit;font-size:13px;
    cursor:pointer}
  .fpv-ctrls button:hover{border-color:var(--acc)}
  .fpv-ctrls button.danger{color:var(--bad);border-color:#3a1f24}
  .fpv-wrap{position:relative;width:100%;background:#000;border-radius:8px;
    overflow:hidden;aspect-ratio:4/3;max-height:520px;margin:0 auto;
    display:flex;align-items:center;justify-content:center}
  .fpv-wrap video{width:100%;height:100%;object-fit:contain;display:block}
  .fpv-wrap.fs{aspect-ratio:auto;max-height:none;height:80vh}
  .fpv-empty{color:var(--muted);text-align:center;padding:30px;font-size:14px;
    line-height:1.6}
  .fpv-empty b{color:var(--txt)}
  .fpv-hud{position:absolute;top:8px;left:8px;background:rgba(0,0,0,.55);
    color:#fff;padding:4px 8px;border-radius:4px;font-size:12px;
    display:none;pointer-events:none}
  .fpv-hud.on{display:block}
  .fpv-hud .rec{color:#ef4444;font-weight:bold}
  .fpv-err{color:var(--bad);font-size:12px;margin-top:6px}

  /* ---- Sensor de gas MQ-2 ---- */
  .gas{display:flex;align-items:center;gap:14px;flex-wrap:wrap;
    background:#0e141b;border:1px solid var(--line);border-radius:8px;
    padding:12px 14px}
  .gas .lab{font-size:11px;color:var(--muted);text-transform:uppercase;
    letter-spacing:.5px;white-space:nowrap}
  .gas .track{flex:1;min-width:160px;height:20px;background:#1f2937;
    border-radius:5px;overflow:hidden;position:relative}
  .gas .fill{height:100%;width:0;border-radius:5px;
    transition:width .25s,background .25s;background:var(--ok)}
  .gas .fill.warn{background:var(--warn)}
  .gas .fill.bad{background:var(--bad)}
  .gas .ticks{position:absolute;inset:0;pointer-events:none}
  .gas .ticks span{position:absolute;top:0;bottom:0;width:1px;
    background:rgba(255,255,255,.18)}
  .gas .val{font-size:18px;color:var(--txt);white-space:nowrap;
    font-variant-numeric:tabular-nums;min-width:140px;text-align:right}
  .gas .val small{color:var(--muted);font-size:12px;margin-left:4px}
  .gas .state{font-size:12px;font-weight:bold;padding:4px 10px;border-radius:999px;
    background:rgba(34,197,94,.18);color:var(--ok);letter-spacing:.5px}
  .gas .state.warn{background:rgba(245,158,11,.18);color:var(--warn)}
  .gas .state.bad {background:rgba(239,68,68,.18); color:var(--bad);
    animation:gasalert 0.9s ease-in-out infinite}
  @keyframes gasalert{0%,100%{opacity:1}50%{opacity:.45}}

  /* ---- Banner modo invertido ---- */
  .invbanner{display:none;align-items:center;gap:10px;
    background:rgba(245,158,11,.14);border:1px solid var(--warn);
    color:var(--warn);border-radius:8px;padding:10px 14px;margin-bottom:14px;
    font-size:14px;font-weight:bold;letter-spacing:.3px}
  .invbanner.on{display:flex;animation:invpulse 1.4s ease-in-out infinite}
  .invbanner .sub{color:var(--muted);font-weight:normal;font-size:12px}
  @keyframes invpulse{0%,100%{opacity:1}50%{opacity:.55}}
  .motor.inv .name{color:var(--warn)}
  .motor.inv .nm::after{content:" ⟳";font-size:11px}

  /* ---- Barra de batería (única, compartida) ---- */
  .battery{display:flex;align-items:center;gap:12px;margin-bottom:14px;
    background:#0e141b;border:1px solid var(--line);border-radius:8px;
    padding:10px 12px}
  .battery .lab{font-size:11px;color:var(--muted);text-transform:uppercase;
    letter-spacing:.5px;white-space:nowrap}
  .battery .track{flex:1;height:18px;background:#1f2937;border-radius:5px;
    overflow:hidden;position:relative}
  .battery .fill{height:100%;width:0;border-radius:5px;
    transition:width .25s,background .25s;background:var(--ok)}
  .battery .fill.warn{background:var(--warn)}
  .battery .fill.bad{background:var(--bad)}
  .battery .val{font-size:15px;color:var(--txt);white-space:nowrap;
    font-variant-numeric:tabular-nums;min-width:120px;text-align:right}
  .battery .val small{color:var(--muted);font-size:12px}

  .motors{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
  @media(max-width:900px){.motors{grid-template-columns:repeat(2,1fr)}}
  @media(max-width:600px){.motors{grid-template-columns:1fr}}
  .motor{background:#0e141b;border:1px solid var(--line);border-radius:8px;
    padding:10px;position:relative;transition:opacity .3s}
  .motor.stale{opacity:.35}
  .motor .name{font-size:12px;color:var(--acc);text-transform:uppercase;
    letter-spacing:.5px;margin-bottom:4px}
  .motor .id{font-size:11px;color:var(--muted)}
  .motor .rpm{font-size:22px;margin:6px 0 2px 0}
  .motor .rpm small{font-size:11px;color:var(--muted)}
  .motor .row{display:flex;justify-content:space-between;font-size:12px;
    margin-top:4px;color:var(--muted)}
  .motor .row b{color:var(--txt);font-weight:normal;
    font-variant-numeric:tabular-nums}
  .motor .bar{height:4px;background:#1f2937;border-radius:2px;overflow:hidden;
    margin:6px 0 2px 0}
  .motor .bar > div{height:100%;background:var(--acc);width:0;
    transition:width .15s,background .15s}
  .motor .bar.neg > div{background:#a78bfa}
  /* Barra de corriente: mas alta y con color segun nivel */
  .motor .ibar{height:8px;background:#1f2937;border-radius:3px;overflow:hidden;
    margin:6px 0 2px 0}
  .motor .ibar > div{height:100%;width:0;border-radius:3px;
    transition:width .15s,background .15s;background:var(--ok)}
  .motor .ibar > div.warn{background:var(--warn)}
  .motor .ibar > div.bad{background:var(--bad)}
  .motor .barlab{display:flex;justify-content:space-between;font-size:11px;
    color:var(--muted);margin-top:2px}
  .motor .barlab b{color:var(--txt);font-weight:normal;
    font-variant-numeric:tabular-nums}
  .tcolor{font-weight:bold}
  .t-ok{color:var(--ok)}
  .t-warn{color:var(--warn)}
  .t-bad{color:var(--bad)}
  .dot{display:inline-block;width:8px;height:8px;border-radius:50%;
    background:var(--ok);margin-right:6px;vertical-align:middle}
  .motor.stale .dot{background:var(--muted)}
</style>
</head>
<body>
<header>
  <h1>🤖 Rescue Robot Monitor</h1>
  <span id="status">Desconectado</span>
</header>
<main>
  <section class="full">
    <div class="fpv-head">
      <h2 style="margin:0">Cámara FPV (5.8 GHz vía receptor USB)</h2>
      <div class="fpv-ctrls">
        <select id="fpvDev" title="Cámara"><option>—</option></select>
        <button id="fpvStart">▶ Iniciar</button>
        <button id="fpvStop" class="danger" style="display:none">■ Detener</button>
        <button id="fpvFs" title="Pantalla completa">⛶</button>
      </div>
    </div>
    <div class="fpv-wrap" id="fpvWrap">
      <div class="fpv-empty" id="fpvEmpty">
        <b>📡 Sin video</b><br>
        Conecta el receptor EWRF a tu laptop, elige la cámara correcta arriba y pulsa <b>Iniciar</b>.<br>
        <span style="font-size:11px">La imagen se procesa localmente en el navegador — no pasa por el ESP32.</span>
      </div>
      <video id="fpvVid" autoplay playsinline muted style="display:none"></video>
      <div class="fpv-hud" id="fpvHud"><span class="rec">●</span> LIVE · <span id="fpvRes">--</span> · <span id="fpvFps">--</span> fps</div>
    </div>
    <div class="fpv-err" id="fpvErr"></div>
  </section>
  <section>
    <h2>Camara Termica MLX90640 (32x24)</h2>
    <div class="canvasWrap">
      <canvas id="therm" width="384" height="288"></canvas>
    </div>
    <div class="legend"></div>
    <div class="legend-lab"><span id="lmin">--</span><span id="lmax">--</span></div>
    <div class="stats">
      <div class="stat"><div class="k">Min</div><div class="v" id="thMin">--</div></div>
      <div class="stat"><div class="k">Max</div><div class="v" id="thMax">--</div></div>
      <div class="stat"><div class="k">Centro</div><div class="v" id="thCen">--</div></div>
    </div>
  </section>
  <section>
    <h2>Magnetometro LIS3MDL (delta vs calibracion)</h2>
    <div class="canvasWrap">
      <canvas id="mag" width="320" height="320"></canvas>
    </div>
    <div class="stats">
      <div class="stat"><div class="k">X</div><div class="v" id="mx">--</div></div>
      <div class="stat"><div class="k">Y</div><div class="v" id="my">--</div></div>
      <div class="stat"><div class="k">Z</div><div class="v" id="mz">--</div></div>
    </div>
    <div class="stats" style="margin-top:8px;grid-template-columns:1fr 1fr">
      <div class="stat"><div class="k">Heading XY</div><div class="v" id="mh">--</div></div>
      <div class="stat"><div class="k">Magnitud</div><div class="v" id="mm">--</div></div>
    </div>
  </section>
  <section class="full">
    <div class="vesc-head">
      <h2 style="margin:0">Detector de gas MQ-2</h2>
      <span id="gasState" class="state">— SIN DATOS —</span>
    </div>
    <div class="gas">
      <span class="lab">💨 Concentración</span>
      <div class="track">
        <div class="fill" id="gasFill"></div>
        <div class="ticks">
          <span style="left:20%"></span>
          <span style="left:40%"></span>
          <span style="left:60%"></span>
          <span style="left:80%"></span>
        </div>
      </div>
      <span class="val"><span id="gasPpm">--</span><small>ppm</small></span>
    </div>
  </section>
  <section class="full">
    <div class="vesc-head">
      <h2 style="margin:0">Motores VESC (CAN bus)</h2>
      <div class="vesc-bus">
        I_total: <b id="ibus">--</b> A &nbsp;|&nbsp;
        P_total: <b id="pbus">--</b> W
      </div>
    </div>
    <div class="invbanner" id="invBanner">
      <span>⚠️ MODO INVERTIDO ACTIVO</span>
      <span class="sub">El frente y la parte trasera están intercambiados (adelante↔atrás, izq↔der). Las etiquetas de los motores muestran su rol actual.</span>
    </div>
    <div class="battery">
      <span class="lab">🔋 Batería</span>
      <div class="track"><div class="fill" id="batFill"></div></div>
      <span class="val"><span id="batV">--</span> V <small>(<span id="batPct">--</span>%)</small></span>
    </div>
    <div class="motors" id="motors"></div>
  </section>
</main>
<footer>Esta interfaz es ajena a cualquier partido político, queda prohibido su uso para fines distintos a los establecidos en el programa. Robotec Marca Registrada.</footer>

<script>
(function(){
  // ============ CONFIG EDITABLE (ajusta a tu hardware) ============
  const CFG = {
    POLE_PAIRS : 7,      // ERPM / POLE_PAIRS = RPM real del eje
    VBAT_MIN   : 19.2,   // Voltaje a 0%  (ej. 6S LiPo: 3.2 V/celda)
    VBAT_MAX   : 25.2,   // Voltaje a 100% (ej. 6S LiPo: 4.2 V/celda)
    IMOTOR_MAX : 45,     // Amperes = fondo de escala de la barra de corriente
    RPM_MAX    : 2000,   // RPM = fondo de escala de la barra de velocidad
    // Sensor de gas MQ-2 (orientativos; ajusta segun calibracion real)
    GAS_MAX    : 2000,   // ppm = fondo de escala de la barra
    GAS_WARN   : 400,    // ppm = entra en estado ATENCION
    GAS_ALERT  : 800     // ppm = entra en estado ALERTA (pulsa rojo)
  };
  // ===============================================================

  // ---------- Conexion WebSocket ----------
  const statusEl = document.getElementById('status');
  let ws=null, retry=0;
  function connect(){
    ws = new WebSocket('ws://' + location.host + '/ws');
    ws.binaryType = 'arraybuffer';
    ws.onopen    = ()=>{ statusEl.textContent='Conectado'; statusEl.className='ok'; retry=0; };
    ws.onclose   = ()=>{ statusEl.textContent='Reconectando...'; statusEl.className='bad';
                         setTimeout(connect, Math.min(3000, 300+retry*300)); retry++; };
    ws.onerror   = ()=>{ try{ws.close();}catch(e){} };
    ws.onmessage = onMsg;
  }
  connect();

  // ---------- Colormap "iron-like" ----------
  const STOPS = [
    [0.00, 0x00,0x00,0x10],
    [0.15, 0x20,0x00,0x60],
    [0.35, 0x60,0x10,0xa0],
    [0.55, 0xd0,0x30,0x60],
    [0.75, 0xff,0x80,0x20],
    [0.90, 0xff,0xd0,0x00],
    [1.00, 0xff,0xff,0xff],
  ];
  function cmap(t){
    if(t<=0) return STOPS[0];
    if(t>=1) return STOPS[STOPS.length-1];
    for(let i=1;i<STOPS.length;i++){
      if(t<=STOPS[i][0]){
        const a=STOPS[i-1], b=STOPS[i];
        const k=(t-a[0])/(b[0]-a[0]);
        return [a[1]+(b[1]-a[1])*k, a[2]+(b[2]-a[2])*k, a[3]+(b[3]-a[3])*k];
      }
    }
    return STOPS[STOPS.length-1];
  }

  // ---------- Render termica ----------
  const thermCv = document.getElementById('therm');
  const tctx = thermCv.getContext('2d');
  const offCv = document.createElement('canvas');
  offCv.width=32; offCv.height=24;
  const octx = offCv.getContext('2d');
  const img  = octx.createImageData(32,24);
  tctx.imageSmoothingEnabled = true;
  tctx.imageSmoothingQuality = 'high';

  function drawTherm(pix, tmin, tmax, tcen){
    for(let y=0;y<24;y++){
      for(let x=0;x<32;x++){
        const src = y*32 + (31-x);
        const t = pix[src]/255;
        const [r,g,b] = cmap(t);
        const di = (y*32 + x)*4;
        img.data[di]=r; img.data[di+1]=g; img.data[di+2]=b; img.data[di+3]=255;
      }
    }
    octx.putImageData(img,0,0);
    tctx.clearRect(0,0,thermCv.width,thermCv.height);
    tctx.drawImage(offCv,0,0,thermCv.width,thermCv.height);

    tctx.strokeStyle='rgba(255,255,255,.7)';
    tctx.lineWidth=1;
    const cx=thermCv.width/2, cy=thermCv.height/2;
    tctx.beginPath();
    tctx.moveTo(cx-8,cy); tctx.lineTo(cx+8,cy);
    tctx.moveTo(cx,cy-8); tctx.lineTo(cx,cy+8);
    tctx.stroke();

    document.getElementById('thMin').textContent=tmin.toFixed(1)+' °C';
    document.getElementById('thMax').textContent=tmax.toFixed(1)+' °C';
    document.getElementById('thCen').textContent=tcen.toFixed(1)+' °C';
    document.getElementById('lmin').textContent=tmin.toFixed(1)+' °C';
    document.getElementById('lmax').textContent=tmax.toFixed(1)+' °C';
  }

  // ---------- Render magnetometro ----------
  const magCv = document.getElementById('mag');
  const mctx  = magCv.getContext('2d');
  let magScale = 60;

  function drawMag(x,y,z){
    const W=magCv.width, H=magCv.height, cx=W/2, cy=H/2;
    mctx.fillStyle='#000';
    mctx.fillRect(0,0,W,H);
    mctx.strokeStyle='#1f2937';
    mctx.lineWidth=1;
    for(let r=40;r<=cx-10;r+=40){
      mctx.beginPath(); mctx.arc(cx,cy,r,0,Math.PI*2); mctx.stroke();
    }
    mctx.beginPath();
    mctx.moveTo(0,cy); mctx.lineTo(W,cy);
    mctx.moveTo(cx,0); mctx.lineTo(cx,H);
    mctx.stroke();
    mctx.fillStyle='#94a3b8';
    mctx.font='12px monospace';
    mctx.textAlign='center';
    mctx.fillText('+Y', cx, 14);
    mctx.fillText('-Y', cx, H-4);
    mctx.fillText('+X', W-12, cy+4);
    mctx.fillText('-X', 12, cy+4);

    const mag2D = Math.hypot(x,y);
    if(mag2D*1.4 > magScale) magScale = mag2D*1.4;
    const px = cx + (x/magScale)*(cx-20);
    const py = cy - (y/magScale)*(cy-20);

    mctx.strokeStyle='#38bdf8';
    mctx.lineWidth=2;
    mctx.beginPath();
    mctx.moveTo(cx,cy); mctx.lineTo(px,py); mctx.stroke();
    mctx.fillStyle='#38bdf8';
    mctx.beginPath();
    mctx.arc(px,py,5,0,Math.PI*2);
    mctx.fill();

    const zmax = magScale;
    const zh = Math.max(-1,Math.min(1, z/zmax))*(cy-20);
    mctx.fillStyle='#22c55e';
    mctx.fillRect(8, cy, 6, -zh);
    mctx.fillStyle='#94a3b8';
    mctx.textAlign='left';
    mctx.fillText('Z', 8, 14);

    document.getElementById('mx').textContent=x.toFixed(2);
    document.getElementById('my').textContent=y.toFixed(2);
    document.getElementById('mz').textContent=z.toFixed(2);
    const heading = ((Math.atan2(x,y)*180/Math.PI)+360)%360;
    document.getElementById('mh').textContent=heading.toFixed(1)+'°';
    document.getElementById('mm').textContent=Math.hypot(x,y,z).toFixed(2);
  }

  // ---------- Motores VESC ----------
  const motorsRoot = document.getElementById('motors');
  // Construir 6 tarjetas vacias (se hidratan con cada msg)
  const cards = []; // {root, els...}
  function buildCards(initial){
    motorsRoot.innerHTML = '';
    cards.length = 0;
    initial.forEach((m, i) => {
      const el = document.createElement('div');
      el.className = 'motor stale';
      el.innerHTML = `
        <div class="name"><span class="dot"></span><span class="nm"></span></div>
        <div class="id">ID <span class="cid"></span> &middot; <span class="age">--</span></div>
        <div class="rpm"><span class="r">--</span><small> RPM</small></div>
        <div class="bar"><div></div></div>
        <div class="ibar"><div></div></div>
        <div class="barlab"><span>Corriente motor</span><b class="im">--</b></div>
        <div class="row"><span>Duty</span><b class="d">--</b></div>
        <div class="row"><span>I batería</span><b class="iin">--</b></div>
        <div class="row"><span>Temp FET</span><b class="tf tcolor">--</b></div>
      `;
      motorsRoot.appendChild(el);
      cards.push({
        root: el,
        nm:  el.querySelector('.nm'),
        cid: el.querySelector('.cid'),
        age: el.querySelector('.age'),
        r:   el.querySelector('.r'),
        bar: el.querySelector('.bar'),
        barFill: el.querySelector('.bar > div'),
        ibarFill: el.querySelector('.ibar > div'),
        d:   el.querySelector('.d'),
        im:  el.querySelector('.im'),
        iin: el.querySelector('.iin'),
        tf:  el.querySelector('.tf')
      });
      cards[i].cid.textContent = m.id;
    });
  }

  // Mapeo de rotacion 180° (frente<->atras, izq<->der). Es una involucion:
  // partner[partner[i]] === i. Indices: 0=LT 1=RT 2=FL 3=FR 4=RL 5=RR
  const INV_PARTNER = [1, 0, 5, 4, 3, 2];

  function tempClass(t){
    if(t < 50) return 't-ok';
    if(t < 70) return 't-warn';
    return 't-bad';
  }

  // Clase de color para la barra de corriente segun % del maximo
  function currentBarClass(pct){
    if(pct < 60) return '';       // verde (default)
    if(pct < 85) return 'warn';   // amarillo
    return 'bad';                 // rojo
  }

  function updateBattery(v_in){
    const fill = document.getElementById('batFill');
    const vEl  = document.getElementById('batV');
    const pEl  = document.getElementById('batPct');
    if(typeof v_in !== 'number' || v_in <= 0){
      vEl.textContent='--'; pEl.textContent='--';
      fill.style.width='0%'; fill.className='fill';
      return;
    }
    let pct = (v_in - CFG.VBAT_MIN) / (CFG.VBAT_MAX - CFG.VBAT_MIN) * 100;
    pct = Math.max(0, Math.min(100, pct));
    fill.style.width = pct + '%';
    // Para bateria: lleno=bien, vacio=mal
    fill.className = 'fill' + (pct < 20 ? ' bad' : (pct < 40 ? ' warn' : ''));
    vEl.textContent = v_in.toFixed(1);
    pEl.textContent = pct.toFixed(0);
  }

  function updateMotors(msg){
    if(!cards.length) buildCards(msg.motors);

    // ---- Modo invertido: banner + re-etiquetado ----
    const inverted = (msg.inverted === true);
    document.getElementById('invBanner').classList.toggle('on', inverted);

    // ---- Bateria unica (compartida) ----
    updateBattery(msg.v_in);

    // ---- Totales del bus ----
    let iSum = 0;
    msg.motors.forEach(m => { if(typeof m.i_in === 'number') iSum += m.i_in; });
    document.getElementById('ibus').textContent = iSum.toFixed(1);
    document.getElementById('pbus').textContent = (iSum * (msg.v_in||0)).toFixed(0);

    msg.motors.forEach((m, i) => {
      const c = cards[i];
      if(!c) return;

      // Nombre segun orientacion actual: en modo invertido cada motor
      // muestra el rol que cumple tras la rotacion de 180°.
      const nameIdx = inverted ? INV_PARTNER[i] : i;
      c.nm.textContent = msg.motors[nameIdx].name;
      c.root.classList.toggle('inv', inverted);

      const stale = (m.age == null) || (m.age > 1500);
      c.root.classList.toggle('stale', stale);
      c.age.textContent = (m.age == null) ? 'sin datos' :
                          (m.age < 1000 ? `${m.age} ms` : `${(m.age/1000).toFixed(1)} s`);
      if(stale) return; // mantener ultimos valores pero atenuados

      // RPM real = ERPM / pole_pairs
      const rpm = m.erpm / CFG.POLE_PAIRS;
      c.r.textContent   = rpm.toFixed(0);

      const dutyPct = (m.duty*100);
      c.d.textContent   = dutyPct.toFixed(0)+'%';
      c.im.textContent  = m.i_m.toFixed(2)+' A';
      c.iin.textContent = m.i_in.toFixed(2)+' A';
      c.tf.textContent  = m.t_f.toFixed(1)+' °C';
      c.tf.className = 'tf tcolor ' + tempClass(m.t_f);

      // Barra de RPM (escala 0..RPM_MAX). Púrpura si gira en reversa.
      const rpmPct = Math.min(100, Math.abs(rpm) / CFG.RPM_MAX * 100);
      c.barFill.style.width = rpmPct + '%';
      c.bar.classList.toggle('neg', rpm < 0);

      // Barra de corriente del motor (verde/amarillo/rojo, escala 0..IMOTOR_MAX)
      const iPct = Math.min(100, Math.abs(m.i_m) / CFG.IMOTOR_MAX * 100);
      c.ibarFill.style.width = iPct + '%';
      c.ibarFill.className = currentBarClass(iPct);
    });
  }

  // ---------- Sensor de gas MQ-2 ----------
  function updateGas(ppm){
    const fill  = document.getElementById('gasFill');
    const val   = document.getElementById('gasPpm');
    const state = document.getElementById('gasState');
    if(typeof ppm !== 'number' || ppm < 0) ppm = 0;

    val.textContent = ppm.toFixed(0);
    const pct = Math.min(100, ppm / CFG.GAS_MAX * 100);
    fill.style.width = pct + '%';

    let cls = '', txt = '✓ NORMAL';
    if(ppm >= CFG.GAS_ALERT){ cls = 'bad';  txt = '⚠ ALERTA'; }
    else if(ppm >= CFG.GAS_WARN){ cls = 'warn'; txt = '! ATENCIÓN'; }
    fill.className  = 'fill'  + (cls ? ' ' + cls : '');
    state.className = 'state' + (cls ? ' ' + cls : '');
    state.textContent = txt;
  }

  // ---------- Dispatcher ----------
  function onMsg(ev){
    if(typeof ev.data === 'string'){
      try{
        const m = JSON.parse(ev.data);
        if(m.type==='mag')  drawMag(m.x, m.y, m.z);
        if(m.type==='vesc') updateMotors(m);
        if(m.type==='gas')  updateGas(m.ppm);
      }catch(e){}
      return;
    }
    const dv = new DataView(ev.data);
    if(dv.byteLength < 13) return;
    if(dv.getUint8(0) !== 0x01) return;
    const tmin = dv.getFloat32(1,true);
    const tmax = dv.getFloat32(5,true);
    const tcen = dv.getFloat32(9,true);
    const pix  = new Uint8Array(ev.data, 13, 768);
    drawTherm(pix, tmin, tmax, tcen);
  }

  // =====================================================================
  // ============ FPV: webcam local del navegador ========================
  // =====================================================================
  // El receptor USB se ve como webcam del SO. Pedimos getUserMedia y la
  // pintamos en un <video>. Esto NO toca el ESP32 ni el WebSocket: el video
  // va del USB a la pantalla directamente, sin pasar por WiFi del robot.
  // =====================================================================
  const fpvDev   = document.getElementById('fpvDev');
  const fpvStart = document.getElementById('fpvStart');
  const fpvStop  = document.getElementById('fpvStop');
  const fpvFs    = document.getElementById('fpvFs');
  const fpvVid   = document.getElementById('fpvVid');
  const fpvWrap  = document.getElementById('fpvWrap');
  const fpvEmpty = document.getElementById('fpvEmpty');
  const fpvHud   = document.getElementById('fpvHud');
  const fpvRes   = document.getElementById('fpvRes');
  const fpvFps   = document.getElementById('fpvFps');
  const fpvErr   = document.getElementById('fpvErr');

  let fpvStream = null;
  let fpvFpsTimer = null;
  let fpvLastFrames = 0;

  const SAVED_DEV_KEY = 'fpv_device_id';

  function fpvShowError(msg){
    fpvErr.textContent = msg || '';
  }

  async function fpvListDevices(){
    if(!navigator.mediaDevices || !navigator.mediaDevices.enumerateDevices){
      fpvShowError('Este navegador no soporta acceso a cámara.');
      return;
    }
    try{
      const devs = await navigator.mediaDevices.enumerateDevices();
      const cams = devs.filter(d => d.kind === 'videoinput');
      fpvDev.innerHTML = '';
      if(cams.length === 0){
        fpvDev.innerHTML = '<option>— sin cámaras detectadas —</option>';
        return;
      }
      // Si los labels vienen vacios (permiso aun no dado), igual mostramos algo
      const saved = localStorage.getItem(SAVED_DEV_KEY);
      cams.forEach((c, i) => {
        const opt = document.createElement('option');
        opt.value = c.deviceId;
        opt.textContent = c.label || `Cámara ${i+1}`;
        if(c.deviceId === saved) opt.selected = true;
        fpvDev.appendChild(opt);
      });
    }catch(e){
      fpvShowError('No se pudieron listar cámaras: ' + e.message);
    }
  }

  async function fpvStartStream(){
    fpvShowError('');
    if(fpvStream) fpvStopStream();
    const deviceId = fpvDev.value;
    const constraints = {
      audio: false,
      video: deviceId ? { deviceId: { exact: deviceId } } : true
    };
    try{
      fpvStream = await navigator.mediaDevices.getUserMedia(constraints);
      fpvVid.srcObject = fpvStream;
      fpvVid.style.display = 'block';
      fpvEmpty.style.display = 'none';
      fpvHud.classList.add('on');
      fpvStart.style.display = 'none';
      fpvStop.style.display  = '';
      localStorage.setItem(SAVED_DEV_KEY, deviceId || '');
      // Refrescar lista ahora que tenemos permiso (los labels aparecen)
      fpvListDevices();

      fpvVid.onloadedmetadata = () => {
        fpvRes.textContent = `${fpvVid.videoWidth}×${fpvVid.videoHeight}`;
      };

      // FPS aproximado mirando rVFC si esta disponible
      if(typeof fpvVid.requestVideoFrameCallback === 'function'){
        const tick = () => {
          fpvLastFrames++;
          if(fpvStream) fpvVid.requestVideoFrameCallback(tick);
        };
        fpvVid.requestVideoFrameCallback(tick);
        clearInterval(fpvFpsTimer);
        fpvFpsTimer = setInterval(() => {
          fpvFps.textContent = fpvLastFrames;
          fpvLastFrames = 0;
        }, 1000);
      } else {
        fpvFps.textContent = '~';
      }
    }catch(e){
      let m = e.message || String(e);
      if(e.name === 'NotAllowedError') m = 'Permiso de cámara denegado. Acepta el permiso en el navegador y vuelve a intentar.';
      if(e.name === 'NotFoundError')  m = 'No se encontró esa cámara. Verifica que el receptor esté conectado.';
      if(e.name === 'NotReadableError') m = 'La cámara está en uso por otra aplicación. Ciérrala y reintenta.';
      fpvShowError('Error: ' + m);
    }
  }

  function fpvStopStream(){
    if(fpvStream){
      fpvStream.getTracks().forEach(t => t.stop());
      fpvStream = null;
    }
    fpvVid.srcObject = null;
    fpvVid.style.display = 'none';
    fpvEmpty.style.display = '';
    fpvHud.classList.remove('on');
    fpvStart.style.display = '';
    fpvStop.style.display  = 'none';
    clearInterval(fpvFpsTimer);
    fpvFpsTimer = null;
    fpvLastFrames = 0;
  }

  fpvStart.addEventListener('click', fpvStartStream);
  fpvStop.addEventListener('click', fpvStopStream);
  fpvFs.addEventListener('click', () => {
    if(document.fullscreenElement){
      document.exitFullscreen();
    } else if(fpvWrap.requestFullscreen){
      fpvWrap.requestFullscreen();
    }
  });
  document.addEventListener('fullscreenchange', () => {
    fpvWrap.classList.toggle('fs', !!document.fullscreenElement);
  });

  // Si el usuario cambia de cámara con el stream activo, reiniciar
  fpvDev.addEventListener('change', () => {
    if(fpvStream) fpvStartStream();
  });

  // Detectar conexion/desconexion de webcams en caliente
  if(navigator.mediaDevices && navigator.mediaDevices.addEventListener){
    navigator.mediaDevices.addEventListener('devicechange', fpvListDevices);
  }

  fpvListDevices();
})();
</script>
</body>
</html>
)HTML";

// ---- Eventos del WebSocket (solo log) ----
void onWsEvent(AsyncWebSocket * /*server*/, AsyncWebSocketClient *client,
               AwsEventType type, void * /*arg*/, uint8_t * /*data*/, size_t /*len*/)
{
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WIFI] WS cliente #%u conectado (%s)\n",
                  client->id(), client->remoteIP().toString().c_str());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WIFI] WS cliente #%u desconectado\n", client->id());
  }
}

void setupWiFiAndServer()
{
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 4);
  if (!ok) {
    Serial.println("[WIFI] softAP() FAIL");
    return;
  }
  delay(100);
  Serial.print("[WIFI] AP '");
  Serial.print(AP_SSID);
  Serial.print("' OK | IP: ");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send_P(200, "text/html; charset=utf-8", index_html);
  });
  server.onNotFound([](AsyncWebServerRequest *req){
    req->send(404, "text/plain", "Not found");
  });

  server.begin();
  Serial.println("[WIFI] HTTP + WS server listo en /  y  /ws");
}

// ====================== NUEVO: BROADCAST TELEMETRIA VESC ======================
// Se llama desde sensorsTask (Core 0). Lee los volatiles que escribe Core 1.
static void broadcastVescTelemetry()
{
  if (ws.count() == 0) return;

  // Voltaje del bus: tomar el de cualquier VESC reciente
  float v_in = 0.0f;
  uint32_t nowMs = millis();
  for (int i = 0; i < 6; i++) {
    if (tele[i].lastUpdateMs != 0 && (nowMs - tele[i].lastUpdateMs) < 1000) {
      if (tele[i].voltage_in > v_in) v_in = tele[i].voltage_in;
    }
  }

  // Buffer suficiente para 6 motores (~500-600 bytes)
  static char buf[900];
  int n = 0;
  n += snprintf(buf + n, sizeof(buf) - n,
                "{\"type\":\"vesc\",\"v_in\":%.1f,\"inverted\":%s,\"motors\":[",
                v_in, g_invertFront ? "true" : "false");

  for (int i = 0; i < 6; i++) {
    if (i > 0) n += snprintf(buf + n, sizeof(buf) - n, ",");

    // Snapshot local para reducir lecturas volatiles consecutivas
    // Nota: enviamos ERPM crudo; la conversion a RPM real (/pole_pairs)
    // se hace en el navegador para poder ajustarla sin reflashear.
    // No enviamos temp_motor (no hay sensor) ni v_in por motor
    // (la bateria es compartida, va una sola en el nivel superior).
    float    erpm = tele[i].erpm;
    float    duty = tele[i].duty;
    float    im   = tele[i].current_motor;
    float    iin  = tele[i].current_in;
    float    tf   = tele[i].temp_fet;
    uint32_t lu   = tele[i].lastUpdateMs;

    long age = (lu == 0) ? -1 : (long)(nowMs - lu);

    n += snprintf(buf + n, sizeof(buf) - n,
        "{\"id\":%u,\"name\":\"%s\",\"erpm\":%.0f,\"duty\":%.3f,"
        "\"i_m\":%.2f,\"i_in\":%.2f,\"t_f\":%.1f,\"age\":%ld}",
        vescIds[i], vescNames[i], erpm, duty, im, iin, tf, age);

    if (n >= (int)sizeof(buf) - 16) break; // proteccion overflow
  }
  n += snprintf(buf + n, sizeof(buf) - n, "]}");
  if (n > 0 && n < (int)sizeof(buf)) {
    ws.textAll(buf, (size_t)n);
  }
}
// ====================== FIN BROADCAST VESC ======================


void resetI2CBus()
{
  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, HIGH);

  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
  }

  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(5);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(100);
}

void calibrarMag()
{
  Serial.println("[SENS] Calibrando magnetometro...");
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 50; i++) {
    sensors_event_t e;
    lis3mdl.getEvent(&e);
    sx += e.magnetic.x;
    sy += e.magnetic.y;
    sz += e.magnetic.z;
    delay(10);
  }
  mag_x0 = sx / 50;
  mag_y0 = sy / 50;
  mag_z0 = sz / 50;
  Serial.println("[SENS] Calibracion OK");
}

void sensorsTask(void *param)
{
  resetI2CBus();

  Serial.println("[SENS] Conectando LIS3MDL...");
  while (!lis3mdl.begin_I2C()) {
    Serial.println("[SENS]   LIS3MDL no encontrado, reintentando...");
    resetI2CBus();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  Serial.println("[SENS] LIS3MDL OK");
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_16_GAUSS);
  calibrarMag();

  Serial.println("[SENS] Conectando MLX90640...");
  while (!mlx.begin(0x33, &Wire)) {
    Serial.println("[SENS]   MLX no encontrado, reintentando...");
    resetI2CBus();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  Serial.println("[SENS] MLX90640 OK");
  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_4_HZ);

  g_sensorsReady = true;
  Serial.println("[SENS] Sistema de sensores listo");

  uint32_t lastMag           = 0;
  uint32_t lastPrint         = 0;
  uint32_t lastMagBroadcast  = 0;
  uint32_t lastVescBroadcast = 0;   // NUEVO
  uint32_t lastMq2Read       = 0;   // NUEVO: muestreo MQ-2
  uint32_t lastMq2Broadcast  = 0;   // NUEVO: envio a la GUI
  uint32_t lastWsCleanup     = 0;

  static uint8_t thermMsg[13 + 768];

  for (;;)
  {
    uint32_t now = millis();

    // --- Magnetometro ~20 Hz ---
    if (now - lastMag >= 50) {
      lastMag = now;
      sensors_event_t e;
      lis3mdl.getEvent(&e);
      g_mag_dx = e.magnetic.x - mag_x0;
      g_mag_dy = e.magnetic.y - mag_y0;
      g_mag_dz = e.magnetic.z - mag_z0;

      // broadcast cada 100ms (10 Hz)
      if (ws.count() > 0 && (now - lastMagBroadcast) >= 100) {
        lastMagBroadcast = now;
        char buf[96];
        float lx = g_mag_dx, ly = g_mag_dy, lz = g_mag_dz;
        int n = snprintf(buf, sizeof(buf),
          "{\"type\":\"mag\",\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}", lx, ly, lz);
        if (n > 0) ws.textAll(buf, (size_t)n);
      }
    }

    // --- NUEVO: Broadcast VESC cada 200ms (5 Hz) ---
    if (now - lastVescBroadcast >= 200) {
      lastVescBroadcast = now;
      broadcastVescTelemetry();
    }

    // --- NUEVO: MQ-2 cada 200ms (5 Hz) ---
    // Promedio rapido de 4 muestras consecutivas para suavizar ruido.
    // Sin delay() entre lecturas: ya es bastante lento el ADC del ESP32.
    if (now - lastMq2Read >= 200) {
      lastMq2Read = now;
      uint32_t sum = 0;
      for (int i = 0; i < 4; i++) sum += analogRead(MQ2_PIN);
      int raw = (int)(sum / 4);
      float ppm = ((float)(raw - MQ2_CLEAN_ADC) /
                   (float)(MQ2_MAX_ADC - MQ2_CLEAN_ADC)) * MQ2_MAX_PPM;
      if (ppm < 0)            ppm = 0;
      if (ppm > MQ2_MAX_PPM)  ppm = MQ2_MAX_PPM;
      g_mq2_ppm = ppm;
    }

    // --- NUEVO: Broadcast MQ-2 cada 500ms (2 Hz, suficiente para gas) ---
    if (ws.count() > 0 && (now - lastMq2Broadcast) >= 500) {
      lastMq2Broadcast = now;
      char buf[64];
      float ppm = g_mq2_ppm;  // snapshot del volatile
      int n = snprintf(buf, sizeof(buf),
        "{\"type\":\"gas\",\"ppm\":%.0f}", ppm);
      if (n > 0) ws.textAll(buf, (size_t)n);
    }

    // --- Camara termica ---
    if (mlx.getFrame(frame) == 0) {
      float mn = frame[0], mx = frame[0];
      for (int i = 1; i < 768; i++) {
        if (frame[i] < mn) mn = frame[i];
        if (frame[i] > mx) mx = frame[i];
      }
      g_thermMin    = mn;
      g_thermMax    = mx;
      g_thermCenter = frame[383];

      if (ws.count() > 0) {
        thermMsg[0] = 0x01;
        memcpy(&thermMsg[1], &mn, 4);
        memcpy(&thermMsg[5], &mx, 4);
        float center = frame[383];
        memcpy(&thermMsg[9], &center, 4);

        float range = mx - mn;
        if (range < 0.01f) range = 0.01f;
        const float inv = 255.0f / range;
        for (int i = 0; i < 768; i++) {
          float t = (frame[i] - mn) * inv;
          if (t < 0)   t = 0;
          if (t > 255) t = 255;
          thermMsg[13 + i] = (uint8_t)t;
        }
        ws.binaryAll(thermMsg, sizeof(thermMsg));
      }
    }

    // --- Imprimir cada 500ms ---
    if (now - lastPrint >= 500) {
      lastPrint = now;
      Serial.print("[SENS] MAG: ");
      Serial.print(g_mag_dx, 2); Serial.print(", ");
      Serial.print(g_mag_dy, 2); Serial.print(", ");
      Serial.print(g_mag_dz, 2);
      Serial.print(" | THERM min/max/c: ");
      Serial.print(g_thermMin, 1);    Serial.print("/");
      Serial.print(g_thermMax, 1);    Serial.print("/");
      Serial.print(g_thermCenter, 1);
      Serial.print(" | MQ2: ");
      Serial.print(g_mq2_ppm, 0);
      Serial.println(" ppm");

      // NUEVO: pequeno dump de telemetria VESC al serial
      Serial.print("[VESC] ");
      for (int i = 0; i < 6; i++) {
        long age = (tele[i].lastUpdateMs == 0)
                   ? -1 : (long)(now - tele[i].lastUpdateMs);
        Serial.printf("[%u] %.0frpm %.2fA %.1fC age=%ldms | ",
                      vescIds[i], tele[i].erpm,
                      tele[i].current_motor, tele[i].temp_fet, age);
      }
      Serial.println();
    }

    if (now - lastWsCleanup >= 2000) {
      lastWsCleanup = now;
      ws.cleanupClients();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(BAUD_RATE);
  delay(500);

  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);

  Serial.println("Rescue Robot - Drive + Flippers + Sensors + WiFi + VESC Telemetry");

  SPI.begin(18, 19, 23, PIN_CS);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("CAN init done");

  // NUEVO: configurar ADC para el MQ-2
  // 12 bits (0..4095) + atenuacion 11dB para rango pleno ~0..3.3V
  pinMode(MQ2_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(MQ2_PIN, ADC_11db);
  Serial.printf("MQ2 init on GPIO %d\n", MQ2_PIN);

  // Inicializar tabla de telemetria a cero
  for (int i = 0; i < 6; i++) {
    tele[i].erpm = 0; tele[i].duty = 0;
    tele[i].current_motor = 0; tele[i].current_in = 0;
    tele[i].voltage_in = 0;
    tele[i].temp_fet = 0; tele[i].temp_motor = 0;
    tele[i].lastUpdateMs = 0;
  }

  setupWiFiAndServer();

  xTaskCreatePinnedToCore(
    sensorsTask,
    "sensorsTask",
    8192,
    NULL,
    1,
    NULL,
    0
  );

  Serial.println("Sensor task lanzada en Core 0");
}

// -------------------- LOOP (corre en Core 1) --------------------
void loop() {
  static uint32_t lastCanMs = 0;
  static bool inFailsafe = false;

  static int32_t rpmLeftTrack  = 0;
  static int32_t rpmRightTrack = 0;
  static int32_t rpmFrontLeft  = 0;
  static int32_t rpmFrontRight = 0;
  static int32_t rpmRearLeft   = 0;
  static int32_t rpmRearRight  = 0;

  uint32_t nowMs = millis();

  if (nowMs - lastCanMs >= CAN_PERIOD_MS)
  {
    lastCanMs = nowMs;

    uint32_t ageMs = (micros() - lastFrameMicros) / 1000;

    // ---- FAILSAFE ----
    if (ageMs > FAILSAFE_MS)
    {
      sendVescRPM(VESC_LEFT_TRACK,  0); delayMicroseconds(500);
      sendVescRPM(VESC_RIGHT_TRACK, 0); delayMicroseconds(500);
      sendVescRPM(VESC_FRONT_LEFT,  0); delayMicroseconds(500);
      sendVescRPM(VESC_FRONT_RIGHT, 0); delayMicroseconds(500);
      sendVescRPM(VESC_REAR_LEFT,   0); delayMicroseconds(500);
      sendVescRPM(VESC_REAR_RIGHT,  0);

      rpmLeftTrack = rpmRightTrack = 0;
      rpmFrontLeft = rpmFrontRight = 0;
      rpmRearLeft  = rpmRearRight  = 0;

      if (!inFailsafe) {
        Serial.println("FAILSAFE -> 0 RPM all motors");
        inFailsafe = true;
      }

      // Aun en failsafe seguimos drenando RX para no perder telemetria
      drainCanRx();
      return;
    }
    else
    {
      inFailsafe = false;
    }

    // ---- READ PPM CHANNELS ----
    uint32_t ch1, ch2, ch3, ch4, ch5, ch6;

    noInterrupts();
    ch1 = ppmValues[0];
    ch2 = ppmValues[1];
    ch3 = ppmValues[2];
    ch4 = ppmValues[3];
    ch5 = ppmValues[4];
    ch6 = ppmValues[5];
    interrupts();

    // ---- DIFFERENTIAL DRIVE MIXING ----
    float throttle = ppmToNormalized((int)ch3);
    float turn     = ppmToNormalized((int)ch4);

    bool invertFront = ((int)ch6 > 1500);
    g_invertFront = invertFront;  // NUEVO: exponer a la interfaz

    if (invertFront){
      throttle = -throttle;
    }

    float leftNorm  = clampFloat(throttle + turn, -1.0f, 1.0f);
    float rightNorm = clampFloat(throttle - turn, -1.0f, 1.0f);

    int32_t targetLeftTrack  = (int32_t)(leftNorm  * RPM_MAX_DRIVE);
    int32_t targetRightTrack = (int32_t)(rightNorm * RPM_MAX_DRIVE);

    // ---- FLIPPER LOGIC ----
    float ch2Norm = ppmToNormalized((int)ch2);
    float ch1Norm = ppmToNormalized((int)ch1);

    float flipLeftNorm, flipRightNorm;

    if (ch1Norm == 0.0f) {
      flipLeftNorm  = ch2Norm;
      flipRightNorm = ch2Norm;
    } else if (ch1Norm < 0.0f) {
      flipLeftNorm  = ch2Norm;
      flipRightNorm = 0.0f;
    } else {
      flipLeftNorm  = 0.0f;
      flipRightNorm = ch2Norm;
    }

    int32_t flipLeftRPM  = (int32_t)(flipLeftNorm  * RPM_MAX_FLIPPER);
    int32_t flipRightRPM = (int32_t)(flipRightNorm * RPM_MAX_FLIPPER);

    bool frontActive = ((int)ch5 < 1500);

    int32_t logicalFrontLeft  = frontActive ? flipLeftRPM  : 0;
    int32_t logicalFrontRight = frontActive ? flipRightRPM : 0;
    int32_t logicalRearLeft   = frontActive ? 0 : flipLeftRPM;
    int32_t logicalRearRight  = frontActive ? 0 : flipRightRPM;

    int32_t targetFrontLeft, targetFrontRight, targetRearLeft, targetRearRight;

    if (invertFront) {
      targetFrontLeft  = -logicalRearRight;
      targetFrontRight = -logicalRearLeft;
      targetRearLeft   = -logicalFrontRight;
      targetRearRight  = -logicalFrontLeft;
    } else {
      targetFrontLeft  = logicalFrontLeft;
      targetFrontRight = logicalFrontRight;
      targetRearLeft   = logicalRearLeft;
      targetRearRight  = logicalRearRight;
    }

    // ---- RAMP ALL MOTORS ----
    rpmLeftTrack  = rampRPM(rpmLeftTrack,  targetLeftTrack);
    rpmRightTrack = rampRPM(rpmRightTrack, targetRightTrack);
    rpmFrontLeft  = rampRPM(rpmFrontLeft,  targetFrontLeft);
    rpmFrontRight = rampRPM(rpmFrontRight, targetFrontRight);
    rpmRearLeft   = rampRPM(rpmRearLeft,   targetRearLeft);
    rpmRearRight  = rampRPM(rpmRearRight,  targetRearRight);

    // ---- SEND ALL CAN FRAMES ----
    sendVescRPM(VESC_LEFT_TRACK,  rpmLeftTrack);  delayMicroseconds(500);
    sendVescRPM(VESC_RIGHT_TRACK, rpmRightTrack); delayMicroseconds(500);
    sendVescRPM(VESC_FRONT_LEFT,  rpmFrontLeft);  delayMicroseconds(500);
    sendVescRPM(VESC_FRONT_RIGHT, rpmFrontRight); delayMicroseconds(500);
    sendVescRPM(VESC_REAR_LEFT,   rpmRearLeft);   delayMicroseconds(500);
    sendVescRPM(VESC_REAR_RIGHT,  rpmRearRight);

    // ---- SERIAL DEBUG ----
    Serial.print("[MOT] DRIVE L:");
    Serial.print(rpmLeftTrack);
    Serial.print(" R:");
    Serial.print(rpmRightTrack);
    Serial.print(" || ");
    Serial.print(frontActive ? "FRONT" : "REAR");
    Serial.print(" FL:");
    Serial.print(frontActive ? rpmFrontLeft : rpmRearLeft);
    Serial.print(" FR:");
    Serial.print(frontActive ? rpmFrontRight : rpmRearRight);
    Serial.print(" || ch1:");
    Serial.print(ch1);
    Serial.print(" ch2:");
    Serial.print(ch2);
    Serial.print(" ch3:");
    Serial.print(ch3);
    Serial.print(" ch4:");
    Serial.print(ch4);
    Serial.print(" ch5:");
    Serial.println(ch5);

    // ---- NUEVO: vaciar el buffer RX del MCP2515 ----
    // Procesa los status frames que mandaron los VESC durante el ciclo.
    // Ocurre DESPUES de los sends para no retrasarlos nunca.
    drainCanRx();
  }
}