#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

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
static const uint8_t CMD_CUSTOM_ANGLE = 100;

// VESC CAN status command IDs
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
  if (err != MCP2515::ERROR_OK) {
    Serial.print("CAN send err to ID ");
    Serial.print(vescID);
    Serial.print(" = ");
    Serial.println((int)err);
  }
}

// =============================================================
// =============== TELEMETRIA VESC ======================
// =============================================================
struct VescTelemetry {
  volatile float    erpm;
  volatile float    duty;            // -1.0 .. 1.0
  volatile float    current_motor;   // A
  volatile float    current_in;      // A (batería)
  volatile float    voltage_in;      // V (batería)
  volatile float    temp_fet;        // °C
  volatile float    temp_motor;      // °C
  // ¡AQUÍ ESTARÁ EL ESPACIO PARA GUARDAR EL TACOMETRO LUEGO!
  volatile uint32_t lastUpdateMs;    // 0 = nunca
};

// Indices fijos para los 6 motores
// 0=left track 1=right track 2=front left 3=front right 4=rear left 5=rear right
static VescTelemetry tele[6];

// IDs CAN de cada indice 
static const uint8_t vescIds[6] = {60, 50, 20, 10, 40, 30};

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

static void processVescStatus(const struct can_frame &rx)
{
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
      int16_t curMot = rdI16BE(&rx.data[4]);  
      int16_t duty1k = rdI16BE(&rx.data[6]);  
      tele[idx].erpm          = (float)erpm;
      tele[idx].current_motor = curMot / 10.0f;
      tele[idx].duty          = duty1k / 1000.0f;
      tele[idx].lastUpdateMs  = nowMs;
    } break;

    case VESC_CMD_STATUS_4: {
      if (rx.can_dlc < 8) return;
      int16_t tFet = rdI16BE(&rx.data[0]); 
      int16_t tMot = rdI16BE(&rx.data[2]); 
      int16_t cIn  = rdI16BE(&rx.data[4]); 
      tele[idx].temp_fet      = tFet / 10.0f;
      tele[idx].temp_motor    = tMot / 10.0f;
      tele[idx].current_in    = cIn  / 10.0f;
      tele[idx].lastUpdateMs  = nowMs;
    } break;

    case VESC_CMD_STATUS_5: {
      if (rx.can_dlc < 6) return;
      int16_t vIn = rdI16BE(&rx.data[4]);
      tele[idx].voltage_in    = vIn / 10.0f;
      tele[idx].lastUpdateMs  = nowMs;
      // AQUÍ PROCESAREMOS EL TACÓMETRO MAS ADELANTE
    } break;

    default:
      break;
  }
}

static void drainCanRx()
{
  static const int MAX_FRAMES = 16;
  struct can_frame rx;
  for (int i = 0; i < MAX_FRAMES; i++) {
    if (mcp2515.readMessage(&rx) != MCP2515::ERROR_OK) break;
    processVescStatus(rx);
  }
}

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
    return (float)(ppm - PPM_DEADBAND_HIGH) / (float)(PPM_MAX_US - PPM_DEADBAND_HIGH);
  } else {
    return -(float)(PPM_DEADBAND_LOW - ppm) / (float)(PPM_DEADBAND_LOW - PPM_MIN_US);
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

Adafruit_LIS3MDL lis3mdl;

float mag_x0 = 0, mag_y0 = 0, mag_z0 = 0;

// Variables compartidas entre cores
volatile float g_mag_dx = 0, g_mag_dy = 0, g_mag_dz = 0;
volatile bool  g_invertFront = false;

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
  Serial.println("[SENS] Calibracion MAG OK");
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
  
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_16_GAUSS);
  calibrarMag();

  Serial.println("[SENS] Sistema de sensores listo");

  uint32_t lastMag = 0;
  uint32_t lastTelemetryBroadcast = 0;

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
    }

    // --- Serial Broadcast a la Jetson (10 Hz) ---
    // Formato JSON puro para fácil parseo en Python con json.loads()
    if (now - lastTelemetryBroadcast >= 100) {
      lastTelemetryBroadcast = now;
      
      static char buf[1024]; // Buffer generoso
      int n = snprintf(buf, sizeof(buf), "{\"mag\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},\"inv\":%s,\"vesc\":[", 
                       g_mag_dx, g_mag_dy, g_mag_dz, g_invertFront ? "true" : "false");

      for (int i = 0; i < 6; i++) {
        if (i > 0) n += snprintf(buf + n, sizeof(buf) - n, ",");

        float erpm = tele[i].erpm;
        float duty = tele[i].duty;
        float im   = tele[i].current_motor;
        float iin  = tele[i].current_in;
        float tf   = tele[i].temp_fet;
        long age   = (tele[i].lastUpdateMs == 0) ? -1 : (long)(now - tele[i].lastUpdateMs);

        n += snprintf(buf + n, sizeof(buf) - n,
            "{\"id\":%u,\"erpm\":%.0f,\"duty\":%.3f,\"i_m\":%.2f,\"i_in\":%.2f,\"t_f\":%.1f,\"age\":%ld}",
            vescIds[i], erpm, duty, im, iin, tf, age);
      }
      
      n += snprintf(buf + n, sizeof(buf) - n, "]}");
      
      if (n > 0 && n < sizeof(buf)) {
        Serial.println(buf);
      }
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

  Serial.println("Rescue Robot - Drive + Flippers + Jetson Serial Telemetry");

  SPI.begin(18, 19, 23, PIN_CS);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("CAN init done");

  for (int i = 0; i < 6; i++) {
    tele[i].erpm = 0; tele[i].duty = 0;
    tele[i].current_motor = 0; tele[i].current_in = 0;
    tele[i].voltage_in = 0;
    tele[i].temp_fet = 0; tele[i].temp_motor = 0;
    tele[i].lastUpdateMs = 0;
  }

  xTaskCreatePinnedToCore(
    sensorsTask,
    "sensorsTask",
    4096,   // Reducimos RAM al no usar WiFi/Web
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

  static float accumFrontLeftAngle = 0.0f;
  static float accumFrontRightAngle = 0.0f;
  static float accumRearLeftAngle = 0.0f;
  static float accumRearRightAngle = 0.0f;

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
    g_invertFront = invertFront;  

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

    // ---- RAMP TRACTION ----
    rpmLeftTrack  = rampRPM(rpmLeftTrack,  targetLeftTrack);
    rpmRightTrack = rampRPM(rpmRightTrack, targetRightTrack);

    // ---- ANGLE ACCUMULATORS
    accumFrontLeftAngle -= (float)targetFrontLeft * 0.00015f;/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    accumFrontRightAngle += (float)targetFrontRight * 0.00015f;
    accumRearLeftAngle -= (float)targetRearLeft * 0.00015f;
    accumRearRightAngle += (float)targetRearRight * 0.00015f;

    // ---- SEND ALL CAN FRAMES ----
    sendVescRPM(VESC_LEFT_TRACK,  rpmLeftTrack);  delayMicroseconds(500);
    sendVescRPM(VESC_RIGHT_TRACK, rpmRightTrack); delayMicroseconds(500);
    
    sendVescRPM(VESC_FRONT_LEFT,  (int32_t)(accumFrontLeftAngle*1000.00f));  delayMicroseconds(500);/////////////////////////////////////////////////////////////////////////////////////////////////////
    sendVescRPM(VESC_FRONT_RIGHT,  (int32_t)(accumFrontRightAngle*1000.00f));  delayMicroseconds(500);
    sendVescRPM(VESC_REAR_LEFT,  (int32_t)(accumRearLeftAngle*1000.00f));  delayMicroseconds(500);
    sendVescRPM(VESC_REAR_RIGHT,  (int32_t)(accumRearRightAngle*1000.00f));
    

    drainCanRx();
  }
}
