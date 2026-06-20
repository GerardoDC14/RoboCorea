#include "Sensors.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

uint8_t Sensors::s_mask   = 0;
MagData Sensors::s_mag     = {};
bool    Sensors::s_mag_ok  = false;

// Short critical sections for the data fields.
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static Adafruit_LIS3MDL s_lis;

bool Sensors::begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);  // 400 kHz fast-mode

    // LIS3MDL magnetometer (Adafruit driver probes its default I2C address).
    if (s_lis.begin_I2C()) {
        s_lis.setPerformanceMode(LIS3MDL_MEDIUMMODE);
        s_lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);
        s_lis.setDataRate(LIS3MDL_DATARATE_155_HZ);
        s_lis.setRange(LIS3MDL_RANGE_16_GAUSS);  // 16 G avoids saturation near the BLDC motors
        s_mag_ok = true;
    }

    return true;  // non-fatal: check s_mag_ok
}

static uint32_t s_last_mag_ms = 0;

uint8_t Sensors::runOnce() {
    uint32_t now  = millis();
    uint8_t  read = 0;

    if ((s_mask & SENSOR_BIT_MAG) && (now - s_last_mag_ms >= 1000 / SENSOR_MAG_HZ)) {
        s_last_mag_ms = now;
        readMag();
        read |= SENSOR_BIT_MAG;
    }
    return read;
}

void Sensors::setEnabledMask(uint8_t mask) {
    portENTER_CRITICAL(&s_mux);
    s_mask = mask;
    portEXIT_CRITICAL(&s_mux);
}

uint8_t Sensors::getEnabledMask() {
    portENTER_CRITICAL(&s_mux);
    uint8_t m = s_mask;
    portEXIT_CRITICAL(&s_mux);
    return m;
}

void Sensors::getMag(MagData& out) {
    portENTER_CRITICAL(&s_mux);
    out = s_mag;
    portEXIT_CRITICAL(&s_mux);
}

void Sensors::readMag() {
    if (!s_mag_ok) return;
    sensors_event_t e;
    s_lis.getEvent(&e);          // magnetic field in µT (1 G = 100 µT)
    MagData d;
    d.x_uT  = (int)e.magnetic.x;
    d.y_uT  = (int)e.magnetic.y;
    d.z_uT  = (int)e.magnetic.z;
    d.valid = true;
    portENTER_CRITICAL(&s_mux);
    s_mag = d;
    portEXIT_CRITICAL(&s_mux);
}
