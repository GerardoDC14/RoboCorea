#include "Sensors.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

uint8_t Sensors::s_mask   = 0;
MagData Sensors::s_mag     = {};
ImuData Sensors::s_imu     = {};
bool    Sensors::s_mag_ok  = false;
bool    Sensors::s_bno_ok  = false;

// Short critical sections for the data fields.
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static Adafruit_LIS3MDL s_lis;
static Adafruit_BNO055  s_bno(55, BNO055_I2C_ADDR);

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

    // BNO055 in NDOF fusion mode.
    if (s_bno.begin()) {
        s_bno.setExtCrystalUse(true);
        s_bno_ok = true;
    }

    return true;  // non-fatal: check s_*_ok individually
}

static uint32_t s_last_mag_ms = 0;
static uint32_t s_last_imu_ms = 0;

uint8_t Sensors::runOnce() {
    uint32_t now  = millis();
    uint8_t  read = 0;

    if ((s_mask & SENSOR_BIT_MAG) && (now - s_last_mag_ms >= 1000 / SENSOR_MAG_HZ)) {
        s_last_mag_ms = now;
        readMag();
        read |= SENSOR_BIT_MAG;
    }
    if ((s_mask & SENSOR_BIT_IMU) && (now - s_last_imu_ms >= 1000 / SENSOR_IMU_HZ)) {
        s_last_imu_ms = now;
        readImu();
        read |= SENSOR_BIT_IMU;
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

void Sensors::getImu(ImuData& out) {
    portENTER_CRITICAL(&s_mux);
    out = s_imu;
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

void Sensors::readImu() {
    if (!s_bno_ok) return;

    imu::Quaternion quat   = s_bno.getQuat();
    imu::Vector<3>  laccel = s_bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3>  gyro   = s_bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    uint8_t cal_sys = 0, cal_gyro = 0, cal_accel = 0, cal_mag = 0;
    s_bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag);

    imu::Vector<3> euler = quat.toEuler();

    ImuData d;
    d.yaw_deg   = euler.x() * (180.0 / M_PI);
    d.pitch_deg = euler.y() * (180.0 / M_PI);
    d.roll_deg  = euler.z() * (180.0 / M_PI);
    d.accel_x = laccel.x(); d.accel_y = laccel.y(); d.accel_z = laccel.z();
    d.gyro_x  = gyro.x();   d.gyro_y  = gyro.y();   d.gyro_z  = gyro.z();
    d.calib = (uint8_t)((cal_sys << 6) | (cal_gyro << 4) | (cal_accel << 2) | cal_mag);
    d.valid = true;

    portENTER_CRITICAL(&s_mux);
    s_imu = d;
    portEXIT_CRITICAL(&s_mux);
}
