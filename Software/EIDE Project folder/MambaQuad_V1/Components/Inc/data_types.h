#pragma once

#include <stdint.h>

typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    uint32_t timestamp_ms;
} IMUData_t;

typedef struct {
    float pressure_Pa;
    float altitude_m;
    uint32_t timestamp_ms;
} BaroData_t;

typedef struct {
    float mx;
    float my;
    float mz;
    uint32_t timestamp_ms;
} MagData_t;

typedef struct {
    uint8_t data[256];
    uint16_t length;
    uint32_t timestamp_ms;
} LoraMessage_t;

typedef struct {
    double lat;
    double lon;
    double elv;

    double speed;
    double dir;

    int year, month, day;
    int hour, min, sec;

    int satInUse;
    int satInView;

    uint32_t timestamp_ms;
} GPSData_t;
