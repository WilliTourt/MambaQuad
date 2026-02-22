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