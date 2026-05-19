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

typedef struct {
    // Header in sector 0
    uint32_t    magic;                  // 0x4D425144 "MBQD" MamBaQuaD     @ 0x000000

    uint32_t    pwr_on_cnt;             // 开机次数                         @ 0x000004
    uint32_t    total_flight_sec;       // 总飞行时间                       @ 0x000008
    uint32_t    total_flight_meters;    // 总飞行距离                       @ 0x00000C
    uint8_t     log_rate_Hz;            // 数据记录频率                     @ 0x000010
    uint32_t    write_ptr;              // 下次写入位置                     @ 0x000014
} FDRHeader_t;

typedef struct {
    // From Sensors
    float       accel[3], gyro[3];      // IMU
    float       mag[3];                 // 磁强
    float       pressure_Pa;            // 气压
    float       temp_C;                 // 温度

    // From GPS
    float       speed;              
    float       dir;
    float       lat, lon;           
    uint32_t    unix_time;

    // From ESC TX
    float       current[4];              // 电机电流
    float       voltage;                 // 电池电压
    uint16_t    motorRPM[4];             // 电机转速

    // From RF
    uint8_t     rf_rssi;                 // 信号强度
    uint8_t     rf_erssi;                // 信道噪声水平

    uint8_t     flags;                   // bit0=lock state, bit1=err, bit2=sens status, bit3=RF aux, bit4-7: reserved
    uint32_t    timestamp_ms;            // 上电后的毫秒
} __attribute__((packed)) FDRData_t;
