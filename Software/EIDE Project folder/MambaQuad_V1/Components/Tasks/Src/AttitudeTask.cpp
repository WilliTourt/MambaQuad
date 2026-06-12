#include "AttitudeTask.h"
#include "ESKF.h"
#include "DBGTask.h"

#include <cmath>

AttitudeTask::AttitudeTask(FreeRTOS::Queue<IMUData_t>  &imuQueue,
                           FreeRTOS::Queue<MagData_t>  &magQueue,
                           FreeRTOS::Queue<AttitudeData_t> &attQueue) :
    Task(tskIDLE_PRIORITY + 3, 1536, "ATTD"),
    _imuQueue(imuQueue),
    _magQueue(magQueue),
    _attQueue(attQueue) {
    _att = {};
    _lastIMUTime = 0;
}

void AttitudeTask::taskFunction() {
    DBGQ.sendToBack((uint8_t*)"AttitudeTask started (ESKF).", 0);

    static ESKF eskf;  // default noise params
    // uint32_t lastPrint = 0;

    for (;;) {
        // wait for next IMU sample
        auto imu = _imuQueue.receive(pdMS_TO_TICKS(3));
        if (!imu) {
            // timeout — publish stale attitude and keep going
            if (_lastIMUTime != 0) {
                _attQueue.sendToBack(_att, 0);
            }
            continue;
        }

        // compute dt
        float dt = 0.002f;  // fallback 500Hz implicit
        if (_lastIMUTime != 0) {
            uint32_t diff = imu->timestamp_ms - _lastIMUTime;
            if (diff > 0 && diff < 100) {
                dt = (float)diff / 1000.0f;
            }
        }
        _lastIMUTime = imu->timestamp_ms;

        // try to get latest magnetometer data (non-blocking)
        float mx = 0, my = 0, mz = 0;
        auto mag = _magQueue.receive(pdMS_TO_TICKS(0));
        if (mag) {
            mx = mag->mx;
            my = mag->my;
            mz = mag->mz;
        } else {
            // no mag data yet — skip mag correction in ESKF
            mx = my = mz = 0;
        }

        // gyro: convert deg/s → rad/s
        const float D2R = 3.14159265358979f / 180.0f;

        // run ESKF
        eskf.update(imu->ax, imu->ay, imu->az,                      // accel  m/s²
                     imu->gx * D2R, imu->gy * D2R, imu->gz * D2R,   // gyro rad/s
                     mx, my, mz,                                    // mag    Gauss
                     dt);                                           // dt     seconds

        // convert quaternion → Euler angles
        _att.roll  = eskf.getRoll();
        _att.pitch = eskf.getPitch();
        _att.yaw   = eskf.getYaw();

        // angular rates: apply LPF to damp vibration noise
        // filtered = beta*raw + (1-beta)*filtered, cutoff ~ beta*fs/(2π)
        // beta=0.2 @ 500Hz → cutoff ≈ 16Hz, good balance for acro
        static float rx_f = 0, ry_f = 0, rz_f = 0;
        float rx_raw = imu->gy * D2R;   // roll rate = about front axis
        float ry_raw = imu->gx * D2R;   // pitch rate = about right axis
        float rz_raw = imu->gz * D2R;   // yaw rate = about up axis
        const float BETA = 0.2f;
        rx_f = BETA * rx_raw + (1.0f - BETA) * rx_f;
        ry_f = BETA * ry_raw + (1.0f - BETA) * ry_f;
        rz_f = BETA * rz_raw + (1.0f - BETA) * rz_f;

        _att.roll_rate  = rx_f;
        _att.pitch_rate = ry_f;
        _att.yaw_rate   = rz_f;

        _att.timestamp_ms = imu->timestamp_ms;

        // publish to attitude queue
        _attQueue.sendToBack(_att, 0);

        // debug print every 1s
        // if (_att.timestamp_ms - lastPrint >= 1000) {
        //     lastPrint = _att.timestamp_ms;
        //     char dbg[96];
        //     snprintf(dbg, sizeof(dbg),
        //              "ATTD R=%.1f P=%.1f Y=%.1f",
        //              _att.roll * 57.3f, _att.pitch * 57.3f, _att.yaw * 57.3f);
        //     DBGQ.sendToBack((uint8_t*)dbg, 0);
        // }
    }
}
