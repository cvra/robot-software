// =====================================================================================================
// MadgwickAHRS.c
// =====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
// =====================================================================================================

#include "MadgwickAHRS.h"
#include <math.h>

void madgwick_filter_init(madgwick_filter_t* f)
{
    f->q[0] = 1.;
    f->q[1] = 0.;
    f->q[2] = 0.;
    f->q[3] = 0.;

    madgwick_filter_set_gain(f, 0.1);
    madgwick_filter_set_sample_frequency(f, 250);
}

float invSqrt(float x);

void madgwick_filter_set_gain(madgwick_filter_t* f, float b)
{
    f->beta = b;
}

void madgwick_filter_set_sample_frequency(madgwick_filter_t* f, float freq)
{
    f->sample_frequency = freq;
}

void madgwick_filter_update(madgwick_filter_t* f,
                            float gx,
                            float gy,
                            float gz,
                            float ax,
                            float ay,
                            float az,
                            float mx,
                            float my,
                            float mz)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2,
        _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        madgwick_filter_updateIMU(f, gx, gy, gz, ax, ay, az);
        return;
    }

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-f->q[1] * gx - f->q[2] * gy - f->q[3] * gz);
    qDot2 = 0.5f * (f->q[0] * gx + f->q[2] * gz - f->q[3] * gy);
    qDot3 = 0.5f * (f->q[0] * gy - f->q[1] * gz + f->q[3] * gx);
    qDot4 = 0.5f * (f->q[0] * gz + f->q[1] * gy - f->q[2] * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * f->q[0] * mx;
        _2q0my = 2.0f * f->q[0] * my;
        _2q0mz = 2.0f * f->q[0] * mz;
        _2q1mx = 2.0f * f->q[1] * mx;
        _2q0 = 2.0f * f->q[0];
        _2q1 = 2.0f * f->q[1];
        _2q2 = 2.0f * f->q[2];
        _2q3 = 2.0f * f->q[3];
        _2q0q2 = 2.0f * f->q[0] * f->q[2];
        _2q2q3 = 2.0f * f->q[2] * f->q[3];
        q0q0 = f->q[0] * f->q[0];
        q0q1 = f->q[0] * f->q[1];
        q0q2 = f->q[0] * f->q[2];
        q0q3 = f->q[0] * f->q[3];
        q1q1 = f->q[1] * f->q[1];
        q1q2 = f->q[1] * f->q[2];
        q1q3 = f->q[1] * f->q[3];
        q2q2 = f->q[2] * f->q[2];
        q2q3 = f->q[2] * f->q[3];
        q3q3 = f->q[3] * f->q[3];

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * f->q[3] + _2q0mz * f->q[2] + mx * q1q1 + _2q1 * my * f->q[2] + _2q1 * mz * f->q[3] - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * f->q[3] + my * q0q0 - _2q0mz * f->q[1] + _2q1mx * f->q[2] - my * q1q1 + my * q2q2 + _2q2 * mz * f->q[3] - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * f->q[2] + _2q0my * f->q[1] + mz * q0q0 + _2q1mx * f->q[3] - mz * q1q1 + _2q2 * my * f->q[3] - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * f->q[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * f->q[3] + _2bz * f->q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * f->q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * f->q[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * f->q[3] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * f->q[2] + _2bz * f->q[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * f->q[3] - _4bz * f->q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * f->q[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * f->q[2] - _2bz * f->q[0]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * f->q[1] + _2bz * f->q[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * f->q[0] - _4bz * f->q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * f->q[3] + _2bz * f->q[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * f->q[0] + _2bz * f->q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * f->q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= f->beta * s0;
        qDot2 -= f->beta * s1;
        qDot3 -= f->beta * s2;
        qDot4 -= f->beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    f->q[0] += qDot1 * (1.0f / f->sample_frequency);
    f->q[1] += qDot2 * (1.0f / f->sample_frequency);
    f->q[2] += qDot3 * (1.0f / f->sample_frequency);
    f->q[3] += qDot4 * (1.0f / f->sample_frequency);

    // Normalise quaternion
    recipNorm = invSqrt(
        f->q[0] * f->q[0] + f->q[1] * f->q[1] + f->q[2] * f->q[2] + f->q[3] * f->q[3]);
    f->q[0] *= recipNorm;
    f->q[1] *= recipNorm;
    f->q[2] *= recipNorm;
    f->q[3] *= recipNorm;
}

void madgwick_filter_updateIMU(madgwick_filter_t* f,
                               float gx,
                               float gy,
                               float gz,
                               float ax,
                               float ay,
                               float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-f->q[1] * gx - f->q[2] * gy - f->q[3] * gz);
    qDot2 = 0.5f * (f->q[0] * gx + f->q[2] * gz - f->q[3] * gy);
    qDot3 = 0.5f * (f->q[0] * gy - f->q[1] * gz + f->q[3] * gx);
    qDot4 = 0.5f * (f->q[0] * gz + f->q[1] * gy - f->q[2] * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * f->q[0];
        _2q1 = 2.0f * f->q[1];
        _2q2 = 2.0f * f->q[2];
        _2q3 = 2.0f * f->q[3];
        _4q0 = 4.0f * f->q[0];
        _4q1 = 4.0f * f->q[1];
        _4q2 = 4.0f * f->q[2];
        _8q1 = 8.0f * f->q[1];
        _8q2 = 8.0f * f->q[2];
        q0q0 = f->q[0] * f->q[0];
        q1q1 = f->q[1] * f->q[1];
        q2q2 = f->q[2] * f->q[2];
        q3q3 = f->q[3] * f->q[3];

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * f->q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * f->q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * f->q[3] - _2q1 * ax + 4.0f * q2q2 * f->q[3] - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= f->beta * s0;
        qDot2 -= f->beta * s1;
        qDot3 -= f->beta * s2;
        qDot4 -= f->beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    f->q[0] += qDot1 * (1.0f / f->sample_frequency);
    f->q[1] += qDot2 * (1.0f / f->sample_frequency);
    f->q[2] += qDot3 * (1.0f / f->sample_frequency);
    f->q[3] += qDot4 * (1.0f / f->sample_frequency);

    // Normalise quaternion
    recipNorm = invSqrt(
        f->q[0] * f->q[0] + f->q[1] * f->q[1] + f->q[2] * f->q[2] + f->q[3] * f->q[3]);
    f->q[0] *= recipNorm;
    f->q[1] *= recipNorm;
    f->q[2] *= recipNorm;
    f->q[3] *= recipNorm;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x)
{
#if 0
    float y;
    // On ARM, compile the fast root algorithm
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
#else
    /* On other platforms we dont know anything about floating point, so better
     * use safe methods. */
    return 1 / sqrtf(x);
#endif
}
