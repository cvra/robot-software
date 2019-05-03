#pragma once

#include "Eigen/Dense"

#define _USE_MATH_DEFINES
#include <math.h>

/* A reference class for 2D controller to compute a smooth 
 * path planning designed around minimizing effort on the end joint
 *
 * Dimensions:
 *      - m: cartesian coordinates 2D
 * Parameters:
 *      - r: the vector of the target position
 *      - a: desired maximum acceleration of the object carried
 */
template <int m>
class RefTracking {
public:
    RefTracking() = default;

    RefTracking(const Eigen::Matrix<float, m, 1>& r, const float a)
    {
        dr = r;
        t0 = std::sqrt(0.25 * M_PI * dr.norm() / a);
        a_max = a;
        v_max = 0.5 * dr.norm() / t0;
    }

    RefTracking(const RefTracking&) = default;
    RefTracking& operator=(const RefTracking&) = default;

    /* Compute the norm of the trajectory at time t
     *
     * Parameters:
     *      - t: time spend from the start of the movement
     *
     * Return:
     *      - norm of traveled distance
     */
    float norm_r(const float t)
    {
        if (t < 0) {
            return 0;
        } else if(t0 == 0){
            return 0;
        } else if (t < t0) {
            return -v_max / 2 * cosf(M_PI / t0 * (t - t0/2)) * t0 / M_PI + v_max / 2 * t;
        } else if (t < 2 * t0) {
            return v_max / 2 * t0 + v_max * (t - t0);
        } else if (t < 3 * t0) {
            return v_max / 2 * t0 + v_max * (t0)-v_max / 2 * cosf(M_PI / t0 * (t + t0/2)) * t0 / M_PI + v_max / 2 * (t - 2 * t0);
        } else {
            return dr.norm();
        }
    }

    /* Compute the norm of the velocity at time t
     *
     * Parameters:
     *      - t: time spend from the start of the movement
     *
     * Return:
     *      - norm of speed at time t
     */
    float norm_v(const float t)
    {
        if (t < 0) {
            return 0;
        } else  if(t0 == 0){
            return 0;
        } else if (t < t0) {
            return v_max / 2 * sinf(M_PI / t0 * (t - t0/2)) + v_max / 2;
        } else if (t < 2 * t0) {
            return v_max;
        } else if (t < 3 * t0) {
            return v_max / 2 * sinf(M_PI / t0 * (t - t0*3/2)) + v_max / 2;
        } else {
            return 0;
        }
    }

    /* Compute the norm of the acceleration at time t
     *
     * Parameters:
     *      - t: time spend from the start of the movement
     *
     * Return:
     *      - norm of acceleration at time t
     */
    float norm_a(const float t)
    {
        if (t < 0) {
            return 0;
        } else if(t0 == 0){
            return 0;
        } else if (t <= t0) {
            return v_max / 2 * cosf(M_PI / t0 * (t - t0/2)) * M_PI / t0;
        } else if (t < 2 * t0) {
            return 0;
        } else if (t <= 3 * t0) {
            return v_max / 2 * cosf(M_PI / t0 * (t - t0*3/2)) * M_PI / t0;
        } else {
            return 0;
        }
    }

    /*
     * Return the expected position at time t in cartesian coordinates
     */
    Eigen::Matrix<float, m, 1> r(const float t)
    {
        if(dr.norm() == 0)
            return dr * 0.f;
        else
            return dr / dr.norm() * norm_r(t);
    }

    /*
     * Return the expected velocity at time t in cartesian coordinates
     */
    Eigen::Matrix<float, m, 1> v(const float t)
    {
        if(dr.norm() == 0)
            return dr * 0.f;
        else
            return dr / dr.norm() * norm_v(t);
    }

    /*
     * Return the expected acceleration at time t in cartesian coordinates
     */
    Eigen::Matrix<float, m, 1> a(const float t)
    {
        if(dr.norm() == 0)
            return dr * 0.f;
        else
            return dr / dr.norm() * norm_a(t);
    }

    /*
     * Return the time necessary to reach the target
     */
    float getTimeToTarget(const float t)
    {
        if (t > 3 * t0)
            return 0;
        else if (t < 0)
            return 3 * t0;
        else
            return 3 * t0 - t;
    }

    Eigen::Matrix<float, m, 1> dr;
    float a_max;
    float v_max;
    float t0;
};
