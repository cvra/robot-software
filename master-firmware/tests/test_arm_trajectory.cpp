#include "control/RefTracking.hpp"

#include <CppUTest/TestHarness.h>

#include <vector>

namespace {
static float tol = 1e-3;

template <int rows, int cols>
void MATRIX_EQUAL(const Eigen::Matrix<float, rows, cols>& expected, const Eigen::Matrix<float, rows, cols>& actual, float tolerance = tol)
{
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            DOUBLES_EQUAL(expected(i, j), actual(i, j), tolerance);
        }
    }
}

template <int size>
void VECTOR_EQUAL(const Eigen::Matrix<float, size, 1>& expected, const Eigen::Matrix<float, size, 1>& actual, float tolerance = tol)
{
    for (int i = 0; i < size; i++) {
        DOUBLES_EQUAL(expected(i), actual(i), tolerance);
    }
}

std::vector<float> range(float start, float stop, float delta)
{
    int total = (stop - start) / delta;
    std::vector<float> r;
    r.reserve(total);
    for (int i = 0; i < total; i++) {
        r.push_back(start + i * delta);
    }
    return r;
}
}

TEST_GROUP (AReferenceTracking) {
    Eigen::Vector2f r{1, 2}; // a position vector in cartesian axis
    float accelerationMax{2}; // end segment arm max accel
    RefTracking<2> ref{r, accelerationMax};
};

TEST(AReferenceTracking, constructorTest)
{
    float expectedV_max = 1.193;
    float expectedT0 = 0.937;
    float a_max = 2;
    Eigen::Vector2f r(1, 2);

    DOUBLES_EQUAL(expectedV_max, ref.v_max, tol);
    DOUBLES_EQUAL(expectedT0, ref.t0, tol);
    DOUBLES_EQUAL(a_max, ref.a_max, tol);
    VECTOR_EQUAL(r, ref.dr, tol);
}

TEST(AReferenceTracking, timeLeftToReachTarget)
{
    float total_time = 2.811;
    float pos_reached_t_left = 0;

    DOUBLES_EQUAL(total_time, ref.getTimeToTarget(0), tol);
    DOUBLES_EQUAL(total_time, ref.getTimeToTarget(-1), tol);
    DOUBLES_EQUAL(pos_reached_t_left, ref.getTimeToTarget(1e12), tol);
}

#include <iostream>
TEST(AReferenceTracking, accelNormInfunctionOfTime)
{
    const auto time = range(-0.3, 3.5, 0.1);
    std::vector<float> a_ref = {
        0,
        0,
        0,
        0,
        0.658,
        1.242,
        1.689,
        1.947,
        1.988,
        1.808,
        1.427,
        0.887,
        0.247,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        -0.1731,
        -0.8190,
        -1.3737,
        -1.7755,
        -1.9795,
        -1.9631,
        -1.7282,
        -1.3007,
        -0.7285,
        -0.0751,
        0,
        0,
        0,
        0,
        0,
        0};

    for (int i = 0; i < 38; i++) {
    	//std::cout << time[i] << std::endl;
        DOUBLES_EQUAL(a_ref[i], ref.norm_a(time[i]), tol);
    }
}

TEST(AReferenceTracking, velNormInfunctionOfTime)
{
    const auto time = range(-0.3, 3.5, 0.1);
    std::vector<float> v_ref = {
        0,
        0,
        0,
        0.0,
        0.03321282579152507,
        0.12915311432515098,
        0.2771380858013034,
        0.460689877915326,
        0.6593703263991999,
        0.8510567181146378,
        1.0144051156546414,
        1.1312269666581076,
        1.1885143666344082,
        1.1931157055517936,
        1.1931157055517936,
        1.1931157055517936,
        1.1931157055517936,
        1.1931157055517936,
        1.1931157055517936,
        1.1931157055517936,
        1.1931157055517936,
        1.1931157055517936,
        1.1908754215132646,
        1.1407933115735318,
        1.0301115551837277,
        0.8711543682641325,
        0.6816213497150218,
        0.4826166626867824,
        0.2962991237005976,
        0.1434148579996733,
        0.040987255551512836,
        0.0004214469751322447,
        0,
        0,
        0,
        0,
        0,
        0};

    for (int i = 0; i < 38; i++) {
        DOUBLES_EQUAL(v_ref[i], ref.norm_v(time[i]), tol);
    }
}

TEST(AReferenceTracking, posNormInfunctionOfTime)
{
     const auto time = range(-0.3, 3.5, 0.1);
     std::vector<float> p_ref = {
         0,
         0,
         0,
         0,
         0.0011112587092005147,
         0.008741345903904754,
         0.028683230158976003,
         0.06535898343281743,
         0.12132739125167008,
         0.19699903651070416,
         0.29059058180185426,
         0.3983233408495523,
         0.5148440284795933,
         0.6340987111768461,
         0.7534102817320255,
         0.872721852287205,
         0.9920334228423842,
         1.1113449933975637,
         1.230656563952743,
         1.3499681345079222,
         1.469279705063102,
         1.588591275618281,
         1.7078835314064167,
         1.825006238383364,
         1.934014606989575,
         2.0294133152044673,
         2.1072224523881173,
         2.165420675223753,
         2.204170280686393,
         2.2257991345645096,
         2.234541467761542,
         2.2360664022462258,
         2.23606797749979,
         2.23606797749979,
         2.23606797749979,
         2.23606797749979,
         2.23606797749979,
         2.23606797749979};     for (int i = 0; i < 38; i++) {
         DOUBLES_EQUAL(p_ref[i], ref.norm_r(time[i]), tol);
     }
 } 

 TEST(AReferenceTracking, accelInfunctionOfTime)
 {
     const auto time = range(-0.3, 3.5, 0.1);
     std::vector<Eigen::Vector2f> a_ref = {
         {0., 0.},
         {0., 0.},
         {0., 0.},
         {0., 0.},
         {0.294, 0.589},
         {0.556, 1.112},
         {0.755, 1.511},
         {0.871, 1.742},
         {0.889, 1.779},
         {0.809, 1.618},
         {0.638, 1.277},
         {0.397, 0.793},
         {0.111, 0.222},
         {0., 0.},
         {0., 0.},
         {0., 0.},
         {0., 0.},
         {0., 0.},
         {0., 0.},
         {0., 0.},
         {0., 0.},
         {0., 0.},
         {-0.077, -0.155},
         {-0.366, -0.733},
         {-0.614, -1.229},
         {-0.794, -1.588},
         {-0.885, -1.771},
         {-0.878, -1.756},
         {-0.773, -1.546},
         {-0.582, -1.163},
         {-0.326, -0.652},
         {-0.034, -0.067},
         {0., 0.},
         {0., 0.},
         {0., 0.},
         {0., 0.},
         {0., 0.},
         {0., 0.}};     for (int i = 0; i < 38; i++) {
         VECTOR_EQUAL(a_ref[i], ref.a(time[i]));
    }
}

TEST(AReferenceTracking, velInfunctionOfTime)
{
    const auto time = range(-0.3, 3.5, 0.1);
    std::vector<Eigen::Vector2f> v_ref = {
        {0., 0.},
        {0., 0.},
        {0., 0.},
        {0., 0.},
        {0.015, 0.03},
        {0.058, 0.116},
        {0.124, 0.248},
        {0.206, 0.412},
        {0.295, 0.59},
        {0.381, 0.761},
        {0.454, 0.907},
        {0.506, 1.012},
        {0.532, 1.063},
        {0.534, 1.067},
        {0.534, 1.067},
        {0.534, 1.067},
        {0.534, 1.067},
        {0.534, 1.067},
        {0.534, 1.067},
        {0.534, 1.067},
        {0.534, 1.067},
        {0.534, 1.067},
        {0.533, 1.065},
        {0.51, 1.02},
        {0.461, 0.921},
        {0.39, 0.779},
        {0.305, 0.61},
        {0.216, 0.432},
        {0.133, 0.265},
        {0.064, 0.128},
        {0.018, 0.037},
        {0., 0.},
        {0., 0.},
        {0., 0.},
        {0., 0.},
        {0., 0.},
        {0., 0.},
        {0., 0.}};

    for (int i = 0; i < 38; i++) {
        VECTOR_EQUAL(v_ref[i], ref.v(time[i]));
    }
}

TEST(AReferenceTracking, posInfunctionOfTime)
{
    const auto time = range(-0.3, 3.5, 0.1);
    std::vector<Eigen::Vector2f> p_ref = {
        {0., 0.},
        {0., 0.},
        {0., 0.},
        {0., 0.},
        {0., 0.001},
        {0.004, 0.008},
        {0.013, 0.026},
        {0.029, 0.058},
        {0.054, 0.109},
        {0.088, 0.176},
        {0.13, 0.26},
        {0.178, 0.356},
        {0.23, 0.46},
        {0.284, 0.567},
        {0.337, 0.674},
        {0.39, 0.781},
        {0.444, 0.887},
        {0.497, 0.994},
        {0.55, 1.101},
        {0.604, 1.207},
        {0.657, 1.314},
        {0.71, 1.421},
        {0.764, 1.528},
        {0.816, 1.632},
        {0.865, 1.73},
        {0.908, 1.815},
        {0.942, 1.885},
        {0.968, 1.937},
        {0.986, 1.971},
        {0.995, 1.991},
        {0.999, 1.999},
        {1., 2.},
        {1., 2.},
        {1., 2.},
        {1., 2.},
        {1., 2.},
        {1., 2.},
        {1., 2.}};

    for (int i = 0; i < 38; i++) {
        VECTOR_EQUAL(p_ref[i], ref.r(time[i]));
    }
}
