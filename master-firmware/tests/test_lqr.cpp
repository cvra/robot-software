#include "control/LinearSystem.hpp"
#include "control/LQRController.hpp"
#include <iostream>

#include <CppUTest/TestHarness.h>

namespace {
template <int rows, int cols>
void MATRIX_EQUAL(const Eigen::Matrix<float, rows, cols>& expected, const Eigen::Matrix<float, rows, cols>& actual, float tolerance = 1e-3)
{
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            DOUBLES_EQUAL(expected(i, j), actual(i, j), tolerance);
        }
    }
}
}

TEST_GROUP (ALinearSystem) {
    LinearSystem<2, 1> system;

    void setup()
    {
        system.A << 1, 0, 0, 1;
        system.B << 2, 0;
    }
};

TEST(ALinearSystem, canBeDiscretized)
{
    auto discrete_system = discretize(system, 0.01);

    Eigen::Matrix<float, 2, 2> discrete_A;
    discrete_A << 1.01005, 0, 0, 1.01005;
    Eigen::Matrix<float, 2, 1> discrete_B;
    discrete_B << 0.0201005, 0;

    MATRIX_EQUAL(discrete_system.A, discrete_A);
    MATRIX_EQUAL(discrete_system.B, discrete_B);
}

TEST_GROUP (AWeirdLinearSystem) {
    LinearSystem<2, 1> system;

    void setup()
    {
        system.A << 0.5, 0.3, -0.2, 1.7;
        system.B << 1.2, -0.1;
    }
};

TEST(AWeirdLinearSystem, canBeDiscretized)
{
    auto discrete_system = discretize(system, 0.01);

    Eigen::Matrix<float, 2, 2> discrete_A;
    discrete_A << 1.00501, 0.0030333, -0.0020222, 1.01714268;
    Eigen::Matrix<float, 2, 1> discrete_B;
    discrete_B << 0.01202854, -0.0010207;

    MATRIX_EQUAL(discrete_system.A, discrete_A);
    MATRIX_EQUAL(discrete_system.B, discrete_B);
}

TEST_GROUP (AnLQRController) {
    LQRController<2, 1, 4> controller;
    LinearSystem<2, 1> system;

    void setup()
    {
        system.A << 0.5, 0.3, -0.2, 1.7;
        system.B << 1.2, -0.1;

        controller.sampling_period = 0.01;
        controller.system = discretize(system, controller.sampling_period);
        controller.Q << 2, 0, 0, 2;
        controller.R << 0.5;
        controller.N << 0.7, 0.3;
    }
};

TEST(AnLQRController, solvesRicatti)
{
    auto P = controller.ricatti_P();

    std::array<Eigen::Matrix<float, 2, 2>, 5> expected;
    expected[0] << 0.0, 0.0, 0.0, 0.0;
    expected[1] << 4.845, -1.309, -1.309, 7.891;
    expected[2] << 3.920, -0.865, -0.865, 5.854;
    expected[3] << 2.972, -0.429, -0.429, 3.892;
    expected[4] << 2.0, 0.0, 0.0, 2.0;

    MATRIX_EQUAL(expected[0], P[0], 1e-3);
    MATRIX_EQUAL(expected[1], P[1], 1e-3);
    MATRIX_EQUAL(expected[2], P[2], 1e-3);
    MATRIX_EQUAL(expected[3], P[3], 1e-3);
    MATRIX_EQUAL(expected[4], P[4], 1e-3);
}

TEST(AnLQRController, computesGains)
{
    auto gain = controller.gain();

    std::array<Eigen::Matrix<float, 1, 2>, 4> expected;
    expected[0] << 1.517, 0.551;
    expected[1] << 1.494, 0.566;
    expected[2] << 1.471, 0.581;
    expected[3] << 1.447, 0.595;

    MATRIX_EQUAL(expected[0], gain[0], 1e-3);
    MATRIX_EQUAL(expected[1], gain[1], 1e-3);
    MATRIX_EQUAL(expected[2], gain[2], 1e-3);
    MATRIX_EQUAL(expected[3], gain[3], 1e-3);
}

TEST_GROUP (A1DPandulum) {
    LinearSystem<3, 1> system1D;
    LQRController<3, 1, 10> controller;
    float delta = 1e-9;

    void setup()
    {
        system1D.A << delta, 1.    , 0.   ,
                    0.     , delta , 1.   ,
                    0.     , 0.    , delta;

        system1D.B << 0., 0., 1.;

        controller.sampling_period = 0.1;
        controller.system = discretize(system1D, controller.sampling_period);
        controller.Q << 10., 0., 0.,
                        0. , 1., 0.,
                        0. , 0., 1.;
        controller.R << 0.3;
        controller.N = system1D.B;
    }
};

TEST(A1DPandulum, canReachATarget)
{
    auto K = controller.gain();
    Eigen::Matrix<float, 3, 1> target;
    Eigen::Matrix<float, 3, 1> state_k;
    Eigen::Matrix<float, 3, 1> state_k_1;
    Eigen::Matrix<float, 3, 1> u;
    std::array<Eigen::Matrix<float, 3,1>, 10> expected;

    target    << 0.1, 0., 0.;
    state_k   << 0.4, 0., 0.;
    state_k_1 << 0., 0., 0.;


    expected[0] << 0.4     ,  0.      , -0.097014;
    expected[1] << 0.399515, -0.009701, -0.164885;
    expected[2] << 0.39772 , -0.02619 , -0.208928;
    expected[3] << 0.394057, -0.047083, -0.233582;
    expected[4] << 0.388181, -0.070441, -0.242577;
    expected[5] << 0.379924, -0.094699, -0.239068;
    expected[6] << 0.369258, -0.118605, -0.225737;
    expected[7] << 0.356269, -0.141179, -0.204873;
    expected[8] << 0.341127, -0.161666, -0.17844 ;
    expected[9] << 0.324068, -0.17951 , -0.14812 ;

    for(int i=0; i<10; i++){
        u = controller.system.B * K[0] * (target - state_k);
        state_k_1 = controller.system.A * state_k + u;
        state_k = state_k_1;
        MATRIX_EQUAL(expected[i],state_k_1,0.0001);
    }
}