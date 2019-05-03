#include "control/Arm2DCtrl.hpp"
#include "control/RefTracking.hpp"
#include "control/LinearSystem.hpp"
#include "control/LQRController.hpp"

#include <CppUTest/TestHarness.h>

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
}

TEST_GROUP(AnArmController) {
    LQRController<9, 3, 10> lqr;
    LinearSystem<9, 3> system;
    Eigen::Matrix<float, 3, 1> l{3,2,1};
    float del = 1e-9;
    Arm2DCtrl<2,3,3,10> controller;

    // traj init
    Eigen::Vector2f r{1, 2}; // a position vector in cartesian axis
    float accelerationMax{2}; // end segment arm max accel
    RefTracking<2> ref{r, accelerationMax};

    void setup()
    {
        system.A << del, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        			0.0, del, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        			0.0, 0.0, del, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        			0.0, 0.0, 0.0, del, 1.0, 0.0, 0.0, 0.0, 0.0,
        			0.0, 0.0, 0.0, 0.0, del, 1.0, 0.0, 0.0, 0.0,
        			0.0, 0.0, 0.0, 0.0, 0.0, del, 0.0, 0.0, 0.0,
        			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, del, 1.0, 0.0,
        			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, del, 1.0,
        			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, del;

        system.B << 0.0, 0.0, 0.0,
        			0.0, 0.0, 0.0,
        			1.0, 0.0, 0.0,
        			0.0, 0.0, 0.0,
        			0.0, 0.0, 0.0,
        			0.0, 1.0, 0.0,
        			0.0, 0.0, 0.0,
        			0.0, 0.0, 0.0,
        			0.0, 0.0, 1.0;
        lqr.sampling_period = 0.1;
        lqr.system = discretize(system, lqr.sampling_period);
        
        lqr.Q << 10., 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            			0.0, 10., 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            			0.0, 0.0, 10., 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            			0.0, 0.0, 0.0, 10., 0.0, 0.0, 0.0, 0.0, 0.0,
            			0.0, 0.0, 0.0, 0.0, 10., 0.0, 0.0, 0.0, 0.0,
            			0.0, 0.0, 0.0, 0.0, 0.0, 10., 0.0, 0.0, 0.0,
            			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10., 0.0, 0.0,
            			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10., 0.0,
            			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.;
        
        lqr.R << 0.3, 0.0, 0.0,
                 0.0, 0.3, 0.0,
                 0.0, 0.0, 0.3;
        
        lqr.N = system.B;

        controller = Arm2DCtrl<2,3,3,10>(l,lqr);
    }
};

TEST(AnArmController, constructorTesting)
{
	std::array<Eigen::Matrix<float, 3, 9>, 10> expected_K;
	expected_K[0] << 1.352, 4.136, 5.758, 0.   , 0.   , 0.   , 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 1.352, 4.136, 5.758, 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.   , 0.   , 0.   , 1.352, 4.136, 5.758;
    expected_K[1] << 1.09 , 3.635, 5.665, 0.   , 0.   , 0.   , 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 1.09 , 3.635, 5.665, 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.   , 0.   , 0.   , 1.09 , 3.635, 5.665;
    expected_K[2] << 0.843, 3.122, 5.57 , 0.   , 0.   , 0.   , 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.843, 3.122, 5.57 , 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.   , 0.   , 0.   , 0.843, 3.122, 5.57 ;
    expected_K[3] << 0.618, 2.603, 5.472, 0.   , 0.   , 0.   , 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.618, 2.603, 5.472, 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.   , 0.   , 0.   , 0.618, 2.603, 5.472;
    expected_K[4] << 0.422, 2.086, 5.375, 0.   , 0.   , 0.   , 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.422, 2.086, 5.375, 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.   , 0.   , 0.   , 0.422, 2.086, 5.375;
    expected_K[5] << 0.26 , 1.576, 5.279, 0.   , 0.   , 0.   , 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.26 , 1.576, 5.279, 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.   , 0.   , 0.   , 0.26 , 1.576, 5.279;
    expected_K[6] << 0.138, 1.087, 5.187, 0.   , 0.   , 0.   , 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.138, 1.087, 5.187, 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.   , 0.   , 0.   , 0.138, 1.087, 5.187;
    expected_K[7] << 0.056, 0.634, 5.104, 0.   , 0.   , 0.   , 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.056, 0.634, 5.104, 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.   , 0.   , 0.   , 0.056, 0.634, 5.104;
    expected_K[8] << 0.012, 0.252, 5.038, 0.   , 0.   , 0.   , 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.012, 0.252, 5.038, 0.   , 0.   , 0.   ,
     				 0.   , 0.   , 0.   , 0.   , 0.   , 0.   , 0.012, 0.252, 5.038;
    expected_K[9] << 0.   , 0.   , 5.   , 0.   , 0.   , 0.   , 0.   , 0.   , 0.,
     				 0.   , 0.   , 0.   , 0.   , 0.   , 5.   , 0.   , 0.   , 0.,
     				 0.   , 0.   , 0.   , 0.   , 0.   , 0.   , 0.   , 0.   , 5.;

    for(int i=0; i<10; i++){
    	MATRIX_EQUAL(expected_K[i], controller.K[i]);
    }

    MATRIX_EQUAL(l, controller.l, tol);
    MATRIX_EQUAL(lqr.system.B, controller.B, tol);
    MATRIX_EQUAL(lqr.system.A, controller.A, tol);
}

TEST(AnArmController, setNewTarget)
{
    float expectedV_max = 1.193;
    float expectedT0 = 0.937;
    float a_max = 2;
    Eigen::Matrix<float, 2, 1> b{0, 0};
    Eigen::Matrix<float, 3, 1> q{0,M_PI,0};
    Eigen::Matrix<float, 2, 1> r{1, 2};

    controller.setNewTarget(r, a_max, b, q);

    DOUBLES_EQUAL(expectedV_max, controller.ref.v_max, tol);
    DOUBLES_EQUAL(expectedT0, controller.ref.t0, tol);
    DOUBLES_EQUAL(a_max, controller.ref.a_max, tol);
    MATRIX_EQUAL(r, controller.ref.dr, tol);
}


TEST(AnArmController, aStep)
{
    float a_max = 2;
    Eigen::Matrix<float, 2, 1> r{1, 2};

    Eigen::Matrix<float, 3, 1> q_in{0,0,0};
    Eigen::Matrix<float, 2, 1> b{0,0};
    controller.setNewTarget(r, a_max, b, q_in);

    Eigen::Matrix<float, 3, 1> Expected_q_out{-0.401, 1.861, 1.498};

    MATRIX_EQUAL(Expected_q_out, controller.step(q_in,b,r), tol);
}

TEST(AnArmController, anOtherStep)
{
    float a_max = 2;
    Eigen::Matrix<float, 2, 1> r{-4, 1};

    Eigen::Matrix<float, 3, 1> q_in{0,0,0};
    Eigen::Matrix<float, 2, 1> b{0,0};
    controller.setNewTarget(r, a_max, b, q_in);

    Eigen::Matrix<float, 3, 1> Expected_q_out{-0.525, -1.504, -0.334};

    MATRIX_EQUAL(Expected_q_out, controller.step(q_in,b,r), tol);
}

TEST(AnArmController, aPreciseStep)
{
    float a_max = 2;
    Eigen::Matrix<float, 2, 1> r{3, 3};

    Eigen::Matrix<float, 3, 1> q_in{0,0,0};
    Eigen::Matrix<float, 2, 1> b{0,0};
    controller.setNewTarget(r, a_max, b, q_in);

    Eigen::Matrix<float, 3, 1> Expected_q_out{0.078208, 1.214375, 0.848277};

    MATRIX_EQUAL(Expected_q_out, controller.step(q_in,b,r), 0.00001);
}

TEST(AnArmController, aStepOutOfReach)
{
    float a_max = 2;
    Eigen::Matrix<float, 2, 1> r{-4, 12};

    Eigen::Matrix<float, 3, 1> q_in{0,0,0};
    Eigen::Matrix<float, 2, 1> b{0,0};
    controller.setNewTarget(r, a_max, b, q_in);

    Eigen::Matrix<float, 3, 1> Expected_q_out{-0.322, -0., 0.};

    MATRIX_EQUAL(Expected_q_out, controller.step(q_in,b,r), tol);
}

TEST(AnArmController, getPosition)
{
    Eigen::Matrix<float, 2, 1> b{0,0};
    Eigen::Matrix<float, 3, 1> q_in{0.7,0.4,-1.1};
    Eigen::Matrix<float, 2, 1> expected_pos{3.715068,4.201719};

    MATRIX_EQUAL(expected_pos, controller.getArmPosition(b,q_in), tol);
}


TEST(AnArmController, isRunning)
{
    float a_max = 4;
    Eigen::Matrix<float, 2, 1> r{2, 1};
    Eigen::Matrix<float, 3, 1> angles;
    Eigen::Matrix<float, 9, 1> q;
    Eigen::Matrix<float, 9, 1> lastq;
    q << 0.,0.,0.,3.14,0.,0.,3.14,0.,0.;
    lastq << 0.,0.,0.,3.14,0.,0.,3.14,0.,0.;

    Eigen::Matrix<float, 3, 1> q_in{0.,3.14,3.14};
    Eigen::Matrix<float, 2, 1> b{0.,0.};
    controller.setNewTarget(r, a_max, b, q_in);

    std::array<Eigen::Matrix<float, 2, 1>,50> expected_pos;

    expected_pos[0] << 4.039834e-09, 1.999997e+00;
    expected_pos[1] << 4.039344e-04, 2.000001e+00;
    expected_pos[2] << 0.002435 ,1.999969;
    expected_pos[3] << 0.00809  ,1.999713;
    expected_pos[4] << 0.019863 ,1.998812;
    expected_pos[5] << 0.040228 ,1.996596;
    expected_pos[6] << 0.07106  ,1.992207;
    expected_pos[7] << 0.113315 ,1.984724;
    expected_pos[8] << 0.16722  ,1.973264;
    expected_pos[9] << 0.232636 ,1.957029;
    expected_pos[10] << 0.309254, 1.935327;
    expected_pos[11] << 0.396696, 1.907559;
    expected_pos[12] << 0.494566, 1.873213;
    expected_pos[13] << 0.602444, 1.831869;
    expected_pos[14] << 0.719456, 1.783322;
    expected_pos[15] << 0.843483, 1.727864;
    expected_pos[16] << 0.970811, 1.66652 ;
    expected_pos[17] << 1.096673, 1.601034;
    expected_pos[18] << 1.216393, 1.533558;
    expected_pos[19] << 1.326517, 1.466188;
    expected_pos[20] << 1.425375, 1.400565;
    expected_pos[21] << 1.512753, 1.337789;
    expected_pos[22] << 1.589195, 1.278558;
    expected_pos[23] << 1.65558 , 1.223281;
    expected_pos[24] << 1.712914, 1.172166;
    expected_pos[25] << 1.762213, 1.125271;
    expected_pos[26] << 1.804455, 1.082544;
    expected_pos[27] << 1.840545, 1.043856;
    expected_pos[28] << 1.87131 , 1.009022;
    expected_pos[29] << 1.897494, 0.977821;
    expected_pos[30] << 1.919755, 0.950012;
    expected_pos[31] << 1.938674, 0.925342;
    expected_pos[32] << 1.954756, 0.903558;
    expected_pos[33] << 1.968441, 0.884413;
    expected_pos[34] << 1.980105, 0.867669;
    expected_pos[35] << 1.990074, 0.853101;
    expected_pos[36] << 1.998621, 0.840501;
    expected_pos[37] << 2.005981, 0.829675;
    expected_pos[38] << 2.012346, 0.820447;
    expected_pos[39] << 2.017881, 0.812659;
    expected_pos[40] << 2.02272 , 0.806167;
    expected_pos[41] << 2.026974, 0.800842;
    expected_pos[42] << 2.030735, 0.796569;
    expected_pos[43] << 2.034075, 0.793245;
    expected_pos[44] << 2.037056, 0.79078 ;
    expected_pos[45] << 2.039726, 0.789092;
    expected_pos[46] << 2.042122, 0.788109;
    expected_pos[47] << 2.044276, 0.787766;
    expected_pos[48] << 2.046211, 0.788005;
    expected_pos[49] << 2.047949, 0.788773;

    float dt = 0.1;

    for(int i=0; i<50; i++){
        q = controller.run(q,b,dt);
        q = controller.A*lastq - controller.B*controller.K[0]*q;
        lastq = q;
        angles(0) = q(0);
        angles(1) = q(3);
        angles(2) = q(6);
        MATRIX_EQUAL(expected_pos[i],controller.getArmPosition(b,angles),tol);
    }
}