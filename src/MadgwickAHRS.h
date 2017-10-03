// =====================================================================================================
// MadgwickAHRS.h
// =====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
// =====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

void madgwick_filter_set_gain(float beta);

void madgwick_filter_get_quaternion(float out[4]);

void magdwick_filter_update(float gx,
                            float gy,
                            float gz,
                            float ax,
                            float ay,
                            float az,
                            float mx,
                            float my,
                            float mz);
void magdwick_filter_updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

#endif
