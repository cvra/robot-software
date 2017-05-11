#ifndef SCARA_JACOBIAN_H
#define SCARA_JACOBIAN_H

#ifdef __cplusplus
extern "C" {
#endif
void scara_jacobian_compute(float f_x, float f_y, float f_theta,
                            float alpha, float beta, float gamma,
                            float l1, float l2, float l3,
                            float* torque_alpha, float* torque_beta, float* torque_gamma);
#ifdef __cplusplus
}
#endif

#endif
