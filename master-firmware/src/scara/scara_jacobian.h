#ifndef SCARA_JACOBIAN_H
#define SCARA_JACOBIAN_H

#ifdef __cplusplus
extern "C" {
#endif
void scara_jacobian_compute(float f_x, float f_y, float alpha, float beta, float l1, float l2, float* torque_alpha, float* torque_beta);
#ifdef __cplusplus
}
#endif

#endif
