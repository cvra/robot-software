#include <math.h>
#include <string.h>

#include "filter/iir.h"

/** IIR filter
 *
 *         b0 + b1*z^-1 + ... + bn*z^-n
 * H(z) = ------------------------------
 *         1 + a1*z^-1 + ... + an*z^-n
 *
 * y[k] = b0*x[k] + b1*x[k-1] + ... + bn*[k-n] - a1*y[k-1] - ... - an*y[k-n]
 */

/*
 * Initialize IIR filter
 * b : filter coefficients b0 to bn (array of size n+1)
 * a : filter coefficients a1 to an (array of size n)
 *     (a can be set to NULL for FIR filter)
 * d : delay buffer (array of size n)
 * n : filter degree, n must be >= 1
 */
void filter_iir_init(filter_iir_t* f, const float* b, const float* a, int n, float* d)
{
    f->b = b;
    f->a = a;
    f->n = n;
    f->d = d;
    memset(f->d, 0, sizeof(float) * n);
}

float filter_iir_apply(filter_iir_t* f, float in)
{
    float out;
    out = f->b[0] * in + f->d[0];
    // update filter
    if (f->a == NULL) { // FIR filter
        int i;
        for (i = 0; i < (f->n - 1); i++) { // delays 0 to n-2
            f->d[i] = f->b[i + 1] * in + f->d[i + 1];
        }
        f->d[f->n - 1] = f->b[f->n] * in; // delay n-1
    } else { // IIR filter
        int i;
        for (i = 0; i < (f->n - 1); i++) { // delays 0 to n-2
            f->d[i] = f->b[i + 1] * in - f->a[i] * out + f->d[i + 1];
        }
        f->d[f->n - 1] = f->b[f->n] * in - f->a[f->n - 1] * out; // delay n-1
    }
    return out;
}
