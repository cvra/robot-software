#ifndef _FAST_MATH_H_
#define _FAST_MATH_H_

/** Initializes the fast math library.
 *
 * This functions precomputes the lookup tables used in others fast_* functions.
 * @warning This function must be called once at the beginning of the code.
 * @author Mathieu Rouvinez, CVRA
 */
void fast_math_init();      //############# DO NOT FUCKING FORGET #############

/** Computes the absolute value of a float.
 *
 * This function computes the absolute value of a float by masking its MSB
 * (sign in IEEE754 floats).
 */
float fast_fabsf(float v);

/** Computes the sine of a float.
 *
 * This function computes the sine by using a Taylor series expansion.
 * //Validity range: [-pi,pi]
 * //Relative error < 0.0178 % on [-3.141592;3.141592] 
 */
float fast_sinf(float v);

/** Computes the cosine of a float.
 *
 * This function computes the cosine by using a Taylor series expansion.
 * //Validity range: [-pi,pi]
 * //Relative error < 0.0042 % on [-3.141592;3.141592]
 */
float fast_cosf(float v);

/** Computes the tangent of a float.
 *
 * This function computes the tangent by using a lookup table.
 * Relative error < 0.0061% on [-1.57,1.57]
 */
float fast_tanf(float v);

/** Computes the arcsine of a float.
 *
 * This function computes the arcsine by using a lookup table.
 * Relative error < 0.004% on [-1,1]
 */
float fast_asinf(float v);

/** Computes the arccosine of a float.
 *
 * This function computes the arccosine by using a lookup table.
 * Relative error < 0.1% on [-1,1]
 */
float fast_acosf(float v);

/** Computes the arctangent of a float.
 *
 * This function computes the arctangent by using a lookup table.
 * Relative error < 0.1%
 */
float fast_atanf(float v);

/** Computes the atan2 of a float.
 *
 * This function computes the atan2 by using a Taylor series expansion.
 * Relative error < [good enough]
 */
float fast_atan2f(float y, float x);

/** Computes the square root of a float
 *
 * This function computes the square root by using a fast lookup table (Hsieh).
 * Relative error < [very good]
 */
float fast_sqrtf(float v);

/** Computes the reciprocal of square root of a float
 *
 * This function computes the reciprocal of square root using Lomont's method.
 * Relative error < 0.07%
 */
float fast_invsqrtf(float v);

/** Computes the reciprocal (or inverse) of a float
 *
 * This function computes the reciprocal of a float using the subtraction trick.
 * Relative error < 0.1%
 */
float fast_invf(float v);

/** Raises a float to some (unsigned) integer power
 *  
 *  uses the "exponentiation by squaring"-algorithm 
 *  to raise a float to some integer power
 */  
float fast_powf(float b, int e);

/** Benchmarks the library. */
void  fast_benchmark(void);

#endif
