/* File        : modules/math/fast_math/fast_math.c
 * Author      : Mathieu Rouvinez
 * Project     : CVRA, Swiss Eurobot 2013
 * State       : still appears to work
 * Creation    : 2012.04.24
 * Last modif. : 2013.03.29
 * Description : quite fast math function, accurate enough, nothing more
 * Notes       : some code borrowed from other programmers, didn't ask them
 * Disclaimer  : works on my machine
 */

#include "fast_math.h"
 
#include <math.h>
#include <float.h>  // FLT_MAX
#include <stdint.h> // int types
#include <stdio.h>  // printf()
#include <stdlib.h> // srand() & rand()
#include <time.h>   // clock()

#ifdef COMPILE_ON_ROBOT
 #include <uptime.h> /* for benchmark */
 #include <aversive.h>
#endif


// GENERAL DEFINES

#define F_2PI   6.283185307179586476925286766559f
#define F_PI    3.1415926535897932384626433832795f
#define F_PI_2  1.5707963267948966192313216916398f
#define F_PI_4  0.78539816339744830961566084581988f
#define F_2_PI  0.63661977236758134307553505349006f
#define F_1_PI  0.31830988618379067153776752674503f
#define F_1_2PI 0.15915494309189533576888376337251f

#if defined(__GNUC__) && (__GNUC__ > 2)
  #define F_LIKELY(x) (__builtin_expect(x, 1))
  #define F_UNLIKELY(x) (__builtin_expect(x, 0))
#else
  #define F_LIKELY(x) (x)
  #define F_UNLIKELY(x) (x)
#endif


// DECLARATIONS OF INTERNAL FUNCTIONS

inline float _fast_fabsf          (float v);
       float _fast_sinf           (float v);
       float _fast_sinf_pmPI2_o5  (float v);
       float _fast_sinf_pmPI2_o7  (float v);
       float _fast_sinf_pmPI_o9   (float v);
       float _fast_sinf_pmPI_o11  (float v);
       void  _fast_sinf_LUT_init  ();
       float _fast_sinf_LUT       (float v);
       float _fast_cosf           (float v);
       float _fast_cosf_pmPI2_o4  (float v);
       float _fast_cosf_pmPI2_o6  (float v);
       float _fast_cosf_pmPI_o8   (float v);
       float _fast_cosf_pmPI_o10  (float v);
       void  _fast_cosf_LUT_init  ();
       float _fast_cosf_LUT       (float v);
       float _fast_tanf           (float v);
       float _fast_tanf_alt       (float v);
       void  _fast_tanf_LUT_init  ();
       float _fast_tanf_LUT       (float v);
       float _fast_acosf          (float v);
       void  _fast_asinf_LUT_init ();
       float _fast_asinf_LUT      (float v);
       void  _fast_acosf_LUT_init ();
       float _fast_acosf_LUT      (float v);
       void  _fast_atanf_LUT_init ();
inline float _fast_atanf_LUT      (float v);
       float _fast_atan2f         (float y, float x);
       void  _fast_sqrtf_LUT_init ();
       float _fast_sqrtf_LUT      (float v);
       float _fast_sqrtf_1        (float v);
       float _fast_sqrtf_2        (float v);
       float _fast_sqrtf_3        (float v);
       float _fast_sqrtf_4        (float v);
       float _fast_sqrtf_5        (float v);
       float _fast_sqrtf_6        (float v);
inline float _fast_invsqrtf       (float v);
inline float _fast_invf           (float v);
       void  _fast_invf_LUT_init  ();
inline float _fast_invf_LUT       (float v);
       float _fast_powf_fs        (float b, int e);
       float _fast_powf_fu        (float b, int e);


// DEFINITIONS OF PUBLIC WRAPPERS

void fast_math_init()
{
    _fast_asinf_LUT_init(); // also used for acosf
    _fast_atanf_LUT_init();
    _fast_sqrtf_LUT_init();
    _fast_invf_LUT_init();
}

inline float fast_fabsf    (float v)          { return _fast_fabsf         (v);   }
inline float fast_sinf     (float v)          { return _fast_sinf          (v);   }
inline float fast_cosf     (float v)          { return _fast_cosf          (v);   }
inline float fast_tanf     (float v)          { return _fast_tanf_alt      (v);   }
inline float fast_asinf    (float v)          { return _fast_asinf_LUT     (v);   }
inline float fast_acosf    (float v)          { return _fast_acosf_LUT     (v);   }
inline float fast_atanf    (float v)          { return _fast_atanf_LUT     (v);   }
inline float fast_atan2f   (float y, float x) { return _fast_atan2f        (y,x); }
inline float fast_sqrtf    (float v)          { return _fast_sqrtf_LUT     (v);   }
inline float fast_invsqrtf (float v)          { return _fast_invsqrtf      (v);   }
inline float fast_invf     (float v)          { return _fast_invf_LUT      (v);   }
inline float fast_powf     (float b, int e)   { return _fast_powf_fu       (b,e); }


// DEFINITION OF INTERNAL FUNCTIONS

inline
float _fast_fabsf(float v)
{
    union {float f; unsigned int u;} i = {v};   // union, for aliasing
    i.u &= 0x7FFFFFFFu;                         // bitmask, set MSB to 0
    return i.f;                                 // done, neat !
}


float _fast_sinf(float v)
{
    int q = -(int)(v<0.0f);     // get number of shifts from quadrant I
    q = q + (int)(v*F_2_PI);    // get number of shifts from quadrant I
    v = v - (float)(q)*F_PI_2;  // our angle now fits in the range [0,PI_2]

    q = q&3;    // quadrant number = number of quadrant shifts modulo 4

    if (q==1 || q==3) v = F_PI_2 - v;   // quad II or IV, complementary angle
    if (q==2 || q==3) v = -v;           // quad III or IV, flip sign of angle

    const float v2 = v*v;
    const float c1 = 0.99989187717437744000f;  //  1.000000f ~= 1/1! = 1/1
    const float c2 =-0.16596019268035889000f;  // -0.166667f ~= 1/3! = 1/6
    const float c3 = 0.00760292448103427890f;  //  0.008333f ~= 1/5! = 1/120

    v = v*(c1+v2*(c2+v2*(c3)));  // compute Taylor series terms and return
    return v;
}


float _fast_sinf_pmPI2_o5(float v) // angle should be in the range -pi/2 to pi/2
{                                  // rel_err < 0.01083%
    const float v2 = v*v;
    const float c1 = 0.99989181756973267000f;   //  1/1
    const float c2 =-0.16596014797687531000f;   // -1/6
    const float c3 = 0.00760291656479239460f;   //  1/120

    return v*(c1+v2*(c2+v2*(c3)));
}


float _fast_sinf_pmPI2_o7(float v) // angle should be in the range -pi/2 to pi/2
{                                  // rel_err < 0.000385%
    const float v2 = v*v;
    const float c1 = 1.00000381469726560000f;   //  1/1
    const float c2 =-0.16668128967285156000f;   // -1/6
    const float c3 = 0.00833638478070497510f;   //  1/120
    const float c4 =-0.00019101370708085597f;   // -1/5040

    return v*(c1+v2*(c2+v2*(c3+v2*(c4))));
}


float _fast_sinf_pmPI_o9(float v)  // angle should be in the range -pi to pi
{                                  // rel_err < 0.1535 % on [-3.141592;3.141592]
    const float v2 = v*v;
    const float c1 = 0.99836528301239014000f;   //  1/1
    const float c2 =-0.16580872237682343000f;   // -1/6
    const float c3 = 0.00824440922588109970f;   //  1/120
    const float c4 =-0.00019879070168826729f;   // -1/5040
    const float c5 = 0.00000275448633146880f;   //  1/362880

    return v*(c1+v2*(c2+v2*(c3+v2*(c4+v2*(c5)))));
}


float _fast_sinf_pmPI_o11(float v) // angle should be in the range -pi to pi
{                                  // rel_err < 0.0178 % on [-3.141592;3.141592]
    const float v2 = v*v;
    const float c1 = 1.00017786026000980000f;   //  1/1
    const float c2 =-0.16675339639186859000f;   // -1/6
    const float c3 = 0.00834206677973270420f;   //  1/120
    const float c4 =-0.00019845737551804632f;   // -1/5040
    const float c5 = 0.00000275710931418871f;   //  1/362880
    const float c6 =-0.00000002506295615490f;   // -1/39916800

    return v*(c1+v2*(c2+v2*(c3+v2*(c4+v2*(c5+v2*(c6))))));
}


#define SIN_LUT_SIZE (2048)
unsigned int _fast_sinf_LUT_array[SIN_LUT_SIZE];
float* _fast_sinf_LUT_ptr = (float*)(&_fast_sinf_LUT_array);

void _fast_sinf_LUT_init()
{
    int i;
    for (i=0; i<SIN_LUT_SIZE; i++)
    {
        _fast_sinf_LUT_ptr[i] = sinf(i*(F_2PI/(float)(SIN_LUT_SIZE-1)));
    }
}

float _fast_sinf_LUT(float v)   // rel. err. on [0,2*PI] is... good enough
{
    int s = -(int)(v<0.0f);     // get number of 2*PI shifts
    s = s + (int)(v*F_1_2PI);   // get number of 2*PI shifts
    v = v - (float)(s)*F_2PI;   // our angle now fits angle in range [0,2*PI]
    
    const float m = v*(float)(SIN_LUT_SIZE-1)*F_1_2PI;
    const int i = (int)(m);                     // that's our LUT index
    const float y1 = _fast_sinf_LUT_ptr[i];     // lookup the smaller value
    const float y2 = _fast_sinf_LUT_ptr[i+1];   // lookup the larger value
    return y1 + (y2-y1)*(m-i);                  // linear approximation
}


float _fast_cosf(float v)
{
    int q = -(int)(v<0.0f);     // get number of shifts from quadrant I
    q = q + (int)(v*F_2_PI);    // get number of shifts from quadrant I
    v = v - (float)(q)*F_PI_2;  // our angle now fits in the range [0,PI_2]

    q = q&3;    // quadrant number = number of quadrant shifts modulo 4

    float s = 1.0f;     // multiplicator, flipping or not the sign of the cos

    if (q==1 || q==3) v = F_PI_2 - v;   // quad II or IV, complementary angle
    if (q==1 || q==2) s = -1.0f;        // quad II or III, flip sign of cos

    const float v2 = v*v;
    const float c1 = 0.99903213977813721000f;  //  1.000000f ~= 1/0! = 1/1
    const float c2 =-0.49264952540397644000f;  // -0.500000f ~= 1/2! = 1/2
    const float c3 = 0.03556701540946960400f;  //  0.041667f ~= 1/4! = 1/24

    return s*(c1+v2*(c2+v2*(c3)));  // compute Taylor series terms and return
}


float _fast_cosf_pmPI2_o4(float v) // angle should be in the range -pi/2 to pi/2
{                                  // rel_err < 0.27 % on [-1.570796;1.570796]
    const float v2 = v*v;
    const float c1 =  0.99737071990966797000f;  //  1/1
    const float c2 = -0.49096757173538208000f;  // -1/2
    const float c3 =  0.03515781834721565200f;  //  1/24

    return (c1+v2*(c2+v2*(c3)));
}


float _fast_cosf_pmPI2_o6(float v) // angle should be in the range -pi/2 to pi/2
{                                  // rel_err < 0.0042 % on [-1.570796;1.570796]
    const float v2 = v*v;
    const float c1 =  1.00007724761962890000f;  //  1/1
    const float c2 = -0.50003027915954590000f;  // -1/2
    const float c3 =  0.04151730611920356800f;  //  1/24
    const float c4 = -0.00126897636801004410f;  // -1/720

    return (c1+v2*(c2+v2*(c3+v2*(c4))));
}


float _fast_cosf_pmPI_o8(float v)  // angle should be in the range -pi to pi
{                                  // rel_err < 0.057 % on [-3.141592;3.141592]
    const float v2 = v*v;
    const float c1 =  0.99970918893814087000f;  //  1/1
    const float c2 = -0.50003212690353394000f;  // -1/2
    const float c3 =  0.04175465926527977000f;  //  1/24
    const float c4 = -0.00139491038862615820f;  // -1/720
    const float c5 =  0.00002210505954280961f;  //  1/40320
    //new best: max_re=0.0566591574 %

    return (c1+v2*(c2+v2*(c3+v2*(c4+v2*(c5)))));
}


float _fast_cosf_pmPI_o10(float v) // angle should be in the range -pi to pi
{                                  // rel_err < 0.0042 % on [-3.141592;3.141592]
    const float v2 = v*v;
    const float c1 =  0.99995851516723633000f;  //  1/1
    const float c2 = -0.49996098875999451000f;  // -1/6
    const float c3 =  0.04165990278124809300f;  //  1/120
    const float c4 = -0.00139067450072616340f;  // -1/720
    const float c5 =  0.00002518510882509872f;  //  1/40320
    const float c6 = -0.00000027364566790311f;  // -1/3628800

    return (c1+v2*(c2+v2*(c3+v2*(c4+v2*(c5+v2*(c6))))));
}


#define COS_LUT_SIZE (2048)
unsigned int _fast_cosf_LUT_array[COS_LUT_SIZE];
float* _fast_cosf_LUT_ptr = (float*)(&_fast_cosf_LUT_array);

void _fast_cosf_LUT_init()
{
    int i;
    for (i=0; i<COS_LUT_SIZE; i++)
    {
        _fast_cosf_LUT_ptr[i] = cosf(i*(F_2PI/(float)(COS_LUT_SIZE-1)));
    }
}

float _fast_cosf_LUT(float v)   // rel. error on [0,2*PI] is... good enough
{
    int s = -(int)(v<0.0f);     // get number of 2*PI shifts
    s = s + (int)(v*F_1_2PI);   // get number of 2*PI shifts
    v = v - (float)(s)*F_2PI;   // our angle now fits angle in range [0,2*PI]
    
    const float m = v*(float)(COS_LUT_SIZE-1)*F_1_2PI;
    const int i = (int)(m);                     // that's our LUT index
    const float y1 = _fast_cosf_LUT_ptr[i];      // lookup the smaller value
    const float y2 = _fast_cosf_LUT_ptr[i+1];    // lookup the larger value
    return y1 + (y2-y1)*(m-i);                  // linear approximation
}


float _fast_tanf(float v)       // 4 coefs : rel. err. on [0,PI/4] < 0.0195%
{                               // 3 coefs : rel. err. on [0,PI/4] < 0.062%
    #define MTS4C           // 2 coefs : rel. error on [0,PI/4] < 0.87%
    // TODO: use fmodf ?
    int q = -(int)(v<0.0f);     // get number of shifts from quadrant I
    q = q + (int)(v*F_2_PI);    // get number of shifts from quadrant I
    v = v - (float)(q)*F_PI_2;  // our angle now fits in the range [0,PI_2]
    
    q = q&3;    // quadrant number = number of quadrant shifts modulo 4
    
    float s = 1.0f;     // multiplicator, flipping or not the sign of the tan
    
    if (q&1) { s = -1.0f; v = F_PI_2-v; }   // quad I or III
    
    int invert = (v > F_PI_4);          
    if (invert) { v = F_PI_2-v; }
    
    #ifdef MTS2C    // modified Taylor series w/ 2 coefficients
        const float v2 = v*v;
        const float c1 = 0.99147701263427734000f;
        const float c2 = 0.43903526663780212000f;
        
        v = s*v*(c1+v2*(c2));
    #endif
    #ifdef MTS3C    // modified Taylor series w/ 3 coefficients
        const float v2 = v*v;
        const float c1 = 1.00061845779418950000f;
        const float c2 = 0.31590843200683594000f;
        const float c3 = 0.20226807892322540000f;
        
        v = s*v*(c1+v2*(c2+v2*(c3)));
    #endif
    #ifdef MTS4C    // modified Taylor series w/ 4 coefficients
        const float v2 = v*v;
        const float c1 = 1.00019323825836180000f;
        const float c2 = 0.32916274666786194000f;
        const float c3 = 0.14286723732948303000f;
        const float c4 = 0.06558319926261901900f;

        v = s*v*(c1+v2*(c2+v2*(c3+v2*(c4))));
    #endif
    
    if (invert) { v = 1.0f/v; }
    
    return v;
}


float _fast_tanf_alt(float v)   // 4 coefs : rel. err. on [-1.57,1.57] < 0.0061%
{                               // 3 coefs : rel. err. on [-1.57,1.57] < 0.162%
    #define EBS4C

    int q = 1 | -(v<0.0f);      // 1 or -1   ( int q = (v<0.0f)?-1:1; )
    q = q + (int)(v*F_1_PI);    // get number of PI shifts
    v = v - (float)(q)*F_PI;    // our angle now fits in the range [-PI_2,PI_2]
    
    // Good approximation with 2 vertical asymptotes :
    //   tan(x) = x/(x+PI_2) - x/(x-PI_2)
    // Now with scaling factor and corrective terms :
    //   tan(x) = a*x*(1/(x+PI_2)-1/(x-PI_2)) + b*x + c*x*x*x
    //     a = 0.63662, b = 0.18943, c = 0.0051
    // Simplified :
    //   tan(x) = x*(c1/(x*x-c2) + c3 + c4*x*x)
    //     c1 = a*PI, c2 = PI_2*PI_2, c3 = a*b, c4 = a*c
    
    #ifdef  EBS3C   // experimental bullshit with 3 coefficients
        const float c1 = -2.00423097610473630000f;
        const float c2 = -2.46741032600402830000f;
        const float c3 =  0.18933959305286407000f;
        const float v2 = v*v;
        
        return (c1/(c2+v2) + c3)*v;
    #endif
    #ifdef  EBS4C   // experimental bullshit with 4 coefficients
        const float c1 = -1.99996936321258540000f;
        const float c2 = -2.46740102767944340000f;
        const float c3 =  0.18938265740871429000f;
        const float c4 =  0.00516806356608867650f;
        const float v2 = v*v;
        
        return (c1/(c2+v2) + c3 + v2*c4)*v;
    #endif
}


#define TAN_LUT_SIZE (65536)
unsigned int _fast_tanf_LUT_array[TAN_LUT_SIZE];
float* _fast_tanf_LUT_ptr = (float*)(&_fast_tanf_LUT_array);

void _fast_tanf_LUT_init()
{
    int i;
    for (i=0; i<TAN_LUT_SIZE; i++)
    {
        _fast_tanf_LUT_ptr[i] = tanf(i*(F_2PI/(float)(TAN_LUT_SIZE-1)));
    }
}

float _fast_tanf_LUT(float v)
{
    int s = -(int)(v<0.0f);     // get number of 2*PI shifts
    s = s + (int)(v*F_1_2PI);   // get number of 2*PI shifts
    v = v - (float)(s)*F_2PI;   // our angle now fits angle in range [0,2*PI]
    
    const float m = v*(float)(TAN_LUT_SIZE-1)*F_1_2PI;
    const int i = (int)(m);                     // that's our LUT index
    const float y1 = _fast_tanf_LUT_ptr[i];     // lookup the smaller value
    const float y2 = _fast_tanf_LUT_ptr[i+1];   // lookup the larger value
    return y1 + (y2-y1)*(m-i);                  // linear approximation
}


float _fast_acosf(float v)  // TODO : characterize max rel. err.
{
    const float av = fabsf(v);
    
    if (av < 0.5f)      // 50% chance
    {
        const float c1 =  1.57210993766784670000f;
        const float c2 = -1.03992009162902830000f;
        return c1 + v*(c2);
    }
    if (av < 0.8f)      // 30% chance
    {
        const float c1 =  1.56902384757995610000f;
        const float c2 = -0.96116626262664795000f;
        const float c3 = -0.30158343911170959000f;
        const float v2  = v  *v  ;
        return c1 + v*(c2 + v2*(c3));
    }
    if (av < 0.9f)      // 10% chance
    {
        const float c1 =  1.56899595260620120000f;
        const float c2 = -0.96650576591491699000f;
        const float c3 = -0.22164604067802429000f;
        const float c4 = -0.17908957600593567000f;
        const float v2  = v  *v  ;
        const float v4  = v2 *v2 ;
        return c1 + v*(c2 + v2*(c3 + v4*(c4)));
    }
    if (av < 0.98f)     // 8% chance
    {
        const float c1 =  1.57075405120849610000f;
        const float c2 = -0.67591530084609985000f;
        const float c3 = -0.73794591426849365000f;
        const float c4 =  0.04363933950662612900f;
        const float c5 =  0.06028866022825241100f;
        const float c6 = -0.17903123795986176000f;
        const float v2  = v  *v  ;
        const float v4  = v2 *v2 ;
        const float v8  = v4 *v4 ;
        const float v16 = v8 *v8 ;
        return c1 + v*(c2 + v2*(c3 + v4*(c4 + v8*(c5 + v16*(c6)))));
    }
    if (av < 0.995f)    // 1.5% chance
    {
        const float c1 =  1.57020831108093260000f;
        const float c2 = -1.13344752788543700000f;
        const float c3 =  0.13332143425941467000f;
        const float c4 = -0.09504155814647674600f;
        const float c5 = -0.69747900962829590000f;
        const float c6 =  0.52639108896255493000f;
        const float c7 = -0.23976626992225647000f;
        const float c8 = -0.02003770694136619600f;
        const float v2  = v  *v  ;
        const float v4  = v2 *v2 ;
        const float v8  = v4 *v4 ;
        const float v16 = v8 *v8 ;
        const float v32 = v16*v16;
        const float v64 = v32*v32;
        return c1 + v*(c2 + v2*(c3 + v4*(c4 + v8*(c5 + v16*(c6 + v32*(c7 + v64*(c8)))))));
    }
    if (av <= 1.0f)     // 0.5% chance
    {
        if (v > 0.0f)   // TODO : reoptimize, especially near -1.0f
        {
            const float c1 =  0.00000752388814362348f;
            const float c2 =  0.99998962879180908000f;
            const float c3 =  0.00000517679336553556f;
            return c1 + sqrtf(c2 - (v-c3)*(v-c3));
        }
        else
        {
            const float c1 =  3.14150023460388180000f;
            const float c2 =  0.99992555379867554000f;
            const float c3 = -0.00003699455919559114f;
            return c1 - sqrtf(c2 - (v-c3)*(v-c3));
        }
    }
    else
    {
        return NAN;
    }
}


#define ASIN_LUT_SIZE (512-1)
unsigned int _fast_asinf_LUT_array[6][ASIN_LUT_SIZE+1];
float* _fast_asinf_LUT_ptr[6] = {
  (float*)(&_fast_asinf_LUT_array[0]), (float*)(&_fast_asinf_LUT_array[1]),
  (float*)(&_fast_asinf_LUT_array[2]), (float*)(&_fast_asinf_LUT_array[3]),
  (float*)(&_fast_asinf_LUT_array[4]), (float*)(&_fast_asinf_LUT_array[5])
};

void _fast_asinf_LUT_init()
{
    int i;
    for (i=0; i<=ASIN_LUT_SIZE; i++)
    {
        const float n = (float)i/(float)(ASIN_LUT_SIZE-1);
        _fast_asinf_LUT_ptr[0][i] = -asinf(0.000000f+0.900000f*n);
        _fast_asinf_LUT_ptr[1][i] = -asinf(0.900000f+0.090000f*n);
        _fast_asinf_LUT_ptr[2][i] = -asinf(0.990000f+0.009000f*n);
        _fast_asinf_LUT_ptr[3][i] = -asinf(0.999000f+0.000900f*n);
        _fast_asinf_LUT_ptr[4][i] = -asinf(0.999900f+0.000090f*n);
        _fast_asinf_LUT_ptr[5][i] = -asinf(0.999990f+0.000010f*n);
    }
}

float _fast_asinf_LUT(float v)      // rel. err. on [-1,1] < 0.004%
{                                   // max. err. near x = -0.899
    const float av = fabsf(v);
    const int is_neg = (v<0.0f);
    
    if (av <= 0.900000f)
    {
        const float m = ((av-0.000000f)/0.900000f)*((ASIN_LUT_SIZE-1));
        const int i = (int)(m);
        const float y1 = _fast_asinf_LUT_ptr[0][i];
        const float y2 = _fast_asinf_LUT_ptr[0][i+1];
        const float r = y1 + (y2-y1)*(m-i);
        return is_neg ? (r) : (-r);
    }
    if (av <= 0.990000f)
    {
        const float m = ((av-0.900000f)/0.090000f)*((ASIN_LUT_SIZE-1));
        const int i = (int)(m);
        const float y1 = _fast_asinf_LUT_ptr[1][i];
        const float y2 = _fast_asinf_LUT_ptr[1][i+1];
        const float r = y1 + (y2-y1)*(m-i);
        return is_neg ? (r) : (-r);
    }
    if (av <= 0.999000f)
    {
        const float m = ((av-0.990000f)/0.009000f)*((ASIN_LUT_SIZE-1));
        const int i = (int)(m);
        const float y1 = _fast_asinf_LUT_ptr[2][i];
        const float y2 = _fast_asinf_LUT_ptr[2][i+1];
        const float r = y1 + (y2-y1)*(m-i);
        return is_neg ? (r) : (-r);
    }
    if (av <= 0.999900f)
    {
        const float m = ((av-0.999000f)/0.000900f)*((ASIN_LUT_SIZE-1));
        const int i = (int)(m);
        const float y1 = _fast_asinf_LUT_ptr[3][i];
        const float y2 = _fast_asinf_LUT_ptr[3][i+1];
        const float r = y1 + (y2-y1)*(m-i);
        return is_neg ? (r) : (-r);
    }
    if (av <= 0.999990f)
    {
        const float m = ((av-0.999900f)/0.000090f)*((ASIN_LUT_SIZE-1));
        const int i = (int)(m);
        const float y1 = _fast_asinf_LUT_ptr[4][i];
        const float y2 = _fast_asinf_LUT_ptr[4][i+1];
        const float r = y1 + (y2-y1)*(m-i);
        return is_neg ? (r) : (-r);
    }
    if (av <= 1.000000f)
    {
        const float m = ((av-0.999990f)/0.000010f)*((ASIN_LUT_SIZE-1));
        const int i = (int)(m);
        const float y1 = _fast_asinf_LUT_ptr[5][i];
        const float y2 = _fast_asinf_LUT_ptr[5][i+1];
        const float r = y1 + (y2-y1)*(m-i);
        return is_neg ? (r) : (-r);
    }
    
    return NAN;
}


#define ACOS_LUT_SIZE ASIN_LUT_SIZE

void _fast_acosf_LUT_init() // acos() shares the same lookup table as asin()
{
    _fast_asinf_LUT_init();
}

float _fast_acosf_LUT(float v)      // rel. err. on [-1,1] < 0.1%
{                                   // max. err. near x = 0.99999
    const float av = fabsf(v);
    const int is_neg = (v<0.0f);
    
    if (av <= 0.900000f)
    {
        const float m = ((av-0.000000f)/0.900000f)*((ACOS_LUT_SIZE-1));
        const int i = (int)(m);
        const float y1 = _fast_asinf_LUT_ptr[0][i];
        const float y2 = _fast_asinf_LUT_ptr[0][i+1];
        const float r = y1 + (y2-y1)*(m-i);
        return is_neg ? (F_PI_2-r) : (F_PI_2+r);
    }
    if (av <= 0.990000f)
    {
        const float m = ((av-0.900000f)/0.090000f)*((ACOS_LUT_SIZE-1));
        const int i = (int)(m);
        const float y1 = _fast_asinf_LUT_ptr[1][i];
        const float y2 = _fast_asinf_LUT_ptr[1][i+1];
        const float r = y1 + (y2-y1)*(m-i);
        return is_neg ? (F_PI_2-r) : (F_PI_2+r);
    }
    if (av <= 0.999000f)
    {
        const float m = ((av-0.990000f)/0.009000f)*((ACOS_LUT_SIZE-1));
        const int i = (int)(m);
        const float y1 = _fast_asinf_LUT_ptr[2][i];
        const float y2 = _fast_asinf_LUT_ptr[2][i+1];
        const float r = y1 + (y2-y1)*(m-i);
        return is_neg ? (F_PI_2-r) : (F_PI_2+r);
    }
    if (av <= 0.999900f)
    {
        const float m = ((av-0.999000f)/0.000900f)*((ACOS_LUT_SIZE-1));
        const int i = (int)(m);
        const float y1 = _fast_asinf_LUT_ptr[3][i];
        const float y2 = _fast_asinf_LUT_ptr[3][i+1];
        const float r = y1 + (y2-y1)*(m-i);
        return is_neg ? (F_PI_2-r) : (F_PI_2+r);
    }
    if (av <= 0.999990f)
    {
        const float m = ((av-0.999900f)/0.000090f)*((ACOS_LUT_SIZE-1));
        const int i = (int)(m);
        const float y1 = _fast_asinf_LUT_ptr[4][i];
        const float y2 = _fast_asinf_LUT_ptr[4][i+1];
        const float r = y1 + (y2-y1)*(m-i);
        return is_neg ? (F_PI_2-r) : (F_PI_2+r);
    }
    if (av <= 1.000000f)
    {
        const float m = ((av-0.999990f)/0.000010f)*((ACOS_LUT_SIZE-1));
        const int i = (int)(m);
        const float y1 = _fast_asinf_LUT_ptr[5][i];
        const float y2 = _fast_asinf_LUT_ptr[5][i+1];
        const float r = y1 + (y2-y1)*(m-i);
        return is_neg ? (F_PI_2-r) : (F_PI_2+r);
    }
    
    return NAN;
}


#define ATAN_LUT_BITS 17            // max rel. err. < 0.1% with a 17-bit LUT
#define ATAN_LUT_SIZE (1<<ATAN_LUT_BITS)
#define ATAN_LUT_BS   (32-ATAN_LUT_BITS-1)  // number or bit shifts to get index

unsigned int _fast_atanf_LUT_array[ATAN_LUT_SIZE+1];

void _fast_atanf_LUT_init()
{
    union { float f; unsigned int u; } a, b, r;
    unsigned int i = 0;
    
    a.u = 0;                                    // compute first element
    r.f = atanf(a.f);
    _fast_atanf_LUT_array[i] = r.u;
    
    for (i=1; i<ATAN_LUT_SIZE; i++)
    {
        a.u = (i+0)<<ATAN_LUT_BS;               // generate current value
        b.u = (i+1)<<ATAN_LUT_BS;               // generate next value
        r.f = (atanf(a.f)+atanf(b.f))/2.0f;     // take average of their arctans
        _fast_atanf_LUT_array[i] = r.u;         // store in LUT
    }                               // averaging reduces by nearly 2 the error
    
    a.u = (ATAN_LUT_SIZE-1)<<ATAN_LUT_BS;       // compute last element
    r.f = atanf(a.f);
    _fast_atanf_LUT_array[i] = r.u;
}

inline
float _fast_atanf_LUT(float v)
{
    union { float f; unsigned int u; } t, r;
    unsigned int idx = 0;
    
    t.f = v;                                // copy input float value in union
    idx = (t.u & 0x7FFFFFFF)>>ATAN_LUT_BS;  // strip sign & gen. index for LUT
    r.u = _fast_atanf_LUT_array[idx];       // table look-up
    r.u = r.u | (t.u & 0x80000000);         // restore sign
    
    return r.f;
}


float _fast_atan2f(float y, float x)
{
    const float absy = fabsf(y);
    const float absx = fabsf(x);

    if (absy < 1e-5f)   // if y near 0, angle is 0 or PI, depending on x
    {
        if (x >= 0.0f) return 0.0f;
        else           return F_PI;
    }

    if (absx < 1e-5f)   // if x near 0, angle is +/-PI_2, depending on y
    {
        if (y >= 0.0f) return  F_PI_2;
        else           return -F_PI_2;
    }

    float v;    // our angle, our value

    const int inv = (int)(absx < absy);

    if (inv)  v = absx/absy;    // invert ratio if y > x in order to make sure
    else      v = absy/absx;    // the ratio remains in the valid range [0,1]

    // source: Fast approximate arctan/atan function - http://nghiaho.com/?p=997
    // angle = F_PI_4*v - v*(fabsf(v) - 1.0f)*(0.2447f + 0.0663f*fabsf(v));
    //
    // simplification & restriction...
    // y = 0.785398163397448309616*v - v*(abs(v) - 1.0)*(0.2447 + 0.0663*abs(v))
    // y = v * (0.785398 - ((abs(v) - 1.0)*(0.2447 + 0.0663*abs(v))))
    // y = v * (0.785398 - 0.2447*abs(v) - 0.0663*v*v + 0.2447 + 0.0663*abs(v))
    // y = v * (1.030098 - 0.1784*abs(v) - 0.0663*v*v)
    // y = v * (1.030098 - 0.1784*v - 0.0663*v*v)   // breaks [-1,0], not [0,1]
    // y = v * (1.030098 + v * ((-0.1784) + v * (-0.0663)))

    const float c1 = 1.00535535812377930000f;  //  1.030098f
    const float c2 =-0.08889085054397583000f;  // -0.178400f
    const float c3 =-0.13527311384677887000f;  // -0.066300f

    v = v*(c1 + v*(c2 + v*(c3)));   // compute angle

    if (inv)  v = F_PI_2 - v;       // if inverse ratio, complementary angle

    const int y_neg = (int)(y < 0);
    const int x_neg = (int)(x < 0);

    if (y_neg)  v = -v;     // if y is negative, flip sign of angle

    if (x_neg)              // if x is negative, supplementary angle
    {
        if (y_neg)  return -F_PI - v;
        else        return  F_PI - v;
    }

    return v;
}


static unsigned int _fast_sqrtf_LUT_array[0x10000u];    // 0x10000 = 65536 = 8192*8

void _fast_sqrtf_LUT_init()
{
    union {float f; unsigned int u;} i;
    unsigned int u;
    
    for (u=0; u<=0x7FFFu; u++)
    {
        // Build a float with the bit pattern u as mantissa
        //  and an exponent of 0, stored as 127
        i.u = (u << 8) | (0x7F << 23);
        i.f = (float)sqrt(i.f);
        
        // Take the square root then strip the first 7 bits of
        //  the mantissa into the table
        _fast_sqrtf_LUT_array[u + 0x8000u] = (i.u & 0x7FFFFFu);
        
        // Repeat the process, this time with an exponent of 1, 
        //  stored as 128
        i.u = (u << 8) | (0x80u << 23);
        i.f = (float)sqrt(i.f);
        
        _fast_sqrtf_LUT_array[u] = (i.u & 0x7FFFFFu);
    }
}

float _fast_sqrtf_LUT(float v)  // Paul Hsieh, http://www.azillionmonkeys.com/qed/sqroot.html
{
    union {float f; unsigned int u;} i = {v};
    
    if (i.u == 0)    return 0.0;     // check for square root of 0
    
    unsigned int dlo =     0xFFFFu;
    unsigned int bm0 = 0x3F800000u; // 00111111100000000000000000000000
    unsigned int bm1 = 0x7F800000u; // 01111111100000000000000000000000
    
    i.u = _fast_sqrtf_LUT_array[(i.u >> 8) & dlo] | ((((i.u - bm0) >> 1) + bm0) & bm1);
    
    return i.f;
}


inline
float _fast_sqrtf_1(float x)    // rel. err. < 0.5%
{                               // http://bits.stephan-brumme.com/squareRoot.html
    union {float f; unsigned int i;} u, v, w;
    float err;

    u.f = x;
    u.i = (u.i + (127 << 23)) >> 1;     // rel. err. < 6.1%
    
    err = x - u.f*u.f;
    v.f = x + 0.874996f*err;
    v.i = (v.i + (127 << 23)) >> 1;     // rel. err. < 1.7%
    
    err = 2.0f*x - u.f*u.f - v.f*v.f;
    w.f = x + 0.874996f*err;
    w.i = (w.i + (127 << 23)) >> 1;     // rel. err. < 0.5%
    
    return w.f;
}


inline
float _fast_sqrtf_2(float x)
{
    union {float f; unsigned int i;} u, v, w;
    float err;
    
    w.i = 0x3F6DFFBD;               // 0x3F6DFFBD

    u.f = x;
    u.i = 0x1FBB9FE5 + (u.i >> 1);  // 0x1FBB9FE5
    
    err = x - u.f*u.f;
    v.f = x + w.f*err;
    v.i = 0x1FBBF9E3 + (v.i >> 1);  // 0x1FBBF9E3   (MAX ERROR = 1.059069 %)
    
    return v.f;
}


inline
float _fast_sqrtf_3(float x)    // rel. err. < 0.5%
{                               // http://bits.stephan-brumme.com/squareRoot.html
    union {float f; unsigned int i;} u, v, w;

    u.f = x;
    u.i = (u.i + (127 << 23)) >> 1;     // rel. err. < 6.1%
    
    /*  Newton Raphson on sqrt()          | v = x^2
        x2 = x1 - f(x1) / f'(x1)          | f (x) = x^2 - v
        x2 = x1 - (x1^2 - v)/(2*x1)       | f'(x) = 2*x
    */
    
    v.i = 0x7EF311C2 - u.i;
    w.f = u.f - 0.5f*(u.f*u.f - x)*v.f;     // rel. err. < 0.03%
    
    return w.f;
}


inline
float _fast_sqrtf_4(float v)    // rel. err. < 3.5%
{
    union {float f; unsigned int i;} x;
    
    x.f = v;
    
  //x.i = 0x5F375A86 - (x.i>>1);    // Lomont's reciprocal sqrt approx.
  //x.i = 0x7EF311C2 - x.i;         // fast multipicative inverse approx.
  
  //x.i = 0x1FBBB73C + (x.i>>1);    // 0x1FBBB73C  (MAX ERROR = 3.699261 %)
    
    x.i = 0x1FBB4F2F + (x.i>>1);    // 0x1FBB4F2F  (MAX ERROR = 3.474728 %)
    
    return x.f;
}


inline
float _fast_sqrtf_5(float v)
{
    union {float f; unsigned int i;} x, t;
    
    x.f = v;
    
  //x.i = 0x1FBB4F2F + (x.i>>1);    // Lomont + multiplicative inverse guess
  //x.f = ((v/x.f) + x.f) / 2.0f;   (MAX ERROR = 0.062461 %)
    
    x.i = 0x1FBBE99C + (x.i>>1);    // 0x1FBBE99C   (sqrt approx)
    
    t.i = 0x7EF37662 - x.i;         // 0x7EF37662   (reciprocal approx)
    t.f = t.f * (2.0f - x.f*t.f);   // Newton-Raphson on reciprocal
    x.f = (v*t.f + x.f);            // Newton-Raphson on sqrt
    
    x.i = x.i - 0x00800004;         // 0x00800004   (division by 2 approx)
    
    return x.f;                     // (MAX ERROR = 0.109105 %)
}


inline
float _fast_sqrtf_6(float v)
{
    return v * _fast_invsqrtf(v);
    
  //return 1.0f/fast_invsqrtf(v);       // am I dumb, or what ?
}


inline
float _fast_invsqrtf(float v)   // rel. err. < 0.07%
{                               // Jan Kaldec, http://rrrola.wz.cz/inv_sqrt.html
    union {float f; unsigned int u;} i = {v};
    i.u = 0x5F1FFFF9u - (i.u >> 1);
    return 0.703952253f * i.f * (2.38924456f - v * i.f * i.f);
}


inline
float _fast_invf(float v)       // http://bits.stephan-brumme.com/inverse.html
{
    union {float f; unsigned int i;} u;
    
    u.f = v;
    u.i = 0x7EF311C2u - u.i;    // 0x7F000000 : 12.5% max rel err, inv(1) == 1
                                // 0x7EEEEEEE : 6.67% max rel err, inv(1) != 1
    return u.f;                 // 0x7EF311C2 : 5.06% max rel err, inv(1) != 1
}


#define INV_LUT_BS      8                   // 8-bit LUT ==> 256-element LUT
#define INV_LUT_SIZE    (1<<INV_LUT_BS)
unsigned int fast_invf_LUT[INV_LUT_SIZE];

void _fast_invf_LUT_init()
{
    const unsigned int strt = 0x40000000u;              // 0x40000000u = 2.0f
    const unsigned int step =   0x800000u/INV_LUT_SIZE; // 0x40800000u = 4.0f
    
    unsigned int i;
    for (i=0; i<INV_LUT_SIZE; i++)  // optimize every coefficient in the LUT
    {
        fast_invf_LUT[i] = 0x7F000000u; // initialize magic number
        
        unsigned int d = 0x80000u;      // delta around magic number
        unsigned int s = 1;             // sign of progression
        
        float remin = FLT_MAX;          // init min relative error
        
        while (d > 1)   // magic number optimization loop
        {
            union {float f; unsigned int u;} t1, t2;
            
            t1.u = strt + (i+0)*step;   // lower bound (LB)
            t2.u = strt + (i+1)*step;   // upper bound (UB)
            
            const float r1 = 1.0f/t1.f; // inverse of LB, reference
            const float r2 = 1.0f/t2.f; // inverse of UB, reference
            
            const unsigned int magic = fast_invf_LUT[i] + ( (s&1) ? -d : d );
            
            t1.u = magic-t1.u;          // inverse of LB, approximation
            t2.u = magic-t2.u;          // inverse of UB, approximation
            
            const float re1 = fabsf(t1.f/r1-1.0f);      // rel. err. made on LB
            const float re2 = fabsf(t2.f/r2-1.0f);      // rel. err. made on UB
            const float re  = (re1>re2) ? re1 : re2;    // select largest error
            
            if (re < remin)     // found a better magic number ? then...
            {
                remin = re;                 // update min relative error
                fast_invf_LUT[i] = magic;   // update magic number in LUT
            }
            else                // otherwise...
            {
                d >>= 2;                    // divide delta by 4
                s ^= 1;                     // alternate s between 0 and 1
            }
        }
    }
}

inline
float _fast_invf_LUT(float x)    // rel. err. < 0.1% for 8-bit LUT
{
    union {float f; unsigned int u;} v = {x};
    const unsigned int idx = (v.u&0x7FFFFFu)>>(23-INV_LUT_BS);
    v.u = fast_invf_LUT[idx] - v.u;
    return v.f;
}


float _fast_powf_fs(float b, int e)      // POWER(float,signed)
{
    if (e < 0) b = 1.0f/b;      // for negative powers, invert base
    e = abs(e);                 // and raise to positive power

    float acc = 1.0f;           // init accumulator
    
    while (e)
    {
        if (e&1) acc *= b;      // if LSB of exponent set, mult. by base
        e >>= 1;                // shift out LSB of exponent
        b *= b;                 // square base
    }

    return acc;
}


float _fast_powf_fu(float b, int e)      // POWER(float,unsigned)
{
    float acc = 1.0f;           // init accumulator
    
    while (e)
    {
        if (e&1) acc *= b;      // if LSB of exponent set, mult. by base
        e >>= 1;                // shift out LSB of exponent
        b *= b;                 // square base
    }

    return acc;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int _is_big_endian()
{
#ifdef COMPILE_ON_ROBOT
    return ALT_CPU_BIG_ENDIAN;      // TODO : why this?
#else
    union {char c[2]; short s;} i;
    i.c[0] = (char)1;
    i.c[1] = (char)0;
    return i.s != 1;
#endif
}


// TODO: add fast_pow to benchmark
void fast_benchmark(void)
{
    #ifdef COMPILE_ON_ROBOT
     #define UPTIME_GET (uptime_get())
     #define UT "us"
     #define NL "\r"    // newline char
    #else
     #define UPTIME_GET ((uint32_t)clock())
     #define UT "clk"
     #define NL "\n"    // newline char
    #endif

    const int CNT = 10000;
    
    uint32_t t;
    
    printf("***Start Benchmark***"NL);
    printf(NL);
    
    printf("endianness: %s endian"NL, _is_big_endian() ? "big" : "little");
    printf(NL);

    printf("Initializing LUT... ");
    
    t = UPTIME_GET;
    fast_math_init();
    printf("[%d "UT"]"NL, (int)(UPTIME_GET-t));
    printf(NL);
    
    int i;
    double r_d = 0.0;
    float  r_f = 0.0f;
    int    r_i = 0;

    double pos_d[CNT+1];    // values between 0 and 1000 (positive)
    float  pos_f[CNT+1];
    
    double png_d[CNT+1];    // values between -1000 and 1000 (Pos'N'neG)
    float  png_f[CNT+1];
    int    png_i[CNT+1];
    
    double trg_d[CNT+1];    // values between -3.141592 and 3.141592 (trigo)
    float  trg_f[CNT+1];
    
    double uni_d[CNT+1];    // values between -1 and 1 (unity)
    float  uni_f[CNT+1];
    
    srand(time(0));
    
    for(i = 0; i < CNT+1; i++)
    {
        double v = (double)rand()/(double)RAND_MAX; // random val in range [0;1]
        
        pos_d[i] = v*1000.0;
        pos_f[i] = (float)pos_d[i];
        
        png_d[i] = (v-0.5)*2000.0;
        png_f[i] = (float)png_d[i];
        png_i[i] = (int)  png_d[i];
        
        trg_d[i] = (v-0.5)*2.0;
        trg_f[i] = (float)trg_d[i];
        
        uni_d[i] = (v-0.5)*3.141592;
        uni_f[i] = (float)uni_d[i];
    }

    // max relative error tests
    double mre_sinf      = 0.0;
    double mre_sinff     = 0.0;
    double mre_cosf      = 0.0;
    double mre_cosff     = 0.0;
    double mre_tanf      = 0.0;
    double mre_tanff     = 0.0;
    double mre_asinf     = 0.0;
    double mre_asinff    = 0.0;
    double mre_acosf     = 0.0;
    double mre_acosff    = 0.0;
    double mre_atanf     = 0.0;
    double mre_atanff    = 0.0;
    double mre_atan2f    = 0.0;
    double mre_atan2ff   = 0.0;
    double mre_sqrtf     = 0.0;
    double mre_sqrtff    = 0.0;
    double mre_invsqrtf  = 0.0;
    double mre_invsqrtff = 0.0;
    double mre_invf      = 0.0;
    double mre_invff     = 0.0;
    double mre_powf      = 0.0;
    double mre_powff     = 0.0;
    
    double tol = 1e-8;          // tolerance
    
    printf("Checking sin..."NL);
    for(i=0; i<CNT; i++)
    {
        double d = sin(trg_d[i]);
        float f = sinf(trg_f[i]);
        float ff = fast_sinf(trg_f[i]);

        if (fabs(f -d) < tol) continue;    // too small difference, skip
        if (fabs(ff-d) < tol) continue;    // too small difference, skip
        if (fabs(   d) < tol) continue;    // too small divider, skip

        double re_f  = fabs(1.0 - (double)f /(double)d);
        double re_ff = fabs(1.0 - (double)ff/(double)d);

        if (re_f  > mre_sinf ) mre_sinf  = re_f;
        if (re_ff > mre_sinff) mre_sinff = re_ff;
    }
    printf("Checking cos..."NL);
    for(i=0; i<CNT; i++)
    {
        double d = cos(trg_d[i]);
        float f = cosf(trg_f[i]);
        float ff = fast_cosf(trg_f[i]);

        if (fabs(f -d) < tol) continue;    // too small difference, skip
        if (fabs(ff-d) < tol) continue;    // too small difference, skip
        if (fabs(   d) < tol) continue;    // too small divider, skip

        double re_f  = fabs(1.0 - (double)f /(double)d);
        double re_ff = fabs(1.0 - (double)ff/(double)d);

        if (re_f  > mre_cosf ) mre_cosf  = re_f;
        if (re_ff > mre_cosff) mre_cosff = re_ff;
    }
    printf("Checking tan..."NL);
    for(i=0; i<CNT; i++)
    {
        double d = tan(png_d[i]);
        float f = tanf(png_f[i]);
        float ff = fast_tanf(png_f[i]);

        if (fabs(f -d) < tol) continue;    // too small difference, skip
        if (fabs(ff-d) < tol) continue;    // too small difference, skip
        if (fabs(   d) < tol) continue;    // too small divider, skip

        double re_f  = fabs(1.0 - (double)f /(double)d);
        double re_ff = fabs(1.0 - (double)ff/(double)d);

        if (re_f  > mre_tanf ) mre_tanf  = re_f;
        if (re_ff > mre_tanff) mre_tanff = re_ff;
    }
    printf("Checking asin..."NL);
    for(i=0; i<CNT; i++)
    {
        double d = asin(uni_d[i]);
        float f = asinf(uni_f[i]);
        float ff = fast_asinf(uni_f[i]);

        if (fabs(f -d) < tol) continue;    // too small difference, skip
        if (fabs(ff-d) < tol) continue;    // too small difference, skip
        if (fabs(   d) < tol) continue;    // too small divider, skip

        double re_f  = fabs(1.0 - (double)f /(double)d);
        double re_ff = fabs(1.0 - (double)ff/(double)d);

        if (re_f  > mre_asinf ) mre_asinf  = re_f;
        if (re_ff > mre_asinff) mre_asinff = re_ff;
    }
    printf("Checking acos..."NL);
    for(i=0; i<CNT; i++)
    {
        double d = acos(uni_d[i]);
        float f = acosf(uni_f[i]);
        float ff = fast_acosf(uni_f[i]);

        if (fabs(f -d) < tol) continue;    // too small difference, skip
        if (fabs(ff-d) < tol) continue;    // too small difference, skip
        if (fabs(   d) < tol) continue;    // too small divider, skip

        double re_f  = fabs(1.0 - (double)f /(double)d);
        double re_ff = fabs(1.0 - (double)ff/(double)d);

        if (re_f  > mre_acosf ) mre_acosf  = re_f;
        if (re_ff > mre_acosff) mre_acosff = re_ff;
    }
    printf("Checking atan..."NL);
    for(i=0; i<CNT; i++)
    {
        double d = atan(png_d[i]);
        float f = atanf(png_f[i]);
        float ff = fast_atanf(png_f[i]);

        if (fabs(f -d) < tol) continue;    // too small difference, skip
        if (fabs(ff-d) < tol) continue;    // too small difference, skip
        if (fabs(   d) < tol) continue;    // too small divider, skip

        double re_f  = fabs(1.0 - (double)f /(double)d);
        double re_ff = fabs(1.0 - (double)ff/(double)d);

        if (re_f  > mre_atanf ) mre_atanf  = re_f;
        if (re_ff > mre_atanff) mre_atanff = re_ff;
    }
    printf("Checking atan2..."NL);
    for(i=0; i<CNT; i++)
    {
        double d = atan2(png_d[i], png_d[i+1]);
        float f = atan2f(png_f[i], png_f[i+1]);
        float ff = fast_atan2f(png_f[i], png_f[i+1]);

        if (fabs(f -d) < tol) continue;    // too small difference, skip
        if (fabs(ff-d) < tol) continue;    // too small difference, skip
        if (fabs(   d) < tol) continue;    // too small divider, skip

        double re_f  = fabs(1.0 - (double)f /(double)d);
        double re_ff = fabs(1.0 - (double)ff/(double)d);

        if (re_f  > mre_atan2f ) mre_atan2f  = re_f;
        if (re_ff > mre_atan2ff) mre_atan2ff = re_ff;
    }
    printf("Checking sqrt..."NL);
    for(i=0; i<CNT; i++)
    {
        double d = sqrt(pos_d[i]);
        float f = sqrtf(pos_f[i]);
        float ff = fast_sqrtf(pos_f[i]);

        if (fabs(f -d) < tol) continue;    // too small difference, skip
        if (fabs(ff-d) < tol) continue;    // too small difference, skip
        if (fabs(   d) < tol) continue;    // too small divider, skip

        double re_f  = fabs(1.0 - (double)f /(double)d);
        double re_ff = fabs(1.0 - (double)ff/(double)d);

        if (re_f  > mre_sqrtf ) mre_sqrtf  = re_f;
        if (re_ff > mre_sqrtff) mre_sqrtff = re_ff;
    }
    printf("Checking invsqrt..."NL);
    for(i=0; i<CNT; i++)
    {
        double d = 1.0/sqrt(pos_d[i]);
        float f = 1.0f/sqrtf(pos_f[i]);
        float ff = fast_invsqrtf(pos_f[i]);

        if (fabs(f -d) < tol) continue;    // too small difference, skip
        if (fabs(ff-d) < tol) continue;    // too small difference, skip
        if (fabs(   d) < tol) continue;    // too small divider, skip

        double re_f  = fabs(1.0 - (double)f /(double)d);
        double re_ff = fabs(1.0 - (double)ff/(double)d);

        if (re_f  > mre_invsqrtf ) mre_invsqrtf  = re_f;
        if (re_ff > mre_invsqrtff) mre_invsqrtff = re_ff;
    }
    printf("Checking inv..."NL);
    for(i=0; i<CNT; i++)
    {
        double d = 1.0/png_d[i];
        float f = 1.0f/png_f[i];
        float ff = fast_invf(png_f[i]);

        if (fabs(f -d) < tol) continue;    // too small difference, skip
        if (fabs(ff-d) < tol) continue;    // too small difference, skip
        if (fabs(   d) < tol) continue;    // too small divider, skip

        double re_f  = fabs(1.0 - (double)f /(double)d);
        double re_ff = fabs(1.0 - (double)ff/(double)d);

        if (re_f  > mre_invf ) mre_invf  = re_f;
        if (re_ff > mre_invff) mre_invff = re_ff;
    }
    printf("Checking pow..."NL);
    for(i=0; i<CNT; i++)
    {
        double d = pow(png_d[i], png_i[i+1]%70);
        float f = powf(png_f[i], png_i[i+1]%70);
        float ff = fast_powf(png_f[i], png_i[i+1]%70);

        if (fabs(f -d) < tol) continue;    // too small difference, skip
        if (fabs(ff-d) < tol) continue;    // too small difference, skip
        if (fabs(   d) < tol) continue;    // too small divider, skip

        double re_f  = fabs(1.0 - (double)f /(double)d);
        double re_ff = fabs(1.0 - (double)ff/(double)d);

        if (re_f  > mre_invf ) mre_invf  = re_f;
        if (re_ff > mre_invff) mre_invff = re_ff;
    }
    
    printf(NL);
    printf("Each function is called %d times"NL, i);
    printf(NL);

    for(t=UPTIME_GET, i=0; i<CNT; i++) r_d = png_d[i] * png_d[i+1];
    printf("     mul (double)     : %7lu "UT", %f"NL,(long unsigned int)(UPTIME_GET-t), r_d);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = png_f[i] * png_f[i+1];
    printf("     mul (float)      : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_f);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_i = png_i[i] * png_i[i+1];
    printf("     mul (int)        : %7lu "UT", %i"NL, (long unsigned int)(UPTIME_GET-t), r_i);
    printf(NL);

    for(t=UPTIME_GET, i=0; i<CNT; i++) r_d = png_d[i] / png_d[i+1];
    printf("     div (double)     : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_d);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = png_f[i] / png_f[i+1];
    printf("     div (float)      : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_f);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_i = png_i[i] / png_i[i+1];
    printf("     div (int)        : %7lu "UT", %i"NL, (long unsigned int)(UPTIME_GET-t), r_i);
    printf(NL);

    for(t=UPTIME_GET, i=0; i<CNT; i++) r_d = fabs(png_d[i]);
    printf("     fabs (double)    : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_d);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = fabsf(png_f[i]);
    printf("     fabsf (float)    : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_f);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = fast_fabsf(png_f[i]);
    printf("fast_fabsf (float)    : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_f);
    printf(NL);

    for(t=UPTIME_GET, i=0; i<CNT; i++) r_d = sin(trg_d[i]);
    printf("     sin (double)     : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_d);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = sinf(trg_f[i]);
    printf("     sinf (float)     : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_sinf);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = fast_sinf(trg_f[i]);
    printf("fast_sinf (float)     : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_sinff);
    printf(NL);

    for(t=UPTIME_GET, i=0; i<CNT; i++) r_d = cos(trg_d[i]);
    printf("     cos (double)     : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_d);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = cosf(trg_f[i]);
    printf("     cosf (float)     : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_cosf);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = fast_cosf(trg_f[i]);
    printf("fast_cosf (float)     : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_cosff);
    printf(NL);

    for(t=UPTIME_GET, i=0; i<CNT; i++) r_d = tan(png_d[i]);
    printf("     tan (double)     : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_d);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = tanf(png_f[i]);
    printf("     tanf (float)     : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_tanf);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = fast_tanf(png_f[i]);
    printf("fast_tanf (float)     : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_tanff);
    printf(NL);
    
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_d = asin(uni_d[i]);
    printf("     asin (double)    : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_d);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = asinf(uni_f[i]);
    printf("     asinf (float)    : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_asinf);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = fast_asinf(uni_f[i]);
    printf("fast_asinf (float)    : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_asinff);
    printf(NL);

    for(t=UPTIME_GET, i=0; i<CNT; i++) r_d = acos(uni_d[i]);
    printf("     acos (double)    : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_d);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = acosf(uni_f[i]);
    printf("     acosf (float)    : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_acosf);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = fast_acosf(uni_f[i]);
    printf("fast_acosf (float)    : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_acosff);
    printf(NL);

    for(t=UPTIME_GET, i=0; i<CNT; i++) r_d = atan(png_d[i]);
    printf("     atan (double)    : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_d);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = atanf(png_f[i]);
    printf("     atanf (float)    : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_atanf);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = fast_atanf(png_f[i]);
    printf("fast_atanf (float)    : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_atanff);
    printf(NL);

    for(t=UPTIME_GET, i=0; i<CNT; i++) r_d = atan2(png_d[i], png_d[i+1]);
    printf("     atan2 (double)   : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_d);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = atan2f(png_f[i], png_f[i+1]);
    printf("     atan2f (float)   : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_atan2f);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = fast_atan2f(png_f[i], png_f[i+1]);
    printf("fast_atan2f (float)   : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_atan2ff);
    printf(NL);

    for(t=UPTIME_GET, i=0; i<CNT; i++) r_d = sqrt(pos_d[i]);
    printf("     sqrt (double)    : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_d);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = sqrtf(pos_f[i]);
    printf("     sqrtf (float)    : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_sqrtf);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = fast_sqrtf(pos_f[i]);
    printf("fast_sqrtf (float)    : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_sqrtff);
    printf(NL);

    for(t=UPTIME_GET, i=0; i<CNT; i++) r_d = 1.0/sqrt(pos_d[i]);
    printf(" 1.0/sqrt (double)    : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_d);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = 1.0f/sqrtf(pos_f[i]);
    printf("1.0f/sqrtf (float)    : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_invsqrtf);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = fast_invsqrtf(pos_f[i]);
    printf("fast_invsqrtf (float) : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_invsqrtff);
    printf(NL);

    for(t=UPTIME_GET, i=0; i<CNT; i++) r_d = 1.0/png_d[i];
    printf(" 1.0/(double)         : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_d);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = 1.0f/png_f[i];
    printf("1.0f/(float)          : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_invf);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = fast_invf(png_f[i]);
    printf("fast_invf (float)     : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_invff);
    printf(NL);

    for(t=UPTIME_GET, i=0; i<CNT; i++) r_d = pow(png_d[i], png_i[i+1]%70);
    printf("     pow (double)   : %7lu "UT", %f"NL, (long unsigned int)(UPTIME_GET-t), r_d);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = powf(png_f[i], png_i[i+1]%70);
    printf("     powf (float)   : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_powf);
    for(t=UPTIME_GET, i=0; i<CNT; i++) r_f = fast_powf(png_f[i], png_i[i+1]%70);
    printf("fast_powf (float)   : %7lu "UT", %f (%lf %%)"NL, (long unsigned int)(UPTIME_GET-t), r_f, 100.0*mre_powff);
    printf(NL);

    printf("*** End Benchmark ***"NL);
    printf(NL);
}


/* Test du 24 avril 2012
***Start Benchmark***
dot (double) 1000 times:13257 us
dot (int) 1000 times:243 us
dot (float) 1000 times:2697 us
divide (double) 1000 times:22387 us
divide (int) 1000 times:268 us
divide (float) 1000 times:8113 us
sqrt (double) 1000 times:18404 us
sqrt (float) 1000 times:7820 us
sin (double) 1000 times:149762 us
sinf (float) 1000 times:66097 us
atan2 (double) 1000 times:259504 us
atan2f (float) 1000 times:98125 us
***End Benchmark**
***End **/


/* Test du 30 avril 2012
endianness: little endian
***Start Benchmark***
Each function called 1000 times
     mul (double)     :   10356 us, 713008.740185
     mul (float)      :     575 us, 713008.750000
     mul (int)        :     477 us, 712632
     div (double)     :   20467 us, 9.353919
     div (float)      :    5290 us, 9.353919
     div (int)        :     486 us, 9
     fabs (double)    :     381 us, 2582.523193
     fabsf (float)    :     311 us, 2582.523193
fast_fabsf (float)    :     575 us, 2582.523193
     sqrt (double)    :   16180 us, 50.818532
     sqrtf (float)    :    5361 us, 50.818531 (0.000006 %)
fast_sqrtf (float)    :    1422 us, 50.818302 (0.001476 %)
fast_invsqrtf (float) :    1380 us, 0.019690
     sin (double)     :  227710 us, 0.133631
     sinf (float)     :  962491 us, 0.133631 (0.000008 %)
fast_sinf (float)     :    6818 us, 0.133619 (0.075732 %)
     cos (double)     :  229336 us, 0.991031
     cosf (float)     :  961424 us, 0.991031 (0.000011 %)
fast_cosf (float)     :    6822 us, 0.990193 (0.343873 %)
     atan2 (double)   :  262330 us, 1.464294
     atan2f (float)   :   97054 us, 1.464294 (0.000010 %)
fast_atan2f (float)   :   17131 us, 1.464498 (0.535477 %)
*** End Benchmark ***/


/* Test du 17 mars 2013
***Start Benchmark***
endianness: little endian
Initializing LUT... [7620 clk]
Checking sin...
Checking cos...
Checking tan...
Checking asin...
Checking acos...
Checking atan...
Checking atan2...
Checking sqrt...
Checking invsqrt...
Checking inv...
Each function is called 10000 times
     mul (double)     :     137 clk, 161602.918803
     mul (float)      :       5 clk, 161602.937500
     mul (int)        :       4 clk, 161090
     div (double)     :     218 clk, 5.075616
     div (float)      :      71 clk, 5.075616
     div (int)        :       4 clk, 5
     fabs (double)    :       3 clk, 905.667939
     fabsf (float)    :       3 clk, 905.667969
fast_fabsf (float)    :       7 clk, 905.667969
     sin (double)     :    1595 clk, -0.786838
     sinf (float)     :     467 clk, -0.786838 (0.000000 %)
fast_sinf (float)     :      27 clk, -0.786940 (0.000000 %)
     cos (double)     :    1841 clk, 0.617160
     cosf (float)     :     511 clk, 0.617160 (0.000000 %)
fast_cosf (float)     :      25 clk, 0.617145 (0.000000 %)
     tan (double)     :    4395 clk, -1.232720
     tanf (float)     :    6779 clk, -1.232795 (48.565084 %)
fast_tanf (float)     :     144 clk, -1.243242 (175611.437255 %)
     asin (double)    :    2019 clk, nan
     asinf (float)    :     636 clk, nan (0.000000 %)
fast_asinf (float)    :     150 clk, nan (0.000000 %)
     acos (double)    :    1833 clk, nan
     acosf (float)    :     581 clk, nan (0.000000 %)
fast_acosf (float)    :     165 clk, nan (0.000000 %)
     atan (double)    :    2701 clk, -1.569692
     atanf (float)    :     706 clk, -1.569692 (0.000000 %)
fast_atanf (float)    :      12 clk, -1.569692 (0.000000 %)
     atan2 (double)   :    3008 clk, -1.765325
     atan2f (float)   :     830 clk, -1.765325 (0.000000 %)
fast_atan2f (float)   :     188 clk, -1.764387 (0.000000 %)
     sqrt (double)    :     167 clk, 6.867753
     sqrtf (float)    :      64 clk, 6.867753 (0.000009 %)
fast_sqrtf (float)    :      15 clk, 6.867752 (0.001460 %)
 1.0/sqrt (double)    :     400 clk, 0.145608
1.0f/sqrtf (float)    :     108 clk, 0.145608 (0.000000 %)
fast_invsqrtf (float) :      15 clk, 0.145562 (0.000000 %)
 1.0/(double)         :     210 clk, -0.001104
1.0f/(float)          :      52 clk, -0.001104 (0.000000 %)
fast_invf (float)     :       7 clk, -0.001104 (0.000000 %)
*** End Benchmark ***/
