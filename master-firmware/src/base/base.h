#ifndef BASE_H
#define BASE_H

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    float left;
    float right;
} wheels_t;

typedef struct {
    float distance;
    float angle;
} polar_t;

typedef struct {
    float x;        // [m]
    float y;        // [m]
    float heading;  // [rad]
} pose2d_t;



#ifdef __cplusplus
}
#endif

#endif /* BASE_H */
