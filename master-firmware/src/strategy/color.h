#ifndef STRATEGY_COLOR_H
#define STRATEGY_COLOR_H

#ifdef __cplusplus
extern "C" {
#endif

/** Team color
 */
enum strat_color_t {
    YELLOW = 0,
    VIOLET
};

/** Compute the symmetrical position depending on color
 */
#define MIRROR_X(color, x) (color == VIOLET ? (x) : 3000.f - (x))
#define MIRROR_A(color, a_deg) (color == VIOLET ? (a_deg) : 180.f - (a_deg))
#define MIRROR(color, value) (color == VIOLET ? (value) : -(value))

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_COLOR_H */
