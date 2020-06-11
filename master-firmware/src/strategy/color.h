#ifndef STRATEGY_COLOR_H
#define STRATEGY_COLOR_H

#ifdef __cplusplus
extern "C" {
#endif

/** Team color
 */
enum strat_color_t {
    BLUE = 0,
    YELLOW
};

/** Compute the symmetrical position depending on color
 */
#define MIRROR_X(color, x) (color == YELLOW ? (x) : 3000.f - (x))
#define MIRROR_A(color, a_deg) (color == YELLOW ? (a_deg) : 180.f - (a_deg))
#define MIRROR(color, value) (color == YELLOW ? (value) : -(value))

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_COLOR_H */
