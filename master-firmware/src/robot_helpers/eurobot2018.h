#ifndef EUROBOT2018_H
#define EUROBOT2018_H

#ifdef __cplusplus
extern "C" {
#endif

/** Cubes color */
enum cube_color {
    CUBE_YELLOW = 0,
    CUBE_GREEN,
    CUBE_BLUE,
    CUBE_ORANGE,
    CUBE_BLACK,
    CUBE_UNKNOWN, // Unknow color
};

/** Get color name from enum */
const char* cube_color_name(enum cube_color color);

/** Retrieve color from character */
enum cube_color cube_color_from_character(char c);

/** Retrieve colors from string, length of colors should exceed string_len */
void cube_color_from_string(char* string, int string_len, enum cube_color* colors);

/** Fill unknown colors in sequence with default from valid colors
 * Only works for a sequence of 5 since there are 5 valid colors
 * and they can only appear once in a block of cubes.
 */
void cube_color_fill_unknown(enum cube_color* colors, int num_colors);

#ifdef __cplusplus
}
#endif

#endif /* EUROBOT2018_H */
