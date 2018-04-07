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

/** Retrieve color from character */
enum cube_color cube_color_from_character(char c);

/** Retrieve colors from string, length of colors should exceed string_len */
void cube_color_from_string(char* string, int string_len, enum cube_color* colors);

#ifdef __cplusplus
}
#endif

#endif /* EUROBOT2018_H */
