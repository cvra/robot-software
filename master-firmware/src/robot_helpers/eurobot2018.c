#include <stdbool.h>

#include "eurobot2018.h"

/** Cubes color to string */
const char* cube_color_name(enum cube_color color)
{
    if      (color == CUBE_YELLOW) { return "YELLOW"; }
    else if (color == CUBE_GREEN)  { return "GREEN"; }
    else if (color == CUBE_BLUE)   { return "BLUE"; }
    else if (color == CUBE_ORANGE) { return "ORANGE"; }
    else if (color == CUBE_BLACK)  { return "BLACK"; }
    else                           { return "UNKNOWN"; }
}

enum cube_color cube_color_from_character(char c)
{
    if      (c == 'Y') { return CUBE_YELLOW; }
    else if (c == 'G') { return CUBE_GREEN; }
    else if (c == 'B') { return CUBE_BLUE; }
    else if (c == 'O') { return CUBE_ORANGE; }
    else if (c == 'K') { return CUBE_BLACK; }
    else /* Unknown */ { return CUBE_UNKNOWN; }
}

void cube_color_from_string(char* string, int string_len, enum cube_color* colors)
{
    for (int i = 0; i < string_len; i++)
    {
        colors[i] = cube_color_from_character(string[i]);
    }
}

static bool color_in_sequence(enum cube_color* colors, int num_colors, enum cube_color color)
{
    for (int i = 0; i < num_colors; i++) {
        if (colors[i] == color) {
            return true;
        }
    }
    return false;
}

void cube_color_fill_unknown(enum cube_color* colors, int num_colors)
{
    for (int i = 0; i < num_colors; i++) {
        if (colors[i] != CUBE_UNKNOWN) {
            continue;
        }

        for (int j = 0; j < (int)CUBE_UNKNOWN; j++) {
            if (!color_in_sequence(colors, num_colors, (enum cube_color)j)) {
                colors[i] = (enum cube_color)j;
                break;
            }
        }
    }
}
