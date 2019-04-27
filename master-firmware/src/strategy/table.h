#ifndef STRATEGY_TABLE_H
#define STRATEGY_TABLE_H

#include "protobuf/strategy.pb.h"

enum PuckOrientiation {
    PuckOrientiation_HORIZONTAL = 0,
    PuckOrientiation_VERTICAL,
};

struct Puck {
    PuckColor color;
    PuckOrientiation orientation;
    float pos_x_mm;
    float pos_y_mm;
};

static const Puck pucks[] = {
    /* in front of start zone */
    {PuckColor_RED, PuckOrientiation_HORIZONTAL, 500, 450},
    {PuckColor_RED, PuckOrientiation_HORIZONTAL, 500, 750},
    {PuckColor_GREEN, PuckOrientiation_HORIZONTAL, 500, 1050},

    /* distributor of 6 */
    {PuckColor_RED, PuckOrientiation_VERTICAL, 500, 1543},
    {PuckColor_GREEN, PuckOrientiation_VERTICAL, 600, 1543},
    {PuckColor_RED, PuckOrientiation_VERTICAL, 700, 1543},
    {PuckColor_BLUE, PuckOrientiation_VERTICAL, 800, 1543},
    {PuckColor_RED, PuckOrientiation_VERTICAL, 900, 1543},
    {PuckColor_GREEN, PuckOrientiation_VERTICAL, 1000, 1543},
};

struct DepositArea {
    float pos_x_mm;
    float pos_y_mm;
};

static const DepositArea areas[] = {
    /* Red   */ {320, 600},
    /* Green */ {420, 600},
    /* Blue  */ {320, 1050},
};

#endif /* STRATEGY_TABLE_H */
