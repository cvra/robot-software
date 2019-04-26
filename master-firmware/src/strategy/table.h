#ifndef STRATEGY_TABLE_H
#define STRATEGY_TABLE_H

#include "protobuf/strategy.pb.h"

struct Puck {
    PuckColor color;
    float pos_x_mm;
    float pos_y_mm;
};

static const Puck pucks[] = {
    {PuckColor_RED, 500, 450},
    {PuckColor_RED, 500, 750},
    {PuckColor_GREEN, 500, 1050},
};

struct DepositArea {
    float pos_x_mm;
    float pos_y_mm;
};

static const DepositArea areas[] = {
    /* Red   */ {320, 450},
    /* Green */ {320, 750},
    /* Blue  */ {320, 1050},
};

#endif /* STRATEGY_TABLE_H */
