#ifndef STRATEGY_TABLE_H
#define STRATEGY_TABLE_H

#include "protobuf/strategy.pb.h"

typedef enum {
    PuckOrientiation_HORIZONTAL = 0,
    PuckOrientiation_VERTICAL,
} PuckOrientiation;

typedef struct {
    PuckColor color;
    PuckOrientiation orientation;
    float pos_x_mm;
    float pos_y_mm;
} Puck;

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

typedef struct {
    PuckColor color;
    float pos_x_mm;
    float pos_y_mm;
} DepositArea;

static const DepositArea areas[] = {
    {PuckColor_RED, 320, 600},
    {PuckColor_GREEN, 420, 600},
    {PuckColor_BLUE, 320, 1050},
};

#endif /* STRATEGY_TABLE_H */
