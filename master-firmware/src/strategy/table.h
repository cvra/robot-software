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
    {PuckColor_REDGREEN, PuckOrientiation_HORIZONTAL, 500, 450},
    {PuckColor_REDGREEN, PuckOrientiation_HORIZONTAL, 500, 750},
    {PuckColor_REDGREEN, PuckOrientiation_HORIZONTAL, 500, 1050},

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
    {PuckColor_REDGREEN, 320, 600},
    {PuckColor_REDGREEN, 420, 600},
    {PuckColor_RED, 450, 450},
    {PuckColor_GREEN, 450, 750},
    {PuckColor_BLUE, 450, 1050},
};

#endif /* STRATEGY_TABLE_H */
