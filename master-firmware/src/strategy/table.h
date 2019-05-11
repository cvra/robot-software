#ifndef STRATEGY_TABLE_H
#define STRATEGY_TABLE_H

#include "protobuf/strategy.pb.h"

const uint32_t MAX_PUCKS_IN_SCALE = 6;

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
    {PuckColor_RED_OR_GREEN, PuckOrientiation_HORIZONTAL, 500, 450},
    {PuckColor_RED_OR_GREEN, PuckOrientiation_HORIZONTAL, 500, 750},
    {PuckColor_RED_OR_GREEN, PuckOrientiation_HORIZONTAL, 500, 1050},

    /* distributor of 6 */
    {PuckColor_RED, PuckOrientiation_VERTICAL, 500, 1543},
    {PuckColor_GREEN, PuckOrientiation_VERTICAL, 600, 1543},
    {PuckColor_RED, PuckOrientiation_VERTICAL, 700, 1543},
    {PuckColor_BLUE, PuckOrientiation_VERTICAL, 800, 1543},
    {PuckColor_RED, PuckOrientiation_VERTICAL, 900, 1543},
    {PuckColor_GREEN, PuckOrientiation_VERTICAL, 1000, 1543},

    /* distributor of 3 */
    {PuckColor_BLUE, PuckOrientiation_VERTICAL, 125, 2000},
    {PuckColor_GREEN, PuckOrientiation_VERTICAL, 225, 2000},
    {PuckColor_RED, PuckOrientiation_VERTICAL, 325, 2000},

    /* accelerator */
    {PuckColor_BLUE, PuckOrientiation_VERTICAL, 1700, 22}, // todo: double check puck position
};

typedef struct {
    PuckColor color;
    float pos_x_mm;
    float pos_y_mm;
} DepositArea;

static const DepositArea areas[] = {
    {PuckColor_RED_OR_GREEN, 320, 600},
    {PuckColor_RED_OR_GREEN, 420, 600},
    {PuckColor_RED_OR_GREEN, 520, 600},
    {PuckColor_RED, 450, 450},
    {PuckColor_GREEN, 450, 750},
    {PuckColor_BLUE, 450, 1050},
};

#endif /* STRATEGY_TABLE_H */
