#ifndef STRATEGY_TABLE_H
#define STRATEGY_TABLE_H

#include "protobuf/strategy.pb.h"

// Cup color for the green side, the yellow side is reversed

typedef enum {
    CupColor_GREEN = 0,
    CupColor_RED,
    CupColor_GREEN_OR_RED
} CupColor;

typedef enum {
    CupOrientiation_UP = 0,
    CupOrientiation_DOWN,
} CupOrientiation;

typedef struct {
    CupColor color;
    CupOrientiation orientation;
    float pos_x_mm;
    float pos_y_mm;
} Cup;

static const Cup cups[] = {
    /* in front of start zone */
    {CupColor_RED, CupOrientiation_DOWN, 300, 400},
    {CupColor_GREEN, CupOrientiation_DOWN, 450, 510},
    {CupColor_RED, CupOrientiation_DOWN, 450, 1080},
    {CupColor_GREEN, CupOrientiation_DOWN, 300, 1200},
    /* diagonal on our side */
    {CupColor_RED, CupOrientiation_DOWN, 670, 100},
    {CupColor_GREEN, CupOrientiation_DOWN, 950, 400},
    {CupColor_RED, CupOrientiation_DOWN, 1100, 800},
    {CupColor_GREEN, CupOrientiation_DOWN, 1270, 1200},
    /* Our distributor */
    {CupColor_RED, CupOrientiation_UP, -67, 1450},
    {CupColor_GREEN, CupOrientiation_UP, -67, 1525},
    {CupColor_RED, CupOrientiation_UP, -67, 1600},
    {CupColor_GREEN, CupOrientiation_UP, -67, 1675},
    {CupColor_RED, CupOrientiation_UP, -67, 1750},
    /* Mixed distrubutor on our side*/
    {CupColor_GREEN_OR_RED, CupOrientiation_UP, 700, -67},
    {CupColor_GREEN_OR_RED, CupOrientiation_UP, 775, -67},
    {CupColor_GREEN_OR_RED, CupOrientiation_UP, 850, -67},
    {CupColor_GREEN_OR_RED, CupOrientiation_UP, 925, -67},
    {CupColor_GREEN_OR_RED, CupOrientiation_UP, 1000, -67},
    /*Rocky area*/
    {CupColor_GREEN, CupOrientiation_DOWN, 1065, 1650},
    {CupColor_RED, CupOrientiation_DOWN, 1005, 1955},
    {CupColor_RED, CupOrientiation_DOWN, 1335, 1650},
    {CupColor_GREEN, CupOrientiation_DOWN, 1395, 1995},
};
#endif /* STRATEGY_TABLE_H */