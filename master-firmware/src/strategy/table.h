#ifndef STRATEGY_TABLE_H
#define STRATEGY_TABLE_H

#include "protobuf/strategy.pb.h"

// Glass color for the green side, the yellow side is reversed

typedef enum {
    GlassColor_UNKNOWN = 0,
    GlassColor_GREEN,
    GlassColor_RED
} GlassColor;

typedef enum {
    GlassOrientiation_UP = 0,
    GlassOrientiation_DOWN,
} GlassOrientiation;

typedef struct {
    GlassColor color;
    GlassOrientiation orientation;
    float pos_x_mm;
    float pos_y_mm;
} Glass;

static const Glass Glasss[] = {
    /* in front of start zone */
    {GlassColor_RED, GlassOrientiation_DOWN, 300, 400},
    {GlassColor_GREEN, GlassOrientiation_DOWN, 450, 510},
    {GlassColor_RED, GlassOrientiation_DOWN, 450, 1080},
    {GlassColor_GREEN, GlassOrientiation_DOWN, 300, 1200},
    /* diagonal on our side */
    {GlassColor_RED, GlassOrientiation_DOWN, 670, 100},
    {GlassColor_GREEN, GlassOrientiation_DOWN, 950, 400},
    {GlassColor_RED, GlassOrientiation_DOWN, 1100, 800},
    {GlassColor_GREEN, GlassOrientiation_DOWN, 1270, 1200},
    /* Our distributor */
    {GlassColor_RED, GlassOrientiation_UP, -67, 1450},
    {GlassColor_GREEN, GlassOrientiation_UP, -67, 1525},
    {GlassColor_RED, GlassOrientiation_UP, -67, 1600},
    {GlassColor_GREEN, GlassOrientiation_UP, -67, 1675},
    {GlassColor_RED, GlassOrientiation_UP, -67, 1750},
    /* Mixed distrubutor on our side*/
    {GlassColor_UNKNOWN, GlassOrientiation_UP, 700, -67},
    {GlassColor_UNKNOWN, GlassOrientiation_UP, 775, -67},
    {GlassColor_UNKNOWN, GlassOrientiation_UP, 850, -67},
    {GlassColor_UNKNOWN, GlassOrientiation_UP, 925, -67},
    {GlassColor_UNKNOWN, GlassOrientiation_UP, 1000, -67},
    /*Rocky area*/
    {GlassColor_GREEN, GlassOrientiation_DOWN, 1065, 1650},
    {GlassColor_RED, GlassOrientiation_DOWN, 1005, 1955},
    {GlassColor_RED, GlassOrientiation_DOWN, 1335, 1650},
    {GlassColor_GREEN, GlassOrientiation_DOWN, 1395, 1995},
};
#endif /* STRATEGY_TABLE_H */