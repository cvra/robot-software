#include "gui_utilities.h"

#include "gui.h"
#include <ch.h>
#include <hal.h>
#include <string.h>
#include <stdio.h>
#include <gfx.h>
#include "main.h"

static GHandle score_label;
static GHandle sensor_label;
static GHandle sensor2_label;
static GHandle console;

void page_1(void)
{
    gwinDestroy(score_label);
    gwinDestroy(sensor_label);
    gwinDestroy(sensor2_label);
    {
        GWidgetInit wi;
        memset(&wi, 0, sizeof(wi));
        wi.g.show = TRUE;
        wi.g.x = 5;
        wi.g.y = 30;
        wi.g.width = gdispGetWidth() / 2;
        wi.g.height = 30;
        wi.text = " page 1score";
        score_label = gwinLabelCreate(0, &wi);
    }

    {
        GWidgetInit wi;
        gwinWidgetClearInit(&wi);
        memset(&wi, 0, sizeof(wi));
        wi.g.show = TRUE;
        wi.g.x = 5;
        wi.g.y = 80;
        wi.g.width = gdispGetWidth() / 2;
        wi.g.height = 30;
        wi.text = " page 1sensor";
        sensor_label = gwinLabelCreate(0, &wi);
    }
    {
        GWidgetInit wi;
        gwinWidgetClearInit(&wi);
        memset(&wi, 0, sizeof(wi));
        wi.g.show = TRUE;
        wi.g.x = 5;
        wi.g.y = 130;
        wi.g.width = gdispGetWidth() / 2;
        wi.g.height = 30;
        wi.text = " page 1sensor2";
        sensor2_label = gwinLabelCreate(0, &wi);
    }
}
void page_2(void)
{
    gwinDestroy(score_label);
    gwinDestroy(sensor_label);
    gwinDestroy(sensor2_label);
    {
        GWidgetInit wi;
        memset(&wi, 0, sizeof(wi));
        wi.g.show = TRUE;
        wi.g.x = 5;
        wi.g.y = 30;
        wi.g.width = gdispGetWidth() / 2;
        wi.g.height = 30;
        wi.text = " page 2 score";
        score_label = gwinLabelCreate(0, &wi);
    }

    {
        GWidgetInit wi;
        gwinWidgetClearInit(&wi);
        memset(&wi, 0, sizeof(wi));
        wi.g.show = TRUE;
        wi.g.x = 5;
        wi.g.y = 80;
        wi.g.width = gdispGetWidth() / 2;
        wi.g.height = 30;
        wi.text = " page 2 sensor";
        sensor_label = gwinLabelCreate(0, &wi);
    }
    {
        GWidgetInit wi;
        gwinWidgetClearInit(&wi);
        memset(&wi, 0, sizeof(wi));
        wi.g.show = TRUE;
        wi.g.x = 5;
        wi.g.y = 130;
        wi.g.width = gdispGetWidth() / 2;
        wi.g.height = 30;
        wi.text = " page 2 sensor2";
        sensor2_label = gwinLabelCreate(0, &wi);
    }
}
void page_3(void)
{
    gwinDestroy(score_label);
    gwinDestroy(sensor_label);
    gwinDestroy(sensor2_label);
    {
        GWidgetInit wi;
        memset(&wi, 0, sizeof(wi));
        wi.g.show = TRUE;
        wi.g.x = 5;
        wi.g.y = 30;
        wi.g.width = gdispGetWidth() / 2;
        wi.g.height = 30;
        wi.text = " page 3 score";
        score_label = gwinLabelCreate(0, &wi);
    }

    {
        GWidgetInit wi;
        gwinWidgetClearInit(&wi);
        memset(&wi, 0, sizeof(wi));
        wi.g.show = TRUE;
        wi.g.x = 5;
        wi.g.y = 80;
        wi.g.width = gdispGetWidth() / 2;
        wi.g.height = 30;
        wi.text = " page 3 sensor";
        sensor_label = gwinLabelCreate(0, &wi);
    }
    {
        GWidgetInit wi;
        gwinWidgetClearInit(&wi);
        memset(&wi, 0, sizeof(wi));
        wi.g.show = TRUE;
        wi.g.x = 5;
        wi.g.y = 130;
        wi.g.width = gdispGetWidth() / 2;
        wi.g.height = 30;
        wi.text = " page 3 sensor2";
        sensor2_label = gwinLabelCreate(0, &wi);
    }
}