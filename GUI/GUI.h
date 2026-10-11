#ifndef __GUI_H
#define __GUI_H

#include "Adafruit_GFX.h"

typedef enum {
    MODE_PICTURE = 0,
    MODE_CALENDAR = 1,
    MODE_CLOCK = 2,
} display_mode_t;

typedef struct {
    display_mode_t mode;
    uint16_t color;
    uint16_t width;
    uint16_t height;
    uint32_t timestamp;
    uint8_t week_start;  // 0: Sunday, 1: Monday
    int8_t temperature;
    uint16_t voltage;
    char ssid[20];
} gui_data_t;

void DrawGUI(gui_data_t* data, buffer_callback callback, void* callback_data);

// Rectangle occupied by the large time digits of the clock screen (MODE_CLOCK).
// x/w are expanded to byte boundaries (8px) as required by e-paper addressing.
void GUI_ClockTimeRect(gui_data_t* data, uint16_t* x, uint16_t* y, uint16_t* w, uint16_t* h);

// Same as DrawGUI, but only renders and reports the given sub-rectangle.
// Used for partial refresh of the clock time strip; only the pixels inside the
// rect are drawn, the callback is invoked with the region coordinates.
void DrawGUIRegion(gui_data_t* data, buffer_callback callback, void* callback_data, uint16_t x, uint16_t y,
                   uint16_t w, uint16_t h);

#endif
