#ifndef DISPLAY_HANDLER_H
#define DISPLAY_HANDLER_H

#include "ssd1306.h"
#include <string.h>
#include "freertos/task.h"
#include "images_data.h"


void init_display(SSD1306_t* dev);

// Update line with flashing effect to indicate update
void display_flash_updated_line(SSD1306_t* dev, int line, float value, const char* format, bool invert); 

void display_image(SSD1306_t *dev, int xpos, int ypos, uint8_t *bitmap, int width, int height, bool invert);

#endif // DISPLAY_HANDLER_H