#include "display_handler.h"

void init_display(SSD1306_t* dev) {
    i2c_master_init(dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    ssd1306_init(dev, 128, 64);
    ssd1306_clear_screen(dev, false);
}

void display_flash_updated_line(SSD1306_t* dev, int line, float value, const char* format, bool invert) {

    char buffer[50];
    snprintf(buffer, sizeof(buffer), format, value);
    ssd1306_display_text(dev, line, buffer, strlen(buffer), !invert);
    vTaskDelay(100 / portTICK_RATE_MS);
    ssd1306_display_text(dev, line, buffer, strlen(buffer), invert);
}

void display_image(SSD1306_t *dev, int xpos, int ypos, uint8_t *bitmap, int width, int height, bool invert) {
    ssd1306_bitmaps(dev, xpos, ypos, bitmap, width, height, invert);
}
