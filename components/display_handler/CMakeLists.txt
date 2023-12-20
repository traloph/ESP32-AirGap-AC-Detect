set(component_srcs "display_handler.c" "images_data.c")

idf_component_register(
    SRCS "${component_srcs}"
    INCLUDE_DIRS "."
    REQUIRES ssd1306 driver
)

