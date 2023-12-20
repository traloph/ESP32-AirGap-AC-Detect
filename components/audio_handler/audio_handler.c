#include "audio_handler.h"

int melody_up[] = {TONE_C, TONE_D, TONE_E};
int melody_down[] = {TONE_E, TONE_D, TONE_C};
int melody_length = sizeof(melody_up) / sizeof(int);


void init_audio(void)
{
    // Initialize the shutdown pin
    esp_rom_gpio_pad_select_gpio(SHUTDOWN_PIN);
    gpio_set_direction(SHUTDOWN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SHUTDOWN_PIN, 0);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = TONE_C,  // Initial frequency set to Middle C
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = LEDC_DUTY,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void play_tone(int freq)
{
    gpio_set_level(SHUTDOWN_PIN, 1);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY)); // Restore the duty cycle
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL)); // Update the duty value
    ESP_ERROR_CHECK(ledc_set_freq(LEDC_MODE, LEDC_TIMER, freq));
    vTaskDelay(pdMS_TO_TICKS(87));  // Play tone for 87ms
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0)); // Set duty to 0
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL)); // Update the duty value
    gpio_set_level(SHUTDOWN_PIN, 0);
}

void play_melody(int *melody){
    for (int i = 0; i < melody_length; i++)
    {
        play_tone(melody[i]);
        vTaskDelay(pdMS_TO_TICKS(8));  // Gap between tones
    }
}
