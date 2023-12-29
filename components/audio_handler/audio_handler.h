#ifndef AUDIO_HANDLER_H
#define AUDIO_HANDLER_H


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5)
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT
#define LEDC_DUTY               (4096)  // 50% duty cycle for max volume

#define TONE_C                  (261)
#define TONE_D                  (293)
#define TONE_E                  (329)

#define SHUTDOWN_PIN            (16)

#define MIN_WATTAGE 30.0
#define MAX_WATTAGE 3000.0
#define MIN_FREQUENCY 261
#define MAX_FREQUENCY 2000

// Define melodies
extern int melody_up[];
extern int melody_down[];
extern int melody_length;

void init_audio(void);
void play_tone(int freq);
void play_melody(int *melody);
void play_tone_based_on_wattage(float current_wattage);


#endif
