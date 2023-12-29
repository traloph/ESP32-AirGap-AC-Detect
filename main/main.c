// -----------------------
// Include Headers
// -----------------------

// Standard includes
#include <stdio.h>
#include <math.h>

// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ESP and Driver includes
#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "esp_timer.h"
#include "esp_dsp.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_sleep.h"

// Custom module includes
#include "ssd1306.h"
#include "lis3mdl.h"
#include "display_handler.h"
#include "audio_handler.h"

// -----------------------
// Macros and Defines
// -----------------------

#define SAMPLE_SIZE 128
#define ESP_INTR_FLAG_DEFAULT 0
#define I2C_BUS       0
#define I2C_SCL_PIN   22
#define I2C_SDA_PIN   21
#define I2C_FREQ      I2C_FREQ_100K
#define WAKE_UP_PIN 34
#define CALIB_SAMPLE_SIZE 300

// -----------------------
// FFT Variables and Structs
// -----------------------

typedef struct {
    float peak_frequency;
    float peak_amplitude;
} fft_result_t;

__attribute__((aligned(16)))
float wind[SAMPLE_SIZE];

static float mag_data[SAMPLE_SIZE];
static int mag_data_index = 0;
float sample_frequency;
fft_result_t result;

// -----------------------
// Timing Variables
// -----------------------

static uint64_t start_time = 0;
static uint64_t end_time = 0;

// -----------------------
// GPIO and Interrupts
// -----------------------

static QueueHandle_t gpio_evt_queue = NULL;

// -----------------------
// Display and I2C
// -----------------------

SSD1306_t dev;

// -----------------------
// Magnetic Sensor and Axis
// -----------------------

typedef enum {
    AXIS_X,
    AXIS_Y,
    AXIS_Z,
    AXIS_NONE  // No axis enabled
} AxisChoice;

static lis3mdl_sensor_t* sensor;
static float axis_data[CALIB_SAMPLE_SIZE];
static int axis_data_index = 0;
AxisChoice selected_axis = AXIS_X;
static bool request_axis_selection = false;

// -----------------------
// Task Handles and Flags
// -----------------------

TaskHandle_t lis3mdlDrdyTaskHandle = NULL;
TaskHandle_t updateScreenTaskHandle = NULL;
bool lis3mdl_reset;

// -----------------------
// Calibration and Wake-up
// -----------------------

static bool has_calibrated_axis = false;
RTC_DATA_ATTR static int wake_count = 0;
RTC_DATA_ATTR static int pointless_wake_count = 0;
RTC_DATA_ATTR static char selected_axis_char = 'N';
RTC_DATA_ATTR static uint16_t threshold_value = 0;

// -----------------------
// Watt calculation
// -----------------------

typedef struct {
    float peak_amplitude;
    float wattage;
} CalibrationPoint;

// To be modified based on calibration data
CalibrationPoint calibration_scale[] = {
    {1, 30.0},
    {50, 2000.0}
};

float current_wattage = 0.0;
RTC_DATA_ATTR static float kwh_since_boot = 0.0;
static float kwh_since_power_started = 0.0;

const float PRICE_PER_KWH_IN_EURO = 0.3816;

// -----------------------
// Current Conditions and Flags
// -----------------------

static const float conditionA_low = 49.0;
static const float conditionA_high = 59.0;
static const float conditionB_amplitude = 0.90;
static bool power_flowed = false;
static bool power_flowing = false;
static bool power_was_flowing = false;
static bool mute = false;


static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void update_screen_task(void* arg) {
    while (1) {
        vTaskDelay(3000 / portTICK_RATE_MS);

        ESP_LOGI("Avg Frequency", "Average GPIO interrupt frequency: %f Hz", sample_frequency);

        display_flash_updated_line(&dev,  0, current_wattage, "%.0f W", power_flowing);
        display_flash_updated_line(&dev,  1, kwh_since_power_started, "%.6f kWh", power_flowing);
        display_flash_updated_line(&dev,  2, kwh_since_power_started * PRICE_PER_KWH_IN_EURO, "%.6f Eur", false);
        display_flash_updated_line(&dev,  3, kwh_since_boot, "%.6f kWh", false);
        display_flash_updated_line(&dev,  4, kwh_since_boot * PRICE_PER_KWH_IN_EURO, "%.6f Eur", false);
        //display_flash_updated_line(&dev,  4, sample_frequency, "%.2f Hz", false);
        display_flash_updated_line(&dev,  5, result.peak_frequency, "%.2f Hz", false);
        display_flash_updated_line(&dev,  6, result.peak_amplitude, "%.2f ", false);

        char buffer[50];
        snprintf(buffer, sizeof(buffer), "%d/%d %c %u", wake_count - pointless_wake_count, wake_count, selected_axis_char, threshold_value);
        display_flash_updated_line(&dev,  7, 0, buffer, false);
    }
}

static float map_peak_to_watt(float peak_amplitude) {
    int size = sizeof(calibration_scale) / sizeof(calibration_scale[0]);
    
    // If below the first point, return the first point's wattage
    if (peak_amplitude <= calibration_scale[0].peak_amplitude) {
        return calibration_scale[0].wattage;
    }

    // If above the last point, return the last point's wattage
    if (peak_amplitude >= calibration_scale[size-1].peak_amplitude) {
        return calibration_scale[size-1].wattage;
    }

    // Otherwise, interpolate between the two points that surround the given amplitude
    for (int i = 0; i < size - 1; i++) {
        if (peak_amplitude >= calibration_scale[i].peak_amplitude && peak_amplitude <= calibration_scale[i+1].peak_amplitude) {
            float fraction = (peak_amplitude - calibration_scale[i].peak_amplitude) /
                             (calibration_scale[i+1].peak_amplitude - calibration_scale[i].peak_amplitude);
            return calibration_scale[i].wattage + fraction * (calibration_scale[i+1].wattage - calibration_scale[i].wattage);
        }
    }

    // Return a default value if for some reason no match is found (should not happen)
    return 0.0;
}

static void update_power_usage(float wattage, float sample_duration) {
    float kwh_this_cycle = (wattage / 1000.0) * (sample_duration / 3600.0);  // Convert watt to kW and second to hour
    kwh_since_power_started += kwh_this_cycle;
    kwh_since_boot += kwh_this_cycle;
}

void power_monitoring_task(void* arg) {
    float last_check_time = 0.0; // Time when the task last checked for power flow
    bool monitored_flowing_power = false;

    while (1) {
        float current_time = esp_timer_get_time() / 1000000.0; // Current time in seconds
        float elapsed_time = current_time - last_check_time;  // Time since the task last ran

        if (power_flowing) {
            current_wattage = map_peak_to_watt(result.peak_amplitude);
            update_power_usage(current_wattage, elapsed_time);
        }
        
        if (power_flowing != monitored_flowing_power) {
            monitored_flowing_power = power_flowing;  // Update the last state
            if (power_flowing) {
                kwh_since_power_started = 0.0; // Reset this counter when power starts flowing
            } else {
                current_wattage = 0.0;
            }
        }

        last_check_time = current_time; // Update the last check time
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

bool init_fft()
{
    esp_err_t ret;

    ret = dsps_fft4r_init_fc32(NULL, SAMPLE_SIZE >> 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE("FFT", "Not possible to initialize FFT4R. Error = %i", ret);
        return false;
    }

    dsps_wind_hann_f32(wind, SAMPLE_SIZE);
    return true;
}

fft_result_t calculate_peak(float* x, float fs)
{
    dsps_fft4r_fc32(x, SAMPLE_SIZE>>1);
    dsps_bit_rev4r_fc32(x, SAMPLE_SIZE>>1);
    dsps_cplx2real_fc32(x, SAMPLE_SIZE>>1);

    float max_magnitude = 0.0;
    int peak_freq_idx = 0;
    // Start from the 3nd bin to skip the DC component (0 Hz)
    for (int i = 3; i < SAMPLE_SIZE/2; i++)
    {
        float magnitude = sqrtf(x[i * 2] * x[i * 2] + x[i * 2 + 1] * x[i * 2 + 1]);
        if (magnitude > max_magnitude)
        {
            max_magnitude = magnitude;
            peak_freq_idx = i;
        }
    }
    float peak_frequency = (float)peak_freq_idx * fs / SAMPLE_SIZE;

    fft_result_t result = {peak_frequency, max_magnitude};
    return result;
}

bool is_current_flowing(float peak_frequency, float peak_amplitude)
{
    return (peak_frequency >= conditionA_low && 
            peak_frequency <= conditionA_high && 
            peak_amplitude > conditionB_amplitude);
}

void select_most_active_axis() {
    lis3mdl_raw_data_t raw_data, prev_data;
    int32_t diff_x = 0, diff_y = 0, diff_z = 0;  // Differences accumulator for each axis

    while (!lis3mdl_new_data(sensor)) {   //TODO use drdy interrupt
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    // Get the initial sample
    lis3mdl_get_raw_data(sensor, &prev_data);

    for(int i = 1; i < CALIB_SAMPLE_SIZE; i++) {
    
        while (!lis3mdl_new_data(sensor)) {  //TODO use drdy interrupt
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        lis3mdl_get_raw_data(sensor, &raw_data);

        // Calculate the absolute differences for each axis
        diff_x += abs(raw_data.mx - prev_data.mx);
        diff_y += abs(raw_data.my - prev_data.my);
        diff_z += abs(raw_data.mz - prev_data.mz);

        // Update the previous data for the next iteration
        prev_data = raw_data;
    }

    // Determine the most active axis based on the accumulated differences
    if(diff_x >= diff_y && diff_x >= diff_z) {
        selected_axis = AXIS_X;
        selected_axis_char = 'X';
    } else if(diff_y >= diff_x && diff_y >= diff_z) {
        selected_axis = AXIS_Y;
        selected_axis_char = 'Y';
    } else {
        selected_axis = AXIS_Z;
        selected_axis_char = 'Z';
    }

    printf("Selected AXIS_%c for calibration.\n", 
           (selected_axis == AXIS_X) ? 'X' : 
           (selected_axis == AXIS_Y) ? 'Y' : 'Z');
    ssd1306_display_text(&dev, 7, "Calib ax", 8, true);

    has_calibrated_axis = true;
    request_axis_selection = false;
}

static void lis3mdl_drdy_read_task(void* arg) {
    uint32_t io_num;
    lis3mdl_float_data_t data;

    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {

            if (request_axis_selection) {
                select_most_active_axis();
                mag_data_index = 0; //reset current sample
            }

            lis3mdl_get_float_data(sensor, &data);

            float magnitude =  fabs(data.mx) + fabs(data.my) + fabs(data.mz); //TODO use only calib axies?
            if (mag_data_index == 0) {
                start_time = esp_timer_get_time();
            }
            mag_data[mag_data_index] = magnitude;
            mag_data_index++;

            if (mag_data_index == SAMPLE_SIZE) {
                mag_data_index = 0; // Reset for the next set of data
                end_time = esp_timer_get_time(); // get the end time in microseconds

                float sample_duration = (end_time - start_time) / 1000000.0;
                sample_frequency = SAMPLE_SIZE / sample_duration;
                result = calculate_peak(mag_data, sample_frequency);

                power_flowing = is_current_flowing(result.peak_frequency, result.peak_amplitude);

                if (power_flowing != power_was_flowing) {
                    if (power_flowing) {
                        if (!mute) play_melody(melody_up);
                        ssd1306_display_text(&dev, 0, "                ", 16, true);
                        ssd1306_display_text(&dev, 1, "                ", 16, true);
                        display_image(&dev, 96, 16, blitz, 32, 48, true);
                        power_flowed = true;
                    } else {
                        if (!mute) play_melody(melody_down);
                        ssd1306_clear_line(&dev, 0, false);
                        ssd1306_clear_line(&dev, 1, false);
                        ssd1306_clear_area(&dev, 96, 16, 32, 48, false);
                    }

                    power_was_flowing = power_flowing;  // Update the last state
                }

                //ESP_LOGI("FFT", "Sample Frequency: %f Hz", sample_frequency);
                //ESP_LOGI("FFT", "Peak Frequency: %f Hz, Peak Amplitude: %f", result.peak_frequency, result.peak_amplitude);
            }
        }
    }
}

void check_power_flow_task(void *pvParameters) {
    while (1) {
        if (power_flowing) {
            if (!has_calibrated_axis && lis3mdl_reset) {
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                if (power_flowing) {
                    request_axis_selection = true;
                }
            }
            vTaskDelay(30000 / portTICK_PERIOD_MS);
        }
        if (power_flowing) {
            //play_tone(TONE_E);
            play_tone_based_on_wattage(current_wattage);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
}

float calibrate_threshold_interrupt() {
    lis3mdl_raw_data_t raw_data;
    int16_t max_value = 0;
    
    printf("Beginning Calibration...\n");
        printf("Calibrating for AXIS_%s...\n", 
           (selected_axis == AXIS_X) ? "X" : 
           (selected_axis == AXIS_Y) ? "Y" : 
           (selected_axis == AXIS_Z) ? "Z" : "NONE");
    
    //TODO make sure current does not start flowing while sampling for calibration
    for(int i = 0; i < CALIB_SAMPLE_SIZE; i++) {
        lis3mdl_get_raw_data(sensor, &raw_data); 
        
        printf("%.3f LIS3MDL (xyz)[LSB] mx=%+7d my=%+7d mz=%+7d\n",
               (double)sdk_system_get_time()*1e-3, 
                raw_data.mx, raw_data.my, raw_data.mz);
                
        switch(selected_axis) {
            case AXIS_X:
                axis_data[axis_data_index] = abs(raw_data.mx);
                break;
            case AXIS_Y:
                axis_data[axis_data_index] = abs(raw_data.my);
                break;
            case AXIS_Z:
                axis_data[axis_data_index] = abs(raw_data.mz);
                break;
            default:
                printf("No axis selected for calibration.\n");
                return -1;  // Error value
        }
        
        if(axis_data[axis_data_index] > max_value) {
            max_value = axis_data[axis_data_index];
        }
        axis_data_index = (axis_data_index + 1) % CALIB_SAMPLE_SIZE;
    }
    
    printf("Max value detected: %d\n", max_value);

    // Power down the sensor to set the interrupt threshold
    lis3mdl_set_mode(sensor, lis3mdl_power_down);
    
    //uint16_t threshold_value = (uint16_t)(max_value * 1.05 + 0.5);
    threshold_value = (uint16_t)(max_value + 220); //TODO have hc value be gen

    lis3mdl_int_config_t int_config;
    int_config.threshold = threshold_value;
    int_config.x_enabled = (selected_axis == AXIS_X);
    int_config.y_enabled = (selected_axis == AXIS_Y);
    int_config.z_enabled = (selected_axis == AXIS_Z);
    int_config.latch = true;
    int_config.signal_level = lis3mdl_high_active;
    
    printf("Setting interrupt threshold to: %u\n", threshold_value);

    lis3mdl_set_int_config(sensor, &int_config);

    // Restart the sensor after setting the interrupt threshold
    lis3mdl_set_scale(sensor, lis3mdl_scale_4_Gs);
    lis3mdl_set_mode(sensor, lis3mdl_uhpm_155);
    
    printf("End Calibration!\n");
    
    return max_value;
}

void print_int_config(lis3mdl_sensor_t* dev)
{
    lis3mdl_int_config_t cfg;

    if (lis3mdl_get_int_config(dev, &cfg))
    {
        printf("Interrupt Configuration:\n");
        printf("X-axis interrupt enabled: %s\n", cfg.x_enabled ? "Yes" : "No");
        printf("Y-axis interrupt enabled: %s\n", cfg.y_enabled ? "Yes" : "No");
        printf("Z-axis interrupt enabled: %s\n", cfg.z_enabled ? "Yes" : "No");
        printf("Interrupt latched: %s\n", cfg.latch ? "Yes" : "No");
        printf("Interrupt signal level: %s\n", cfg.signal_level ? "High" : "Low");
        printf("Interrupt threshold: %d\n", cfg.threshold);
    }
    else
    {
        printf("Failed to get interrupt configuration.\n");
    }
}

void deep_sleep_task(void *pvParameters) {
    while (1) {
        if (!power_flowing && (has_calibrated_axis || !lis3mdl_reset)) {
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            if (!power_flowing) {
                vTaskSuspend(updateScreenTaskHandle);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                if (lis3mdl_reset) {
                    calibrate_threshold_interrupt();
                } else {
                    print_int_config(sensor); 
                }
                if (!power_flowed) {
                    pointless_wake_count++;
                }
                ssd1306_clear_screen(&dev, false);
                printf("Entering deep sleep...\n");
                esp_deep_sleep_start();
            }
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void app_main() {

    wake_count++;
    
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
        printf("Woken up by threshold interrupt!\n");
        lis3mdl_int_source_t int_src;
        lis3mdl_get_int_source (sensor, &int_src);
    }
    
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    
    if (wakeup_cause == ESP_SLEEP_WAKEUP_UNDEFINED) 
    {
        printf("Debug: First boot or normal reset.\n");
        lis3mdl_reset = true;
    } 
    else
    {
        printf("Debug: Wakeup from deep sleep.\n");
        lis3mdl_reset = false;
    }


    init_display(&dev);

    ssd1306_display_text(&dev, 2, "SSD1306.......ok", 16, false);
    
    if (init_fft())
        ssd1306_display_text(&dev, 3, "FFT...........ok", 16, false);
    else
        ssd1306_display_text(&dev, 3, "FFT........ERROR", 16, true);

    init_audio();

    ssd1306_display_text(&dev, 4, "AUDIO.........ok", 16, false);

    // Configure GPIO
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = 1ULL << GPIO_NUM_4;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    
    gpio_pulldown_en(WAKE_UP_PIN);
    esp_sleep_enable_ext0_wakeup(WAKE_UP_PIN, 1);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_NUM_4, gpio_isr_handler, (void*) GPIO_NUM_4);

    ssd1306_display_text(&dev, 5, "ISR...........ok", 16, false);

    i2c_init_lis3mdl(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);
    sensor = lis3mdl_init_sensor(I2C_BUS, LIS3MDL_I2C_ADDRESS_1, 0, lis3mdl_reset);

    if (sensor)
    {
        lis3mdl_set_scale(sensor, lis3mdl_scale_4_Gs);
        lis3mdl_set_mode(sensor, lis3mdl_uhpm_155);
        ssd1306_display_text(&dev, 6, "LIS3MDL.......ok", 16, false);
    }
    else
        printf("Could not initialize LIS3MDL sensor\n");

    
    print_int_config(sensor); 

    ssd1306_clear_screen(&dev, false);

    xTaskCreate(update_screen_task, "update_screen_task", 2048, NULL, 2, &updateScreenTaskHandle);
    xTaskCreate(lis3mdl_drdy_read_task, "lis3mdl_drdy_read_task", 2048, NULL, 5, &lis3mdlDrdyTaskHandle);
    xTaskCreate(&check_power_flow_task, "CheckPowerFlowTask", 2048, NULL, 4, NULL);
    xTaskCreate(&deep_sleep_task, "DeepSleepTask", 2048, NULL, 5, NULL);
    xTaskCreate(power_monitoring_task, "Power Monitoring Task", 2048, NULL, 2, NULL);
}