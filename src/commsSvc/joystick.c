#include "joystick.h"

// -------------------------------------------------------------------------------
// Includes
#include <stdbool.h>
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

// -------------------------------------------------------------------------------
// Definitions
#define JOYSTICK_X_CHANNEL   ADC_CHANNEL_6   // GPIO14
#define JOYSTICK_Y_CHANNEL   ADC_CHANNEL_7   // GPIO27
#define JOYSTICK_SW_PIN      26              // GPIO26

#define ADC_MIN_READING      0
#define ADC_MAX_READING      4095
#define X_RAW_MID            1980
#define Y_RAW_MID            2025

#define JOYSTICK_PAN_RANGE   270.0f
#define JOYSTICK_PAN_MID     0.0f
#define JOYSTICK_PAN_MIN     (JOYSTICK_PAN_MID - JOYSTICK_PAN_RANGE / 2)
#define JOYSTICK_PAN_MAX     (JOYSTICK_PAN_MID + JOYSTICK_PAN_RANGE / 2)
#define JOYSTICK_TILT_RANGE  90.0f
#define JOYSTICK_TILT_MID    90.0f
#define JOYSTICK_TILT_MIN    (JOYSTICK_TILT_MID - JOYSTICK_TILT_RANGE / 2 )
#define JOYSTICK_TILT_MAX    (JOYSTICK_TILT_MID + JOYSTICK_TILT_RANGE / 2)

// -------------------------------------------------------------------------------
// Type defines
typedef struct
{
	uint16_t xRaw;
	uint16_t yRaw;
} joystickReadingRaw_t;


// -------------------------------------------------------------------------------
// Local variables
static const char *TAG = "> joystick";
static adc_oneshot_unit_handle_t s_adc_handle = NULL;

// -------------------------------------------------------------------------------
// Local function declarations
// -------------------------------------------------------------------------------
static bool readJoystickRaw(joystickReadingRaw_t *sample);
static float joystickMapAxis(int value, int inMin, int inMid, int inMax, float outMin, float outMax);
static float joystickMapHalf(int value, int inMin, int inMax, float outMin, float outMax);
void deadBandFilter(uint16_t* value, uint16_t center, uint16_t deadbandSize);

// -------------------------------------------------------------------------------
// Global function definitions
// -------------------------------------------------------------------------------
void joystick_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_2,
    };
    
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC oneshot init failed: %d", err);
        return;
    }
    
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,  // 0-3.3V range for joystick
    };

    err = adc_oneshot_config_channel(s_adc_handle, JOYSTICK_X_CHANNEL, &chan_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC X channel config failed: %d", err);
        return;
    }
    
    err = adc_oneshot_config_channel(s_adc_handle, JOYSTICK_Y_CHANNEL, &chan_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC Y channel config failed: %d", err);
        return;
    }
}


bool joystick_getInputAttitude(gimbalAttitude_t *attitude)
{
    if (attitude == NULL)
    {
        return false;
    }

    joystickReadingRaw_t joystickSample = {0};
    if (!readJoystickRaw(&joystickSample))
    {
        return false;
    }

    deadBandFilter(&joystickSample.yRaw, Y_RAW_MID, 400);
    // do not deadband X because gimbalControlSvc needs to know when joystick is centered

    attitude->tiltDeg = joystickMapAxis((float)joystickSample.xRaw, ADC_MIN_READING, X_RAW_MID, ADC_MAX_READING, JOYSTICK_TILT_MIN, JOYSTICK_TILT_MAX);
    attitude->panDeg = -joystickMapAxis((float)joystickSample.yRaw, ADC_MIN_READING, Y_RAW_MID, ADC_MAX_READING, JOYSTICK_PAN_MIN, JOYSTICK_PAN_MAX);
    //ESP_LOGI(TAG, "Joystick attitude - pan: %.1f, tilt: %.1f", attitude->panDeg, attitude->tiltDeg);
    
    return true;
}

// -------------------------------------------------------------------------------
// Local function definitions
// -------------------------------------------------------------------------------
static bool readJoystickRaw(joystickReadingRaw_t *sample)
{
    if (sample == NULL)
    {
        return false;
    }

    int xRaw = 0;
    int yRaw = 0;
    esp_err_t xErr = adc_oneshot_read(s_adc_handle, JOYSTICK_X_CHANNEL, &xRaw);
    esp_err_t yErr = adc_oneshot_read(s_adc_handle, JOYSTICK_Y_CHANNEL, &yRaw);

    if (xErr != ESP_OK || yErr != ESP_OK)
    {
        ESP_LOGW(TAG, "ADC read failed (x=%d y=%d)", xErr, yErr);
        return false;
    }

    if (xRaw < 0) xRaw = 0;
    if (yRaw < 0) yRaw = 0;
    if (xRaw > (int)ADC_MAX_READING) {
        ESP_LOGW(TAG, "xRaw ADC reading out of range: %d", xRaw);
        xRaw = (int)ADC_MAX_READING;
    }
    if (yRaw > (int)ADC_MAX_READING) {
        ESP_LOGW(TAG, "yRaw ADC reading out of range: %d", yRaw);
        yRaw = (int)ADC_MAX_READING;
    }

    //ESP_LOGI(TAG, "Joystick ADC raw readings - x: %d, y: %d", xRaw, yRaw);
    sample->xRaw = (uint16_t)xRaw;
    sample->yRaw = (uint16_t)yRaw;

    return true;
}

static float joystickMapAxis(int value, int inMin, int inMid, int inMax, float outMin, float outMax)
{
    // joystick axes are not symmetrical so we map the lower and upper halves separately to maintain sensitivity around the center
    float outValue = 0.0f;
    if (value < inMin) value = inMin;
    if (value > inMax) value = inMax;

    if (value < inMid) {
        outValue = joystickMapHalf(value, inMin, inMid, outMin, (outMin + outMax) / 2.0f);
    } else {
        outValue = joystickMapHalf(value, inMid, inMax, (outMin + outMax) / 2.0f, outMax);
    }
    
    return outValue;
}

static float joystickMapHalf(int value, int inMin, int inMax, float outMin, float outMax)
{
    if (value < inMin) value = inMin;
    if (value > inMax) value = inMax;
    float ratio = (value - inMin) / (float)(inMax - inMin);
    return outMin + (ratio * (outMax - outMin));
}

void deadBandFilter(uint16_t* value, uint16_t center, uint16_t deadbandSize) {
    if (value == NULL) return;

    if (*value > center - deadbandSize && *value < center + deadbandSize) {
        //ESP_LOGI(TAG, "Apply deadband: %d is within %d of center %d, setting to center", *value, deadbandSize, center);
        *value = center;
    }
}

/** @} */
