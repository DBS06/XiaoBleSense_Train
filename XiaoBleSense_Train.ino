/* Edge Impulse Arduino examples
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Includes ---------------------------------------------------------------- */
#include <LSM6DS3.h>
// #include <PullBack_inferencing.h>
#include <U8g2lib.h>
#include <U8x8lib.h>
#include <Wire.h>
#include <detectPullBack_inferencing.h>
#include <stdio.h>

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2 9.80665f
#define MAX_ACCEPTED_RANGE \
    2.0f  // starting 03/2022, models are generated setting range to +-2, but this example use Arudino library which set
          // range to +-4g. If you are using an older model, ignore this value and use 4.0f instead

/*
 ** NOTE: If you run into TFLite arena allocation issue.
 **
 ** This may be due to may dynamic memory fragmentation.
 ** Try defining "-DEI_CLASSIFIER_ALLOCATION_STATIC" in boards.local.txt (create
 ** if it doesn't exist) and copy this file to
 ** `<ARDUINO_CORE_INSTALL_PATH>/arduino/hardware/<mbed_core>/<core_version>/`.
 **
 ** See
 ** (https://support.arduino.cc/hc/en-us/articles/360012076960-Where-are-the-installed-cores-located-)
 ** to find where Arduino installs cores on your machine.
 **
 ** If the problem persists then there's not enough memory for this model and application.
 */

// U8X8_SSD1306_64X48_ER_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/PIN_WIRE_SCL,
                                       /* data=*/PIN_WIRE_SDA,
                                       /* reset=*/U8X8_PIN_NONE);  // OLEDs without Reset of the Display

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;  // Set this to true to see e.g. features generated from the raw signal
LSM6DS3 myIMU(I2C_MODE, 0x6A);
/**
 * @brief      Arduino setup function
 */

const int RED_ledPin   = 11;
const int BLUE_ledPin  = 12;
const int GREEN_ledPin = 13;

const uint32_t displayResultTimeMs = 2000;

enum RgbColor
{
    OFF,
    RED,
    GREEN,
    BLUE,
    YELLOW,
    PURPLE,
    WHITE
};

void printResultToDisp(ei_impulse_result_classification_t &classResult, uint8_t color);
void printResultToDisp(const char *label, float value, uint8_t color);

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    // u8g2.begin();
    u8x8.begin();

    Serial.println("Edge Impulse Inferencing Demo");

    // if (!IMU.begin()) {
    if (!myIMU.begin())
    {
        ei_printf("Failed to initialize IMU!\r\n");
    }
    else
    {
        ei_printf("IMU initialized\r\n");
    }

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3)
    {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
        return;
    }

    ei_printf("\nStarting inferencing in 2 seconds...\n");
    delay(2000);
}

/**
 * @brief Return the sign of the number
 *
 * @param number
 * @return int 1 if positive (or 0) -1 if negative
 */
float ei_get_sign(float number)
{
    return (number >= 0.0) ? 1.0 : -1.0;
}

/**
 * @brief      Get data and run inferencing
 *
 * @param[in]  debug  Get debug info if true
 */
void loop()
{
    digitalWrite(RED_ledPin, HIGH);
    digitalWrite(BLUE_ledPin, HIGH);
    digitalWrite(GREEN_ledPin, HIGH);

    u8x8.clear();
    u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);

    // u8x8.drawString(2, 3, "Sampling... 3");
    // u8x8.refreshDisplay();
    // delay(1000);
    u8x8.drawString(2, 3, "Sampling... 2");
    ei_printf("Sampling... 2\n");
    u8x8.refreshDisplay();
    delay(1000);
    u8x8.drawString(2, 3, "Sampling... 1");
    ei_printf("Sampling... 1\n");
    u8x8.refreshDisplay();
    delay(1000);
    u8x8.clear();
    u8x8.drawString(2, 3, "Sampling...");
    ei_printf("Sampling... 0\n");
    u8x8.refreshDisplay();

    // Allocate a buffer here for the values we'll read from the IMU
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3)
    {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        buffer[ix]     = myIMU.readFloatAccelX();
        buffer[ix + 1] = myIMU.readFloatAccelY();
        buffer[ix + 2] = myIMU.readFloatAccelZ();

        // buffer[ix] = myIMU.readFloatGyroX();
        // buffer[ix+1] = myIMU.readFloatGyroY();
        // buffer[ix+2] = myIMU.readFloatGyroZ();

        for (int i = 0; i < 3; i++)
        {
            if (fabs(buffer[ix + i]) > MAX_ACCEPTED_RANGE)
            {
                buffer[ix + i] = ei_get_sign(buffer[ix + i]) * MAX_ACCEPTED_RANGE;
            }
        }

        buffer[ix + 0] *= CONVERT_G_TO_MS2;
        buffer[ix + 1] *= CONVERT_G_TO_MS2;
        buffer[ix + 2] *= CONVERT_G_TO_MS2;

        delayMicroseconds(next_tick - micros());
    }

    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0)
    {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = {0};

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK)
    {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)", result.timing.dsp, result.timing.classification,
              result.timing.anomaly);
    ei_printf(": \n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
    {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif

    u8x8.clear();
    if (result.anomaly > 0.5)
    {
        printResultToDisp("anomaly", result.anomaly, WHITE);
    }
    // idle
    else if (result.classification[0].value > 0.5)
    {
        printResultToDisp(result.classification[0], RED);
    }
    // pullback
    else if (result.classification[1].value > 0.5)
    {
        printResultToDisp(result.classification[1], GREEN);
    }
    // pullforward
    else if (result.classification[2].value > 0.5)
    {
        printResultToDisp(result.classification[2], BLUE);
    }
    else
    {
        printResultToDisp("uncertain", 0.0, OFF);
    }
    delay(displayResultTimeMs);
}

void printResultToDisp(ei_impulse_result_classification_t &classResult, uint8_t color)
{
    printResultToDisp(classResult.label, classResult.value, color);
}

void printResultToDisp(const char *label, float value, uint8_t color)
{
    switch (color)
    {
        case OFF:
            digitalWrite(RED_ledPin, HIGH);
            digitalWrite(GREEN_ledPin, HIGH);
            digitalWrite(BLUE_ledPin, HIGH);
            break;
        case RED:
            digitalWrite(RED_ledPin, LOW);
            digitalWrite(GREEN_ledPin, HIGH);
            digitalWrite(BLUE_ledPin, HIGH);
            break;
        case GREEN:
            digitalWrite(RED_ledPin, HIGH);
            digitalWrite(GREEN_ledPin, LOW);
            digitalWrite(BLUE_ledPin, HIGH);
            break;

        case BLUE:
            digitalWrite(RED_ledPin, HIGH);
            digitalWrite(GREEN_ledPin, HIGH);
            digitalWrite(BLUE_ledPin, LOW);
            break;

        case YELLOW:
            digitalWrite(RED_ledPin, LOW);
            digitalWrite(GREEN_ledPin, LOW);
            digitalWrite(BLUE_ledPin, HIGH);
            break;

        case PURPLE:
            digitalWrite(RED_ledPin, LOW);
            digitalWrite(GREEN_ledPin, HIGH);
            digitalWrite(BLUE_ledPin, LOW);
            break;
        case WHITE:
            digitalWrite(RED_ledPin, LOW);
            digitalWrite(GREEN_ledPin, LOW);
            digitalWrite(BLUE_ledPin, LOW);
            break;
        default:
            break;
    }

    char numBuf[20] = {0};
    snprintf(numBuf, 20, "%.2f%", value * 100.0);

    u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
    u8x8.drawString(2, 3, label);
    u8x8.drawString(2, 5, numBuf);

    u8x8.refreshDisplay();
}