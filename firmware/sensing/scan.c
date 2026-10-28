/**
 * @file    scan.c
 * @brief   IR sweep scanning, object segmentation, and distance-budget logic.
 *
 * A servo sweeps the Sharp IR sensor across the field of view. At each angle
 * the ADC is sampled three times (on top of the ADC's 16x hardware averaging)
 * and the mean is converted to centimetres with an empirical power-law model.
 *
 * Object segmentation is a single-pass run-length grouping:
 *
 *     angle:   70 71 72 73 74 75 76 77 78 79 80 ...
 *     hit?      0  0  1  1  1  1  0  0  1  1  0 ...
 *                     \_________/        \___/
 *                      object #1        object #2
 *
 * A contiguous run of "hits" (distance <= 70 cm) becomes one object. For each
 * object the firmware records its start/end angle, midpoint, closest IR
 * distance, and an estimated linear width (from angular span + distance).
 *
 * Results are streamed over UART as one line per object:
 *     "Min Degree: <a> MaxDegree: <b>, Irsensor: <cm>, Midpoint: <m>, ObjectWidth: <w>"
 * which the Python clients in /visualization parse and plot.
 *
 * The nearest object distance becomes the forward "distance budget" that
 * main() uses to limit how far the robot may drive before rescanning.
 */

#include "cyBot_Scan.h"
#include <stdio.h>
#include "new_uart_interrupt.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "adc.h"


/* ========================================================================== */
/*  Distance budget                                                           */
/* ========================================================================== */

int maxDistance = 70; // Default max distance in cm

// Sets the max distance the robot can move forward
void setMaxDistance(int maxDist) {
    maxDistance = maxDist;
}

// Returns the max allowed distance
int getMaxDistance() {
    return maxDistance;
}


/* ========================================================================== */
/*  IR sensor model                                                           */
/* ========================================================================== */

// Converts IR ADC value to distance in cm.
// Readings at or below 450 counts are treated as "nothing in range".
float IR_to_cm(uint16_t irVal) {
    if(irVal <= 450) {
        return 9999.0f; // Invalid reading or no object
    }
    float average = (float)irVal;
    return 120000.0f * powf(average, -1.23f); // Empirical formula: d = 120000 * adc^-1.23
}


/* ========================================================================== */
/*  Object model                                                              */
/* ========================================================================== */

// Structure to store object detection data
typedef struct {
    int minDegree;      // First angle at which the object was seen
    int maxDegree;      // Last angle at which the object was seen
    float Irsensor;     // Closest IR distance across the object (cm)
    int midPoint;       // Centre angle of the object
    float objectWidth;  // Estimated linear width (cm)
} ObjectData;

// Dynamically grown array of objects found in the most recent scan
ObjectData *detectedObject = NULL;


/* ========================================================================== */
/*  Short scan: narrow forward cone, 1 deg resolution                         */
/* ========================================================================== */

// Performs a short IR scan from 70 to 110 degrees
void shortScan() {
    // Enable servo (bit 0) + IR (bit 2); PING stays disabled
    cyBOT_init_Scan(0b0101);
    adc_init();

    // Per-robot servo calibration (pulse values for 0 deg and 180 deg)
    right_calibration_value = 274750;
    left_calibration_value = 1214500;

    cyBOT_Scan_t scan;
    int IRValue, degree, objectCount = 0, inObject = 0, temp, minIndex = 0;
    int IRscan[3], IRtotal;
    float currentMinIR = 9999.0;

    /* ---------------- Sweep + segment ---------------- */
    for(degree = 70; degree <= 110; degree++) {
        // Point the sensor, then take three ADC samples and average them
        cyBOT_Scan(degree, &scan);
        IRscan[0] = adc_read();
        IRscan[1] = adc_read();
        IRscan[2] = adc_read();

        IRtotal = IRscan[0] + IRscan[1] + IRscan[2];
        IRValue = IRtotal / 3;
        float IR_cm = IR_to_cm(IRValue);

        if(IR_cm > 0 && IR_cm <= 70) {
            uart_sendChar('1'); // Object detected

            if (!inObject) {
                // Rising edge: start a new object record
                currentMinIR = 9999;
                detectedObject = realloc(detectedObject, (objectCount + 1) * sizeof(*detectedObject));
                detectedObject[objectCount].minDegree = degree;
                detectedObject[objectCount].maxDegree = degree;
                inObject = 1;
                objectCount++;
            } else {
                // Still inside the same object: extend its angular span
                detectedObject[objectCount - 1].maxDegree = degree;
            }

            // Track the closest reading across the object
            if (IR_cm < currentMinIR) currentMinIR = IR_cm;
        } else {
            uart_sendChar('0'); // No object
            inObject = 0;

            // Falling edge / gap: finalize the most recent object's stats
            if(objectCount > 0) {
                int i = objectCount - 1;
                temp = detectedObject[i].maxDegree - detectedObject[i].minDegree;
                detectedObject[i].Irsensor = currentMinIR;
                detectedObject[i].objectWidth = calculateLinearWidth(currentMinIR, temp);
                detectedObject[i].midPoint = (detectedObject[i].minDegree + detectedObject[i].maxDegree) / 2;
            }
        }

        // Track the index of the narrowest object seen so far
        for(int i = 0; i < objectCount; i++) {
            if(detectedObject[i].objectWidth < detectedObject[minIndex].objectWidth) {
                minIndex = i;
            }
        }
    }

    // Close out an object that was still in view when the sweep ended
    if (inObject && objectCount > 0) {
        int i = objectCount - 1;
        temp = detectedObject[i].maxDegree - detectedObject[i].minDegree;
        detectedObject[i].Irsensor = currentMinIR;
        detectedObject[i].objectWidth = (currentMinIR < 9999 && currentMinIR > 0) ? calculateLinearWidth(currentMinIR, temp) : 0;
        detectedObject[i].midPoint = (detectedObject[i].minDegree + detectedObject[i].maxDegree) / 2;
    }

    uart_sendChar('\n');
    uart_sendChar('\r');

    /* ---------------- Report + update distance budget ---------------- */
    char Buffer[200];
    int b, min = 9999;

    for(b = 0; b < objectCount; b++) {
        // One telemetry line per object (parsed by the Python radar clients)
        sprintf(Buffer, "Min Degree: %d MaxDegree: %d, Irsensor: %f, Midpoint: %d, ObjectWidth: %f",
                detectedObject[b].minDegree, detectedObject[b].maxDegree, detectedObject[b].Irsensor,
                detectedObject[b].midPoint, detectedObject[b].objectWidth);
        uart_sendStr(Buffer);
        uart_sendChar('\n');
        uart_sendChar('\r');

        // Limit forward travel to this object's distance
        if(detectedObject[b].Irsensor < min && objectCount != 0)
            setMaxDistance(detectedObject[b].Irsensor);

        // Target signature: an object 10-15 cm wide
        if(detectedObject[b].objectWidth >= 10 && detectedObject[b].objectWidth < 15) {
            char FoundObject[20];
            sprintf(FoundObject, "Found! Angle: %d", detectedObject[b].midPoint);
            uart_sendStr(FoundObject);
        }
    }

    // Path verdict for the operator
    if(objectCount == 0) {
        uart_sendStr("Clear to go straight min distance");
        setMaxDistance(70);
    } else {
        uart_sendStr("Cannot go straight object in path");
    }

    uart_sendChar('\n');
    uart_sendChar('\r');
}


/* ========================================================================== */
/*  Long scan: full 180 deg field of view, 3 deg resolution                   */
/*  Same pipeline as shortScan(); see the comments there for each stage.      */
/* ========================================================================== */

// Performs a wide scan from 1 to 180 degrees
void longScan() {
    cyBOT_init_Scan(0b0101);    // Servo + IR
    adc_init();

    right_calibration_value = 274750;
    left_calibration_value = 1214500;

    cyBOT_Scan_t scan;
    int IRValue, degree, objectCount = 0, inObject = 0, temp, minIndex = 0;
    int IRscan[3], IRtotal;
    float currentMinIR = 9999.0;

    /* ---------------- Sweep + segment ---------------- */
    for(degree = 1; degree <= 180; degree += 3) {
        cyBOT_Scan(degree, &scan);
        IRscan[0] = adc_read();
        IRscan[1] = adc_read();
        IRscan[2] = adc_read();

        IRtotal = IRscan[0] + IRscan[1] + IRscan[2];
        IRValue = IRtotal / 3;
        float IR_cm = IR_to_cm(IRValue);

        if(IR_cm > 0 && IR_cm <= 70) {
            uart_sendChar('1');

            if (!inObject) {
                // Rising edge: new object
                currentMinIR = 9999;
                detectedObject = realloc(detectedObject, (objectCount + 1) * sizeof(*detectedObject));
                detectedObject[objectCount].minDegree = degree;
                detectedObject[objectCount].maxDegree = degree;
                inObject = 1;
                objectCount++;
            } else {
                detectedObject[objectCount - 1].maxDegree = degree;
            }

            if (IR_cm < currentMinIR) currentMinIR = IR_cm;
        } else {
            uart_sendChar('0');
            inObject = 0;

            // Finalize the most recent object
            if(objectCount > 0) {
                int i = objectCount - 1;
                temp = detectedObject[i].maxDegree - detectedObject[i].minDegree;
                detectedObject[i].Irsensor = currentMinIR;
                detectedObject[i].objectWidth = calculateLinearWidth(currentMinIR, temp);
                detectedObject[i].midPoint = (detectedObject[i].minDegree + detectedObject[i].maxDegree) / 2;
            }
        }

        for(int i = 0; i < objectCount; i++) {
            if(detectedObject[i].objectWidth < detectedObject[minIndex].objectWidth)
                minIndex = i;
        }
    }

    // Close out an object still in view at the end of the sweep
    if (inObject && objectCount > 0) {
        int i = objectCount - 1;
        temp = detectedObject[i].maxDegree - detectedObject[i].minDegree;
        detectedObject[i].Irsensor = currentMinIR;
        detectedObject[i].objectWidth = (currentMinIR < 9999 && currentMinIR > 0) ? calculateLinearWidth(currentMinIR, temp) : 0;
        detectedObject[i].midPoint = (detectedObject[i].minDegree + detectedObject[i].maxDegree) / 2;
    }

    uart_sendChar('\n');
    uart_sendChar('\r');

    /* ---------------- Report + update distance budget ---------------- */
    char Buffer[200];
    int b, min = 9999;

    for(b = 0; b < objectCount; b++) {
        sprintf(Buffer, "Min Degree: %d MaxDegree: %d, Irsensor: %f, Midpoint: %d, ObjectWidth: %f",
                detectedObject[b].minDegree, detectedObject[b].maxDegree, detectedObject[b].Irsensor,
                detectedObject[b].midPoint, detectedObject[b].objectWidth);

        uart_sendStr(Buffer);
        uart_sendChar('\n');
        uart_sendChar('\r');

        if(detectedObject[b].Irsensor < min && objectCount != 0)
            setMaxDistance(detectedObject[b].Irsensor);

        // Wide-scan target signature: anything at least 9.5 cm wide
        if(detectedObject[b].objectWidth >= 9.5) {
            char FoundObject[20];
            sprintf(FoundObject, "Found! Angle: %d", detectedObject[b].midPoint);
            uart_sendStr(FoundObject);
        }

        if(objectCount == 0) setMaxDistance(70);
    }
}


/* ========================================================================== */
/*  Shared state accessors and conversion helpers                             */
/* ========================================================================== */

// Stores and returns the most recent Ping (not used here, but may be for future PING sensor logic)
float returnPing;
void setDistance(float Ping) { returnPing = Ping; }
float getPing() { return returnPing; }

// Stores and returns the midpoint of the final target object
int ReturnMidpoint;
void setMidpoint(int midPoint) { ReturnMidpoint = midPoint; }
int getMidPoint() { return ReturnMidpoint; }

// Converts IR ADC value to distance in meters using voltage
// (12-bit ADC, 3.3 V reference; returns -1 below the sensor's 0.42 V floor)
int IrToMeters(int adc_value) {
    float voltage = (adc_value * 3.3) / 4090.2;
    if(voltage <= 0.42) return -1;
    float distance = 27.86 / (voltage - 0.42);
    return distance;
}

// Calculates the linear width of an object given
