#ifndef CYBOT_SCAN_H_
#define CYBOT_SCAN_H_

// Author: Phillip Jones
// Data: Updated 10/27/2021
// Version: 10 27 2021  (you can use cyBOT_scan_version() to confirm version)
//
// Interface to the course-provided libcybotScan library (servo + PING + IR),
// extended with this project's scan / navigation API (implemented in scan.c).


/* ========================================================================== */
/*  Project scan / navigation API (scan.c)                                    */
/* ========================================================================== */

// Target-object midpoint (deg)
void setMidpoint(int midPoint);
int getMidPoint();

// Last PING distance (reserved for PING-based scanning)
void setDistance(float Ping);
float getPing();

// Convert an object's distance (cm) and angular span (deg) into a linear width (cm)
float calculateLinearWidth(float distance, int angleWidth);

// Forward-travel distance budget (cm), set from the nearest object in the last scan
void setMaxDistance(int maxDist);
int getMaxDistance();


/* ========================================================================== */
/*  libcybotScan library API                                                  */
/* ========================================================================== */

// Scan value
typedef struct{
    float sound_dist;  // Distance from PING sensor (cyBOT_Scan returns -1.0 if PING is not enabled)
    int IR_raw_val;    // Raw ADC value from IR sensor (cyBOT_Scan returns -1 if IR is not enabled)
} cyBOT_Scan_t;


// Name: cyBOT_init_Scan
// Input: feature, int where each feature is associated with a bit as follows
//  Bit 0: 1 enable servo
//  Bit 1: 1 enable PING
//  Bit 2: 1 enable IR
//  Bit 3 - 31: reserved (0's)
// Example: cyBOT_init_Scan(0b0111); // Enables servo, PING, and IR
void cyBOT_init_Scan(int feature);


// Point sensors to angle, and get a scan value
// Input Parameters:
// angle: Direction to point the sensors for taking a measurement
// getScan : The address of a declared cyBOT_Scan_t structure
// NOTE 1: If PING is not enabled, then getScan->sound_dist returns -1.0
// NOTE 2: If IR is not enabled, then getScan->IR_raw_val returns -1
void cyBOT_Scan(int angle, cyBOT_Scan_t* getScan);


// Returns the version of the libcybotScan library
// Hex encoding of Month Day Year: MMDDYYYY
unsigned int cyBOT_scan_version(void);


/* ========================================================================== */
/*  Project scan routines (scan.c)                                            */
/* ========================================================================== */

void longScan();              // Full 1-180 deg sweep, 3 deg steps
void shortScan();             // Forward 70-110 deg sweep, 1 deg steps
int IrToMeters(int adc_value);  // Voltage-based IR distance conversion (-1 if out of range)


////////////////////////////////////////////
//// Calibrate physical CyBOT            ///
////////////////////////////////////////////

// These will be different for each physical CyBOT servo,
// where 0 degrees (right) and 180 degrees (left) are located.
// These values can be found by running the servo calibrate function.
// Once you know the values, then you can assign them in main().
int right_calibration_value;
int left_calibration_value;


// Servo Calibrate value struct
typedef struct{
    int right;  // Right (0 degree) calibration value
    int left;   // Left (180 degree) calibration value
} cyBOT_SERVRO_cal_t;


// Return a struct containing the right (0 degree) and
// left (180 degree) values that can be assigned to variables
// right_calibration_value and left_calibration_value
// NOTE: If servo is not enabled, then returns -1 for right and left.
cyBOT_SERVRO_cal_t cyBOT_SERVO_cal(void);


#endif /* CYBOT_SCAN_H_ */
