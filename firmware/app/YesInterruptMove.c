/**
 * @file    YesInterruptMove.c
 * @brief   Main control loop for the autonomous radar-mapping robot.
 *
 * Commands arrive one character at a time over UART1 (bridged to TCP/Wi-Fi
 * by the CyBot radio module). The UART1 RX interrupt in new_uart_interrupt.c
 * writes each byte into `received_char`; this loop polls that value, executes
 * one step of the matching command, then clears it.
 *
 * Because the command is cleared after every pass, a pass with no new byte
 * falls through to the default case and stops the wheels. Motion only
 * continues while the operator keeps sending keys (key-repeat), which acts
 * as a dead-man's switch.
 *
 *   Command map
 *   -----------
 *   'w'  drive forward  (hazard-checked, limited by the last scan's distance budget)
 *   's'  reverse slowly
 *   'a'  rotate left 1 deg          'd'  rotate right 1 deg
 *   '9'  rotate left 90 deg         '0'  rotate right 90 deg
 *   '1'  short IR scan (70-110 deg)
 *   '2'  full IR scan (1-180 deg), re-arms forward motion, resets heading
 *   'q'  play a tune on the Create speaker
 *   else stop
 *
 * Target: TI TM4C123GH6PM (Tiva C) driving an iRobot Create 2 base.
 */

#include "open_interface.h"          // Interface for the Roomba robot
#include "Timer.h"                   // Timer utilities
#include "new_uart_interrupt.h"      // UART interrupt handling
#include "adc.h"                     // ADC sensor readings
#include "cyBot_Scan.h"              // Scanning utilities

volatile char received_char;         // Stores the most recent UART character received


/* ========================================================================== */
/*  Audio                                                                     */
/* ========================================================================== */

// Function to play a song using the Roomba speaker.
// Notes are MIDI note numbers; durations are in 1/64th-second units.
void play_music() {
    unsigned char notes[54] =    {79, 76, 72, 76, 79, 84, 88, 86, 84, 76, 78, 79, 79, 79, 88, 86, 84, 83, 81, 83, 84, 84, 79, 76, 72};
    unsigned char duration[54] = {36, 24, 48, 48, 48, 92, 36, 24, 48, 48, 48, 92, 24, 24, 60, 24, 36, 92, 24, 24, 48, 48, 48, 48, 48};
    oi_loadSong(0, 54, notes, duration);    // Store the sequence in song slot 0
    oi_play_song(0);                        // Play song slot 0
}


/* ========================================================================== */
/*  Hazard detection                                                          */
/*                                                                            */
/*  Each detector refreshes the Open Interface sensor packet and then checks  */
/*  the four downward-facing cliff sensors (or the bumpers).                  */
/*    cliff signal >= 2600  -> highly reflective surface (white boundary tape) */
/*    cliff signal <= 1000  -> little or no reflection (black area / drop-off) */
/* ========================================================================== */

// Detects if the robot is near a boundary (white tape)
int boundary_detect(oi_t *sensor_data) {
    oi_update(sensor_data);
    return (sensor_data->cliffLeftSignal >= 2600 || sensor_data->cliffRightSignal >= 2600 ||
            sensor_data->cliffFrontLeftSignal >= 2600 || sensor_data->cliffFrontRightSignal >= 2600);
}

// Detects if the robot is near a hole (black area)
int hole_detect(oi_t *sensor_data) {
    oi_update(sensor_data);
    return (sensor_data->cliffLeftSignal <= 1000 || sensor_data->cliffRightSignal <= 1000 ||
            sensor_data->cliffFrontLeftSignal <= 1000 || sensor_data->cliffFrontRightSignal <= 1000);
}

// Detects if the robot has bumped into a tall object (e.g., tree or rock)
int tall_tree_detect(oi_t *sensor_data) {
    oi_update(sensor_data);
    return (sensor_data->bumpLeft || sensor_data->bumpRight);
}


/* ========================================================================== */
/*  Main control loop                                                         */
/* ========================================================================== */

int main(void) {

    /* ---------------------------------------------------------------- */
    /*  Peripheral bring-up                                              */
    /* ---------------------------------------------------------------- */
    timer_init();                    // Initialize timers (TIMER5 microsecond timebase)
    uart_interrupt_init();           // Initialize UART1 with RX interrupts
    oi_t *sensor_data = oi_alloc();  // Allocate memory for sensor data
    oi_init(sensor_data);            // Initialize Roomba sensors

    lcd_init();                      // Initialize the LCD display

    /* ---------------------------------------------------------------- */
    /*  Navigation state                                                 */
    /* ---------------------------------------------------------------- */
    int boundary, hole, tall_tree;   // Detection flags
    int sum = 0;                     // Total distance moved since the last scan
    double currentDegree = 90;       // Starting angle (90 = facing straight ahead)
    char sendAngle[100];             // Buffer to send angle and distance data
    int pleaseScan = 1;              // Forward motion is armed (cleared when the distance budget is used up)

    while (1) {
        // Continuously update sensor flags
        boundary = boundary_detect(sensor_data);
        hole = hole_detect(sensor_data);
        tall_tree = tall_tree_detect(sensor_data);

        switch(received_char) {

        /* ------------------------------------------------------------ */
        case 'q': // Play music
            play_music();
            break;

        /* ------------------------------------------------------------ */
        case 'w': // Move forward with obstacle checking
            if (boundary || hole || tall_tree) {
                // Hazard present: stop immediately and report which sensor(s) tripped.
                // Priority order: boundary > hole > bump.
                oi_setWheels(0, 0);
                if (boundary) {
                    uart_sendStr("\r\nBoundary detected: ");
                    if (sensor_data->cliffLeftSignal >= 2600) uart_sendStr("[Left] ");
                    if (sensor_data->cliffRightSignal >= 2600) uart_sendStr("[Right] ");
                    if (sensor_data->cliffFrontLeftSignal >= 2600) uart_sendStr("[Front_Left] ");
                    if (sensor_data->cliffFrontRightSignal >= 2600) uart_sendStr("[Front_Right] ");
                } else if (hole) {
                    uart_sendStr("\r\nHole detected: ");
                    if (sensor_data->cliffLeftSignal <= 1000) uart_sendStr("[Left] ");
                    if (sensor_data->cliffRightSignal <= 1000) uart_sendStr("[Right] ");
                    if (sensor_data->cliffFrontLeftSignal <= 1000) uart_sendStr("[Front_Left] ");
                    if (sensor_data->cliffFrontRightSignal <= 1000) uart_sendStr("[Front_Right] ");
                } else if (tall_tree) {
                    uart_sendStr("\r\nRock detected: ");
                    if (sensor_data->bumpLeft) uart_sendStr("[Left] ");
                    if (sensor_data->bumpRight) uart_sendStr("[Right] ");
                }
            } else {
                // Path is safe. Drive only while there is distance budget left from
                // the most recent scan (getMaxDistance()) and motion is still armed.
                if (sum < getMaxDistance() && pleaseScan) {
                    oi_setWheels(150, 150);             // 150 mm/s on both wheels
                    sum += sensor_data->distance;       // Accumulate odometry
                    oi_update(sensor_data);
                    lcd_printf("%d", sum);              // Show progress on the LCD...
                    char newThing[3];
                    sprintf(newThing, "%d", sum);
                    uart_sendStr(newThing);             // ...and stream it to the client
                } else {
                    // Budget exhausted: stop, disarm forward motion, and ask for a rescan.
                    oi_setWheels(0, 0);
                    pleaseScan = 0;
                    uart_sendStr("Please rescan");
                    sum = 0;
                }
            }
            break;

        /* ------------------------------------------------------------ */
        case 's': // Move backward slowly
            oi_setWheels(-25, -25);
            break;

        /* ------------------------------------------------------------ */
        /*  Fine turns: closed-loop on the Create's angle odometry.       */
        /*  Spin in place, polling every 10 ms until >= 1 deg is turned.  */
        /* ------------------------------------------------------------ */
        case 'a': // Turn left by 1 degree
            if (currentDegree < 180.0) {
                double angle_turned = 0.0;
                sensor_data->angle = 0;
                oi_setWheels(25, -25);
                while (angle_turned < 1.0) {
                    timer_waitMillis(10);
                    oi_update(sensor_data);
                    angle_turned += fabs(sensor_data->angle);
                }
                oi_setWheels(0, 0);
                currentDegree += 1.0;
                if (currentDegree > 180.0) currentDegree = 180.0;

                // Map the internal heading onto the 0-180 deg scale shown to the
                // operator (calibrated: internal 60..121 -> displayed 0..180), then clamp.
                double displayedDegree = ((currentDegree - 60.0) * (180.0 / 61.0));
                if (displayedDegree < 0) displayedDegree = 0;
                if (displayedDegree > 180) displayedDegree = 180;

                // Report heading + odometry to the LCD and the Python client
                lcd_printf("Turning Left: %.1lf", displayedDegree);
                sprintf(sendAngle, "Angle: %0.2lf, Distance Moved: %d, Max Distance: %d", displayedDegree, sum, getMaxDistance());
                uart_sendStr(sendAngle);
                uart_sendChar('\n');
                uart_sendChar('\r');
            }
            break;

        case 'd': // Turn right by 1 degree
            if (currentDegree > 0.0) {
                double angle_turned = 0.0;
                sensor_data->angle = 0;
                oi_setWheels(-25, 25);
                while (angle_turned < 1.0) {
                    timer_waitMillis(10);
                    oi_update(sensor_data);
                    angle_turned += fabs(sensor_data->angle);
                }
                oi_setWheels(0, 0);
                currentDegree -= 1.0;
                if (currentDegree < 0.0) currentDegree = 0.0;

                // Same heading-to-display mapping as 'a'
                double displayedDegree = ((currentDegree - 60.0) * (180.0 / 61.0));
                if (displayedDegree < 0) displayedDegree = 0;
                if (displayedDegree > 180) displayedDegree = 180;

                lcd_printf("Turning Right: %.1lf", displayedDegree);
                sprintf(sendAngle, "Angle: %0.2lf, Distance Moved: %d, Max Distance: %d", displayedDegree, sum, getMaxDistance());
                uart_sendStr(sendAngle);
                uart_sendChar('\n');
                uart_sendChar('\r');
            }
            break;

        /* ------------------------------------------------------------ */
        /*  Scans (implemented in sensing/scan.c)                         */
        /* ------------------------------------------------------------ */
        case '1': // Perform short scan
            shortScan();
            break;

        case '2': // Perform long scan, then re-arm forward motion and reset heading
            longScan();
            pleaseScan = 1;
            currentDegree = 90;
            break;

        /* ------------------------------------------------------------ */
        /*  Coarse turns: same closed-loop approach, 90 deg target.       */
        /* ------------------------------------------------------------ */
        case '9': // Turn left 90 degrees
            if (currentDegree <= 90) {
                double angle_turned = 0.0;
                sensor_data->angle = 0;
                oi_setWheels(25, -25);
                while (angle_turned < 90.0) {
                    timer_waitMillis(10);
                    oi_update(sensor_data);
                    angle_turned += fabs(sensor_data->angle);
                }
                oi_setWheels(0, 0);
                currentDegree += 90.0;
                if (currentDegree > 180.0) currentDegree = 180.0;
                lcd_printf("Turning Left 90: %.1lf", currentDegree);

                sprintf(sendAngle, "Angle: %0.2lf, Distance Moved: %d, Max Distance: %d", currentDegree, sum, getMaxDistance());
                uart_sendStr(sendAngle);
                uart_sendChar('\n');
                uart_sendChar('\r');
            }
            break;

        case '0': // Turn right 90 degrees
            if (currentDegree >= -180.0) {
                double angle_turned = 0.0;
                sensor_data->angle = 0;
                oi_setWheels(-25, 25);
                while (angle_turned < 90.0) {
                    timer_waitMillis(10);
                    oi_update(sensor_data);
                    angle_turned += fabs(sensor_data->angle);
                }
                oi_setWheels(0, 0);
                currentDegree -= 90.0;
                if (currentDegree < -180.0) currentDegree = 90.0;
                lcd_printf("Turning Right 90: %.1lf", currentDegree);

                sprintf(sendAngle, "Angle: %0.2lf, Distance Moved: %d, Max Distance: %d", currentDegree, sum, getMaxDistance());
                uart_sendStr(sendAngle);
                uart_sendChar('\n');
                uart_sendChar('\r');
            }
            break;

        /* ------------------------------------------------------------ */
        default: // Stop if unknown key is pressed (or no key this pass: dead-man's switch)
            oi_setWheels(0, 0);
            break;
        }

        received_char = 0; // Reset received char for next command
    }
}
