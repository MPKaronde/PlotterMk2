#include <Arduino.h>
#include <AccelStepper.h>

// some useful facts
#define INTEGER_MAX_VAL 32767
#define INTEGER_MIN_VAL -32768

// Motor constants
#define HALFSTEP 4
// TODO: DETERMINE VALUES
const int CART_STEPS_PER_CM = 0; // Closest # steps/cm for x & y
const int Z_HALF_DIST = 0;       // how far z moves for down to half up or half up to full up
const int CARTESIAN_ACCEL = 10000000;
const int CARTESIAN_MAX_SPEED = 1200;
const int Z_ACCEL = 50;
const int Z_MAX_SPEED = 50;

// Motor declaration
// TODO: DETERMINE PORT VALUES
AccelStepper x(HALFSTEP, 0, 1, 2, 3);
AccelStepper y1(HALFSTEP, 4, 5, 6, 7);
AccelStepper y2(HALFSTEP, 8, 9, 10, 11);
AccelStepper z(HALFSTEP, 12, 13, 14, 15);

// Bump Switch Positions
// TODO: DETERMINE PORT VALUES
const int X_SWITCH_PORT = 16;
const int Y1_SWITCH_PORT = 17;
const int Y2_SWITCH_PORT = 18;

// Global Position
int x; // steps from zero position
int y; // steps from zero position
int z; // 0 = down, 1 = half up, 2 = full up

// zeros out x & y axis (not z)
// return true when complete, false if something went wrong
// NOTE: THIS FUNCTION BYPASSES INTERNAL ABSTRACTION AND DIRECTLY ACCESSES MOTORS
bool zeroAxis()
{
    const int stepAmt = -100;
    const int resetAmt = 150;

    bool x_zeroed = digitalRead(X_SWITCH_PORT) == LOW;
    bool y1_zeroed = digitalRead(Y1_SWITCH_PORT) == LOW;
    bool y2_zeroed = digitalRead(Y2_SWITCH_PORT) == LOW;

    while (!x_zeroed || !y1_zeroed || !y2_zeroed)
    {
        if (!x_zeroed)
        {
            x.setCurrentPosition(0);
            x.moveTo(stepAmt);
        }
        if (!y1_zeroed)
        {
            y1.setCurrentPosition(0);
            y1.moveTo(stepAmt);
        }
        if (!y2_zeroed)
        {
            y2.setCurrentPosition(0);
            y2.moveTo(stepAmt);
        }

        bool x_zeroed = digitalRead(X_SWITCH_PORT) == LOW;
        bool y1_zeroed = digitalRead(Y1_SWITCH_PORT) == LOW;
        bool y2_zeroed = digitalRead(Y2_SWITCH_PORT) == LOW;
    }

    x.setCurrentPosition(0);
    x.moveTo(resetAmt);
    y1.setCurrentPosition(0);
    y1.moveTo(resetAmt);
    y2.setCurrentPosition(0);
    y2.moveTo(resetAmt);

    x = 0;
    y = 0;
}

void setup()
{
    // Motor setup
    x.setAcceleration(CARTESIAN_ACCEL);
    y1.setAcceleration(CARTESIAN_ACCEL);
    y2.setAcceleration(CARTESIAN_ACCEL);
    x.setMaxSpeed(CARTESIAN_MAX_SPEED);
    y1.setMaxSpeed(CARTESIAN_MAX_SPEED);
    y2.setMaxSpeed(CARTESIAN_MAX_SPEED);
    z.setAcceleration(Z_ACCEL);
    z.setMaxSpeed(Z_MAX_SPEED);

    // bump switch port setup
    pinMode(X_SWITCH_PORT, INPUT_PULLUP);
    pinMode(Y1_SWITCH_PORT, INPUT_PULLUP);
    pinMode(Y2_SWITCH_PORT, INPUT_PULLUP);

    zeroAxis();
    z = 2;
}

// returns num cm represented by a given num steps in cartesian axis's
double cmFromSteps(int steps)
{
    return (double)steps * (1.0 / (double)CART_STEPS_PER_CM);
}

// returns num steps needed to achieve num cm in cartesian axis's
int stepsFromCm(double cm)
{
    return (int)cm * CART_STEPS_PER_CM;
}
