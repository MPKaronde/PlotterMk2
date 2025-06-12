#include <Arduino.h>
#include <AccelStepper.h>

// some useful facts
#define INTEGER_MAX_VAL 32767
#define INTEGER_MIN_VAL -32768
#define BAUD_RATE 9600

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
int x_pos; // steps from zero position
int y_pos; // steps from zero position
int z_pos; // 0 = full up, 1 = half up, 2 = down

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

        while (x.distanceToGo() != 0)
        {
            x.run();
        }
        while (y1.distanceToGo() != 0)
        {
            y1.run();
        }
        while (y2.distanceToGo() != 0)
        {
            y2.run();
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

    x_pos = 0;
    y_pos = 0;

    return true;
}

void setup()
{
    // sets up serial
    Serial.begin(9600);

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
    z_pos = 2;
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

/*
Lowest Level Control Commands: The following commands interact directly with the motors via the accelstepper library.
Additionally, they update machine state accordingly to their movements.
All other motor manipulation should be done through these methods to avoid future headache
*/

// moves pen to given position
// 0 = full up, 1 = half up, 2 = down
// return true if move successful, false otherwise
bool movePenToPos(int pos)
{
    // illegal position
    if (pos < 0 || pos > 2)
    {
        return false;
    }
    int movAmt = pos - z_pos;
    z.setCurrentPosition(0);
    z.moveTo(movAmt * Z_HALF_DIST);
    while (z.distanceToGo() != 0)
    {
        z.run();
    }
    z_pos = pos;
    return true;
}

// moves x by given number of steps
// if bump switch hit, return false and stop
bool moveXBySteps(int steps)
{
    x.setCurrentPosition(0);
    x.moveTo(steps);
    while (x.distanceToGo() != 0)
    {
        x.run();

        // bump switch collided / bounds reached, exit
        if (digitalRead(X_SWITCH_PORT) == LOW)
        {
            return false;
        }
    }

    x_pos += steps;
    return true;
}

// moves both y by given number of steps
// if either bump switch is hit, return false and stop
bool moveYBySteps(int steps)
{
    y1.setCurrentPosition(0);
    y2.setCurrentPosition(0);
    y1.moveTo(steps);
    y2.moveTo(steps);
    while (y1.distanceToGo() != 0 || y2.distanceToGo() != 0)
    {
        y1.run();
        y2.run();
        // bump switch collided, exit
        if (digitalRead(Y1_SWITCH_PORT) == LOW || digitalRead(Y2_SWITCH_PORT) == LOW)
        {
            return false;
        }
    }

    y_pos += steps;
    return true;
}

// moves x and y simultaneously to their expected positions
// stop and return false if bump switch hit
bool runToPoint(int x_steps, int y_steps)
{
    x.setCurrentPosition(0);
    y1.setCurrentPosition(0);
    y2.setCurrentPosition(0);
    x.moveTo(x_steps);
    y1.moveTo(y_steps);
    y2.moveTo(y_steps);

    while (x.distanceToGo() != 0 || y1.distanceToGo() != 0 || y2.distanceToGo() != 0)
    {
        x.run();
        y1.run();
        y2.run();

        // bound hit, exit
        if (digitalRead(X_SWITCH_PORT) == LOW || digitalRead(Y1_SWITCH_PORT) == LOW || digitalRead(Y2_SWITCH_PORT) == LOW)
        {
            return false;
        }
    }

    x_pos += x_steps;
    y_pos += y_steps;
    return true;
}

/*
End of the Lowest Level Control Commands.
From here on out, NOTHING should directly interact with AccelStepper.
All Movement commands go through the above methods instead.
*/

// move x by a given number of cm
// if bump switch hit, stops and return false
bool moveXByCm(double cm)
{
    return moveXBySteps(stepsFromCm(cm));
}

// move y by a given number of cm
// if bump switch hit, stops and return false
bool moveYByCm(double cm)
{
    return moveYBySteps(stepsFromCm(cm));
}

// move x and y simultaneously by given number of cm
// if bump switch hit, stops and return false
bool runToPointByCm(double x_cm, double y_cm)
{
    return runToPoint(stepsFromCm(x_cm), stepsFromCm(y_cm));
}

// parses and executes a given command, communicates with user
// return command execution state, false if command not known
bool parseCommand(String command)
{
    int space1 = command.indexOf(" ");
    String identifier = command;
    if (space1 != -1)
    {
        identifier = command.substring(0, space1);
    }

    // z =  zero axis
    if (command == "z")
    {
        return zeroAxis();
    }
    // xbs = move x by steps
    if (command == "xbs")
    {
        int numSteps = command.substring(space1).toInt();
        return moveXBySteps(numSteps);
    }
    // ybs = move y by steps
    else if (command == "ybs")
    {
        int numSteps = command.substring(space1).toInt();
        return moveYBySteps(numSteps);
    }
    // rtp = run to point (steps)
    else if (command == "rtp")
    {
        int space2 = command.indexOf(" ", space1);
        int xSteps = command.substring(space1, space2).toInt();
        int ySteps = command.substring(space2).toInt();
        return runToPoint(xSteps, ySteps);
    }
    // xbcm = move x by cm
    else if (command == "xbcm")
    {
        int numSteps = command.substring(space1).toDouble();
        return moveXByCm(numSteps);
    }
    // ybcm = move y by cm
    else if (command == "ybcm")
    {
        int numSteps = command.substring(space1).toDouble();
        return moveYByCm(numSteps);
    }
    // rtpcm
    else if (command == "rtpcm")
    {
        int space2 = command.indexOf(" ", space1);
        int xSteps = command.substring(space1, space2).toDouble();
        int ySteps = command.substring(space2).toDouble();
        return runToPointByCm(xSteps, ySteps);
    }
    // unknown command
    else
    {
        return false;
    }
    return true;
}
