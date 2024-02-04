#include "ACRL_trajectories.h"

// ACRL_trajectories.cpp
// This file contains the trajectories for running ACRL's adaptive flight mode.
// Copyright 2021 Sheng Cheng, all rights reserved.

// Dr. Sheng Cheng, Nov. 2021
// Email: chengs@illinois.edu
// Advanced Controls Research Laboratory
// Department of Mechanical Science and Engineering
// University of Illinois Urbana-Champaign
// Urbana, IL 61821, USA

void ACRL_trajectory_takeoff(float timeInThisRun,
                             Vector3f *targetPos,
                             Vector3f *targetVel,
                             Vector3f *targetAcc,
                             Vector3f *targetJerk,
                             Vector3f *targetSnap,
                             Vector2f *targetYaw,
                             Vector2f *targetYaw_dot,
                             Vector2f *targetYaw_ddot)
{
    // The function ACRL_trajectory_takeoff function evaluates the takeoff trajectory.
    // The order of the polynomial coefficients follow the matlab definition.
    float polyCoef[8] = {-0.1563, 1.0938, -2.6250, 2.1875, 0, 0, 0, 0};
    if (timeInThisRun < 2) // only takeoff during the first 2 seconds when entering mode ADAPTIVE
    {
        *targetPos = (Vector3f){0, 0, -polyEval(polyCoef, timeInThisRun, 8)};
        
        *targetVel = (Vector3f){0, 0, -polyDiffEval(polyCoef, timeInThisRun, 8)};

        *targetAcc = (Vector3f){0, 0, -polyDiff2Eval(polyCoef, timeInThisRun, 8)};

        *targetJerk = (Vector3f){0, 0, -polyDiff3Eval(polyCoef, timeInThisRun, 8)};

        *targetSnap = (Vector3f){0, 0, -polyDiff4Eval(polyCoef, timeInThisRun, 8)};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
    else
    {
        // print the error information that the quadrotor cannot take off
        gcs().send_text(MAV_SEVERITY_INFO, "Quadrotor cannot takeoff: time is %f > 2 s.", timeInThisRun);
    }
}

void ACRL_trajectory_transition_to_start(float timeInThisRun,
                             float radiusX,
                             float timeOffset,
                             Vector3f *targetPos,
                             Vector3f *targetVel,
                             Vector3f *targetAcc,
                             Vector3f *targetJerk,
                             Vector3f *targetSnap,
                             Vector2f *targetYaw,
                             Vector2f *targetYaw_dot,
                             Vector2f *targetYaw_ddot)
{
    // The function ACRL_trajectory_transition_to_start function evaluates transition trajectory from (0,0,-1) to (0,-radiusX, -1), where radiusX is the circle radius.
    // The order of the polynomial coefficients follow the matlab definition.
    float polyCoef[8] = {-0.1563, 1.0938, -2.6250, 2.1875, 0, 0, 0, 0};
    if (timeInThisRun >= timeOffset && timeInThisRun <= timeOffset + 2) // only do the transition in 2-4 seconds when entering mode ADAPTIVE
    {
        *targetPos = (Vector3f){0, -radiusX * polyEval(polyCoef, timeInThisRun - timeOffset, 8), -1};
        
        *targetVel = (Vector3f){0, -radiusX * polyDiffEval(polyCoef, timeInThisRun - timeOffset, 8), 0};

        *targetAcc = (Vector3f){0, -radiusX * polyDiff2Eval(polyCoef, timeInThisRun - timeOffset, 8), 0};

        *targetJerk = (Vector3f){0, -radiusX * polyDiff3Eval(polyCoef, timeInThisRun - timeOffset, 8), 0};

        *targetSnap = (Vector3f){0, -radiusX * polyDiff4Eval(polyCoef, timeInThisRun - timeOffset, 8), 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
    }
    else
    {
        // print the error information that the quadrotor cannot take off
        gcs().send_text(MAV_SEVERITY_INFO, "Quadrotor can't transition to circle start position: time is %f, not in [2,4] s.", timeInThisRun);
    }
}

// ACRL_trajectory_transition_to_start(timeInThisRun, radiusX, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);

void ACRL_trajectory_circle_variable_yaw(float timeInThisRun,
                                         float radius,
                                         float initialTimeOffset,
                                         float targetSpeed,
                                         Vector3f *targetPos,
                                         Vector3f *targetVel,
                                         Vector3f *targetAcc,
                                         Vector3f *targetJerk,
                                         Vector3f *targetSnap,
                                         Vector2f *targetYaw,
                                         Vector2f *targetYaw_dot,
                                         Vector2f *targetYaw_ddot)
{
    // The function ACRL_trajectory_variable_yaw function evaluates the circular trajectory that always aligns the yaw angle with the tangent speed direction.
    static float currentSpeed = 0;    // current speed of the rotation
    static float timeOffset = 0;      // offset in time for the circular trajectory at different speeds 
    static float currentLoopTime = 0; // time to complete one circle under current speed
    float netTime = 0;                // time for the circular trajectory at different speeds
    static u_int8_t accComplete = 0;  // indicator for whether acceleration is complete

    if ((currentSpeed <= 0.1) && (targetSpeed >= 0.5)) // initialize and only accelerate for targetSpeed >= 0.51 m/s
    {
        currentSpeed = 0.5 / radius;           // initialize as 0.5 m/s (currentSpeed is angular rate, the linear speed is currentSpeed * radius)
        timeOffset = initialTimeOffset;        // initial time for starting the trajectory
        currentLoopTime = 6.28 / currentSpeed; // time to complete one circle under current speed: (2 * pi * radius) / (currentSpeed * radius)
        gcs().send_text(MAV_SEVERITY_INFO, "Acceleration initiated.");
        gcs().send_text(MAV_SEVERITY_INFO, "Initial speed at 0.5 m/s (complete in %f s)", currentLoopTime);
    }
    else if ((timeInThisRun - timeOffset >= currentLoopTime) && (!accComplete)) // if the drone has completed one circle at the current speed and acceleration is not complete
    {
        if ((currentSpeed*radius) >= 1.98) // increase speed by 0.1 m/s if the linear speed goes beyond 1.98 m/s (approx. 2 m/s) 
        {
            currentSpeed += 0.1 / radius; 
        }
        else                                // otherwise, increase speed by 0.5 m/s
        {
            currentSpeed += 0.5 / radius; 
        }
        if (currentSpeed > (targetSpeed / radius))
        {
            timeOffset += currentLoopTime;         // update the loop time
            currentSpeed = targetSpeed / radius; // set current speed to the circspeed
            accComplete = 1;                     // flag the acceleration as complete
            gcs().send_text(MAV_SEVERITY_INFO, "Reached target speed %f m/s", (double)targetSpeed);
        }
        else
        {
            timeOffset += currentLoopTime;         // update the loop time
            currentLoopTime = 6.28 / currentSpeed; // time to complete one circle under current speed
            gcs().send_text(MAV_SEVERITY_INFO, "Starting %f m/s (complete in %f s)", currentSpeed * radius, currentLoopTime);
        }
    }
    netTime = timeInThisRun - timeOffset; // time reference for this speed

    #if (!REAL_OR_SITL) // SITL
        *targetPos = (Vector3f){radius * sinf(currentSpeed * netTime), radius * (1 - cosf(currentSpeed * netTime)), -1};
    #elif (REAL_OR_SITL) // Real 
        *targetPos = (Vector3f){radius * sinf(currentSpeed * netTime), radius * (-cosf(currentSpeed * netTime)), -1};
    #endif

    *targetVel = (Vector3f){radius * currentSpeed * cosf(currentSpeed * netTime), radius * currentSpeed * sinf(currentSpeed * netTime), 0};

    *targetAcc = (Vector3f){-radius * powf(currentSpeed, 2) * sinf(currentSpeed * netTime), radius * powf(currentSpeed, 2) * cosf(currentSpeed * netTime), 0};

    *targetJerk = (Vector3f){-radius * powf(currentSpeed, 3) * cosf(currentSpeed * netTime), -radius * powf(currentSpeed, 3) * sinf(currentSpeed * netTime), 0};

    *targetSnap = (Vector3f){radius * powf(currentSpeed, 4) * sinf(currentSpeed * netTime), -radius * powf(currentSpeed, 4) * cosf(currentSpeed * netTime), 0};

    *targetYaw = (Vector2f){cosF(currentSpeed * netTime), sinF(currentSpeed * netTime)};
    *targetYaw_dot = (Vector2f){-currentSpeed * sinF(currentSpeed * netTime), currentSpeed * cosF(currentSpeed * netTime)};
    *targetYaw_ddot = (Vector2f){-powf(currentSpeed, 2) * cosF(currentSpeed * netTime), -powf(currentSpeed, 2) * sinF(currentSpeed * netTime)};
}

void ACRL_trajectory_circle_fixed_yaw(float timeInThisRun,
                                      float radius,
                                      float initialTimeOffset,
                                      float targetSpeed,
                                      Vector3f *targetPos,
                                      Vector3f *targetVel,
                                      Vector3f *targetAcc,
                                      Vector3f *targetJerk,
                                      Vector3f *targetSnap,
                                      Vector2f *targetYaw,
                                      Vector2f *targetYaw_dot,
                                      Vector2f *targetYaw_ddot)
{
    // The function ACRL_trajectory_fixed_yaw function evaluates the circular trajectory that has a fixed yaw angle (=0).
    static float currentSpeed = 0;    // current speed of the rotation
    static float timeOffset = 0;      // offset in time for the circular trajectory at different speeds 
    static float currentLoopTime = 0; // time to complete one circle under current speed
    float netTime = 0;                // time for the circular trajectory at different speeds
    static u_int8_t accComplete = 0;  // indicator for whether acceleration is complete

    if ((currentSpeed <= 0.1) && (targetSpeed >= 0.5)) // initialize and only accelerate for targetSpeed >= 0.51 m/s
    {
        currentSpeed = 0.5 / radius;           // initialize as 0.5 m/s (currentSpeed is angular rate, the linear speed is currentSpeed * radius)
        timeOffset = initialTimeOffset;        // initial time for starting the trajectory
        currentLoopTime = 6.28 / currentSpeed; // time to complete one circle under current speed: 2 * pi * radius / currentSpeed
        gcs().send_text(MAV_SEVERITY_INFO, "Acceleration initiated.");
        gcs().send_text(MAV_SEVERITY_INFO, "Initial speed at 0.5 m/s (complete in %f s)", currentLoopTime);
    }
    else if ((timeInThisRun - timeOffset >= currentLoopTime) && (!accComplete)) // if the drone has completed one circle at the current speed and acceleration is not complete
    {
        if ((currentSpeed*radius) >= 1.98) // increase speed by 0.1 m/s if the linear speed goes beyond 1.98 m/s (approx. 2 m/s) 
        {
            currentSpeed += 0.1 / radius; 
        }
        else                                // otherwise, increase speed by 0.5 m/s
        {
            currentSpeed += 0.5 / radius; 
        }
        if (currentSpeed > (targetSpeed / radius))
        {   
            timeOffset += currentLoopTime;         // update the loop time
            currentSpeed = targetSpeed / radius; // set current speed to the circspeed
            accComplete = 1;                     // flag the acceleration as complete
            gcs().send_text(MAV_SEVERITY_INFO, "Reached target speed %f m/s", (double)targetSpeed);
        }
        else
        {
            timeOffset += currentLoopTime;         // update the loop time
            currentLoopTime = 6.28 / currentSpeed; // time to complete one circle under current speed
            gcs().send_text(MAV_SEVERITY_INFO, "Starting %f m/s (complete in %f s)", currentSpeed * radius, currentLoopTime);
        }
    }
    netTime = timeInThisRun - timeOffset; // time reference for this speed

    #if (!REAL_OR_SITL) // SITL
        *targetPos = (Vector3f){radius * sinf(currentSpeed * netTime), radius * (1 - cosf(currentSpeed * netTime)), -1};
    #elif (REAL_OR_SITL) // Real 
        *targetPos = (Vector3f){radius * sinf(currentSpeed * netTime), radius * (-cosf(currentSpeed * netTime)), -1};
    #endif

    *targetVel = (Vector3f){radius * currentSpeed * cosf(currentSpeed * netTime), radius * currentSpeed * sinf(currentSpeed * netTime), 0};

    *targetAcc = (Vector3f){-radius * powf(currentSpeed, 2) * sinf(currentSpeed * netTime), radius * powf(currentSpeed, 2) * cosf(currentSpeed * netTime), 0};

    *targetJerk = (Vector3f){-radius * powf(currentSpeed, 3) * cosf(currentSpeed * netTime), -radius * powf(currentSpeed, 3) * sinf(currentSpeed * netTime), 0};

    *targetSnap = (Vector3f){radius * powf(currentSpeed, 4) * sinf(currentSpeed * netTime), -radius * powf(currentSpeed, 4) * cosf(currentSpeed * netTime), 0};

    *targetYaw = (Vector2f){1, 0};
    *targetYaw_dot = (Vector2f){0, 0};
    *targetYaw_ddot = (Vector2f){0, 0};
}

void ACRL_trajectory_figure8_fixed_yaw(float timeInThisRun,
                                       float radiusX,
                                       float radiusY,
                                       float targetSpeed,
                                       Vector3f *targetPos,
                                       Vector3f *targetVel,
                                       Vector3f *targetAcc,
                                       Vector3f *targetJerk,
                                       Vector3f *targetSnap,
                                       Vector2f *targetYaw,
                                       Vector2f *targetYaw_dot,
                                       Vector2f *targetYaw_ddot)
{
    // The function ACRL_trajectory_figure8_fixed_yaw function evaluates the figure8 trajectory that has a fixed yaw angle (=0).
    static float currentRate = 0;     // current max speed at origin
    static float currentEqvRate = 0;  // current equivalent speed to ease computation
    static float timeOffset = 0;      // offset in time for the circular trajectory at different speeds 
    static float currentLoopTime = 0; // time to complete one circle under current speed
    float netTime = 0;                // time for the circular trajectory at different speeds
    static u_int8_t accComplete = 0; // indicator for whether acceleration is complete
    static uint8_t positiveLoop = 1; // This is the indicator of positive loop, where the drone flies in the positive region of x
                                     // The value of positiveLoop is 0 if the drone flies in the negative loop
    
    const float scaleFactor = sqrtf(radiusX * radiusX + 4 * radiusY * radiusY); // scale the coefficients for the cos/sin functions to the speed so the final speed when passing through the origin equals targetSpeed

    if ((currentRate <= 0.1) && (targetSpeed >= 0.5)) // initialize and only accelerate for targetSpeed >= 0.5 m/s
    {
        currentRate = 0.5;                  // initialize as 0.5 m/s
        currentEqvRate = currentRate / scaleFactor;
        timeOffset = 2;                   // initial time after takeoff
        currentLoopTime = 3.14 / currentEqvRate; // time to complete one circle under current speed: 2 * pi * radius / currentRate
        gcs().send_text(MAV_SEVERITY_INFO, "Acceleration initiated.");
        gcs().send_text(MAV_SEVERITY_INFO, "Starting %f m/s (complete in %f s)", currentRate, currentLoopTime);
    }
    else if ((timeInThisRun - timeOffset >= currentLoopTime) && (!accComplete)) // if the drone has completed one circle at the current speed and acceleration is not complete
    {
        if (abs(currentRate - targetSpeed) <= 0.01) // acceleration complete
        {
            currentRate = targetSpeed; // set current speed to the circspeed
            currentEqvRate = currentRate / scaleFactor;
            accComplete = 1; // flag the acceleration as complete
            gcs().send_text(MAV_SEVERITY_INFO, "Reached target speed %f m/s", (double)targetSpeed);
        }
        else // accelerating
        {
            currentRate += 0.5;            // increase speed by 0.5
            if (currentRate > targetSpeed) // fix the currentRate if it goes beyond target speed
            {
                currentRate = targetSpeed;
            }
            currentEqvRate = currentRate / scaleFactor;
            timeOffset += currentLoopTime; // update the loop time
            currentLoopTime = 3.14 / currentEqvRate; // time to complete one circle under current speed
            if (positiveLoop)                        // flip the direction upon acceleration
            {
                positiveLoop = 0;
            }
            else
            {
                positiveLoop = 1;
            }
            gcs().send_text(MAV_SEVERITY_INFO, "Starting %f m/s (complete in %f s)", currentRate, currentLoopTime);
        }
    }
    netTime = timeInThisRun - timeOffset; // time reference for this speed

    float sf = sinf(currentEqvRate * netTime);
    float s2f = sinf(2 * currentEqvRate * netTime);
    float cf = cosf(currentEqvRate * netTime);
    float c2f = cosf(2 * currentEqvRate * netTime);

    if (positiveLoop)
    {
        *targetPos = (Vector3f){radiusX * sf, radiusY * s2f, -1};

        *targetVel = (Vector3f){radiusX * currentEqvRate * cf, radiusY * 2 * currentEqvRate * c2f, 0};

        *targetAcc = (Vector3f){-radiusX * currentEqvRate * currentEqvRate * sf, -radiusY * 4 * currentEqvRate * currentEqvRate * s2f, 0};

        *targetJerk = (Vector3f){-radiusX * powf(currentEqvRate, 3) * cf, -radiusY * 8 * powf(currentEqvRate, 3) * c2f, 0};

        *targetSnap = (Vector3f){radiusX * powf(currentEqvRate, 4) * sf, radiusY * 16 * powf(currentEqvRate, 4) * s2f, 0};
    }
    else
    {   
        *targetPos = (Vector3f){-radiusX * sf, radiusY * s2f, -1};

        *targetVel = (Vector3f){-radiusX * currentEqvRate * cf, radiusY * 2 * currentEqvRate * c2f, 0};

        *targetAcc = (Vector3f){radiusX * currentEqvRate * currentEqvRate * sf, -radiusY * 4 * currentEqvRate * currentEqvRate * s2f, 0};

        *targetJerk = (Vector3f){radiusX * powf(currentEqvRate, 3) * cf, -radiusY * 8 * powf(currentEqvRate, 3) * c2f, 0};

        *targetSnap = (Vector3f){-radiusX * powf(currentEqvRate, 4) * sf, radiusY * 16 * powf(currentEqvRate, 4) * s2f, 0};
    }
    *targetYaw = (Vector2f){1, 0};
    *targetYaw_dot = (Vector2f){0, 0};
    *targetYaw_ddot = (Vector2f){0, 0};
}

// trajectory 4
void ACRL_trajectory_figure8_tilted(float timeInThisRun,
                                       float radiusX,
                                       float radiusY,
                                       float targetSpeed,
                                       Vector3f *targetPos,
                                       Vector3f *targetVel,
                                       Vector3f *targetAcc,
                                       Vector3f *targetJerk,
                                       Vector3f *targetSnap,
                                       Vector2f *targetYaw,
                                       Vector2f *targetYaw_dot,
                                       Vector2f *targetYaw_ddot)
{
    // The function ACRL_trajectory_figure8_tilted function evaluates the figure8 trajectory that has a tilted altitude.
    static float currentRate = 0;     // current max speed at origin
    static float currentEqvRate = 0;  // current equivalent speed to ease computation
    static float timeOffset = 0;      // offset in time for the circular trajectory at different speeds 
    static float currentLoopTime = 0; // time to complete one circle under current speed
    float netTime = 0;                // time for the circular trajectory at different speeds
    static u_int8_t accComplete = 0; // indicator for whether acceleration is complete
    static uint8_t positiveLoop = 1; // This is the indicator of positive loop, where the drone flies in the positive region of x
                                     // The value of positiveLoop is 0 if the drone flies in the negative loop

    float z_amp = 0.2;               // amplitude of tilted altitude (z axis)     
    const float scaleFactor = sqrtf(radiusX * radiusX + 4 * radiusY * radiusY + z_amp * z_amp); // scale the coefficients for the cos/sin functions to the speed so the final speed when passing through the origin equals targetSpeed

    if ((currentRate <= 0.1) && (targetSpeed >= 0.5)) // initialize and only accelerate for targetSpeed >= 0.5 m/s
    {
        currentRate = 0.5;                  // initialize as 0.5 m/s
        currentEqvRate = currentRate / scaleFactor;
        timeOffset = 2;                   // initial time after takeoff
        currentLoopTime = 3.14 / currentEqvRate; // time to complete one circle under current speed: 2 * pi * radius / currentRate
        gcs().send_text(MAV_SEVERITY_INFO, "Acceleration initiated.");
        gcs().send_text(MAV_SEVERITY_INFO, "Starting %f m/s (complete in %f s)", currentRate, currentLoopTime);
    }
    else if ((timeInThisRun - timeOffset >= currentLoopTime) && (!accComplete)) // if the drone has completed one circle at the current speed and acceleration is not complete
    {
        if (abs(currentRate - targetSpeed) <= 0.01) // acceleration complete
        {
            currentRate = targetSpeed; // set current speed to the circspeed
            currentEqvRate = currentRate / scaleFactor;
            accComplete = 1; // flag the acceleration as complete
            gcs().send_text(MAV_SEVERITY_INFO, "Reached target speed %f m/s", (double)targetSpeed);
        }
        else // accelerating
        {
            currentRate += 0.5;            // increase speed by 0.5
            if (currentRate > targetSpeed) // fix the currentRate if it goes beyond target speed
            {
                currentRate = targetSpeed;
            }
            currentEqvRate = currentRate / scaleFactor;
            timeOffset += currentLoopTime; // update the loop time
            currentLoopTime = 3.14 / currentEqvRate; // time to complete one circle under current speed
            if (positiveLoop)                        // flip the direction upon acceleration
            {
                positiveLoop = 0;
            }
            else
            {
                positiveLoop = 1;
            }
            gcs().send_text(MAV_SEVERITY_INFO, "Starting %f m/s (complete in %f s)", currentRate, currentLoopTime);
        }
    }
    netTime = timeInThisRun - timeOffset; // time reference for this speed

    float sf = sinf(currentEqvRate * netTime);
    float s2f = sinf(2 * currentEqvRate * netTime);
    float cf = cosf(currentEqvRate * netTime);
    float c2f = cosf(2 * currentEqvRate * netTime);

    if (positiveLoop)
    {
        *targetPos = (Vector3f){radiusX * sf, radiusY * s2f, -1 - z_amp * sf};

        *targetVel = (Vector3f){radiusX * currentEqvRate * cf, radiusY * 2 * currentEqvRate * c2f, -z_amp * currentEqvRate * cf};

        *targetAcc = (Vector3f){-radiusX * currentEqvRate * currentEqvRate * sf, -radiusY * 4 * currentEqvRate * currentEqvRate * s2f, z_amp * currentEqvRate * currentEqvRate * sf};

        *targetJerk = (Vector3f){-radiusX * powf(currentEqvRate, 3) * cf, -radiusY * 8 * powf(currentEqvRate, 3) * c2f, z_amp * powf(currentEqvRate, 3) * cf};

        *targetSnap = (Vector3f){radiusX * powf(currentEqvRate, 4) * sf, radiusY * 16 * powf(currentEqvRate, 4) * s2f, -z_amp * powf(currentEqvRate, 4) * sf};
    }
    else
    {
        *targetPos = (Vector3f){-radiusX * sf, radiusY * s2f, -1 + z_amp * sf};

        *targetVel = (Vector3f){-radiusX * currentEqvRate * cf, radiusY * 2 * currentEqvRate * c2f, z_amp * currentEqvRate * cf};

        *targetAcc = (Vector3f){radiusX * currentEqvRate * currentEqvRate * sf, -radiusY * 4 * currentEqvRate * currentEqvRate * s2f, -z_amp * currentEqvRate * currentEqvRate * sf};

        *targetJerk = (Vector3f){radiusX * powf(currentEqvRate, 3) * cf, -radiusY * 8 * powf(currentEqvRate, 3) * c2f, -z_amp * powf(currentEqvRate, 3) * cf};

        *targetSnap = (Vector3f){-radiusX * powf(currentEqvRate, 4) * sf, radiusY * 16 * powf(currentEqvRate, 4) * s2f, z_amp * powf(currentEqvRate, 4) * sf};
    }
    
    *targetYaw = (Vector2f){1, 0};
    *targetYaw_dot = (Vector2f){0, 0};
    *targetYaw_ddot = (Vector2f){0, 0};
}

// landing trajectory
uint8_t ACRL_trajectory_land(float currentTime,
                          Vector3f currentPosition,
                          Vector3f currentVelocity,
                          float currentYaw,
                          float decRate,
                          Vector3f *targetPos,
                          Vector3f *targetVel,
                          Vector3f *targetAcc,
                          Vector3f *targetJerk,
                          Vector3f *targetSnap,
                          Vector2f *targetYaw,
                          Vector2f *targetYaw_dot,
                          Vector2f *targetYaw_ddot)
{
    // The function ACRL_trajectory_land function executes the landing procedure
    // The landing procedure first hault the quadrotor into hover for 2 seconds
    // Then the quadrotor will follow a descending trajectory to complete the descending in 3 seconds

    static Vector3f enterPosition;    // position when landing/hover triggered
    static Vector3f enterVelocity;    // velocity when landing/hover triggered
    static float enterYaw;            // yaw angle when landing/hover triggered
    static float enterLinVelInv;      // inverse of linear velocity when landing triggered
    static u_int8_t decComplete = 0;  // indicator for whether deceleration is complete
    static float switchSpeed = 0.8;   // switch to hover when less than this speed
    static float decTime = 0;         // time for deceleration. 3 times of deceleration at most
    const float hoverTime = 3;        // time for hover before landing in seconds
    const float landTime = 3;         // time for landing in seconds
    float linearVelocity = 0;         // calculate the linear velocity based on currentVelocity
    linearVelocity = sqrtf(currentVelocity[0] * currentVelocity[0] + currentVelocity[1] * currentVelocity[1] + currentVelocity[2] * currentVelocity[2]);
    
    if (currentTime < 0.0025)
    {
        enterPosition = currentPosition;
        enterVelocity = currentVelocity;
        enterYaw = currentYaw;
        enterLinVelInv = 1 / linearVelocity;
        decTime = (linearVelocity - switchSpeed) / decRate; // switch to hover at 1m/s

        gcs().send_text(MAV_SEVERITY_INFO, "current position = (%.3f, %.3f, %.3f), current velocity = %.3f", enterPosition[0], enterPosition[1], enterPosition[2], linearVelocity);
    }
    if ((linearVelocity > switchSpeed) && (!decComplete)) // decelerating
    {   
        if (currentTime < 0.0025)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "deceleration rate = %.3f, decelerate in %.3f seconds", decRate, decTime);  
        }
        float decTimeInv = 1 / decTime; 
        *targetVel = (Vector3f){enterVelocity[0] * (1 - currentTime * decTimeInv) + switchSpeed * enterVelocity[0] * enterLinVelInv * currentTime * decTimeInv, enterVelocity[1] * (1 - currentTime * decTimeInv) + switchSpeed * enterVelocity[1] * enterLinVelInv * currentTime * decTimeInv, 0};
        *targetPos = (Vector3f){enterPosition[0] + (enterVelocity[0] + (targetVel->x)) * currentTime / 2, enterPosition[1] + (enterVelocity[1] + (targetVel->y)) * currentTime / 2, enterPosition[2]};

        *targetAcc = (Vector3f){-powf(-1, signbit(enterVelocity[0])) * decTimeInv * (1 - switchSpeed * enterLinVelInv), -powf(-1, signbit(enterVelocity[1])) * decTimeInv * (1 - switchSpeed * enterLinVelInv), 0};
        *targetJerk = (Vector3f){0,0,0};
        *targetSnap = (Vector3f){0,0,0};
    
        *targetYaw = (Vector2f){cosf(enterYaw), sinf(enterYaw)};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};
        
        return (uint8_t) 0; // landing is not completed, return 0
    }
    else // hovering and landing
    {
        if (!decComplete)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "Landing mode phase 0: Deceleration complete");
            decComplete = 1;
            enterPosition = currentPosition;
            enterVelocity = currentVelocity;
            enterYaw = currentYaw;
            decTime = currentTime;
        }
        if (currentTime <= decTime + hoverTime) // hover the quadrotor at the current position
        {   
            if (currentTime <= decTime + 0.005)
            {
                gcs().send_text(MAV_SEVERITY_INFO, "Landing mode phase 1: Hovering for %f seconds", hoverTime);
            }
            *targetPos = enterPosition; // currentPosition works for SITL, but cannot provide stable behavior in real cases
            *targetVel = (Vector3f){0,0,0};
            *targetAcc = (Vector3f){0,0,0};
            *targetJerk = (Vector3f){0,0,0};
            *targetSnap = (Vector3f){0,0,0};
        
            *targetYaw = (Vector2f){cosf(enterYaw), sinf(enterYaw)};
            *targetYaw_dot = (Vector2f){0, 0};
            *targetYaw_ddot = (Vector2f){0, 0};

            return (uint8_t) 0; // landing is not completed, return 0
        }
        else if ((currentTime > decTime + hoverTime) && (currentTime <= decTime + hoverTime + landTime)) // execute the land trajectory
        {   
            printf("currentTime is %f\n.",currentTime);
            if (currentTime <= decTime + hoverTime + 0.005)
            {
                gcs().send_text(MAV_SEVERITY_INFO, "Landing mode phase 2: Descending for %f seconds", landTime);
            }

            float polyCoef[8] = {-0.00122109375, 0.017090625, -0.08203125, 0.13671875,0,0,0,0};
            *targetPos = (Vector3f) {enterPosition[0],enterPosition[1],enterPosition[2] - enterPosition[2] * polyEval(polyCoef, currentTime - decTime - hoverTime, 8)};
            *targetVel = (Vector3f){0,0,-enterPosition[2] * polyDiffEval(polyCoef, currentTime - decTime - hoverTime, 8)};
            *targetAcc = (Vector3f){0,0,-enterPosition[2] * polyDiff2Eval(polyCoef, currentTime - decTime - hoverTime, 8)};
            *targetJerk = (Vector3f){0,0,-enterPosition[2] * polyDiff3Eval(polyCoef, currentTime - decTime - hoverTime, 8)};
            *targetSnap = (Vector3f){0,0,-enterPosition[2] * polyDiff4Eval(polyCoef, currentTime - decTime - hoverTime, 8)};
        
            *targetYaw = (Vector2f){cosf(enterYaw), sinf(enterYaw)};
            *targetYaw_dot = (Vector2f){0, 0};
            *targetYaw_ddot = (Vector2f){0, 0};

            return (uint8_t) 0; // landing is not completed, return 0
        }
        else // landing is completed
        {   
            if (currentTime < decTime + hoverTime + landTime + 0.005)
            {
                gcs().send_text(MAV_SEVERITY_INFO, "Landing completed. Please disarm by hand.");
            }
            
            // load dummy trajectory (to be used by the controllers)
            *targetPos = (Vector3f) {enterPosition[0],enterPosition[1],0};
            *targetVel = (Vector3f){0,0,0};
            *targetAcc = (Vector3f){0,0,0};
            *targetJerk = (Vector3f){0,0,0};
            *targetSnap = (Vector3f){0,0,0};
        
            *targetYaw = (Vector2f){cosf(enterYaw), sinf(enterYaw)};
            *targetYaw_dot = (Vector2f){0, 0};
            *targetYaw_ddot = (Vector2f){0, 0};

            return (uint8_t) 1; // landing is completed, return 1
        }
    }    
}

float polyEval(float polyCoef[], float x, int N)
{
    // Evaluate the polynomial given by polyCoef at variable x.
    // The order of polynomial is N.
    float result = 0;
    for (int i = 0; i < N; i++)
    {
        result += polyCoef[i] * powf(x, N - 1 - i);
    }
    return result;
}

float polyDiffEval(float polyCoef[], float x, int N)
{
    // Evaluate the polynomial's derivative given by polyCoef at variable x.
    // The order of polynomial is N.
    float result = 0;
    for (int i = 0; i < N - 1; i++)
    {
        result += polyCoef[i] * powf(x, N - 2 - i) * (float)(N - 1 - i);
    }
    return result;
}

float polyDiff2Eval(float polyCoef[], float x, int N)
{
    // Evaluate the polynomial's 2nd derivative given by polyCoef at variable x.
    // The order of polynomial is N.
    float result = 0;
    for (int i = 0; i < N - 2; i++)
    {
        result += polyCoef[i] * powf(x, N - 3 - i) * (float)(N - 1 - i) * (float)(N - 2 - i);
    }
    return result;
}

float polyDiff3Eval(float polyCoef[], float x, int N)
{
    // Evaluate the polynomial's 3rd derivative given by polyCoef at variable x.
    // The order of polynomial is N.
    float result = 0;
    for (int i = 0; i < N - 3; i++)
    {
        result += polyCoef[i] * powf(x, N - 4 - i) * (float)(N - 1 - i) * (float)(N - 2 - i) * (float)(N - 3 - i);
    }
    return result;
}

float polyDiff4Eval(float polyCoef[], float x, int N)
{
    // Evaluate the polynomial's 4th derivative given by polyCoef at variable x.
    // The order of polynomial is N.
    float result = 0;
    for (int i = 0; i < N - 4; i++)
    {
        result += polyCoef[i] * powf(x, N - 5 - i) * (float)(N - 1 - i) * (float)(N - 2 - i) * (float)(N - 3 - i) * (float)(N - 4 - i);
    }
    return result;
}
