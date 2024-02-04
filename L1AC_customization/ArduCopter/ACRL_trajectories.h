/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
// ACRL_trajectories.h
// Copyright 2021 Sheng Cheng, all rights reserved.

// Dr. Sheng Cheng, Nov. 2021
// Email: chengs@illinois.edu
// Advanced Controls Research Laboratory
// Department of Mechanical Science and Engineering
// University of Illinois Urbana-Champaign
// Urbana, IL 61821, USA



// #include <math.h>
// #include <AP_Math/AP_Math.h>
// #include <AP_Math/Vector3f.h>
// #include <AP_Math/vectorN.h>
// #include <AP_Math/quaternion.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/matrixN.h>
#include <AP_Math/ftype.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include "Copter.h"



void ACRL_trajectory_takeoff(float timeInThisRun,
                             Vector3f *targetPos, 
                             Vector3f *targetVel, 
                             Vector3f *targetAcc, 
                             Vector3f *targetJerk, 
                             Vector3f *targetSnap, 
                             Vector2f *targetYaw, 
                             Vector2f *targetYaw_dot, 
                             Vector2f *targetYaw_ddot);
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
                             Vector2f *targetYaw_ddot);
void ACRL_trajectory_circle_variable_yaw(float timeInThisRun,
                                      float radius,
                                      float InitialTimeOffset,
                                      float targetSpeed,
                                      Vector3f *targetPos,
                                      Vector3f *targetVel,
                                      Vector3f *targetAcc,
                                      Vector3f *targetJerk,
                                      Vector3f *targetSnap,
                                      Vector2f *targetYaw,
                                      Vector2f *targetYaw_dot,
                                      Vector2f *targetYaw_ddot);
void ACRL_trajectory_circle_fixed_yaw(float timeInThisRun,
                                      float radius,
                                      float InitialTimeOffset,
                                      float targetSpeed,
                                      Vector3f *targetPos,
                                      Vector3f *targetVel,
                                      Vector3f *targetAcc,
                                      Vector3f *targetJerk,
                                      Vector3f *targetSnap,
                                      Vector2f *targetYaw,
                                      Vector2f *targetYaw_dot,
                                      Vector2f *targetYaw_ddot);
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
                                       Vector2f *targetYaw_ddot);
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
                                       Vector2f *targetYaw_ddot); 
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
                          Vector2f *targetYaw_ddot);                                

float polyEval(float polyCoef[],float x,int N);
float polyDiffEval(float polyDiffCoef[],float x,int N);
float polyDiff2Eval(float polyDiffDiffCoef[],float x,int N);
float polyDiff3Eval(float polyDiffDiffCoef[],float x,int N);
float polyDiff4Eval(float polyDiffDiffCoef[],float x,int N);
