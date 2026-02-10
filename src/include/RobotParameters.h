#pragma once
double maxSpeed = 10;      // m/s
double maxAccel = 5;       // m/s^2
double wheelRadius = 0.02; // m
double wheelDistance = 0.1;
double maxRotationalSpeed = (2 * maxSpeed) / wheelDistance;
double maxRotationalAccel = (2 * maxAccel) / wheelDistance;
double mu = 1;
double driveGearing = 1;