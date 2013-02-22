#ifndef MONTECARLO_H
#define MONTECARLO_H

#include "sample.h";

void initialise();
void navigateToWaypoint(float xtarget, float ytarget);
float calcDistance(float xtarget, float ytarget);
float calcAngle(float xtarget, float ytarget);

// **** locomotion methods ****
void updateParticleArraysForward(float distanceMoved);
void updateParticleArraysRotate(float degTurned);
float findAverageX();
float findAverageY();
float findAverageTheta();
void driveToWaypoint (float new_x, float new_y);
void moveForward(float d);
void turnNDegrees(float a);
// ******************************

task main();

#endif
