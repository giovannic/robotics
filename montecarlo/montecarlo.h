#ifndef MONTECARLO_H
#define MONTECARLO_H

#include "sample.h";

void initialise();
void navigateToWaypoint(float xtarget, float ytarget);

float calcDistance(float xtarget, float ytarget);
float calcAngle(float xtarget, float ytarget);
bool atTarget(float target, float currentLocation);


// **** monte carlo methods ****
void monteCarlo();
void measurementUpdate(float sonar);

// likelihood functions
float scaleForAngle(float sample, float angle);
float getGaussianValue(float m, float z);
bool between(float mid, float sta, float fin);
int getClosestWallForward(float xValue, float yValue, float thetaValue);
int getClosestWallForwardDistance(float x, float y, float theta, int wall);
float angleToWall(float theta, int wall);
float calculateLikelihood(float x, float y, float theta, float z);
// **************

void normalisation();
void resampling();
int getRandomParticleIndex();
// ******************************



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
