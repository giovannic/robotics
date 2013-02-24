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
bool between(float middle, float start, float finish);
int getClosestWallForward(float xValue, float yValue, float thetaValue);
int getClosestWallForwardDistance(float xValue, float yValue, float thetaValue, int wall);
float angleToWall(float phi, int wall);
float calculateLikelihood(float p_x, float p_y, float phi, float z);
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
void rotateWMonte(float newAngle);
// ******************************

// **** drawing methods ****

void drawMap();
void drawPosition(float p_x, float p_y);
void drawParticles();

// ******************************

task main();

#endif
