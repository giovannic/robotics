#ifndef PARTICLES_H
#define PARTICLES_H

#include "sample.h"

//float pow(float a, int b);
void drawMap();
void drawPosition(float x, float y);
void drawParticles();
float encoderToCm(float encoder);
void initParticleArrays();
void updateParticleArraysForward(float distanceMoved);
void updateParticleArraysRotate(float degTurned);
float findAverageX();
float findAverageY();
float findAverageTheta();
float calculate_likelihood(float x, float y, float theta, float z);
float sample_gaussian(float mean, float x, float k, float sigma);
int between(float middle, float start, float finish);
void normalise();
void resample();
void copyParticleData(int from, int to);
void copyParticle(int from, int to);
void navigateToWaypoint (float new_x, float new_y);
void moveForward(float d);
void turnNDegrees(float a);

task main();

#endif
