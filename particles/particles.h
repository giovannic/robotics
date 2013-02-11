#ifndef PARTICLES_H
#define PARTICLES_H

void drawPosition(float x, float y);
void drawMap();
void drawParticles();
void initParticleArrays();
void updateParticleArraysForward(float distanceMoved);
void updateParticleArraysRotate(float theta, float degTurned);
float encoderToCm(float encoder);
task main();

#endif
