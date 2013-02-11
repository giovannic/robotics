// Some suitable data structures for particle filter
// Andrew Davison 2008-2012

#include "squareDrawing.c"
#include "sample.h"
//#include "particles.h"

const int NUMBER_OF_PARTICLES = 100;
const int NUMBER_OF_WALLS = 8;

const float ENC_P_CM = 21.157;
const int OFFSET = 10;

float e,f = 0;

// Arrays for storing information about particles
float xArray[NUMBER_OF_PARTICLES];
float yArray[NUMBER_OF_PARTICLES];
float thetaArray[NUMBER_OF_PARTICLES];
//float weightArray[NUMBER_OF_PARTICLES];
float weight = 1/NUMBER_OF_PARTICLES;
float cumulativeWeightArray[NUMBER_OF_PARTICLES];


// Definitions of walls
// a: O to A
// b: A to B
// c: C to D
// d: D to E
// e: E to F
// f: F to G
// g: G to H
// h: H to O
//                                      a    b    c    d    e    f    g    h
float wallAxArray[NUMBER_OF_WALLS] = {  0,   0,  84,  84, 168, 168, 210, 210};
float wallAyArray[NUMBER_OF_WALLS] = {  0, 168, 126, 210, 210,  84,  84,   0};
float wallBxArray[NUMBER_OF_WALLS] = {  0,  84,  84, 168, 168, 210, 210,   0};
float wallByArray[NUMBER_OF_WALLS] = {168, 168, 210, 210,  84,  84,   0,   0};


// Number of cm per pixel in display
const float DISPLAY_SCALE = 1.0;

void drawMap()
{
	// Display the map
	for (int j = 0; j < NUMBER_OF_WALLS; j++)
	{
		nxtDrawLine((int)(wallAxArray[j]/DISPLAY_SCALE), (int)(wallAyArray[j]/DISPLAY_SCALE),
			(int)(wallBxArray[j]/DISPLAY_SCALE), (int)(wallByArray[j]/DISPLAY_SCALE));
	}
}

void drawPosition(float x, float y)
{
	// Draw new position
	nxtSetPixel((int) (encoderToCm(x)/DISPLAY_SCALE) + OFFSET, (int) (encoderToCm(y)/DISPLAY_SCALE) + OFFSET);
}

void drawParticles()
{
	// Draw the particle set
	for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
	{
		drawPosition(xArray[i], yArray[i]);
	}

}

float encoderToCm(float encoder)
{
	return encoder/ENC_P_CM;
}

void initParticleArrays()
{
	for (int particle = 0; particle < NUMBER_OF_PARTICLES; particle++)
	{
		xArray[particle] = 0;
		yArray[particle] = 0;
		thetaArray[particle] = 0;
	}
}

void updateParticleArraysForward(float distanceMoved)
{
	for (int particle = 0; particle < NUMBER_OF_PARTICLES; particle++)
	{
		//float uniform_float = sampleUniform(1.0);
		e = sampleGaussian(0.0, 0.005);
		f = sampleGaussian(0.0, 0.005);
		xArray[particle] = xArray[particle] + (distanceMoved + e)*cos(thetaArray[particle]);
		yArray[particle] = yArray[particle] + (distanceMoved + e)*sin(thetaArray[particle]);
		thetaArray[particle] = thetaArray[particle] + f;
	}
}

void updateParticleArraysRotate(float theta, float degTurned)
{
	for (int particle = 0; particle < NUMBER_OF_PARTICLES; particle++)
	{
		//float uniform_float = sampleUniform(1.0);
		float g = sampleGaussian(0.0, 0.005);
		thetaArray[particle] = thetaArray[particle] + (degTurned + g);
	}
}

task main()
{
	eraseDisplay();

	initParticleArrays();

	//drawMap();

	//drawParticles();

	square();

	wait1Msec(5000);

}
