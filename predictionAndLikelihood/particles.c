#include "particles.h"

const int NUMBER_OF_PARTICLES = 96;
const int NUMBER_OF_WALLS = 8;

const float ENC_P_CM = 21.157;
const int OFFSET = 10;

float e,f = 0;

// Arrays for storing information about particles
float xArray[NUMBER_OF_PARTICLES];
float yArray[NUMBER_OF_PARTICLES];
float thetaArray[NUMBER_OF_PARTICLES];

float xArrayCopy[NUMBER_OF_PARTICLES];
float yArrayCopy[NUMBER_OF_PARTICLES];
float thetaArrayCopy[NUMBER_OF_PARTICLES];

float weightArray[NUMBER_OF_PARTICLES];

//unnecessary
//float cumulativeWeightArray[NUMBER_OF_PARTICLES];


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
  int particles = 0;
  //create particles for 168*168 square
  for(int x = 1; x < 168; x += 20)
    for(int y = 1; y < 168; y+= 22)
    {
      xArray[particles] = x;
      yArray[particles] = y;
      thetaArray[particles++] = random(31415)/10000 - PI/2;
    }

  //create particles in top sqaure
  for(int x = 85; x < 168; x += 25)
   for(int y = 169; y < 210; y += 15)
   {
      xArray[particles] = x;
      yArray[particles] = y;
      thetaArray[particles++] = random(31415)/10000 - PI/2;
   }
  //create particles in right sqaure
  for(int x = 169; x < 210; x += 14)
   for(int y = 1; y < 84; y += 25)
   {
      xArray[particles] = x;
      yArray[particles] = y;
      thetaArray[particles++] = random(31415)/10000 - PI/2;
   }
}
void updateParticleArraysForward(float distanceMoved)
{
	for (int particle = 0; particle < NUMBER_OF_PARTICLES; particle++)
	{
		//float uniform_float = sampleUniform(1.0);
		e = sampleGaussian(0.0, 0.005);
		f = sampleGaussian(0.0, 0.008);
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

float findAverageX()
{
  float total;
  int i = 0;
   for ( i = 0; i < NUMBER_OF_PARTICLES; ++i)
   {
     total += xArray[i];
   }
   return (total / NUMBER_OF_PARTICLES);
}

float findAverageY()
{
  float total;
  int i = 0;
   for ( i = 0; i < NUMBER_OF_PARTICLES; ++i)
   {
     total += yArray[i];
   }
   return (total / NUMBER_OF_PARTICLES);
}

float findAverageTheta()
{
  float total;
  int i = 0;
   for ( i = 0; i < NUMBER_OF_PARTICLES; ++i)
   {
     total += thetaArray[i];
   }
   return (total / NUMBER_OF_PARTICLES);

}

float calculate_likelihood(float x, float y, float theta, float z)
{
  //sentinel minimum and wall values
  float By = 0, Bx = 0, Ax = 0, Ay = 0, m = 0, minimum = -1, wall = -1;
  float dx = 0, dy = 0;
  float numerator = 0, denominator = 0;
  float interX = 0, interY = 0;

   for (int i = 0; i < NUMBER_OF_WALLS; ++i)
   {
      Ax = wallAxArray[i];
      Ay = wallAyArray[i];
      Bx = wallBxArray[i];
      By = wallByArray[i];
	  
	  dx = wallAxArray[i] - wallBxArray[i];
	  dy = wallAyArray[i] - wallByArray[i];
	  
	  //calculate m
	  numerator = dy*(Ax-x) - dx*(Ay-y);
	  denominator = dy*cos(theta) - dx*sin(theta);
      m = numerator/denomitator;
	  
      if ((m > 0) && (minimum > m || minimum == -1))
      {
		//check bound
		//round or truncate?
		interX = (int) (x + m*cos(theta));
		interY = (int) (y + m*sin(theta));
		if (between(interX, Ax, Bx) && between(interY, Ay, By))
		{
			minimum = m;
			wall = i;
		}
      }
	  
	  return sample_gaussian(z, m);
	  
   }
}

float sample_gaussian(float mean, float x)
{
	float k = 1;
	//maybe dependant on distance
	float sigma = 1;
	return exp((-pow((mean - x),2)) / (2*pow(sigma,2))) + k;
}

int between(float middle, float start, float finish)
{
	//round for comparisons
	int mid = (int) middle + 0.5;
	int sta = (int) start + 0.5;
	int fin = (int) finish + 0.5;
	
	//check that middle is between start and finish
	//even if start > finish
	return ( mid >= sta && mid <= fin) || 
		(mid >= fin && mid <= sta);
}

void normalise()
{
	float weightTotal = 0;
	for (int i = 0; i < NUMBER_OF_PARTICLES; ++i) 
	{
		weightTotal += weightArray[i];
	}
	
	for (int j = 0; j < NUMBER_OF_PARTICLES; ++j)
	{
		weightArray[j] = weightArray[j] / weightTotal;
	}
}

void resample()
{
	float randomSelected;
	int currentIndex;
	
	copyParticleData();
	
	for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
	{
		randomSelected = random(1000)/1000;
		currentIndex = 0;
		while (randomSelected > 0) 
		{
			randomSelected -= weightArray[currentIndex];
			++currentIndex;
		}
		--currentIndex;
		copyParticle(currentIndex, i);
	}
}

void copyParticleData(int from, int to)
{
	for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
	{
		xArrayCopy[i] = xArray[i];
		yArrayCopy[i] = yArray[i];
		thetaArrayCopy[i] = thetaArray[i];
	}
	
}

void copyParticle(int from, int to)
{
	xArray[to] = xArrayCopy[from];
	yArray[to] = yArrayCopy[from];
	thetaArray[to] = thetaArrayCopy[from];
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

		e = sampleGaussian(0.0, 0.005);
		f = sampleGaussian(0.0, 0.008);

		xArray[particle] = xArray[particle] + (distanceMoved + e)*sin(thetaArray[particle]);
		yArray[particle] = yArray[particle] + (distanceMoved + e)*cos(thetaArray[particle]);
		thetaArray[particle] = thetaArray[particle] + f;
	}
}

void updateParticleArraysRotate(float degTurned)
{
	for (int particle = 0; particle < NUMBER_OF_PARTICLES; particle++)
	{

		float g = sampleGaussian(0.0, 0.005);
		thetaArray[particle] = thetaArray[particle] + (degTurned + g);
	}
}

float findAverageX()
{
  float total = 0;
  int i = 0;
   for ( i = 0; i < NUMBER_OF_PARTICLES; ++i)
   {
     total += xArray[i];
   }
   return (total / NUMBER_OF_PARTICLES);
}

float findAverageY()
{
  float total = 0;
  int i = 0;
   for ( i = 0; i < NUMBER_OF_PARTICLES; ++i)
   {
     total += yArray[i];
   }
   return (total / NUMBER_OF_PARTICLES);
}

float findAverageTheta()
{
  float total = 0;
  int i = 0;
   for ( i = 0; i < NUMBER_OF_PARTICLES; ++i)
   {
     total += thetaArray[i];
   }
   return (total / NUMBER_OF_PARTICLES);

}

//x and y in meters
void navigateToWaypoint (float new_x, float new_y)
{
	new_x *= 100;
	new_y *= 100;
	float dx = new_x - x;
	float dy = new_y - y;
	nxtDisplayCenteredTextLine(1, "x: %f", dx);
	nxtDisplayCenteredTextLine(2, "y: %f", dy);
	float targetAngle = 0;
	if (dy != 0)
	{
		targetAngle = atan( dx / dy );
	} else {
		targetAngle = PI/2;

	}
	nxtDisplayCenteredTextLine(4, "TA: %f", targetAngle);
	//SW quadrant
	nxtDisplayCenteredTextLine(5, "HERE: %f", (float)0);
	if(dy < 0 && dx >= 0)
	{
		targetAngle += PI;
		nxtDisplayCenteredTextLine(5, "HERE: %f", (float)1);
	}
	//NW
	if (dy <= 0 && dx < 0)
	{
		targetAngle -= PI;
		nxtDisplayCenteredTextLine(5, "HERE: %f", (float)2);
	}
	if(dy > 0 && dx < 0)
	{
		targetAngle = -atan( abs(dx) / dy );
		nxtDisplayCenteredTextLine(5, "HERE: %f", (float)3);
	}

	float newAngle = targetAngle - theta;
	nxtDisplayCenteredTextLine(3, "NA: %f", newAngle);

	wait1Msec(1000);
	turnNDegrees(newAngle);

	moveForward(sqrt((dx*dx) + (dy*dy)));
}

// Functions that draw on the screen
//d in cm
void moveForward(float d)
{
  float lastMotorValue = 0;
  float distanceMoved = 0;
  float currentDistance = 0;
  nSyncedMotors = synchAB;
  nSyncedTurnRatio = 100;
  float lineStart = nMotorEncoder[motorA];
  motor[motorA] = motorPower;
  float  encoderLimit = ENC_P_CM*d;

  while((nMotorEncoder[motorA] - lineStart) < encoderLimit)
  {
  }


  updateParticleArraysForward(d);

  theta = findAverageTheta();
  x = findAverageX();
  y = findAverageY();
  motor[motorA] = 0;  // turn the motors off.

}

void turnNDegrees(float a)
{
  nSyncedMotors = synchAB;
  nSyncedTurnRatio = -100;
  float rotStart = nMotorEncoder[motorA];

  while (a <= -PI){
	a += 2*PI;
  }

  while (a > PI){
	a -= 2*PI;
  }

  if (a < 0){
    motor[motorA] = motorPower;
  }else{
    motor[motorA] = -motorPower;
  }
  float rotLimit = (400/PI) * a;

  while(abs(nMotorEncoder[motorA] - rotStart) < abs(rotLimit) )
  {
  }

  motor[motorA] = 0;
  wait1Msec(1000);
  updateParticleArraysRotate(a);
  theta = findAverageTheta();
  //theta += a;
}

task main (){
	initParticleArrays();
	navigateToWaypoint(0.5,0.5);
	wait1Msec(1000);
	navigateToWaypoint(0.5,-0.2);
	wait1Msec(1000);
	navigateToWaypoint(0,0);
	wait1Msec(1000);
}
