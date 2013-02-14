#include "locomotion.h"
#include "particles.h"

int x = 0;
int y = 0;
float theta = 0;

float motorPower = 30;
const float ENC_P_CM = 21.157;

const int NUMBER_OF_PARTICLES = 100;

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
		f = sampleGaussian(0.0, 0.008);
		//xArray[particle] = xArray[particle] + (distanceMoved + e)*cos(thetaArray[particle]);
		//yArray[particle] = yArray[particle] + (distanceMoved + e)*sin(thetaArray[particle]);
		xArray[particle] = xArray[particle] + (distanceMoved + e)*sin(thetaArray[particle]);
		yArray[particle] = yArray[particle] + (distanceMoved + e)*cos(thetaArray[particle]);
		thetaArray[particle] = thetaArray[particle] + f;
	}
}

void updateParticleArraysRotate(float degTurned)
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

  //x = x + d * cos(theta);
  //y = y + d * sin(theta);

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
