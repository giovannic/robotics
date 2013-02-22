#pragma config(Sensor, S1,     sonar_sensor,        sensorSONAR)
#pragma config(Motor,  motorA,          Right_motor,   tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          Left_motor,    tmotorNormal, PIDControl, encoder)

#include "montecarlo.h"

const int NUMBER_OF_PARTICLES = 100;

int x = 0;
int y = 0;
float theta = 0;
float motorPower = 30;

const int NUMBER_OF_WALLS = 8;

const float ENC_P_CM = 21.157;
//const int OFFSET = 10;

const float stepDistance = 20; //cm

float xArray[NUMBER_OF_PARTICLES];
float yArray[NUMBER_OF_PARTICLES];
float thetaArray[NUMBER_OF_PARTICLES];
float weightArray[NUMBER_OF_PARTICLES];

//                                      a    b    c    d    e    f    g    h
float wallAxArray[NUMBER_OF_WALLS] = {  0,   0,  84,  84, 168, 168, 210, 210};
float wallAyArray[NUMBER_OF_WALLS] = {  0, 168, 126, 210, 210,  84,  84,   0};
float wallBxArray[NUMBER_OF_WALLS] = {  0,  84,  84, 168, 168, 210, 210,   0};
float wallByArray[NUMBER_OF_WALLS] = {168, 168, 210, 210,  84,  84,   0,   0};

void initialise()
{
    for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
    {
        xArray[i] = 0;
        yArray[i] = 0;
        thetaArray[i] = 0;
        weightArray[i] = 1.0/((float) NUMBER_OF_PARTICLES);
    }
}


void navigateToWaypoint(float xtarget, float ytarget)
{
    while ((xtarget != x) || (ytarget != y))
    {
        float distance = calcDistance(xtarget, ytarget);
        float angle = calcAngle(xtarget, ytarget);

        float driveDistance = 0; //initialise
        if (distance < stepDistance)
        {
            driveDistance = distance;
        }
        else
        {
            driveDistance = stepDistance;
        }

        // co-ordinate system is different so cos and sin are swapped - take from current location not origin
        float tempWaypointX = x + (driveDistance * cos(angle));
        float tempWaypointY = y + (driveDistance * sin(angle));

        driveToWaypoint(tempWaypointX, tempWaypointY);
        //monte carlo here *********
        wait1Msec(500);

    }
}


float calcDistance(float xtarget, float ytarget)
{
    float dx = x - xtarget;
    float dy = y - ytarget;

    return sqrt((dx*dx) + (dy*dy));
}

float calcAngle(float xtarget, float ytarget)
{
    float dy = xtarget - x;
	  float dx = ytarget - y;
    float targetAngle = 0;
	  if (dy != 0)
	  {
		  targetAngle = atan( dx / dy );
	  } else {
		  targetAngle = PI/2;

	  }
	  //SW quadrant
	  if(dy < 0 && dx >= 0)
	  {
		  targetAngle += PI;
	  }
	  //NW
		if (dy <= 0 && dx < 0)
		{
			targetAngle -= PI;
		}
		if(dy > 0 && dx < 0)
		{
			targetAngle = -atan( abs(dx) / dy );
		}

		//float newAngle = targetAngle - theta;

		return targetAngle;
}



/***********************************************************************
***********************************************************************
*********************** Locomotion Methods *****************************/

void updateParticleArraysForward(float distanceMoved)
{
  float e = 0;
  float f = 0;

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
void driveToWaypoint (float new_x, float new_y)
{
	float dx = new_x - x;
	float dy = new_y - y;
	float targetAngle = 0;
	if (dy != 0)
	{
		targetAngle = atan( dx / dy );
	} else {
		targetAngle = PI/2;

	}
	//SW quadrant
	if(dy < 0 && dx >= 0)
	{
		targetAngle += PI;
	}
	//NW
	if (dy <= 0 && dx < 0)
	{
		targetAngle -= PI;
	}
	if(dy > 0 && dx < 0)
	{
		targetAngle = -atan( abs(dx) / dy );
	}

	float newAngle = targetAngle - theta;

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
}



/*********************************************************************
**********************************************************************/

task main ()
{
    initialise();

    navigateToWaypoint (84, 30);
    navigateToWaypoint (180, 30);
	  navigateToWaypoint (180, 54);
	  navigateToWaypoint (126, 54);
	  navigateToWaypoint (126, 168);
	  navigateToWaypoint (126, 126);
		navigateToWaypoint (30, 54);
		navigateToWaypoint (84, 54);
		navigateToWaypoint (84, 30);
    //driveToWaypoint(9.14,3.36);
    //driveToWaypoint(84,30);
}
