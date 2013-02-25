#pragma config(Sensor, S1,     sonar_sensor,        sensorSONAR)
#pragma config(Motor,  motorA,          Right_motor,   tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          Left_motor,    tmotorNormal, PIDControl, encoder)

#include "montecarlo.h"

const int NUMBER_OF_PARTICLES = 100;

float x = 0;
float y = 0;
float theta = 0;
float motorPower = 30;
float bound = 3;
bool leftMotorDrive = false;

const int NUMBER_OF_WALLS = 8;
const float ENC_P_CM = 18;

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
        xArray[i] = 84;
        yArray[i] = 30;
        thetaArray[i] = PI/2;
        weightArray[i] = 1.0/((float) NUMBER_OF_PARTICLES);
    }
    x = findAverageX();
    y = findAverageY();
    theta = findAverageTheta();
}

bool atTarget(float target, float currentLocation)
{
    return ((target - bound <= currentLocation) && (currentLocation <= bound + target));
}


void navigateToWaypoint(float xtarget, float ytarget)
{
    while(!atTarget(xtarget, x) || (!atTarget(ytarget, y)))
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
        monteCarlo();

        eraseDisplay();
        drawMap();
        drawParticles();
        wait1Msec(100);

    }

    PlayTone(784, 15);
}


/***********************************************************************
***********************************************************************
*********************** Calculate Distance and Angle *******************/

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

		return targetAngle;
}


/***********************************************************************
***********************************************************************
*********************** Monte Carlo Methods ***************************/


void monteCarlo()
{
  float sonar = SensorValue(sonar_sensor) - 6; // allow for distance between sonar and centre of wheelbase
  //nxtDisplayCenteredTextLine(1, "sonar: %f", (float)sonar);
  if (sonar < 10 || sonar > 150) {return;}

  measurementUpdate(sonar);
  normalisation();
  resampling();
}

void measurementUpdate(float sonar)
{

    for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
    {
        float particlesX = xArray[i];
        float particlesY = yArray[i];
        float particlesTheta = thetaArray[i];
        weightArray[i] *= calculateLikelihood(particlesX, particlesY, particlesTheta, sonar);
    }
}

/***********************************************************************
************************ Likelihood Calculating *************************/

float scaleForAngle(float sample, float angle)
{
	if (angle == 0)
	{
		angle = 1;
	}

	return (sample * 1.0/angle);
}

float getGaussianValue(float m, float z)
{
	float sigma = z/100; //growing
	float constant = 0.001;

	float numerator = - ((z-m) * (z-m));
	float denominator = 2 * (sigma * sigma);
	float power = numerator/denominator;

	return (exp(power) + constant);
}

bool between(float middle, float start, float finish)
{

	int mid = middle + 0.5;
	int sta = start + 0.5;
	int fin = finish + 0.5;

	return ( mid >= sta && mid <= fin) || (mid >= fin && mid <= sta);
}

int getClosestWallForward(float xValue, float yValue, float thetaValue)
{
	float closestDistance = -1;
	int closestWall = -1;

	for(int i = 0 ; i < NUMBER_OF_WALLS ; ++i)
	{
		float ax = wallAxArray[i];
		float bx = wallBxArray[i];
		float ay = wallAyArray[i];
		float by = wallByArray[i];

		float dx = bx - ax;
		float dy = by - ay;

		float numerator = dy*(ax-xValue) - dx*(ay-yValue);
		float denominator = dy*sin(thetaValue) - dx*cos(thetaValue);
		float distance = numerator/denominator;

		float interX = (xValue + distance*sin(thetaValue));
		float interY = (yValue + distance*cos(thetaValue));



		bool collide;

    if (abs(interX) > 10000 || abs(interY) > 10000)
    {
      collide = false;
    }
    else
    {
      collide = between(interX, ax, bx) && between(interY, ay, by);
    }


		if(distance >= 0 && (closestDistance == -1 || distance < closestDistance) && collide )
		{
			closestDistance = distance;
			closestWall = i;
		}
	}

	return closestWall;
}

int getClosestWallForwardDistance(float xValue, float yValue, float thetaValue, int wall)
{

	float ax = wallAxArray[wall];
	float bx = wallBxArray[wall];
	float ay = wallAyArray[wall];
	float by = wallByArray[wall];

	float dx = bx - ax;
	float dy = by - ay;

		float numerator = dy*(ax-xValue) - dx*(ay-yValue);
		float denominator = dy*sin(thetaValue) - dx*cos(thetaValue); //swapped
		float distance = numerator/denominator;

	return distance;
}

// Calculate the angle to the wall.
float angleToWall(float theta, int wall)
{
	float ax = wallAxArray[wall];
	float bx = wallBxArray[wall];
	float ay = wallAyArray[wall];
	float by = wallByArray[wall];

	float dx = bx - ax;
	float dy = ay - by;

	float numerator = dy*sin(theta) + dx*cos(theta);
	float denominator = sqrt((dy*dy) + (dx*dx));
	float fraction = numerator/denominator;

	return  abs(asin(fraction));
}

float calculateLikelihood(float x, float y, float theta, float z)
{
  int wall = getClosestWallForward(x,y,theta);
  if(wall == -1)
  {
    return 0; //Particle out the map - assign it 0 weight.
  }

	float expectedDepth = getClosestWallForwardDistance(x,y,theta,wall);

	float sample = getGaussianValue(expectedDepth,z);
	return sample;
}



/************************************************************************/

void normalisation()
{
  float weightTotal = 0;
	for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
	{
		weightTotal += weightArray[i];
	}

	for (int j = 0; j < NUMBER_OF_PARTICLES; ++j)
	{
		weightArray[j] /= weightTotal;
	}
}

void resampling()
{
  float newX[NUMBER_OF_PARTICLES];
	float newY[NUMBER_OF_PARTICLES];
	float newTheta[NUMBER_OF_PARTICLES];
	float newWeight[NUMBER_OF_PARTICLES];


	for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
	{
	    int newIndex = getRandomParticleIndex();
	    newX[i] = xArray[newIndex];
	    newY[i] = yArray[newIndex];
	    newTheta[i] = thetaArray[newIndex];
	    newWeight[i] = weightArray[newIndex];
	}

	xArray = newX;
	yArray = newY;
	thetaArray = newTheta;
	weightArray = newWeight;

	x = findAverageX();
	y = findAverageY();
	theta = findAverageTheta();
}

int getRandomParticleIndex()
{
    float randomFloat = 0;
    float cumulativeWeight = 0;
    int currentIndex = 0;

    while (randomFloat == 0)
    {
	    randomFloat = sampleUniform();
	  }

	  while (cumulativeWeight < randomFloat)
	  {
	      cumulativeWeight = cumulativeWeight + weightArray[currentIndex];
	      ++currentIndex;
	  }

	  return currentIndex-1;
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


		e = sampleGaussian(0.0, 0.5); // was 1
		f = sampleGaussian(0.0, 0.08);

		xArray[particle] = xArray[particle] + (distanceMoved + e)*sin(thetaArray[particle]);
		yArray[particle] = yArray[particle] + (distanceMoved + e)*cos(thetaArray[particle]);
		thetaArray[particle] += f;
	}
}

void updateParticleArraysRotate(float degTurned)
{
	for (int particle = 0; particle < NUMBER_OF_PARTICLES; particle++)
	{
	  float g = sampleGaussian(0.0, 0.1);

		thetaArray[particle] += (degTurned + g);
	}
}

float findAverageX()
{
  float total = 0;
  int i = 0;
   for ( i = 0; i < NUMBER_OF_PARTICLES; ++i)
   {
     total = total + (xArray[i] * weightArray[i]);
   }
   return total;
}

float findAverageY()
{
  float total = 0;
  int i = 0;
   for ( i = 0; i < NUMBER_OF_PARTICLES; ++i)
   {
          total = total + (yArray[i] * weightArray[i]);
   }
   return total;
}

float findAverageTheta()
{
  float total = 0;
  int i = 0;
   for ( i = 0; i < NUMBER_OF_PARTICLES; ++i)
   {
     total = total + (thetaArray[i] * weightArray[i]);
   }
   return total;

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

	//wait1Msec(1000);
	turnNDegrees(newAngle);

	moveForward(sqrt((dx*dx) + (dy*dy)));
}

// Functions that draw on the screen
//d in cm
void moveForward(float d)
{
  float lineStart;
   nSyncedTurnRatio = 100;

  float  encoderLimit = ENC_P_CM*d;

  // switch the slave and master
  if (leftMotorDrive)
  {
     nSyncedMotors = synchBA;
     lineStart = nMotorEncoder[motorB];
     motor[motorB] = motorPower;
      while((nMotorEncoder[motorB] - lineStart) < encoderLimit)
      {
      }
  }
  else
  {
     nSyncedMotors = synchAB;
     lineStart = nMotorEncoder[motorA];
     motor[motorA] = motorPower;
      while((nMotorEncoder[motorA] - lineStart) < encoderLimit)
      {
      }
  }

  leftMotorDrive = !leftMotorDrive;

  updateParticleArraysForward(d);

  theta = findAverageTheta();
  x = findAverageX();
  y = findAverageY();
  motor[motorB] = 0;
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
  updateParticleArraysRotate(a);
  theta = findAverageTheta();
  wait1Msec(1000);
}

/*********************************************************************
**********************************************************************/


/***********************************************************************
***********************************************************************
*********************** Draw Particles ********************************/

// Number of cm per pixel in display
const float DISPLAY_SCALE = 3.5;
const int OFFSET = 3;

void drawMap()
{
	// Display the map
	for (int j = 0; j < NUMBER_OF_WALLS; j++)
	{
		nxtDrawLine((int)(wallAxArray[j]/DISPLAY_SCALE) + OFFSET, (int)(wallAyArray[j]/DISPLAY_SCALE) + OFFSET,
			(int)(wallBxArray[j]/DISPLAY_SCALE) + OFFSET, (int)(wallByArray[j]/DISPLAY_SCALE) + OFFSET);
	}
}

void drawPosition(float x, float y)
{
	// Draw new position
	nxtSetPixel((int) (x/DISPLAY_SCALE) + OFFSET, (int) (y/DISPLAY_SCALE) + OFFSET);
}

void drawParticles()
{
	// Draw the particle set
	for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
	{
		drawPosition(xArray[i], yArray[i]);
	}

}

/*********************************************************************
**********************************************************************/

task main ()
{
    initialise();
    drawMap();


    navigateToWaypoint (180, 30);
	  navigateToWaypoint (180, 54);
	  navigateToWaypoint (126, 54);
	  navigateToWaypoint (126, 168);
	  navigateToWaypoint (126, 126);
		navigateToWaypoint (30, 54);
		navigateToWaypoint (84, 54);
		navigateToWaypoint (84, 30);
}
