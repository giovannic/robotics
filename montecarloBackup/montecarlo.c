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
        xArray[i] = 84;
        yArray[i] = 30;
        thetaArray[i] = 0;
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
    //while ((xtarget != x) || (ytarget != y))
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

        nxtDisplayCenteredTextLine(3, "DD: %f", driveDistance);

        float temp1 = x;
	      float temp2 = y;
        nxtDisplayCenteredTextLine(1, "X: %f", temp1);
	      nxtDisplayCenteredTextLine(2, "Y: %f", temp2);
	      //nxtDisplayCenteredTextLine(3, "Angle: %f", angle);
	      //wait1Msec(5000);

        // co-ordinate system is different so cos and sin are swapped - take from current location not origin
        float tempWaypointX = x + (driveDistance * cos(angle));
        float tempWaypointY = y + (driveDistance * sin(angle));

        nxtDisplayCenteredTextLine(5, "tempX: %f", tempWaypointX);
	      nxtDisplayCenteredTextLine(6, "tempY: %f", tempWaypointY);

        driveToWaypoint(tempWaypointX, tempWaypointY);
        //monteCarlo();

        //float temp1 = x;
	      //float temp2 = y;

        nxtDisplayCenteredTextLine(1, "X: %f", temp1);
	      nxtDisplayCenteredTextLine(2, "Y: %f", temp2);
	      nxtDisplayCenteredTextLine(3, "Theta: %f", theta);
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
  measurementUpdate();
  normalisation();
  resampling();
}

void measurementUpdate()
{
    float sonar = SensorValue(sonar_sensor) - 8; // allow for distance between sonar and centre of wheelbase
    for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
    {
        int particlesX = xArray[i];
        int particlesY = yArray[i];
        int particlesTheta = thetaArray[i];
        weightArray[i] = weightArray[i] * calculateLiklihood(particlesX, particlesY, particlesTheta, sonar);
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
	float sigma = 1;
	float constant = 0.1;

	float numerator = - ((z-m) * (z-m));
	float denominator = 2 * (sigma * sigma);
	float power = numerator/denominator;

	return (exp(power) + constant);
}

int between(float middle, float start, float finish)
{
	float mid = middle + 0.5;
	float sta = start + 0.5;
	float fin = finish + 0.5;

	return ( mid >= sta && mid <= fin) || (mid >= fin && mid <= sta);
}

int getClosestWallForward(float x, float y, float theta)
{
	float closestDistance = -1;
	int closestWall = 0;

	for(int i = 0 ; i < NUMBER_OF_WALLS ; i++)
	{
		float ax = wallAxArray[i];
		float bx = wallBxArray[i];
		float ay = wallAyArray[i];
		float by = wallByArray[i];

		float dx = bx - ax;
		float dy = by - ay;

		float numerator = dy*(ax-x) - dx*(ay-y);
		float denominator = dy*cos(theta) - dx*sin(theta);
		float distance = numerator/denominator;

		float interX = (x + distance*sin(theta));   // CHECK THIS!!!!!!!!!!!
		float interY = (y + distance*cos(theta));

		int collide = between(interX, ax, bx) && between(interY, ay, by);

		if(distance >= 0 && (closestDistance == -1 || distance < closestDistance) && collide )
		{
			closestDistance = distance;
			closestWall = i;
		}
	}

	return closestWall;
}

int getClosestWallForwardDistance(float x, float y, float theta, int wall)
{

	float ax = wallAxArray[wall];
	float bx = wallBxArray[wall];
	float ay = wallAyArray[wall];
	float by = wallByArray[wall];

	float dx = bx - ax;
	float dy = by - ay;

	float numerator = dy*(ax-x) - dx*(ay-y);
	float denominator = dy*cos(theta) - dx*sin(theta);
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

	float numerator = dy*cos(theta) + dx*sin(theta);
	float denominator = sqrt((dy*dy) + (dx*dx));
	float fraction = numerator/denominator;

	return  abs(acos(fraction)); //check
}

float calculateLiklihood(float x, float y, float theta, float z)
{
  int wall = getClosestWallForward(x,y,theta);
	float expectedDepth = getClosestWallForwardDistance(x,y,theta,wall);
	float sample = getGaussianValue(expectedDepth,z);
	float angle = angleToWall(theta,wall);
	float result = scaleForAngle(sample,angle);
	return result;
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
		weightArray[j] = weightArray[j] / weightTotal;
	}
}

void resampling()
{
  float newX[NUMBER_OF_PARTICLES];
	float newY[NUMBER_OF_PARTICLES];
	float newTheta[NUMBER_OF_PARTICLES];

	for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
	{
	    int newIndex = getRandomParticleIndex();
	    newX[i] = xArray[newIndex];
	    newY[i] = yArray[newIndex];
	    newTheta[i] = thetaArray[newIndex];

	}

	xArray = newX;
	yArray = newY;
	thetaArray = newTheta;

	for (int j = 0; j < NUMBER_OF_PARTICLES; ++j )
	{
	    weightArray[j] = 1.0/ (float)NUMBER_OF_PARTICLES;
	}

	//Update values after resampling!!
	x = findAverageX();
	y = findAverageY();
	theta = findAverageTheta();

	float temp1 = x;
	float temp2 = y;

  nxtDisplayCenteredTextLine(1, "X: %f", temp1);
	nxtDisplayCenteredTextLine(2, "Y: %f", temp2);
	nxtDisplayCenteredTextLine(3, "Theta: %f", theta);

	wait1Msec(3000);
}

int getRandomParticleIndex()
{
    float randomFloat = 0;
    float cumulativeWeight = 0;
    int currentIndex = 0;
    int negposRng = 0;

    while (randomFloat == 0)
    {
      negposRng = rand();//%65536;
	    randomFloat = (float)(negposRng) / 65536.0;    // CHECK THIS
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


		//e = sampleGaussian(0.0, 0.005);
	  //e = sampleGaussian(0.0, 1.5);
		//f = sampleGaussian(0.0, 0.008);
	  //f = sampleGaussian(0, 0.1); //Biased mean on purpose as one motor/wheel stronger than the other

	  //nxtDisplayCenteredTextLine(1, "MG: %f", e);
	  //nxtDisplayCenteredTextLine(2, "RG: %f", f);

	  //wait1Msec(500);

		xArray[particle] = xArray[particle] + (distanceMoved + e)*sin(thetaArray[particle]);
		yArray[particle] = yArray[particle] + (distanceMoved + e)*cos(thetaArray[particle]);
		thetaArray[particle] = thetaArray[particle] + f;
	}
}

void updateParticleArraysRotate(float degTurned)
{
  float total = 0;
	for (int particle = 0; particle < NUMBER_OF_PARTICLES; particle++)
	{
    //float g = 0;
		float g = sampleGaussian(0.0, 0.005); //UPDATE THIS.
	  //float g = sampleGaussian(0.0, 0.5);
	  //float g =sampleGaussian(0.0, 0.5);

	  total += g;

		thetaArray[particle] = thetaArray[particle] + (degTurned + g);
	}
	nxtDisplayCenteredTextLine(7, "G: %f", total/(float)NUMBER_OF_PARTICLES);
	wait1Msec(500);
}

float findAverageX()
{
  float total = 0;
  int i = 0;
   for ( i = 0; i < NUMBER_OF_PARTICLES; ++i)
   {
     //total = total + (xArray[i] * weightArray[i]);
     total += xArray[i];
   }
   return total/((float)NUMBER_OF_PARTICLES); //Fixed to allow for weights...
}

float findAverageY()
{
  float total = 0;
  int i = 0;
   for ( i = 0; i < NUMBER_OF_PARTICLES; ++i)
   {
          //total = total + (yArray[i] * weightArray[i]);
      total+=yArray[i];
   }
   return total/((float)NUMBER_OF_PARTICLES); //Fixed to allow for weights...
}

float findAverageTheta()
{
  float total = 0;
  int i = 0;
   for ( i = 0; i < NUMBER_OF_PARTICLES; ++i)
   {
     //total = total + (thetaArray[i] * weightArray[i]);
     total += thetaArray[i];
   }
   return total/((float)NUMBER_OF_PARTICLES); //Fixed to allow for weights...

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
  //float lastMotorValue = 0;
  //float distanceMoved = 0;  //Why are these here?...
  //float currentDistance = 0;





   // ATTEMPTS TO CORRECT SLAVE AND MASTER
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
  wait1Msec(1000);
  updateParticleArraysRotate(a);
  theta = findAverageTheta();
}



/*********************************************************************
**********************************************************************/

task main ()
{
    initialise();

    //navigateToWaypoint (84, 30);
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
