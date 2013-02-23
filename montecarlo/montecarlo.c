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

        // co-ordinate system is different so cos and sin are swapped - take from current location not origin
        float tempWaypointX = x + (driveDistance * cos(angle));
        float tempWaypointY = y + (driveDistance * sin(angle));


        driveToWaypoint(tempWaypointX, tempWaypointY);
        monteCarlo();


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
        weightArray[i] = weightArray[i] * calculateLikelihood(particlesX, particlesY, particlesTheta, sonar);
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

bool between(int mid, int sta, int fin)
{

	//float mid = middle + 0.5;
	//float sta = start + 0.5;
	//float fin = finish + 0.5;

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
		nxtDisplayCenteredTextLine(4, "ax: %f", (float)ax);
		nxtDisplayCenteredTextLine(5, "bx: %f", (float)bx);
		nxtDisplayCenteredTextLine(6, "ay: %f", (float)ay);
		//nxtDisplayCenteredTextLine(7, "by: %f", (float)by);

		float dx = bx - ax;
		float dy = by - ay; //test with absoluute values CHECK THIS!!

		float numerator = dy*(ax-xValue) - dx*(ay-yValue);
		float denominator = dy*sin(thetaValue) - dx*cos(thetaValue); //swapped
		float distance = numerator/denominator;

		float interX = (x + distance*sin(thetaValue));   // CHECK THIS!!!!!!!!!!! swapped   //SWAPPED INTERX AND Y
		float interY = (y + distance*cos(thetaValue));



		bool collide = between(interX, ax, bx) && between(interY, ay, by);

		nxtDisplayCenteredTextLine(1, "interX: %f", (float)interX);
		nxtDisplayCenteredTextLine(2, "interY: %f", (float)interY);
		nxtDisplayCenteredTextLine(3, "wall: %f", (float)i);
		nxtDisplayCenteredTextLine(7, "collide: %f", (float)collide);
		//nxtDisplayCenteredTextLine(5, "distance: %f", (float)distance);
		//nxtDisplayCenteredTextLine(6, "XValue: %f", (float)xValue);
		//nxtDisplayCenteredTextLine(7, "YValue: %f", (float)yValue);
		//nxtDisplayCenteredTextLine(3, "theta: %f", (float)thetaValue);
		wait1Msec(1000);

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

float calculateLikelihood(float x, float y, float theta, float z)
{
  int wall = getClosestWallForward(x,y,theta);
  nxtDisplayCenteredTextLine(1, "Wall: %f", (float)wall);
	float expectedDepth = getClosestWallForwardDistance(x,y,theta,wall);
	nxtDisplayCenteredTextLine(2, "DToWall: %f", (float)expectedDepth);
	float sample = getGaussianValue(expectedDepth,z);
	nxtDisplayCenteredTextLine(3, "Sample: %f", (float)sample);
	float angle = angleToWall(theta,wall);
	nxtDisplayCenteredTextLine(4, "Angle: %f", (float)angle);
	float result = scaleForAngle(sample,angle);
	wait1Msec(100);
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

	// put all weights back to an equal distribution
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

		xArray[particle] = xArray[particle] + (distanceMoved + e)*sin(thetaArray[particle]);
		yArray[particle] = yArray[particle] + (distanceMoved + e)*cos(thetaArray[particle]);
		thetaArray[particle] = thetaArray[particle] + f;
	}
}

void updateParticleArraysRotate(float degTurned)
{
	for (int particle = 0; particle < NUMBER_OF_PARTICLES; particle++)
	{
    //float g = 0;
		float g = 0;//sampleGaussian(0.0, 0.005); //UPDATE THIS.
	  //float g = sampleGaussian(0.0, 0.5);
	  //float g =sampleGaussian(0.0, 0.5);


		thetaArray[particle] = thetaArray[particle] + (degTurned + g);
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
