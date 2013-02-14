#include "locomotion.h"
//#include "particles.h"


int x = 0;
int y = 0;
float theta = 0;

float motorPower = 30;
const float ENC_P_CM = 21.157;

//x and y in meters
void navigateToWaypoint (float new_x, float new_y)
{
	new_x *= 100;
	new_y *= 100;
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

	float newAngle = targetAngle - theta;
	nxtDisplayCenteredTextLine(3, "NA: %f", newAngle);
	nxtDisplayCenteredTextLine(4, "TA: %f", targetAngle);
	wait1Msec(1000);
	turnNDegrees(newAngle);

	//moveForward(sqrt((dx*dx) + (dy*dy)));
}

// Functions that draw on the screen
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

  x = d * cos(theta);
  y = d * sin(theta);
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
  theta += a;
}

task main (){
	navigateToWaypoint(0,2);
	wait1Msec(100000);
	navigateToWaypoint(1,1);
	wait1Msec(100000);
	navigateToWaypoint(1,0);
	wait1Msec(100000);
	navigateToWaypoint(1,-1);
	wait1Msec(100000);
	navigateToWaypoint(0,-1);
	wait1Msec(100000);
	navigateToWaypoint(-1,-1);
	wait1Msec(100000);
	navigateToWaypoint(-1,0);
	wait1Msec(100000);
	navigateToWaypoint(-1,1);
	wait1Msec(100000);
}
