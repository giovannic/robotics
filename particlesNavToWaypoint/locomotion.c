#include "locomotion.h"
//#include "particles.h"


int x = 0;
int y = 0;
float theta = 0;

const int squareSize = 40;
const int wheelMoveLimit = 847;
const int wheelRotateLimit = 200;
const int motorPower = 10;
const float offset = 0.9999;
const float scale = wheelMoveLimit/(squareSize+offset);


int orientation = 0;
/*
void drawPosition()
{
  nxtSetPixel((x )/scale +10,(y )/scale+10 );
}
*/

// Functions that draw on the screen
void forward40cmdraw()
{
  float lastMotorValue = 0;
  float distanceMoved = 0;
  float currentDistance = 0;
  nSyncedMotors = synchAB;
  nSyncedTurnRatio = 100;
  float lineStart = nMotorEncoder[motorA];
  motor[motorA] = motorPower;

  while((nMotorEncoder[motorA] - lineStart) < wheelMoveLimit)
  {
    currentDistance = nMotorEncoder[motorA] - lineStart;
    distanceMoved = currentDistance - lastMotorValue;

    x = x + distanceMoved * cos(theta);
    y = y + distanceMoved * sin(theta);

    drawPosition(x, y);
    lastMotorValue = currentDistance;
	updateParticleArraysForward(distanceMoved);
	drawParticles();
  }

  motor[motorA] = 0;  // turn the motors off.

}
void left90dgdraw()
{
  nSyncedMotors = synchAB;
  nSyncedTurnRatio = -100;
  float rotStart = nMotorEncoder[motorA];
  float degTurned = PI/2;

  while((nMotorEncoder[motorA] - rotStart) > -wheelRotateLimit )
  {
    motor[motorA] = -motorPower;
  }
  motor[motorA] = 0;
  wait1Msec(1000);

  updateParticleArraysRotate(theta, degTurned);
  theta = theta + degTurned;
}
