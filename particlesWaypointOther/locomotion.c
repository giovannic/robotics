#include "locomotion.h"
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
void drawPosition()
{
  nxtSetPixel((x )/scale +10,(y )/scale+10 );
}
void forward40cm()
{
  nSyncedMotors = synchAB;
  nSyncedTurnRatio = 100;
  nMotorEncoder[motorA] = 0;
  while(nMotorEncoder[motorA] < wheelMoveLimit)
  {
    motor[motorA] = motorPower;
  }
  motor[motorA] = 0;  // turn the motors off.
}
void backward40cm()
{
  nSyncedMotors = synchAB;
  nSyncedTurnRatio = 100;
  nMotorEncoder[motorA] = 0;
  while(nMotorEncoder[motorA] > -wheelMoveLimit)
  {
    motor[motorA] = -motorPower;
  }
  motor[motorA] = 0;  // turn the motors off.
}
void left90deg()
{
  nSyncedMotors = synchAB;
  nSyncedTurnRatio = -100;
  nMotorEncoder[motorA] = 0;
  while(nMotorEncoder[motorA] > -wheelRotateLimit )
  {
    motor[motorA] = -motorPower;
  }
  motor[motorA] = 0;
  wait1Msec(1000);
}
void right90deg()
{
  nSyncedMotors = synchAB;
  nSyncedTurnRatio = -100;
  nMotorEncoder[motorA] = 0;
  while(nMotorEncoder[motorA] < wheelRotateLimit)
  {
    motor[motorA] = motorPower;
  }
    motor[motorA] = 0;
      wait1Msec(1000);
}
// Functions that draw on the screen
void forward40cmdraw()
{
  double lastMotorValue = 0;
  double distanceMoved = 0;
  double currentDistance;
  nSyncedMotors = synchAB;
  nSyncedTurnRatio = 100;
  nMotorEncoder[motorA] = 0;
    motor[motorA] = motorPower;
  while(nMotorEncoder[motorA] < wheelMoveLimit)
  {
    currentDistance = nMotorEncoder[motorA];
    distanceMoved = currentDistance - lastMotorValue;
    x = x + distanceMoved * cos(theta);
    y = y + distanceMoved * sin(theta);

    drawPosition();
    lastMotorValue = currentDistance;
  }
  motor[motorA] = 0;  // turn the motors off.


}
void left90dgdraw()
{
  nSyncedMotors = synchAB;
  nSyncedTurnRatio = -100;
  nMotorEncoder[motorA] = 0;
  while(nMotorEncoder[motorA] > -wheelRotateLimit )
  {
    motor[motorA] = -motorPower;
  }
  motor[motorA] = 0;
  wait1Msec(1000);

  theta = theta + PI/2;
}
