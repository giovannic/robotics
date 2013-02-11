#include "locomotion.h"

int x = 0;
int y = 0;

const int squareSize = 40;
const int wheelMoveLimit = 847;
const int wheelRotateLimit = 200;
const int motorPower = 10;
const float offset = 0.9999;
const float scale = wheelMoveLimit/(squareSize+offset);





int orientation = 0;

void drawPosition()
{
  nxtSetPixel(x+10,y+10 );
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
  int lastMotorValue = -1;
  nSyncedMotors = synchAB;
  nSyncedTurnRatio = 100;
  nMotorEncoder[motorA] = 0;
    motor[motorA] = motorPower;
  while(nMotorEncoder[motorA] < wheelMoveLimit)
  {
    if (nMotorEncoder[motorA] != lastMotorValue) {

    switch( orientation )
    {
    case 0:
    //    if ( (nMotorEncoder[motorA]   ) == 0) {
      x = nMotorEncoder[motorA] / scale;

   //     }

        break;
    case 1:
    //    if ( (nMotorEncoder[motorA]   ) == 0) {
          y = nMotorEncoder[motorA] / scale;
    //    }
        break;
    case 2:
    //    if ( (nMotorEncoder[motorA]  ) == 0) {
          x = squareSize - (nMotorEncoder[motorA] / scale);
    //    }
        break;
    case 3:
    //    if ( (nMotorEncoder[motorA]   ) == 0) {
          y = squareSize - (nMotorEncoder[motorA] / scale);
    //    }
        break;
    }


    drawPosition();
  }
    lastMotorValue = nMotorEncoder[motorA];
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
  // Set a new orientation (if past 3 then put back to 0)
  orientation = (orientation+1)%4  ;

}
