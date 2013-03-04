#pragma config(Sensor, S1,     sonar,               sensorSONAR)
#pragma config(Sensor, S2,     bump,                sensorTouch)
#pragma config(Motor,  motorA,          left,          tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          right,         tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorC,          sonarMotor,    tmotorNormal, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "circularScan.c"
#include "Utils.c"

int first;
int second;
int third;

int sonarOffset = 4;

/* Global to fix memory error. */
loc_sig scan;

void getIntoCorridor()
{

  //Scan 360 and work out which array element is the largest sonar reading.
  circularScan(scan);

  int result = -1;
  int boundary = 0;

  while(result == -1)
  {
    for(int i = 0 ; i < NO_BINS ; i++)
    {
      if(scan.sig[i] == (63 - sonarOffset + boundary) || scan.sig[i] == (63 - sonarOffset - boundary))
      {
        result = i;
      }
    }
    boundary++;
  }

  nxtDisplayCenteredTextLine(1, "ArrElt: %d", result);
  wait1Msec(1000);

  //Rotate so that you face that way.
  float turnAngle = (result / 180.0)*PI;

  nxtDisplayCenteredTextLine(2, "TurnAngle: %f", turnAngle);
  wait1Msec(1000);

  turnRadiansClockwise(-turnAngle);

  nSyncedMotors = synchAB;
  nSyncedTurnRatio = 100;
  motor[motorA] = 30;

  while(SensorValue[sonar] > 21 - sonarOffset)
  {

  }

  motor[motorA] = 0;

  //Scan 360 and work out which array element is the largest sonar reading.
  //Rotate so that you face that way.
  //Drive until you are 21 away from the wall infront (allow for sonar wheelbase offset)
}

int calculateCurrentWaypoint()
{
  int waypoint = 0;

  //Turn the sonar right.
  nMotorEncoder[motorC] = 0;
  motor[motorC] = -20;
  while(nMotorEncoder[motorC] > -90)
    ;

  if (SensorValue[sonar] < 50)
  {
    //fix and return
    nMotorEncoder[motorC] = 0;
    motor[motorC] = 20;
    while(nMotorEncoder[motorC] < 0)
      ;  
    motor[motorC] = 0;
    return 1;
  }

  //Turn the sonar left.
  nMotorEncoder[motorC] = 0;
  motor[motorC] = 20;
  while(nMotorEncoder[motorC] < 90)
    ;

  if (SensorValue[sonar] < 50)
  {
    waypoint = 2;
  } else {
    waypoint = 3;
  }

  //fix and return
  nMotorEncoder[motorC] = 0;
  motor[motorC] = -20;
  while(nMotorEncoder[motorC] > 0)
    ;  
  motor[motorC] = 0;
  return waypoint;
}

void corridorTurn(int currentWaypoint, int destinationWaypoint)
{
  //Turn based on arguments.
  //Face sonar to the correct wall.
  float a = 0;
  switch(currentWaypoint)
  {
    case 1:
      a = -PI/2;
      break;
    case 2:
      switch(destinationWaypoint)
      {
        case 1:
          a = PI/2;
          break;
        case 3:
          a = -PI/2;
          break;
      }
    case 3:
      a = PI/2;
      break;
  }
  turnRadiansClockwise(a);
}

void downCorridor(int dest)
{
  //If the destination is waypoint 2 we need to train the odometry to drive
  //the correct distance whilst wall following.

  //Otherwise we need to wall follow and occasionaly shoot the sonar forwards
  //to gauage our depth from the wall infront. We want to stop at 21 from the
  //wall.
}

void turnToWaypoint(int previous, int dest)
{
  //Rotate to face the waypoint, for 1 and 3 this is dependent only on the
  //destination whereas for waypoint 2 we need to know where we came from.
}

void driveToBackWall()
{
  //Drive forwards till the sonar gives a reading of 21.

  nSynchedMotors = synchAB;
  nSyncedTurnRatio = 100;
  motor[motorA] = 10;
  while(SensorValue(sonar) > 21)
    ;
  motor[motorA] = 0;

}

void chamberAdjust()
{
  //Sonar look left + take reading.
  nMotorEncoder[motorC] = 0;
  motor[motorC] = 5; 
  while(nMotorEncoder[motorC] < 90)
    ;
  motor[motorC] = 0;
  int leftReading = SensorValue(sonar);

  //Sonar look right + take reading.
  motor[motorC] = -5; 
  while(nMotorEncoder[motorC] > -90)
    ;
  motor[motorC] = 0;
  int rightReading = SensorValue(sonar);
  //Recenter sonar
  motor[motorC] = 5; 
  while(nMotorEncoder[motorC] < 0)
    ;
  motor[motorC] = 0;
  //Adjust the position of the robot so that the left reading = right reading.
  if(leftReading > rightReading)
  {
    turnRadiansClockwise(-PI/2);
    nSynchedMotors = synchAB;
    nSyncedTurnRatio = 100;
    motor[motorA] = 5;
    while( SensorValue(sonar) > 21)
      ;
    motor[motorA] = 0;
    turnRadiansClockwise(PI/2);
  }
  else if(leftReading < rightReading)
  {
    turnRadiansClockwise(PI/2);
    nSynchedMotors = synchAB;
    nSyncedTurnRatio = 100;
    motor[motorA] = 5;
    while( SensorValue(sonar) > 21)
      ;
    motor[motorA] = 0;
    turnRadiansClockwise(-PI/2);
  }

}

void beep()
{
  //beep when done.
  PlayImmediateTone(780, 100)
}

task main()
{
  getIntoCorridor();

  int currentWaypoint = calculateCurrentWaypoint();

  if(currentWaypoint == 1)
  {
    first = 2;
    second = 3;
    third = 1;
  }
  else if(currentWaypoint == 2)
  {
    first = 3;
    second = 1;
    third = 2;
  }
  else //currentWaypoint == 3
  {
    first = 2;
    second = 1;
    third = 3;
  }

  /* First */
  corridorTurn(currentWaypoint, first);
  downCorridor(first);
  turnToWaypoint(currentWaypoint, first);
  driveToBackWall();
  chamberAdjust();
  beep();
  /* End First */

  /* Second */
  currentWaypoint = first;
  getIntoCorridor();
  corridorTurn(currentWaypoint, second);
  downCorridor(second);
  turnToWaypoint(currentWaypoint, second);
  driveToBackWall();
  chamberAdjust();
  beep();
  /* End Second */

  /* Third */
  currentWaypoint = second;
  getIntoCorridor();
  corridorTurn(currentWaypoint, third);
  downCorridor(third);
  turnToWaypoint(currentWaypoint, third);
  driveToBackWall();
  chamberAdjust();
  beep();
  /* End Third */

}
