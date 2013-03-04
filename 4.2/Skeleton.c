#include "circularScan.c"

int first;
int second;
int third;

int sonarOffset = 4;

void getIntoCorridor()
{
  loc_sig scan;
  circularScan(scan);

  int result = -1;

  while(result == -1)
  {
    for(int i = 0 ; i < NO_BINS ; i++)
    {
      if(scan.sig[i] == 63 - sonarOffset)
      {
        result = i;
      }
    }
  }



  //Scan 360 and work out which array element is the largest sonar reading.
  //Rotate so that you face that way.
  //Drive until you are 21 away from the wall infront (allow for sonar wheelbase offset)
}

int calculateCurrentWaypoint()
{
  //Turn the sonar right.
  //If reading < 50 you're at 1
  //Turn it left.
  //If reading < 50 you're at 3
  //Else you're at 2#
  return 0;
}

void corridorTurn(int currentWaypoint, int destinationWaypoint)
{
  //Turn based on arguments.
  //Face sonar to the correct wall.
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
