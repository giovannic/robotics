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
  nMotorEncoder[motorC] = 0;
  motor[motorC] = -20;
  while(nMotorEncoder[motorC] > -90)
    ;
  
  if (SensorValue[sonar] < 50)
  {
     return 1;
  }

  //Turn the sonar left.
  nMotorEncoder[motorC] = 0;
  motor[motorC] = 20;
  while(nMotorEncoder[motorC] > 180)
    ;
  
  if (SensorValue[sonar] < 50)
  {
     return 2;
  }
  return 3;
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
}

void chamberAdjust()
{
	//Sonar look left + take reading.
	//Sonar look right + take reading.

	//Adjust the position of the robot so that the left reading = right reading.
}

void beep()
{
	//beep when done.
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
