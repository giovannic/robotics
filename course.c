#include "sensors.h"

#include "course.h"
#include "towards_light.c"
#include "obstacles.c"
//left is A, right is B
int courseState = 0;

task main()
{
  StartTask(course);
  while(true){}
}

task course()
{
  //first task
  StartTask(towards_light);
  while (true)
  {

    //light found
    //state 1
    //go forward
    if(centred() || bumping())
    {
      newCourseTask(1);
    }
    //base - look for light
    //state 0
	  else
	  {
	    newCourseTask(0);
	  }

  }
}

void newCourseTask(int newT)
{
  if (courseState != newT)
  {
    stopCurrentCourseTask(courseState);
    courseState = newT;
	  switch(newT){
    case 1:
      StartTask(avoid_obstacles);
      break;
    case 0:
      StartTask(towards_light);
      break;
    }
  }
}

void stopCurrentCourseTask(int oldT)
{
  switch(oldT){
    case 1:
      StopTask(avoid_obstacles);
      break;
    case 0:
      StopTask(towards_light);
      break;
  }
}

bool centred()
{
	return (abs(
	  SensorValue(leftLight) - SensorValue(rightLight)
	  ) < 7.5);
}

bool bumping()
{
  return (SensorValue(leftBumper) || SensorValue(rightBumper));
}
