
int motorPower = 30;
const float stepAngle = PI/8;

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
}

void rotateWMonte(float newAngle)
{
	while(newAngle != 0)
	{
		float turnAngle = 0; //initialise
	  if (newAngle < stepAngle)
	  {
	     turnAngle = newAngle;
	  }
	  else
	  {
	     turnAngle = stepAngle;
	  }
		turnNDegrees(turnAngle);
		newAngle -= turnAngle;
		wait1Msec(100);
  }
}

task main()
{
  rotate(PI);
}
