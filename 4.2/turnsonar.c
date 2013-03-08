#pragma config(Sensor, S1,     sonar,               sensorSONAR)
#pragma config(Sensor, S2,     bump,                sensorTouch)
#pragma config(Motor,  motorA,          left,          tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          right,         tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorC,          sonarMotor,    tmotorNormal, PIDControl, encoder)

void sonarTo(int degrees)
{
  degrees = - degrees;
  const int speed = 20;
  int difference = nMotorEncoder[motorC] - degrees;

  if (difference < 0)
  {
    motor[motorC] = speed;
    while(nMotorEncoder[motorC] < degrees)
      ;
  } else {
    motor[motorC] = -speed;
    while(nMotorEncoder[motorC] > degrees)
      ;
  }
  motor[motorC] = 0;
}

task main()
{
  sonarTo(0);
  wait1Msec(1000);
  sonarTo(-90);
  wait1Msec(1000);
  sonarTo(90);
  wait1Msec(1000);
  sonarTo(180);
  wait1Msec(1000);
  sonarTo(115);
  wait1Msec(1000);
  sonarTo(0);
}
