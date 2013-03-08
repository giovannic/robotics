
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

void follow_wall(int distance)
{
  int balance = 0;
  int reading = 0;

  const int wall_follow_distance = 20;
  const float damper = 0.75;
  const int drive_power = 50;
  const int base = 7;

  nMotorPIDSpeedCtrl[motorA] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;
  motor[motorA] = drive_power;
  motor[motorB] = drive_power;

  float lineStart = nMotorEncoder[motorA];
  float encoderLimit = ENC_P_CM*distance;

  while((nMotorEncoder[motorA] - lineStart) < encoderLimit/2)
  {
    reading = SensorValue[S1];
    if(reading < 100)
    {
      balance = damper*(SensorValue[S1] - wall_follow_distance);
    } else {
      if (balance < 0)
      {
        balance = -base;
      } else {
        balance = base;
      }
    }
    motor[motorA] = drive_power - balance;
    motor[motorB] = drive_power + balance;
  }
  motor[motorA] = 0;
  motor[motorB] = 0;
}

void follow_outer_wall()
{
  const int step = 20;
  const int drive_power = 20;

  //redundant
  sonarTo(0);
  reading = SensorValue[S1];
  while(reading > 21 + step)
  {
    sonarTo(-90);
    follow_wall(step);
    sonarTo(0);
    reading = SensorValue[S1];
  }

  motor[motorA] = drive_power;
  motor[motorB] = drive_power;

  while(SensorValue[S1] > 21)
    ;

  motor[motorA] = 0;
  motor[motorB] = 0;

}
