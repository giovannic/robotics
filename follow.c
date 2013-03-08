#pragma config(Sensor, S1,     sonar,               sensorSONAR)
#pragma config(Sensor, S2,     bump,                sensorTouch)
#pragma config(Motor,  motorA,          left,          tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          right,         tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorC,          sonarMotor,    tmotorNormal, PIDControl, encoder)

void follow_wall()
{
  const int wall_follow_distance = 20;
  const float damper = 0.75;
  int balance = 0;
  int reading = 0;
  const int drive_power = 50;

  nMotorPIDSpeedCtrl[motorA] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;
  motor[motorA] = drive_power;
  motor[motorB] = drive_power;

  while(true)
  {
    reading = SensorValue[S1];
    if(reading < 100)
    {
      balance = damper*(SensorValue[S1] - wall_follow_distance);
    } else {
      if (balance < 0)
      {
        balance = -7;
      } else {
        balance = 7;
      }
    }
    nxtDisplayCenteredTextLine(1, "bal: %d", balance);
      motor[motorA] = drive_power - balance;
      motor[motorB] = drive_power + balance;
  }
}

task main(){
 follow_wall();
}
