#pragma config(Sensor, S1,     sonar_sensor,        sensorSONAR)
#pragma config(Motor,  motorA,          Right_motor,   tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          Left_motor,    tmotorNormal, PIDControl, encoder)

const float ENC_P_CM = 21.157;

void follow_wall(int distance)
{

  float lastMotorValue = 0;
  float distanceMoved = 0;
  float currentDistance = 0;

  nSyncedMotors = synchAB;
  nSyncedTurnRatio = 100;

  float lineStart = nMotorEncoder[motorA];
  motor[motorA] = motorPower;
  float  encoderLimit = ENC_P_CM*d;

  while((nMotorEncoder[motorA] - lineStart) < encoderLimit)
  {}

}

task main()
{
  follow_wall(40);
}
