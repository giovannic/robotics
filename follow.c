void follow_wall()
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
  {
  }


  updateParticleArraysForward(d);

  theta = findAverageTheta();
  x = findAverageX();
  y = findAverageY();
}
