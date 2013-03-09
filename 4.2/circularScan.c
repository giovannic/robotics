#pragma config(Sensor, S1,     sonar,               sensorSONAR)
#pragma config(Sensor, S2,     bump,                sensorTouch)
#pragma config(Motor,  motorA,          left,          tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          right,         tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorC,          sonarMotor,    tmotorNormal, PIDControl, encoder)

float scanMotorSpeed = 2;

#define NO_BINS 360

typedef struct
{
	short sig[NO_BINS];
} loc_sig;

/*
void resetSonar()
{
	motor[motorC] = -20;
	while(nMotorEncoder[motorC] > -360)
	  ;
	motor[motorC] = 0;
}
*/
/*

void circularScan(loc_sig& signature)
{
	float end = 360;

	motor[motorC] = scanMotorSpeed;

	while( nMotorEncoder[motorC] < end)
	{
		int value = nMotorEncoder[motorC];
		if (value < 360)
		{
			signature.sig[value] = SensorValue(sonar);
		}
	}

	resetSonar();

	PlayTone(523,40);
}
*/
void circularScan(loc_sig& signature)
{
  nSyncedMotors = synchAB;
  nSyncedTurnRatio = -100;
  float rotStart = nMotorEncoder[motorA];
	float rotLimit = 800;

	motor[motorA] = 20;

	while(nMotorEncoder[motorA] - rotStart < rotLimit)
	{
		int value = (nMotorEncoder[motorA]/800) * 360;
		if (value < 360)
		{
			signature.sig[value] = SensorValue(sonar);
		}
	}

	PlayTone(523,40);
}

/*task main()
{
	loc_sig matt;
	circularScan(matt);
	wait10Msec(10000);
}*/
