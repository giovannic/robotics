#pragma config(Sensor, S1, sonar_sensor, sensorSONAR)
#pragma config(Motor, motorC, Sonar_Motor, tmotorNormal, PIDControl, encoder)

float scanMotorSpeed = 2;

#define NO_BINS 360

typedef struct
{
	short sig[NO_BINS];
} loc_sig;

void resetSonar()
{
	nMotorEncoder[motorC] = 0;
	motor[motorC] = -20;
	while(nMotorEncoder[motorC] > -360)
	{

	}
	motor[motorC] = 0;
}

void circularScan(loc_sig& signature)
{
	nMotorEncoder[motorC] = 0;
	float end = 360;

	motor[motorC] = scanMotorSpeed;

	while( nMotorEncoder[motorC] < end)
	{
		int value = nMotorEncoder[motorC];
		if (value < 360)
		{
			signature.sig[value] = SensorValue(sonar_sensor);
		}
	}

	resetSonar();

	PlayTone(523,40);
}

/*task main()
{
	loc_sig matt;
	circularScan(matt);
	wait10Msec(10000);
}*/
