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

void FrereJacque()
{
	PlayTone(523,40);	wait10Msec(50);
	PlayTone(587,40);	wait10Msec(50);
	PlayTone(659,40);	wait10Msec(50);
	PlayTone(523,40);	wait10Msec(50);
	PlayTone(523,40);	wait10Msec(50);
	PlayTone(587,40);	wait10Msec(50);
	PlayTone(659,40);	wait10Msec(50);
	PlayTone(523,40);	wait10Msec(50);
	PlayTone(659,40);	wait10Msec(50);
	PlayTone(698,40);	wait10Msec(50);
	PlayTone(784,80);	wait10Msec(90);
	PlayTone(659,40);	wait10Msec(50);
	PlayTone(698,40);	wait10Msec(50);
	PlayTone(784,80);	wait10Msec(90);
	PlayTone(784,20);	wait10Msec(30);
	PlayTone(880,20);	wait10Msec(30);
	PlayTone(784,20);	wait10Msec(30);
	PlayTone(698,20);	wait10Msec(30);
	PlayTone(659,40);	wait10Msec(50);
	PlayTone(523,40);	wait10Msec(50);
	PlayTone(784,20);	wait10Msec(30);
	PlayTone(880,20);	wait10Msec(30);
	PlayTone(784,20);	wait10Msec(30);
	PlayTone(698,20);	wait10Msec(30);
	PlayTone(659,40);	wait10Msec(50);
	PlayTone(523,40);	wait10Msec(50);
	PlayTone(523,40);	wait10Msec(50);
	PlayTone(392,40);	wait10Msec(50);
	PlayTone(523,80);	wait10Msec(90);
	PlayTone(523,40);	wait10Msec(50);
	PlayTone(392,40);	wait10Msec(50);
	PlayTone(523,80);	wait10Msec(90);
	return;
}

task main()
{
  FrereJacque();
	loc_sig matt;
	circularScan(matt);
	wait10Msec(10000);
}
