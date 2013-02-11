displaySonarValue()
{
	nxtDisplayCenteredTextLine(3, "Sensor Value: %d", SensorValue[sonarSensor]);
	wait1Msec(1000);
}

task main()
{
	while(1)
	{
		displaySonarValue();
	}
}

