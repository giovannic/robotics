void navigateToWaypoint (float new_x, float new_y)
{
	float dx = new_x - x;
	float dy = new_y - y;
	float targetAngle = atan( dx / dy );
	float newAngle = targetAngle - theta;
	float dTheta = newAngle
	if (abs(newAngle) > PI/2)
	{
		//there is a smaller difference angle,
		//set difference to the alternative angle
		dTheta = newAngle > 0 ? newAngle - PI : PI + newAngle;
	}
	turnNDegrees(dTheta);
	
	moveForward(sqrt(pow(dx) + pow(dy)));
}