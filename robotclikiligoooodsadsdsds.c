#include <cmath>
#include <iostream>

using namespace std;

const int NUMBER_OF_WALLS = 8;
//                                      a    b    c    d    e    f    g    h
float wallAxArray[NUMBER_OF_WALLS] = {  0,   0,  84,  84, 168, 168, 210, 210};
float wallAyArray[NUMBER_OF_WALLS] = {  0, 168, 126, 210, 210,  84,  84,   0};
float wallBxArray[NUMBER_OF_WALLS] = {  0,  84,  84, 168, 168, 210, 210,   0};
float wallByArray[NUMBER_OF_WALLS] = {168, 168, 210, 210,  84,  84,   0,   0};

float scaleForAngle(float sample, float angle)
{
	if (angle == 0)
	{
		angle = 1;
	}

	return (sample * 1.0/angle);
}

float sampleGaussian(float m, float z)
{
	float sigma = 1;
	float constant = 1;
	
	float numerator = - ((z-m) * (z-m));
	float denominator = 2 * sigma * sigma;
	float power = numerator/denominator;

	return (exp(power) + constant);	
}

int between(float middle, float start, float finish)
{
	int mid = (int) middle + 0.5;
	int sta = (int) start + 0.5;
	int fin = (int) finish + 0.5;
	
	return ( mid >= sta && mid <= fin) || (mid >= fin && mid <= sta);
}

int getClosestWallForward(float x, float y, float theta)
{
	float closestDistance = -1;
	int closestWall = 0;

	for(int i = 0 ; i < NUMBER_OF_WALLS ; i++)
	{
		float ax = wallAxArray[i];
		float bx = wallBxArray[i];
		float ay = wallAyArray[i];
		float by = wallByArray[i];

		float dx = bx - ax;
		float dy = by - ay;

		float numerator = dy*(ax-x) - dx*(ay-y);
		float denominator = dy*cos(theta) - dx*sin(theta);
		float distance = numerator/denominator;

		int interX = (int) (x + distance*cos(theta));
		int interY = (int) (y + distance*sin(theta));

		int collide = between(interX, ax, bx) && between(interY, ay, by);
		
		if(distance >= 0 && (closestDistance == -1 || distance < closestDistance) )
		{
			closestDistance = distance;
			closestWall = i;
		}
	}

	return closestWall;
}

int getClosestWallForwardDistance(float x, float y, float theta, int wall)
{

	float ax = wallAxArray[wall];
	float bx = wallBxArray[wall];
	float ay = wallAyArray[wall];
	float by = wallByArray[wall];

	float dx = bx - ax;
	float dy = by - ay;

	float numerator = dy*(ax-x) - dx*(ay-y);
	float denominator = dy*cos(theta) - dx*sin(theta);
	float distance = numerator/denominator;	
	
	return distance;
}

// Calculate the angle to the wall.
float angleToWall(float theta, int wall)
{
	float ax = wallAxArray[wall];
	float bx = wallBxArray[wall];
	float ay = wallAyArray[wall];
	float by = wallByArray[wall];

	float dx = bx - ax;
	float dy = ay - by;

	float numerator = dy*cos(theta) + dx*sin(theta);
	float denominator = sqrt((dy*dy) + (dx*dx));
	float fraction = numerator/denominator;

	return  abs(acos(fraction));
}

float calculate_likelihood(float x, float y, float theta, float z)
{
	int wall = getClosestWallForward(x,y,theta);
	float expectedDepth = getClosestWallForwardDistance(x,y,theta,wall);
	float sample = sampleGaussian(expectedDepth,z);
	float angle = angleToWall(theta,wall);
	float result = scaleForAngle(sample,angle);
	return result;
}

int main()
{
	cout << calculate_likelihood(0,0,0,10) << endl;
	return 0;
}
