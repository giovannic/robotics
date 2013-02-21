// x,y,theta for a specific particle. z is the sonar reading.
float calculate_likelihood(float x, float y, float theta, float z)
{
  //sentinel minimum and wall values
  //minimum = closest wall so far.
  //
  float By = 0, Bx = 0, Ax = 0, Ay = 0, m = 0, minimum = -1; //wall = -1;
  float dx = 0, dy = 0, numerator = 0, denominator = 0, interX = 0, interY = 0;

   for (int i = 0; i < NUMBER_OF_WALLS; ++i)
   {
      Ax = wallAxArray[i];
      Ay = wallAyArray[i];
      Bx = wallBxArray[i];
      By = wallByArray[i];

	  dx = Bx - Ax;
	  dy = By - Ay;

	  //calculate m
	  numerator = dy*(Ax-x) - dx*(Ay-y);
	  denominator = dy*cos(theta) - dx*sin(theta);
      m = numerator/denominator;

		if ((m > 0) && (minimum > m || minimum == -1))
		{
			//check bound
			//round or truncate?
			interX = (int) (x + m*cos(theta));
			interY = (int) (y + m*sin(theta));
			if (between(interX, Ax, Bx) && between(interY, Ay, By))
			{
				minimum = m;
				//wall = i;
			}
		}

   }
   
   	  return sample_gaussian(z, minimum, 1, 1);
}