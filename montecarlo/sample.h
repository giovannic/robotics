// Return a random number sampled uniformly from the range 0 to max
float sampleUniform()
{
		// Converts random int to correct float range
		//return max * (float)(random(30000)) / 30000.0;


    // NEW RANDOM
    int negposRng = 0;
    negposRng = rand();//%65536;
	  return (float)(negposRng) / 32768.0;    // CHECK THIS

}

// Return a random number sampled from a Gaussian distribution with
// mean = mean, standard deviation = sigma
float sampleGaussian(float mean, float sigma)
{
  float u     = sampleUniform();
  float theta = sampleUniform() * 2 * PI;
  //float theta = sampleUniform(2 * PI);

  // Fix to avoid infinity problem
  if (u == 0) {
    u = 0.0001;
  }

  float r = sqrt(-2*log(u));

  float x = r * cos(theta);

  return mean + (sigma * x);
}
