#include "squareDrawing.h"

task main ()
{
  eraseDisplay();

	initParticleArrays();

   for(int i = 0 ; i < 4 ; i++)
   {
		forward40cmdraw();
		left90dgdraw();
		wait1Msec(1000);
   }
   wait1Msec(5000);
}
