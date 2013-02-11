#include "locomotion.c"

void square()
{
   for(int i = 0 ; i < 4 ; i++)
   {
		forward40cmdraw();
		left90dgdraw();
		wait1Msec(1000);
   }
}
