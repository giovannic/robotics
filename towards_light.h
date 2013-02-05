#ifndef TOWARDS_LIGHT_H
#define TOWARDS_LIGHT_H

task towards_light();
task seek();
task approach_light();
void stopCurrentLightTask(int oldT);
void newLightTask(int newT);

#endif
