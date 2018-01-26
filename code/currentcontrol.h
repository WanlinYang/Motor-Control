#ifndef CURRENTCONTROL_H
#define CURRENTCONTROL_H

void current_timer_initialize(void);
float current_mAm(void);
float anti_windup(float current_Eint, float wind_abs);

#endif
