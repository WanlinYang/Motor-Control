#ifndef UTILITIES_H
#define UTILITIES_H

typedef enum {IDLE,PWM,ITEST,HOLD,TRACK} Mode_t;

void set_mode(Mode_t m);
Mode_t get_mode(void);

#endif
