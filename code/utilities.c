#include "utilities.h"
#include <xc.h>

static volatile Mode_t mode;

void set_mode(Mode_t m){    // set at the beginning of main to IDLE
  mode = m;
}

Mode_t get_mode(void){
  return mode;
}
