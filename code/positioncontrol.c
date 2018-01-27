#include "positioncontrol.h"
#include <xc.h>

void position_timer_initialize(void){

  T4CONbits.TCKPS = 0b110;  // prescaler N = 64   page126
  TMR4 = 0;             // initial TMR2 count is 0
  PR4 = 6249;           // period = (PR2+1)*N*12.5ns = 1/(200Hz) = 5,000,000ns
  T4CONbits.ON = 1;
  IPC4bits.T4IP = 6;
  IPC4bits.T4IS = 0;
  IFS0bits.T4IF = 0;    // step5: clear flag
  IEC0bits.T4IE = 1;    // step6: enable T4

  TRISDbits.TRISD7 = 0;   // D7 as digital output
  TRISDbits.TRISD7 = 0;
}
