#include "currentcontrol.h"
#include <xc.h>

void current_timer_initialize(void){
  OC1CONbits.ON = 1;        // turn on OC1
  OC1CONbits.OCTSEL = 1;    // use Timer3
  T3CONbits.TCKPS = 0b000;  // Timer3 prescaler N=0
  PR3 = 3999;               // period = (PR3+1) * N * 12.5 ns = 50000ns, 20 kHz
  TMR3 = 0;                 // initial TMR3 count is 0
  OC1CONbits.OCM = 0b110;   // PWM mode without fault pin; other OC1CON bits are defaults
  OC1R = 3000;              // initialize before turning OC1 on
  T3CONbits.ON = 1;         // turn on Timer3

  T2CONbits.TCKPS = 0b011;  // prescaler N = 8   page126
  TMR2 = 0;             // initial TMR2 count is 0
  PR2 = 1999;           // period = (PR2+1)*N*12.5ns = 1/5kHz = 20000ns
  T2CONbits.ON = 1;
  IPC2bits.T2IP = 5;
  IPC2bits.T2IS = 0;
  IFS0bits.T2IF = 0;    // step5: clear flag
  IEC0bits.T2IE = 1;    // step6: enable T2

  TRISDbits.TRISD6 = 0;   // D6 as digital output
}

float current_mAm(void){
  return (adc_sample_convert(12)*1.9207)-977.4;
}

float anti_windup(float current_Eint, float wind_abs){
  if (current_Eint > wind_abs)
    current_Eint = wind_abs;
  else if (current_Eint<-wind_abs)
    current_Eint = -wind_abs;
}
