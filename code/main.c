#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "utilities.h"
#include "isense.h"
#include "currentcontrol.h"
#define PLOTPTS 100  // number of data points to plot
#define BUF_SIZE 200
#define SAMPLE_TIMES 100     // total sample times in case k
#define REFCURRENT_ABS 200   // absolute reference value in case k
#define TRACK_MAX 2000       // max length of tracking array in case o
#define FORGETTING 1       // forgetting factor used for position_Eint
#define WIND_CURRENT_ABS 1000  // boundary of wind up
#define WIND_POSITION_ABS 1000
// include other header files here


static volatile Mode_t Mode;
static volatile int pwm_val;               // pwm wave value, from -100 to 100
static volatile float Kp_c = 2.0, Ki_c = 0.1;     // current Kp and Ki
static volatile float Kp_p = 20, Ki_p = 2, Kd_p = 500;   // position Kp, Ki, Kd, and Ki should be pretty small
static volatile int test_count = 0;        // counter used in case k and o
static volatile int ref_current_test;      // reference current in case k
static volatile int ref_current;           // reference current in position control
static volatile float current_Eint = 0.0;
static volatile int CURarray[PLOTPTS];     // real current array in case k
static volatile int REFarray[PLOTPTS];     // reference current array in case k
static volatile float ref_position;
static volatile float position_Eint = 0.0;
static volatile float position_eprev = 0.0;
static volatile int track_array_length = 0;  // input track length in case o
static volatile float TRAarray[TRACK_MAX];   // construct reference tracking array (2000 terms, 10s) 
static volatile float ANGarray[TRACK_MAX];   // real angle value in tracking control

void __ISR(_TIMER_2_VECTOR,IPL5SOFT)Current_Controller(void){
  static float current_value = 0.0;        // real current value
  static float current_err = 0.0;
  static float u = 0.0, unew = 0.0;        // control signal for current
  switch(Mode){
    case IDLE:
		{
      OC1RS = 0;                       // duty cycle = OC1RS/(PR3+1)
      LATDbits.LATD6 = 0;
      break;
    }
    case PWM:
    {
      if (pwm_val>=0){
				OC1RS = pwm_val*40;          // from 0 to 4000
				LATDbits.LATD6 = 1;          // using D6 as direction output
			}
			else{
				OC1RS = -pwm_val*40;
				LATDbits.LATD6 = 0;
			}
			break;
		}
  		case ITEST:
		{
			if (test_count==25||test_count==50||test_count==75)
				ref_current_test = -ref_current_test;      // generate square wave
			if (test_count == SAMPLE_TIMES-1)
				Mode = IDLE;                      // after case k, come back to IDLE
			current_value = current_mAm();        // helper function in "isense.h"
			current_err = ref_current_test - current_value;
			current_Eint = current_Eint + current_err;
			current_Eint = anti_windup(current_Eint, WIND_CURRENT_ABS);    // anti wind up in "currentcontrol.c"
			u = Kp_c*current_err + Ki_c*current_Eint;    // u is a percentage(> or < 0)
			if (u >= 0.0){
				LATDbits.LATD6 = 1;
				unew = u;
			} else if (u < 0.0){
				LATDbits.LATD6 = 0;            // change direction according to the sign of u
				unew = -u;
			}
			if (unew > 100.0){
				unew = 100.0;
			} else if (unew < 0.0){
				unew = 0.0;
			}
			OC1RS = (unsigned int)((unew/100.0)*PR3);  // duty cycle = OC1RS/(PR3+1)
			REFarray[test_count] = (int)ref_current_test;  // store reference and real values in arrays
			CURarray[test_count] = (int)current_value;
			test_count++;
			break;
		}
  		case HOLD:
		{
			current_value = current_mAm();
			current_err = ref_current - current_value;
			current_Eint = current_Eint + current_err;
			current_Eint = anti_windup(current_Eint, WIND_CURRENT_ABS);
			u = Kp_c*current_err + Ki_c*current_Eint;     // u is a percentage
			if (u >= 0.0){
				LATDbits.LATD6 = 1;
				unew = u;
			} else if (u < 0.0){
				LATDbits.LATD6 = 0;
				unew = -u;
			}
			if (unew > 100.0){
				unew = 100.0;
			} else if (unew < 0.0){
				unew = 0.0;
			}
			OC1RS = (unsigned int)((unew/100.0)*PR3);  // duty cycle = OC1RS/(PR3+1)
			break;
		}
		case TRACK:
		{
			current_value = current_mAm();
			current_err = ref_current - current_value;
			current_Eint = current_Eint + current_err;
			current_Eint = anti_windup(current_Eint, WIND_CURRENT_ABS);
			u = Kp_c*current_err + Ki_c*current_Eint;
			if (u >= 0.0){
				LATDbits.LATD6 = 1;
				unew = u;
			} else if (u < 0.0){
				LATDbits.LATD6 = 0;
				unew = -u;
			}
			if (unew > 100.0){
				unew = 100.0;
			} else if (unew < 0.0){
				unew = 0.0;
			}
			OC1RS = (unsigned int)((unew/100.0)*PR3);  // duty cycle = OC1RS/(PR3+1)
			break;
		}
		default:
		{
			NU32_LED2 = 0;  // turn on LED2 to indicate an error
			break;
		}
	}
	IFS0bits.T2IF = 0;      // clear flag
}

void __ISR(_TIMER_4_VECTOR,IPL6SRS)Position_Controller(void){
	static float position_value = 0.0;
	static float position_err = 0.0;
	static float position_edot = 0.0;
	switch(Mode){
		case HOLD:
		{
			position_value = encoder_deg();            // helper function in "encoder.h"
			position_err = ref_position - position_value;     // degree
			position_Eint = FORGETTING * position_Eint + position_err;  // forgetting factor
			position_Eint = anti_windup(position_Eint, WIND_POSITION_ABS);   // Eint can be very big, so setting a range
			position_edot = position_err - position_eprev;
			position_eprev = position_err;
			ref_current = Kp_p*position_err + Ki_p*position_Eint + Kd_p*position_edot;

			//if (TRISDbits.TRISD7==0)       // check whether timer4 is working
			//	TRISDbits.TRISD7 = 1;
			//else if (TRISDbits.TRISD7==1)
			//TRISDbits.TRISD7 = 0;
		    break;
		}
		case TRACK:
		{
			position_value = encoder_deg();
			ANGarray[test_count] = position_value;
			position_err = TRAarray[test_count] - position_value;     // degree
			position_Eint = FORGETTING * position_Eint + position_err;  // forgetting factor
			position_Eint = anti_windup(position_Eint, WIND_POSITION_ABS);
			position_edot = position_err - position_eprev;
			position_eprev = position_err;
			ref_current = Kp_p*position_err + Ki_p*position_Eint + Kd_p*position_edot;
			test_count++;
			if (test_count == track_array_length){
				ref_position = TRAarray[test_count-1];
				Mode = HOLD;
			}
			break;
		}
		default:
		{
			NU32_LED2 = 0;  // turn on LED2 to indicate an error
			break;
		}
	}
	IFS0bits.T4IF = 0;
}

int main()
{
  char buffer[BUF_SIZE];
	int i = 0;                // count only in main
  NU32_Startup();           // cache on, min flash wait, interrupts on, LED/button init, UART init

	set_mode(IDLE);           // Mode = IDLE;
  NU32_LED1 = 1;            // turn off the LEDs
  NU32_LED2 = 1;

  __builtin_disable_interrupts();
  encoder_init();
  AD1PCFGbits.PCFG12 = 0;    // B12 as ADC pin
  AD1CON3bits.ADCS = 2;
  AD1CON1bits.ADON = 1;
  current_timer_initialize();
  position_timer_initialize();
  __builtin_enable_interrupts();

  while(1)
  {
		NU32_ReadUART3(buffer,BUF_SIZE); // we expect the next character to be a menu command
		NU32_LED2 = 1;                   // clear the error LED
		switch (buffer[0]) {
			case 'a':
			{
				sprintf(buffer,"%d\r\n",adc_sample_convert(12));   // sample and convert
				NU32_WriteUART3(buffer);      // send adc count to client (0-1023)
				break;
			}
			case 'b':
			{
				sprintf(buffer,"%.3f\r\n",current_mAm());       // adc in mA
				NU32_WriteUART3(buffer);
				break;
			}
			case 'c':
			{
				sprintf(buffer,"%d\r\n",encoder_counts());
				NU32_WriteUART3(buffer);                        // send encoder count to client
				break;
			}
			case 'd':
			{
				sprintf(buffer,"%.3f\r\n",encoder_deg());       // encoder in degree
				NU32_WriteUART3(buffer);
				break;
			}
 			case 'e':
			{
				encoder_reset();
				break;
			}
			case 'f':
			{
				NU32_ReadUART3(buffer,BUF_SIZE);                // input value from -100 to 100
				sscanf(buffer,"%d",&pwm_val);
				Mode = PWM;
				break;
			}
			case 'g':
			{
				NU32_ReadUART3(buffer,BUF_SIZE);                // input current Kp, Ki value
				sscanf(buffer,"%f  %f", &Kp_c, &Ki_c);
				break;
			}
			case 'h':
			{
				sprintf(buffer,"%.3f  %.3f\r\n", Kp_c, Ki_c);   // return current Kp, Ki value
				NU32_WriteUART3(buffer);
				break;
			}
			case 'i':
			{
				NU32_ReadUART3(buffer,BUF_SIZE);                // input postion Kp, Ki, Kd value
				sscanf(buffer,"%f %f %f", &Kp_p, &Ki_p, &Kd_p);
				break;
			}
			case 'j':
			{
				sprintf(buffer,"%.3f  %.3f  %.3f", Kp_p, Ki_p, Kd_p); // return postion Kp, Ki, Kd value
				NU32_WriteUART3(buffer);
			}
			case 'k':                               // ITEST for current control
			{
				test_count = 0;
				ref_current_test = REFCURRENT_ABS;
				current_Eint = 0.0;
				Mode = ITEST;
				while (Mode == ITEST){;}                 // wait until finish ITEST
				sprintf(buffer,"%d\r\n",SAMPLE_TIMES);   // Sample 100 times
				NU32_WriteUART3(buffer);
				for(i=0; i<PLOTPTS; i++){                // send two arrays to Matlab
					sprintf(buffer,"%d %d\r\n", REFarray[i], CURarray[i]);
					NU32_WriteUART3(buffer);
				}
				break;
			}
  			 case 'l':
			{
				__builtin_disable_interrupts();
				Mode = HOLD;
				current_Eint = 0.0;
				position_Eint = 0.0;
				position_eprev = 0.0;
				encoder_reset();
				NU32_ReadUART3(buffer,BUF_SIZE);         // get desired angle from client
				sscanf(buffer, "%f", &ref_position);     // degree
				IFS0bits.T4IF = 0;
				__builtin_enable_interrupts();
				break;
			}
			case 'm':								     // step
			{											 // m and n are totaly same in c
				NU32_ReadUART3(buffer,BUF_SIZE);
				sscanf(buffer, "%d", &track_array_length);
				for(i=0; i<track_array_length; i++){
					NU32_ReadUART3(buffer,BUF_SIZE);
					sscanf(buffer, "%f", &TRAarray[i]);
				}
				break;
			}
			case 'n':									 // cubic
			{
				NU32_ReadUART3(buffer,BUF_SIZE);
				sscanf(buffer, "%d", &track_array_length);
				for(i=0; i<track_array_length; i++){
					NU32_ReadUART3(buffer,BUF_SIZE);
					sscanf(buffer, "%f", &TRAarray[i]);
				}
				break;
			}
			case 'o':
			{
				__builtin_disable_interrupts();
				Mode = TRACK;
				test_count = 0;
				current_Eint = 0.0;
				position_Eint = 0.0;
				position_eprev = 0.0;
				encoder_reset();
				IFS0bits.T4IF = 0;
				__builtin_enable_interrupts();
				while (Mode == TRACK) {;}
				sprintf(buffer,"%d\r\n", track_array_length);   // same plot method as k
				NU32_WriteUART3(buffer);
				for(i=0; i<track_array_length; i++){
					sprintf(buffer,"%f %f\r\n", TRAarray[i], ANGarray[i]);  // ATTENTION: float
					NU32_WriteUART3(buffer);
				}
				break;
			}
			case 'p':
			{
				Mode = IDLE;
				break;
			}
			case 'q':
			{
				// handle q for quit. Later you may want to return to IDLE mode here. 
				break;
			}
			case 'r':
			{
				sprintf(buffer,"%d\r\n",Mode);
				NU32_WriteUART3(buffer);
				break;
			}
			default:
			{
				NU32_LED2 = 0;  // turn on LED2 to indicate an error
				break;
			}
        }
    }
    return 0;
}

