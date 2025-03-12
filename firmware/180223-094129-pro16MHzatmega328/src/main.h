
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <avr/wdt.h>

/*
https://www.maxonmotor.ch/medias/sys_master/root/8797816094750/maxonMotorData-Handzettel.pdf?attachment=true

https://www.heise.de/select/make/2016/6/1482398401198797
*/

/* Lochraster (013/xx)
//Arduino Mini Pro
//5V, 16MHz quarz
//Onboard 5V/150mA LDO
//Pin 13 (PB5) Onboard green LED (active high)
*/

/* PCB 013/00
5V 8MHz

  [NAME]       [Arduino]   [Atmega]
---------------------------------------
BTN_PIN_PROG      D2      PD2
WS2811_PIN        D9      PB7 rework:PB1
RC_PULLUP_PIN     D0      PD0

MOTOR_PIN...
    _GateP_Neg    D3      PD3
    _GateP_Pos    D4      PD4
    _GateN_Pos    D5      PD5 (OC0B)
    _GateN_Neg    D6      PD6 (OC0A)

ADC_PIN...
    _MOTOR_NEG    A0      PC0 (ADC0)
    _MOTOR_POS    A1      PC1 (ADC1)
    _MOTOR_Rs     A2      PC2 (ADC2)
    _AD_MON_SUP   A3      PC3 (ADC3) (not used)
    _RC           A4      PC4 (ADC4)

*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//RL basic defines
#define INIT_PIN_OUT(ddrx,num)			ddrx |= (1<<num)
#define INIT_PIN_IN(ddrx,num)			ddrx &= ~(1<<num)

#define INIT_PIN_PULLUP(port,num)		port |= (1<<num)
#define INIT_PIN_NO_PULLUP(ddrx,num)	port &= ~(1<<num)

#define SET_PIN_HIGH(port,num)			port |= (1<<num)
#define SET_PIN_LOW(port,num)			port &= ~(1<<num)
#define SET_PIN_TOG(port,num)			port^= (1<<num)

#define PIN_IS_HIGH(pin,num)			pin & (1<<num)
#define PIN_IS_LOW(pin,num)				!(pin & (1<<num))

///////////////////////////////////////////////////////////////////////////////
//system mode
#define BTN_MODE_PAUSE 1  //pause and continue program if "start" button is pressed
#define BTN_MODE_BREAK 0  //stop (go to program PROG_STOP) if "start" button is pressed

///////////////////////////////////////////////////////////////////////////////
/* PIN defines */
//  digital I/Os:
#define BTN_PIN_PROG  2  //switch program

//onboard pullup for RC control
//#define RC_PULLUP_PIN 1  //PD1 GPIO1
#define RC_PULLUP_PIN 0 //REWORK 09-06-2020: use RXD pin to control pull-up

//  Motor control:
#if MOTOR_HBRIDGE
  #define MOTOR_PIN_PWM   9  //
  #define MOTOR_PIN_DIR1  12   //
  #define MOTOR_PIN_DIR2  11  //

  #define MOTOR_PIN_GateP_Neg   3 //PD3
  #define MOTOR_PIN_GateP_Pos   4 //PD4
  #define MOTOR_PIN_GateN_Pos   5 //PD5 PWM (OC0B)
  #define MOTOR_PIN_GateN_Neg   6 //PD6 PWM (OC0A)

  #define ADC_PIN_MOTOR_POS 1
  #define ADC_PIN_MOTOR_NEG 0
#endif

//  analog inputs:
//Motor current sense
#define ADC_PIN_MOTOR_Rs 2 //PC2 ADC2

//Remote Control
//#define ADC_PIN_RC  5     //PC5 ADC5
#define ADC_PIN_RC  4     //PC4 ADC4 RXD //REWORK 09-06-2020:

//Supply voltage measurement
#define ADC_PIN_MON_SUP  3     //PC3 ADC3


///////////////////////////////////////////////////////////////////////////////
//increase speed in prog "speed up"
#define SPEEDUP 1
///////////////////////////////////////////////////////////////////////////////
//control bits (button press...)
unsigned char sem_nextProgram = 0;  //is set to 1 if PROG button is pressed
unsigned char sem_starStop = 0;     //is set to 1 if START button is pressed
///////////////////////////////////////////////////////////////////////////////
//list of all available programs
#define MAX_PROGRAM 5

#define PROG_NORMAL   1
#define PROG_SPEEDUP  2
#define PROG_CRACY    3
#define PROG_MAXSPEED 4
#define PROG_RC       5

unsigned char prog_current = 1;	//current selected Programm
///////////////////////////////////////////////////////////////////////////////
//list of all program status
#define STAT_PAUSE  0
#define STAT_RUN    1
#define STAT_STOP   3

unsigned char stat_current = 0;  //1: stat_current/continue program, 0: program is stopped
///////////////////////////////////////////////////////////////////////////////
//Motor speed (140->255)

//Vin=7.5V SPEED_CORRECTION_FACTOR = 1
//Vin= 12V SPEED_CORRECTION_FACTOR = 0.6
#define SPEED_CORRECTION_FACTOR 1

#define ABSMINSPEED (20 * SPEED_CORRECTION_FACTOR)
#define MINSPEED  (70  * SPEED_CORRECTION_FACTOR)//with a lower value the motor will not move
#define NORMSPEED (70 * SPEED_CORRECTION_FACTOR)//70
#define MAXSPEED  (255 * SPEED_CORRECTION_FACTOR)

unsigned char MOTOR_speed = 0;	//
unsigned char MOTOR_speed_new = 0;
///////////////////////////////////////////////////////////////////////////////
//motor direction
#define STOP  0
#define RWD   1	//backward
#define FWD   2	//forward
#define BREAK 3	//break
#define FREE  4	//freewheel

unsigned char MOTOR_direction = 0;	//motor dir
unsigned char MOTOR_direction_new = 0;	//motor dir
///////////////////////////////////////////////////////////////////////////////
unsigned char PWM_val = 255;	//aktuelle PWM OFF Zeit
unsigned char LED_speed = MINSPEED;

int loopCount = 0;  //count the main loops (to get a timebase)
///////////////////////////////////////////////////////////////////////////////
//start/stop detection via motor connection
int Motor_Pos_ADC = 0;  //analog input from positive motor connection
int Motor_Neg_ADC = 0;  //analog input from negative motor connection
int Motor_Rs_ADC  = 0; //analog input from motor current sense resistor

//calculated filtered values:
int Motor_Rs_ADC_FILTER = 0;

// Vsup--|22k|---ADC---|1k|---GND
#define MOTOR_VOLTAGE_ADC_OFF_LEVEL 50
#define MOTOR_CURRENT_ADC_OFF_LEVEL 5

//after switch activation or motor direction change, the current sense
//will be deactivated until the motor is running again to avoid a switch-off detection
#define CURRENT_SENSE_DEACTIVATE_TIME 10

int currentSenseDeactivate = 0;

///////////////////////////////////////////////////////////////////////////////
//RemoteControl analog input

//RC control (extern variable resistor to GND)
//Schematic: (PIN-OUT-5V)---|4k7|---(ADC)---|poti|---(GND)

// voltage measured at PC5 in mV (black RC Felix&Nadja) 013/00 (4k7 onboard pullup)
// 5V supply: 4700
// ADC_max =  1810  (full speed forward + full fine regulator forward)
// ADC_norm = 1650 (speed regulator medium forward + fine regulator in mid position)
// ADC_mid =  1490  (speed regulator in mid position + fine regulator in mid position)
// ADC_low =  1060  (full speed backward + full fine regulator backward)
//
//calculated ADC black RC case (1024/4700 * measured)
// int ADC_max =   394;
// int ADC_norm =  359;
// int ADC_mid =   325;
// int ADC_low =   231;


// voltage measured at PC4 in mV (blue RC) 013/00 (4k7 onboard pullup)
// 5V supply: 5100
// ADC_max =  2027  (full speed forward + full fine regulator forward)
// ADC_norm = 1967 (speed regulator medium forward + fine regulator in mid position)
// ADC_mid =  1827  (speed regulator in mid position + fine regulator in mid position)
// ADC_low =  1434  (full speed backward + full fine regulator backward)
//
//calculated ADC blue RC case (1024/5100 * measured) 09-06-2020
int ADC_max =   406;
int ADC_norm =  395;
int ADC_mid =   367;
int ADC_low =   289;


int ADC_hyst =  5;  //hysterese around ADC_mid
int ADC_hyst2 = 20; //hysterese from/to ADC_max/ADC_low

int RC_ADC = 0; //analog input from RC control

/*
poti resistance range:
max: 3k27 (max speed forward)
typ: 2k99 (norm speed forward)
mid: 2k66 (stop, idle position)
min: 1k95 (max speed backward)
*/

#define FILTER_LP_POINTS 10
int xRs[10] = { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5 }; //current sense ADC filter

int xRC[10] = { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5 }; //remote control ADC filter

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void ISRbtnPROG();
//void ISRbtnSTART(); //not needed if ON/OFF switch is in series with motor

void debugDiagnose();

void setPwmFrequency(int pin, int divisor);
int lowPassAveraging_Rs(int input); //current sense ADC filter
int lowPassAveraging_RC(int input); //remote control ADC filter

void executeProgram();
void checkStartSwitch();
void incrementProgram();
void engineControl();
void LEDshowProgram();
void RemoteControl (int RC_ADC);

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////






//void setPwmFrequency(int pin, int divisor);

//void getMotorData();
//void MotorRegulator2(double V_EMK_set);
//void printSerialData();
// void getMotorCurrent();
// void MotorRegulator(int current_set);
// void MotorRegulatorV(int V_motor_set);
// void MotorRegulatorEMK(int V_EMK_set);
//void engineControl(int newspeed, int dir);
//void engineControlRamp(int oldSpeed, int newSpeed);
//void pauseProgram();


/*
#if MOTOR_L9110
  #define MOTOR_PIN_DIR 12  //L9110: "IB"
  #define MOTOR_PIN_PWM 9   //L9110: "IA"
#endif

#if MOTOR_L293
  #define MOTOR_PIN_PWM   9  //L293: "1,2EN"
  #define MOTOR_PIN_DIR1  12   //L293: "1A"
  #define MOTOR_PIN_DIR2  11  //L293: "2A"
#endif
*/





///////////////////////////////////////////////////////////////////////////////
//motor MotorRegulator
/*
int V_supply = 6000; //supply voltage in mV

//double R_Motor = 19.4; //Rv of Motor in Ohm (measured one time)
double R_Motor = 25.0; //Rv of Motor in Ohm (measured one time)
int I_Motor = 0; //measurend motor current in mA
int V_Motor = 0; //measurend motor voltage in mV
int V_EMK = 0;


int V_Rsense = 0;
double R_Rsense = 17.0; //R_cs in Ohm


//constants
double MOTOR_CONST_Rs = 17.0; //Current sense resistor [Ohm]
double MOTOR_CONST_Rv = 19.4; //DC motor resistance [Ohm]
double MOTOR_CONST_km = 0.2078; //Drehmoment konstante [mNm/A]
double MOTOR_CONST_kn = 39.0921; //Drehzahlkonstante [min^-1/V]
  //ke Generatorkonstante [mV/min^-1]

//measured values
double MOTOR_MEAS_Vp = 0; //voltage at motor positive connection
double MOTOR_MEAS_Vn = 0; //voltage at motor negative connection
double MOTOR_MEAS_Vrs = 0; //voltage over current sense resistor

//calculated values
double MOTOR_CALC_Vm = 0; //voltage at motor connectors
double MOTOR_CALC_Im = 0; //current through motor
double MOTOR_CALC_Vemk = 0; //generator voltage

int MOTOR_CALC_n = 0; //Drehzahl / speed
double MOTOR_CALC_M = 0; //Drehmoment / torque
*/
