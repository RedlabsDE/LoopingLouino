/*
 * LoopingLoui v6
 *
 * Created: 17.07.2018
 *  Author: Julian Hauser
 *  www.redlabs.de
 */

/*
 * Board: "Arduino Pro or Pro Mini"
 * Prozessor: "ATmega328 5V 16MHz"
 * Programmer: "AVR ISP"
 */

 //redlabs PCB 013-00
 //Vin DC 7.5V / 12V (set correct SPEED_CORRECTION_FACTOR)

/*
v00
v01
v02
23.02.2018  v03   switch to Atom IDE
08.03.2018  v06   switch to MOTOR_L9110 and Arduino Mini Pro (013-xx breadboard)
17.03.2018  v07   WS2811 control corrected, new engineControl2()
19.03.2018  v08   tidy up code, add LEDcntrl, capacitor added at "raw" pin -> no resets at change of direction
20.03.2018  R1.0  decission: Vin=7.5V, ramp up/down improved
25.03.2018  v1.1  start to add analog RC-Control
26.03.2018  v1.2  RC-control
26.03.2018  R1.3  RC control implemented correct, tidy up code, [Osterwidum '18 release]

08.01.2019  v1.4  switch to PCB 013/00: pin mapping changed / use of MOTOR_HBRIDGE / ON/OFF switch in series with motor
18.11.2019        checkStartSwitch2 implemented correct
19.11.2019        LEDs control done / RC control done
21.11.2019 R1.6   Release (Felix&Nadja)
02.12.2019 R1.7   Release (Felix&Nadja), RC control adapted to new (black) RC control
09.06.2020 1.8    Change RC-pin to PC4/ADC4/RXD (ADC5 seems to be damaged), also changed pinning in blue RC-Control to mid-pin, TODO: correct control of 4 LEDs (instead of 11)
15.06.2020 R1.9   Running Version - Change to 12V supply/ control of 4 LEDs fixed / change random speed change time from 3 to 5 sec / add ADC lowpass filter to Remote control

12.03.2025        Add Comments & Tidy up. Use 7.5V supply voltage

  
TODO: extra program with LED show - or LED show after some time if motor is not running
*/

#define MOTOR_L9110   0 //obsolete
#define MOTOR_L293    0 //obsolete
#define MOTOR_HBRIDGE 1 //use discrete MOSFET H-Bridge

#define LOOPDELAY_MS      10
#define DELAY_CORRECTION  64 //TIMER0 is changed to set the motor PWM -> affects arduino delay() / workaround: delay( time_ms *DELAY_CORRECTION);

#define DEBUG 0

#include "main.h"
#include "LEDcntrl.h"

///////////////////////////////////////////////////////////////////////////////
//This function will be called when PROG Button is pressed
void ISRbtnPROG()
{
  sem_nextProgram = 1; //button press event will be processed in main loop
  delay(10*DELAY_CORRECTION);
}
///////////////////////////////////////////////////////////////////////////////
//setup
void setup()
{
  #if DEBUG
    Serial.begin(19200);
  #endif

//user button "program selection"
  pinMode(BTN_PIN_PROG, INPUT);
  digitalWrite(BTN_PIN_PROG, HIGH); //enable intern pullup
  attachInterrupt(digitalPinToInterrupt(BTN_PIN_PROG), ISRbtnPROG, FALLING);

//RC Control onboard pullup pin
  pinMode(RC_PULLUP_PIN, OUTPUT); //enable internal pullup
  digitalWrite(RC_PULLUP_PIN, HIGH); //enable intern pullup

//WS2811 OUTPUT
  pinMode(WS2811_PIN, OUTPUT);

//Motor control
  #if MOTOR_L9110
    pinMode(MOTOR_PIN_DIR, OUTPUT);
    pinMode(MOTOR_PIN_PWM, OUTPUT);
    digitalWrite(MOTOR_PIN_DIR, LOW);
    digitalWrite(MOTOR_PIN_PWM, LOW);
  #endif

  #if MOTOR_HBRIDGE
    pinMode(MOTOR_PIN_GateP_Neg, OUTPUT);
    pinMode(MOTOR_PIN_GateP_Pos, OUTPUT);
    pinMode(MOTOR_PIN_GateN_Pos, OUTPUT); //PWM
    pinMode(MOTOR_PIN_GateN_Neg, OUTPUT); //PWM

    digitalWrite(MOTOR_PIN_GateP_Neg, LOW);
    digitalWrite(MOTOR_PIN_GateP_Pos, LOW);
    digitalWrite(MOTOR_PIN_GateN_Pos, LOW);
    digitalWrite(MOTOR_PIN_GateN_Neg, LOW);

  #endif

  //Set Motor PWM frequency, (default: 490 Hz), //this changes the delay() time!!!!
  setPwmFrequency(MOTOR_PIN_GateN_Pos,1); //set PWM freq to 32kHz (31250/1 Hz)
  setPwmFrequency(MOTOR_PIN_GateN_Neg,1); //set PWM freq to 32kHz (31250/1 Hz)

//watchdog enable
  //wdt_enable(WDTO_2S);
  //wdt_reset();
  //wdt_disable();

//WS2812 start:
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.clear();
  rainbow(5*DELAY_CORRECTION);  //start up LED show

  #if DEBUG
    Serial.println("xxxxxxxxxxxxxxxxxxxxxxx");
    Serial.println("redlabs Looping Loui V6");
    Serial.println("DEBUG enabled");
  #endif

  sem_nextProgram = 0;
  sem_starStop = 1;
}

///////////////////////////////////////////////////////////////////////////////
//main loop
void loop()
{
  ///////////////////////////////////////////////////////////////////////////////
  //main loop count (dt: 10ms)
  loopCount++;
  if(loopCount>1000) //start again at 0 after ~10 sec
  {
    loopCount = 0;
    #if DEBUG
      debugDiagnose();
    #endif
  }
  //wdt_reset();

  ///////////////////////////////////////////////////////////////////////////////
  //if "PROG" button was pressed: switch to next program
  if(sem_nextProgram)
    incrementProgram();
  ///////////////////////////////////////////////////////////////////////////////
  //Check status of main LoopingLoui switch & set stat_current to RUN/PAUSE
  checkStartSwitch();
  ///////////////////////////////////////////////////////////////////////////////
  //execute RUN / PAUSE mode
  if(stat_current == STAT_RUN) //while stauts = RUN, execute current program
  {
    executeProgram();
  }
  else if(stat_current == STAT_PAUSE) //if status = "PAUSE", pause current program
  {
    MOTOR_speed_new = 0;
    MOTOR_direction_new = STOP;
  }
  //////////////////////////////////////////////////////////////////////////////
  //set motor speed & direction
  engineControl();
  //show current program/state with LEDs
  LEDshowProgram();
  //main loop delay
  delay(LOOPDELAY_MS*DELAY_CORRECTION); //10ms loop
}


///////////////////////////////////////////////////////////////////////////////
//if PROGRAM button is pushed, go to next program
void incrementProgram()
{
  prog_current++; //increment progam number

  if(prog_current>MAX_PROGRAM) //if max program reached, go to program 1
  {
    prog_current=1;
    MOTOR_speed = MINSPEED; //set speed to low value, to start again the "PROG_SPEEDUP"
  }

  #if DEBUG
    Serial.println("next program selected: ");
    Serial.print(prog_current);
  #endif

  delay(300*DELAY_CORRECTION); //debounce
  sem_nextProgram = 0; //reset the button press semaphor
}


///////////////////////////////////////////////////////////////////////////////
//check status of main LoopinLoui I/O switch and set status to RUN/PAUSE
// I/O switch is in series with motor
/*
// Start switch decission table:
if (Motor == stopped) set "GateP-Pos" active
  switch off: ADC-Neg = 0V
  switch on:  ADC-Neg = VCOM
if (Motor == running)
  if (direction == forward) GateP-Pos is active
    switch off: ADC-Neg = 0V
    switch on: ADC-Neg > 0V
  if (direction == backward) GateP-Neg is active
    switch off: ADC-Pos = 0V
    switch on: ADC-Pos > 0V
*/
void checkStartSwitch()
{
  static char switchIsOn = 0;
  ///////////////////////////////////////////////////////////////////////////////
  //get analog inputs - voltage at motor connection or motor current
  Motor_Neg_ADC = analogRead(ADC_PIN_MOTOR_NEG);
  Motor_Pos_ADC = analogRead(ADC_PIN_MOTOR_POS);

  if(currentSenseDeactivate) //motor current sense is deactivated for a short time when the switch is turned on - to give the motor current time to rise
  {
    Motor_Rs_ADC_FILTER = MOTOR_CURRENT_ADC_OFF_LEVEL + 1;
    currentSenseDeactivate--;
  }
  else
  {
    Motor_Rs_ADC  = analogRead(ADC_PIN_MOTOR_Rs); //measurement of motor current over shunt resistor
    Motor_Rs_ADC_FILTER = lowPassAveraging_Rs(Motor_Rs_ADC); //apply filter
  }

  ////////////////////////////////////////////////////
  //check voltage at motor connection depending on run mode
  if(MOTOR_direction == STOP)
  {
    if(Motor_Neg_ADC < MOTOR_VOLTAGE_ADC_OFF_LEVEL) //switch == OFF
    {
      switchIsOn = 0;
    }
    else //switch == just turned ON
    {
      switchIsOn = 1;
      currentSenseDeactivate = CURRENT_SENSE_DEACTIVATE_TIME;
    }
  }
  else if ( (MOTOR_direction == FWD) || (MOTOR_direction == RWD) )
  {
    if(Motor_Rs_ADC_FILTER < MOTOR_CURRENT_ADC_OFF_LEVEL) //no motor current -> switch == OFF
    {
      switchIsOn = 0;
    }
    else //switch = ON
    {
      switchIsOn = 1;
    }
  }
  ////////////////////////////////////////////////////
  //set status according to switch position
  if(switchIsOn == 0 ) //switch == OFF
  {
    stat_current = STAT_PAUSE;
    if(sem_starStop == 1)
    {
      #if DEBUG
        Serial.println("status: pause");
      #endif
    }
    sem_starStop = 0;
  }
  else //switch == ON
  {
    stat_current = STAT_RUN;
    if(sem_starStop == 0)
    {
      #if DEBUG
        Serial.println("status: run");
      #endif
    }
    sem_starStop = 1;
  }

}

///////////////////////////////////////////////////////////////////////////////
//goto selected program
void executeProgram()
{
  switch (prog_current)
  {
    ////////////////////////////////////////////////////
    //normal motor speed,chaser lights
    case PROG_NORMAL:

        MOTOR_speed_new = NORMSPEED;
        MOTOR_direction_new = FWD;
    break;
    ////////////////////////////////////////////////////
    //incresase motor speed with time
    case PROG_SPEEDUP:

        if(MOTOR_speed_new < MINSPEED)
          MOTOR_speed_new = MINSPEED;

        if(MOTOR_speed_new < MAXSPEED) //while max speed is not reached, increment speed
        {
          if(!(loopCount % (500/LOOPDELAY_MS))) //increase speed every 500 ms
          {
            #if DEBUG
              Serial.println("speed up");
            #endif
            MOTOR_speed_new += SPEEDUP;
            MOTOR_direction_new = FWD;
          }
        }
    break;
    ////////////////////////////////////////////////////
    //random change of speed and direction - redlabs best choice :)
    case PROG_CRACY:

        if(MOTOR_speed_new < MINSPEED)
        {
          MOTOR_speed_new = MINSPEED;
          MOTOR_direction_new = FWD;
        }

        if(!(loopCount %  (5000/LOOPDELAY_MS) )) //change speed/direction every 5 seconds
        {
          unsigned char direction = 0;

          MOTOR_speed_new = random(MINSPEED,MAXSPEED);
          //MOTOR_speed_new = rand()%175+70; //get random speed value in range (70:255)
          direction = random(1,100);
          //direction = rand()%100+1; //get random direction value in range (1:100)

          if(direction>15) //85% chance to go forward
            MOTOR_direction_new = FWD;
          else
            MOTOR_direction_new = RWD;
        }
    break;
    ////////////////////////////////////////////////////
    //go forward with max speed
    case PROG_MAXSPEED:

        MOTOR_direction_new = FWD;
        MOTOR_speed_new = MAXSPEED;
    break;
    ////////////////////////////////////////////////////
    //use analog input as remote control
    case PROG_RC:

        if(!(loopCount %  (50/LOOPDELAY_MS) )) //read remote control every 50 milli seconds
        {
          //RC_ADC = analogRead(ADC_PIN_RC);
          RC_ADC = lowPassAveraging_RC(analogRead(ADC_PIN_RC)); //apply filter

          #if DEBUG
            Serial.println("ADC: ");
            Serial.print(RC_ADC);
          #endif
          RemoteControl (RC_ADC);
        }
    break;
    ////////////////////////////////////////////////////
    //should not appear
    default:
      //trigger watchdog to reset controller
    break;
  }
}

///////////////////////////////////////////////////////////////////////////////
//set motor speed / direction
void engineControl()
{
  if(MOTOR_direction != MOTOR_direction_new) //ramp speed to 0, change direction, ramp speed up
  {
    //change of direction from FWD or RWD
    if((MOTOR_direction==FWD)||(MOTOR_direction==RWD))
    {
      MOTOR_speed -= 5; //ramp down speed

      if(MOTOR_speed<10) //if speed is low enough to stop, change direction
      {
        MOTOR_direction = MOTOR_direction_new;
        currentSenseDeactivate = CURRENT_SENSE_DEACTIVATE_TIME;
      }
      #if DEBUG
        Serial.print("x");
      #endif
    }
    else //change from stop to other direction
    {
      MOTOR_direction = MOTOR_direction_new;
    }
  }
  else if(MOTOR_speed != MOTOR_speed_new) //change in motor speed, without change of direction
  {
    if(MOTOR_speed > MOTOR_speed_new+1) //if actual speed is higher than new speed
    {
      MOTOR_speed -= 2; //ramp down speed

      #if DEBUG
        Serial.print("<");
      #endif
    }
    else if(MOTOR_speed < MOTOR_speed_new-1) //if actual speed is lower than new speed
    {
      MOTOR_speed += 2; //ramp up speed
      #if DEBUG
        Serial.print(">");
      #endif
    }
  }
  else //no change
  {
    MOTOR_speed = MOTOR_speed_new; //just set direction again
  }

  //output speed and direction to motor driver (use one dirction GPIO and one PWM)
  #if MOTOR_L9110
    if(MOTOR_direction == FWD)
    {
      digitalWrite(MOTOR_PIN_DIR, LOW);
      analogWrite(MOTOR_PIN_PWM,MOTOR_speed); //PWM output to motor driver
    }
    else if (MOTOR_direction == RWD)
    {
      digitalWrite(MOTOR_PIN_DIR, HIGH);
      analogWrite(MOTOR_PIN_PWM,255-MOTOR_speed); //PWM inverse output to motor driver
    }
    else
    {
      digitalWrite(MOTOR_PIN_DIR, LOW);
      analogWrite(MOTOR_PIN_PWM,0); //PWM output to motor driver
    }
  #endif

  //output speed and direction to H-Bridge (use two GPIOs and two PWMs)
  #if MOTOR_HBRIDGE
    if(MOTOR_direction == FWD)
    {
      digitalWrite(MOTOR_PIN_GateP_Pos, HIGH); //set Motor-Pos to supply voltage
      digitalWrite(MOTOR_PIN_GateP_Neg, LOW); //disconnect Motor-Neg from supply voltage
      analogWrite(MOTOR_PIN_GateN_Neg,MOTOR_speed); //PWM output to low-side switch on Motor-Neg
      analogWrite(MOTOR_PIN_GateN_Pos,0); //stop PWM output to low-side switch on Motor-Pos
    }
    else if (MOTOR_direction == RWD)
    {
      digitalWrite(MOTOR_PIN_GateP_Neg, HIGH); //set Motor-Neg to supply voltage
      digitalWrite(MOTOR_PIN_GateP_Pos, LOW); //disconnect Motor-Pos from supply voltage
      analogWrite(MOTOR_PIN_GateN_Pos,MOTOR_speed); //PWM output to low-side switch on Motor-Pos
      analogWrite(MOTOR_PIN_GateN_Neg,0); //stop PWM output to low-side switch on Motor-Neg
    }
    else //motor stopped
    {
      digitalWrite(MOTOR_PIN_GateP_Pos, HIGH); //set Motor-Pos to supply voltage  <--- used for start/stop detection
      digitalWrite(MOTOR_PIN_GateP_Neg, LOW); //disconnect Motor-Neg from supply voltage
      analogWrite(MOTOR_PIN_GateN_Neg,0); //stop PWM output to low-side switch on Motor-Neg
      analogWrite(MOTOR_PIN_GateN_Pos,0); //stop PWM output to low-side switch on Motor-Pos
    }
  #endif

}

///////////////////////////////////////////////////////////////////////////////
//if program is in pause mode, the program is shown as color coded (static color)
void LEDshowProgram()
{
  static int LEDcount = 0;    //0-99
  static int LEDsection = 0;  //0,1,2,3

  //LEDsection = LEDcount/9; //get led number 0-10 from LEDcount 0-99 (11 LEDs)
  LEDsection = LEDcount/25; //get led number 0-3 from LEDcount 0-99 (4 LEDs)

  if(stat_current == STAT_RUN)
  {
    strip.clear();

    switch(prog_current)
    {
      case PROG_NORMAL:  //green
          WS28xxSetPixelColor(LEDsection, COLOR_NORMAL);
      break;

      case PROG_SPEEDUP: //purple
          WS28xxSetPixelColor(LEDsection, COLOR_SPEEDUP);
      break;

      case PROG_CRACY: //orange
        WS28xxSetPixelColor(LEDsection, COLOR_CRAZY);
      break;

      case PROG_MAXSPEED: //red
        WS28xxSetPixelColor(LEDsection, COLOR_MAXSPEED);
      break;

      case PROG_RC: //blue
        if(MOTOR_direction == FWD || MOTOR_direction == RWD)
          WS28xxSetPixelColor(LEDsection, COLOR_RC);
        else
          WS28xxSetColor(COLOR_RC);
      break;
    }
  }
  else if(stat_current == STAT_PAUSE)
  {

    switch(prog_current)
    {
      case PROG_NORMAL:  //green
        WS28xxSetColor(COLOR_NORMAL);
      break;

      case PROG_SPEEDUP: //purple
        WS28xxSetColor(COLOR_SPEEDUP);
      break;

      case PROG_CRACY: //orange
        WS28xxSetColor(COLOR_CRAZY);
      break;

      case PROG_MAXSPEED: //red
        WS28xxSetColor(COLOR_MAXSPEED);
      break;

      case PROG_RC: //blue
        WS28xxSetColor(COLOR_RC);
      break;
    }
  }

  if(MOTOR_direction == FWD)
  {
    LEDcount ++;
    if(LEDcount > 99)
      LEDcount = 0;
  }
  else if(MOTOR_direction == RWD)
  {
    if(LEDcount == 0)
      LEDcount = 99;

    LEDcount --;
  }
}

///////////////////////////////////////////////////////////////////////////////
//set direction+speed according to RC (analog) input
void RemoteControl (int adc_val)
{
  static double RC_speedfactor = 0;

  if(  (adc_val > (ADC_low-ADC_hyst2) ) && (adc_val < (ADC_mid - ADC_hyst)) ) //RWD
  {
    RC_speedfactor = 1.0 -  (1.0 *adc_val - ADC_low + ADC_hyst) / (1.0*ADC_mid - ADC_hyst - ADC_low);
    MOTOR_direction_new = RWD;
    #if DEBUG
      Serial.println("RWD");
    #endif
  }
  else if( (adc_val < ADC_max+ADC_hyst2) && (adc_val > (ADC_mid + ADC_hyst))  ) //FWD
  {
    RC_speedfactor = (1.0*adc_val - ADC_mid - ADC_hyst) / (1.0* ADC_max - ADC_mid - ADC_hyst);
    MOTOR_direction_new = FWD;
    #if DEBUG
      Serial.println("FWD");
    #endif
  }
  else if( (adc_val >= (ADC_mid-ADC_hyst)) && (adc_val <= (ADC_mid+ADC_hyst) ) ) //Stop (middle position)
  {
    #if DEBUG
      Serial.println("STOP");
    #endif
    MOTOR_direction_new = STOP;
    RC_speedfactor = 0;
  }
  else //fault, RC disconnect
  {
    #if DEBUG
      Serial.println("FAULT");
    #endif
    MOTOR_direction_new = STOP;
    RC_speedfactor = 0;
  }

  if(RC_speedfactor > 1.0)
  {
    RC_speedfactor = 1.0;
    #if DEBUG
      Serial.println("lim max");
    #endif
  }
  else if (RC_speedfactor < 0)
  {
    RC_speedfactor = 0;
    #if DEBUG
      Serial.println("lim min");
    #endif
  }

  MOTOR_speed_new = ( RC_speedfactor * (MAXSPEED-ABSMINSPEED) + ABSMINSPEED );
  #if DEBUG
    Serial.println("speed: ");
    Serial.print(MOTOR_speed_new);
  #endif
}

int lowPassAveraging_Rs(int input)
{
    int sum = 0;
    for(int i = FILTER_LP_POINTS - 1; i > 0; --i)
    {
        xRs[i] = xRs[i-1];
        sum += xRs[i];
    }
    sum += input;
    xRs[0] = input;
    return sum/FILTER_LP_POINTS;
}

int lowPassAveraging_RC(int input)
{
    int sum = 0;
    for(int i = FILTER_LP_POINTS - 1; i > 0; --i)
    {
        xRC[i] = xRC[i-1];
        sum += xRC[i];
    }
    sum += input;
    xRC[0] = input;
    return sum/FILTER_LP_POINTS;
}


void setPwmFrequency(int pin, int divisor)
{
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10)
  {
    switch(divisor)
    {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6)
    {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else
    {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}


void debugDiagnose()
{
  Serial.println("\n------------ debugDiagnose() ------------");
  Serial.print("  stat: ");
  Serial.print(stat_current);
  Serial.print("  prog: ");
  Serial.print(prog_current);
  Serial.print("  M dir: ");
  Serial.print(MOTOR_direction);
  Serial.print("  M speed: ");
  Serial.print(MOTOR_speed);
  Serial.print("  M speed new: ");
  Serial.print(MOTOR_speed_new);
  Serial.print("  Motor_Neg_ADC: ");
  Serial.print(Motor_Neg_ADC);
  Serial.print("  Motor_Pos_ADC: ");
  Serial.print(Motor_Pos_ADC);
  Serial.print("  Rs_ADC: ");
  Serial.print(Motor_Rs_ADC);
  Serial.print("  Rs_ADC_filterd: ");
  Serial.print(Motor_Rs_ADC_FILTER);
}



/*
void RemoteControl (int adc_val)
{
  static double RC_speedfactor = 0;

  if(  (adc_val > (ADC_low-ADC_hyst2) ) && (adc_val < (ADC_mid - ADC_hyst)) ) //RWD
  {
    RC_speedfactor = 1.0 -  (1.0 *adc_val - ADC_low + ADC_hyst) / (1.0*ADC_mid - ADC_hyst - ADC_low);
    MOTOR_direction_new = RWD;
    Serial.println("RWD");
  }
  else if( (adc_val < ADC_max+ADC_hyst2) && (adc_val > (ADC_mid + ADC_hyst))  ) //FWD
  {
    RC_speedfactor = (1.0*adc_val - ADC_mid - ADC_hyst) / (1.0* ADC_max - ADC_mid - ADC_hyst);
    MOTOR_direction_new = FWD;
    Serial.println("FWD");
  }
  else if( (adc_val >= (ADC_mid-ADC_hyst)) && (adc_val <= (ADC_mid+ADC_hyst) ) ) //Stop (middle position)
  {
    Serial.println("STOP");
    MOTOR_direction_new = STOP;
    RC_speedfactor = 0;
  }
  else //fault, RC disconnect
  {
    Serial.println("FAULT");
    MOTOR_direction_new = STOP;
    RC_speedfactor = 0;
  }

  if(RC_speedfactor > 1.0)
  {
    RC_speedfactor = 1.0;
    Serial.println("lim max");
  }
  else if (RC_speedfactor < 0)
  {
    RC_speedfactor = 0;
    Serial.println("lim min");
  }

  MOTOR_speed_new = ( RC_speedfactor * (MAXSPEED-ABSMINSPEED) + ABSMINSPEED );
  Serial.println("speed: ");
  Serial.print(MOTOR_speed_new);
}
*/


///////////////////////////////////////////////////////////////////////////////
/////      ALTERNATIVE FUNCTIONS FOR DIFFERENT HARDWARE SETTING     ///////////
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
//check status of main LoopinLoui I/O switch and set status to RUN/PAUSE
// I/O switch is seperated from motor, use of extra GPIO
/*
void checkStartSwitch()
{
  if(digitalRead(BTN_PIN_START))  //switch is in position "OFF"
  {
    stat_current = STAT_PAUSE;
    if(sem_starStop == 1)
    {
      #if DEBUG
        Serial.println("status: pause");
      #endif
    }
    sem_starStop = 0;
  }
  else  //switch is in position "ON"
  {
    stat_current = STAT_RUN;
    if(sem_starStop == 0)
    {
      #if DEBUG
        Serial.println("status: run");
      #endif
    }
    sem_starStop = 1;
  }
}
*/


///////////////////////////////////////////////////////////////////////////////
///////////////////////        OLD STUFF           ////////////////////////////
///////////////////////////////////////////////////////////////////////////////




////////////////////////// OLD STUFF //////////////////////////////////

//static int Motor_Rs_ADC_FILTER = 50;//DEBUG
////////////////////////////////////////////////////
//get voltage at motor connection
/*
Motor_Neg_ADC = analogRead(ADC_PIN_MOTOR_NEG);
Motor_Pos_ADC = analogRead(ADC_PIN_MOTOR_POS);
Motor_Rs_ADC  = analogRead(ADC_PIN_MOTOR_Rs);
*/
//Motor_Rs_ADC_FILTER = (Motor_Rs_ADC_FILTER +  Motor_Rs_ADC)/2;
//Motor_Rs_ADC_FILTER = lowPassAveraging(Motor_Rs_ADC);

/*
  ///////////////////////////////////////////////////////////////////////////////
  //get analog inputs
  Motor_Neg_ADC = analogRead(ADC_PIN_MOTOR_NEG);
  Motor_Pos_ADC = analogRead(ADC_PIN_MOTOR_POS);

  if(currentSenseDeactivate)
  {
    Motor_Rs_ADC_FILTER = MOTOR_CURRENT_ADC_OFF_LEVEL + 1;
    currentSenseDeactivate--;
  }
  else
  {
    Motor_Rs_ADC  = analogRead(ADC_PIN_MOTOR_Rs);
    Motor_Rs_ADC_FILTER = lowPassAveraging_Rs(Motor_Rs_ADC); //apply filter
  }
*/

//DEBUG loop
/*
void loop()
{
  loopCount++;
  if(loopCount>100) //start again at 0 after 1 sec
  {
    loopCount = 0;
    debugDiagnose();

    digitalWrite(MOTOR_PIN_GateP_Pos, HIGH); //set Motor-Pos to supply voltage
    digitalWrite(MOTOR_PIN_GateP_Neg, LOW); //disconnect Motor-Neg from supply voltage

    analogWrite(MOTOR_PIN_GateN_Neg,70); //PWM output to low-side switch on Motor-Neg
    analogWrite(MOTOR_PIN_GateN_Pos,0); //stop PWM output to low-side switch on Motor-Pos
  }

  static int Motor_Rs_ADC_FILTER = 0;//DEBUG
  ////////////////////////////////////////////////////
  //get voltage at motor connection
  Motor_Rs_ADC  = analogRead(ADC_PIN_MOTOR_Rs);

  //Motor_Rs_ADC_FILTER = (Motor_Rs_ADC_FILTER +  Motor_Rs_ADC)/2;
  Motor_Rs_ADC_FILTER = lowPassAveraging(Motor_Rs_ADC);

  Serial.println(" data ");
  Serial.print(" rs ");
  Serial.print(Motor_Rs_ADC);
  Serial.print(" filt: ");
  Serial.print(Motor_Rs_ADC_FILTER);


  //main loop delay
  delay(LOOPDELAY_MS); //10ms loop

}
*/


/*
void getMotorData()
{
  double V_Rsense_ADC = 0;
  double V_Motor_pos_ADC = 0;
  double V_Motor_neg_ADC = 0;

  digitalWrite(4, HIGH);
  //ca 120us pro ADC Messung
  for(int i=0;i<10;i++)
  {
    V_Rsense_ADC    += analogRead(ADC_PIN_MOTOR_Rs);
    V_Motor_pos_ADC += analogRead(ADC_PIN_MOTOR_VM_P);
    V_Motor_neg_ADC += analogRead(ADC_PIN_MOTOR_VM_N);
  }
  digitalWrite(4, LOW);

  V_Rsense_ADC    /= 10;
  V_Motor_pos_ADC /= 10;
  V_Motor_neg_ADC /= 10;

  double V_ref = 4700.0; //logic supply and ADC ref mV

  MOTOR_MEAS_Vrs = V_Rsense_ADC*(V_ref/1024);
  MOTOR_MEAS_Vp = V_Motor_pos_ADC * (V_ref/1024*2.013);
  MOTOR_MEAS_Vn = V_Motor_neg_ADC * (V_ref/1024*2.031);

  MOTOR_CALC_Im = (MOTOR_MEAS_Vrs / MOTOR_CONST_Rs);
  MOTOR_CALC_Vm = MOTOR_MEAS_Vp - MOTOR_MEAS_Vn;
  MOTOR_CALC_Vemk = MOTOR_CALC_Vm - (MOTOR_CALC_Im*MOTOR_CONST_Rv);

  // if(MOTOR_CALC_Vemk < 50)
  // {
  //   MOTOR_CALC_Vemk = 0
  // }
  // else
  // {
  //   MOTOR_CALC_Vemk -= 15;
  // }


  MOTOR_CALC_n = MOTOR_CONST_kn * MOTOR_CALC_Vemk/1000;
  MOTOR_CALC_M = MOTOR_CALC_Im * MOTOR_CONST_km;

}

void MotorRegulator2(double V_EMK_set)
{
  int hyst = 50;

  if((V_EMK_set > (MOTOR_CALC_Vemk + hyst)) && (MOTOR_speed<255))
  {
    MOTOR_speed++;
    //Serial.print("+");
  }
  else if ((V_EMK_set < (MOTOR_CALC_Vemk-hyst)) && (MOTOR_speed>0))
  {
    MOTOR_speed--;
    //Serial.println("-");
  }
  else
  {
    //Serial.println("motor set current reached");
  }
}
*/


/*
void getMotorCurrent()
{

  double V_Rsense_ADC = 0;

  for(int i=0;i<10;i++)
  {
    V_Rsense_ADC += analogRead(ADC_PIN_MOTOR_Rs);
  }

  V_Rsense_ADC /= 10;
  V_Rsense = V_Rsense_ADC*4.88;

  I_Motor = (V_Rsense / R_Rsense);
  V_Motor = V_supply - V_Rsense;

  V_EMK = V_supply - V_Rsense - (I_Motor * R_Motor);


  // Serial.print("V: ");
  // Serial.print(V_Rsense);
  // Serial.print("I: ");
  // Serial.println(I_Motor);



}

void MotorRegulator(int current_set)
{
  int hyst = 3;

  if(I_Motor < (current_set -hyst) )
  {
    MOTOR_speed++;
    Serial.println("speed ++ ");
  }
  else if (I_Motor > (current_set + hyst))
  {
    MOTOR_speed--;
    Serial.println("speed -- ");
  }
  else
  {
    //Serial.println("motor set current reached");
  }

}

void MotorRegulatorV(int V_motor_set)
{
  int hyst = 100;

  if(V_Motor < (V_motor_set -hyst) )
  {
    MOTOR_speed--;
    Serial.print("-");
  }
  else if (V_Motor > (V_motor_set + hyst))
  {
    MOTOR_speed++;
    Serial.println("+");
  }
  else
  {
    //Serial.println("motor set current reached");
  }

}

void MotorRegulatorEMK(int V_EMK_set)
{
  int hyst = 50;

  if((V_EMK_set < V_EMK) && (MOTOR_speed<255))
  {
    MOTOR_speed++;
    //Serial.print("+");
  }
  else if ((V_EMK_set > V_EMK) && (MOTOR_speed>0))
  {
    MOTOR_speed--;
    //Serial.println("-");
  }
  else
  {
    //Serial.println("motor set current reached");
  }

}
*/





/*
void loop() //DEBUG
{

  Serial.println("Hallo");
  delay(200);
  digitalWrite(4, LOW);
  //PORTB |= (1<<7); //set PB7 high
  SET_PIN_HIGH(PORTB,7);
  delay(200);
  digitalWrite(4, HIGH);
  //PORTB &= ~(1<<7); //set PB7 low
  SET_PIN_LOW(PORTB,7);

}
*/

/*
//debug
//int prog_last = 0;
//int currentSet = 0;
//double EMK_set = 0;
void loop()
{
  loopCount++;
  if(loopCount>1000) //start again at 0 after 10 sec
    loopCount = 0;

  ///////////////////////////////////////////////////////////////////////////////
  //if "PROG" button was pressed: switch to next program
  if(sem_nextProgram)
    incrementProgram();

    if(prog_current != prog_last)
    {
      //currentSet = (prog_current-1)*1000; //max= 5*50=250
      EMK_set = ( prog_current-1)*500;

      //MOTOR_speed = (prog_current-1)*28; //max= 5*50=250

      prog_last = prog_current;
    }


    getMotorData();
    //MotorRegulator(currentSet);
    //MotorRegulatorV(5300);

    //MotorRegulatorEMK(EMK_set);

    //MOTOR_speed=165;
    analogWrite(MOTOR_PIN_PWM,MOTOR_speed); //PWM output to motor driver

   MotorRegulator2(EMK_set);

    if(!(loopCount % 100))
    {
      printSerialData();
    }

    delay(10);
}
*/




/**
 * Divides a given PWM pin frequency by a divisor.
 *
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 *
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 *
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 *
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://forum.arduino.cc/index.php?topic=16612#msg121031
 */





/*
void printSerialData()
{
  Serial.print("Vp: ");
  Serial.print(MOTOR_MEAS_Vp);
  Serial.print(" Im: ");
  Serial.print(MOTOR_CALC_Im);
  Serial.print("  Vm: ");
  Serial.print(MOTOR_CALC_Vm);
  Serial.print("  Vemk: ");
  Serial.print(MOTOR_CALC_Vemk);
  Serial.print("  n: ");
  Serial.print(MOTOR_CALC_n);
  Serial.print("  PWM: ");
  Serial.println(MOTOR_speed);

  // double MOTOR_MEAS_Vp = 0; //voltage at motor positive connection
  // double MOTOR_MEAS_Vn = 0; //voltage at motor negative connection
  // double MOTOR_MEAS_Vrs = 0; //voltage over current sense resistor
  //
  // //calculated values
  // double MOTOR_CALC_Vm = 0; //voltage at motor connectors
  // double MOTOR_CALC_Im = 0; //current through motor
  // double MOTOR_CALC_Vemk = 0; //generator voltage
  //
  // int MOTOR_CALC_n = 0; //Drehzahl / speed
  // double MOTOR_CALC_M = 0; //Drehmoment / torque
}
*/





///////////////////////////////////////////////////////////////////////////////
//if "START" button was pressed: pause/continue program
/*
if(sem_starStop) //STAR/STOP Button was pressed
{
  //toggle stat_current to stop or continue program
  if(stat_current == STAT_RUN)
  {
    stat_current = STAT_PAUSE;
    MOTOR_speed = MINSPEED;
    #if DEBUG
      Serial.println("status: pause");
    #endif
  }
  else if (stat_current == STAT_PAUSE)
  {
    stat_current = STAT_RUN;
    colorWipe(strip.Color(0, 0, 255), 50); // set all LEDs Red
    #if DEBUG
      Serial.println("status: run");
    #endif
  }

  delay(200);
  sem_starStop = 0; //reset the button press semaphor
}
*/

//setup
/*
#if MOTOR_L293
  pinMode(MOTOR_PIN_DIR1, OUTPUT);
  pinMode(MOTOR_PIN_DIR2, OUTPUT);
  pinMode(MOTOR_PIN_PWM, OUTPUT);

  digitalWrite(MOTOR_PIN_DIR1, LOW);
  digitalWrite(MOTOR_PIN_DIR2, LOW);
  digitalWrite(MOTOR_PIN_PWM, LOW);
#endif

*/


///////////////////////////////////////////////////////////////////////////////
//set motor speed / direction
/*
void engineControl(int newspeed, int dir)
{
  static int old_speed = MINSPEED;
  static int old_dir = 0;

  if(dir != old_dir) //direction changed (or direction and speed changed)
  {
    //change of direction: ramp speed donw -> change direction -> ramp speed up
    engineControlRamp(old_speed, 0); //ramp speed down to zero

    switch (dir)
    {
      case FWD:
        #if MOTOR_L293
          digitalWrite(MOTOR_PIN_DIR1, LOW);
          digitalWrite(MOTOR_PIN_DIR2, HIGH);
        #endif
        #if MOTOR_L9110
          digitalWrite(MOTOR_PIN_DIR, LOW);
        #endif
        engineControlRamp(MINSPEED, newspeed); //ramp speed up from zero to current value
      break;
      case RWD:
        #if MOTOR_L293
          digitalWrite(MOTOR_PIN_DIR1, HIGH);
          digitalWrite(MOTOR_PIN_DIR2, LOW);
        #endif
        #if MOTOR_L9110
          digitalWrite(MOTOR_PIN_DIR, HIGH);
        #endif
        engineControlRamp(MINSPEED, newspeed); //ramp speed up from zero to current value
      break;
      case BREAK:
        #if MOTOR_L293
          digitalWrite(MOTOR_PIN_DIR1, LOW);
          digitalWrite(MOTOR_PIN_DIR2, LOW);
          analogWrite(MOTOR_PIN_PWM,255);
        #endif
        #if MOTOR_L9110
          digitalWrite(MOTOR_PIN_DIR, LOW);
          analogWrite(MOTOR_PIN_PWM,0);
        #endif
      break;
      case FREE:
        #if MOTOR_L293
          analogWrite(MOTOR_PIN_PWM,0);
        #endif
        #if MOTOR_L9110 //freewheel is not possible with L9110: break
          digitalWrite(MOTOR_PIN_DIR, LOW);
          analogWrite(MOTOR_PIN_PWM,0);
        #endif
      break;

    }

  }
  else if (MOTOR_speed_new != MOTOR_speed) //same direction, speed changed
  {
    engineControlRamp(old_speed,newspeed);
  }

  old_speed = newspeed;
  old_dir = dir;
}
*/

///////////////////////////////////////////////////////////////////////////////
//ramp speed from current to new speed to protect the mechanics
/*
void engineControlRamp(int oldSpeed, int newSpeed)
{
    #if DEBUG
      Serial.println("ramp");
    #endif

  int currentSpeed = oldSpeed;

  //increment / decrement speed
  while (newSpeed != currentSpeed)
  {

    if(newSpeed > currentSpeed)
    {
      currentSpeed += SPEEDUP;
      #if DEBUG
        Serial.print(">");
      #endif
    }
    else if(newSpeed < currentSpeed)
    {
      currentSpeed -= SPEEDUP;
     #if DEBUG
      Serial.print("<");
    #endif
    }

    #if MOTOR_L9110
      if(MOTOR_direction == RWD)
      {
        analogWrite(MOTOR_PIN_PWM,255-currentSpeed); //PWM inverse output to motor driver
      }
      else
      {
        analogWrite(MOTOR_PIN_PWM,currentSpeed); //PWM inverse output to motor driver
      }
    #endif
    #if MOTOR_L293
      analogWrite(MOTOR_PIN_PWM,currentSpeed); //PWM output to motor driver
    #endif
    delay(10);
  }

}
*/

///////////////////////////////////////////////////////////////////////////////
//stop motor and show selected program
/*
void pauseProgram()
{
  //engineControl(MINSPEED,FREE);
  //LEDshowProgram();
}
*/
///////////////////////////////////////////////////////////////////////////////
/** This function will be called when "START/STOP" Button is pressed */
/*
void ISRbtnSTART()
{
  //diable interrupts
  sem_starStop = 1;
  delay(1);
  //enable interrupts
}
*/

///////////////////////////////////////////////////////////////////////////////
