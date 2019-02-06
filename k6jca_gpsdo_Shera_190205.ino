//-----------------------------------------------------
//  Arduino GPSDO PLL Controller based upon 
//  Brooks Shera's GPSDO Algorithm
//
//  Author:  Jeff Anderson, K6JCA
//  
//  To echo Brooks Shera (in his PIC code):
//    "This program may be used only for non-commercial
//     purposes, and carries no warranty of any kind, 
//     including any implied warranty of fitness for
//     any particular purpose."
//  
//
//  Base Rev:   160801a
//    o  FLL-based GPSDO
//
//  Rev: 181120
//    o  Modify base rev for PLL operation.
//       o  Delete reception of SV serial data.
//          Move uart RX pin to an unused NANO pin.
//    o  Delete accumulated_error stuff.  Never used.
//
//  Rev: 181215
//    o  Changed phase delta period to 400 ns
//       (from 200). 
//
//  Rev 190101
//    o  Start incorportating Shera into my original 
//       Frequency-locked loop design.
//    o  External attenuation has not yet been added,
//       So compensate with internal (pre-DAC)
//       attenuation:
//       (If N = 4 (2.5 MHz clk), set internal atten to
//       1/57).
//       (If N = 8 (1.25 MHz clk), set internal atten to
//       1/28).
//    o  190104: incorporated external attenuation after 
//       the DAC. So remove the internal attenuation stuff.
//       Compensated the FLL for this ext. atten by 
//       dividing the run_error by the amount of 
//       attenuation (the latter less than one).
//    o  190107 Add eeprom code
//
//  Rev 190115
//    o  Adding automatic filter-selection code...
//
//  Rev 190126
//    o  Change root Kcpu from 2^5 to 2^6
//
//  Rev 190205
//    o  Add dropback code: if the magnitude of the
//       phase delta error is > DROPBACK_LIMIT,
//       then drop back to a lower filter & increment
//       dropback_count.
//-----------------------------------------------------

#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <PWM.h>
#include <Wire.h>
#include <stdlib.h>
#include <EEPROM.h>

// Compiler Directives
//#define PRINT_CSV             // In FLL mode, print FLL parameters
//#define PRINT_DELTA           // In FLL mode, print Delta, second by second
//#define HOLD_DAC              // In FLL mode, hold the DAC at its initialized valu
//#define PRINT_PHASEDELTA      // Print second-by-second phase-delta
//#define TEST_ADC_READ_TIMING  // Test time-to-read ADC
//#define PRINT_PLL             // print PLL parameters every 30 seconds !!! VERY USEFUL

#define SW_REV 190205  // SW Revision!  (yr,mo,day)

// Address Defines
#define MAX5217  0x1C  // DAC I2C address

// NANO Pin Defines
#define INT0_PIN    2  // NANO pin, INTERRUPT 0
#define GPS_RST     7  // NANO pin, reset GNSS-2 receiver
#define TX_IN      10  // NANO pin, software UART RX
#define ICP1        8  // NANO pin, T1's ICP1 input
#define T1_CLK      5  // NANO pin, T1's clock input
#define SERIAL_IN   6  // NANO pin, software uart, RX from pc.
#define SERIAL_OUT 11  // NANO pin, software uart, TX to pc
#define DIG9        9  // NANO pin, Digital In (or Out, for testing)
#define RAMP_IN    A0  // NANO pin, Analog In0, PLL Ramp


// Starting Values for Frequency-Locked Loop algorithm
#define MAX_DAC_STEP     2816   // 16-second DAC step
#define GAIN             15.0
#define START_DAC        0x8000
#define OSC_PPB_DRIFT_PER_HOUR      0.0     // DON'T CORRECT FOR AGING
#define PPB_CORRECTION_PER_DAC_STEP 0.0044  // from osc. measurements
#define IDEAL_DELTA 19264   // for a 1 second period.

// gpsdo Frequency-Lock Mode states:
#define WAIT   0
#define START  1
#define SETTLE 2
#define GO     3
#define SETTLE_2 4
#define PLL_SETTLE_1 5
#define PLL_SETTLE_2 6

// run states
#define HOLD 0
#define RUN 1

// constants for Phase-Locked Loop operation
#define ADC_MAX_INIT 822    // maximum adc value
#define D_INIT        30    // for phase-delta filter
#define F1_INIT      256    // F1 for Shera's "fastest" IIR Filter
#define F2_INIT        8    // F2 for Shera's "fastest" IIR Filter
#define KCPU_INIT     32    // Kcpu for Shera's "fastest" IIR Filter
#define KCPU_INIT     64    // Kcpu for Shera's "fastest" IIR Filter
#define KCPU_T1_INIT   8    // kcpu for Shera's Type 1 Filter
#define N_INIT         8    //  = 10MHz / 1.25 MHz
#define SHERA_MAX_CNT 2304  // max phase-delta count in shera's design
#define KV_INIT     -0.32   // 10 MHz Kv for my HP oscillator module.
#define RECIP_EXT_ATTEN 29   // Reciprocal of extern_atten.  
//                              Use 29 for K6JCA HW gains (and N=8). 
//                              Note:  larger = more ringing)

// constants for auto filter selection
#define MIN_IIR_INIT       2    // minimum IIR Filter
#define MAX_IIR_INIT       5    // maximum IIR filter
#define SETTLING_INIT     2000  // Settling time, in seconds (3000 chosen for minimum IIR filter)
#define DELTA_LIMIT_INIT 100*30 // To switch filters up, PD error must be less than this.
#define DROPBACK_LIMIT    3000  // drop to lower filter if |phase delta error| exceeds this value.

#define FLL  0   // Frequency-lock mode
#define PLL  1   // Phase-lock mode

#define AUTO 1   // Auto IIR filter select
#define MAN  0   // Manual IIR filter select

#define TYPE1 0  // Shera Type 1 filter
#define IIR   1  // Shera IIR filter

// eeprom addresses:
#define ADDR_DATA_VALID    0   // 1 byte.  '1' = eeprom contents valid.  Else = invalid
#define ADDR_LOCK_MODE     1   // 1 byte.  Either FLL or PLL
#define ADDR_ADC_MAX       2   // 2 bytes  Maximum expeced value from ADC
#define ADDR_F1_ROOT       4   // 2 bytes  F1 for IIR Filter 2
#define ADDR_F2            6   // 2 bytes  F2
#define ADDR_KCPU_ROOT     8   // 2 bytes  Kcpu for IIR Filter 2
#define ADDR_KCPU_T1       10  // 2 bytes  Kcpu for Type 1 Filter
#define ADDR_KV            12  // 2 bytes  VCO Kv (Hz/V, signed)
#define ADDR_REC_EXT_ATTEN 14  // 2 bytes  The RECIPROCAL of attenuation between DAC & VCO
#define ADDR_DAC_START     16  // 2 bytes  Initalized DAC to this value
#define ADDR_SETTLING_ROOT 18  // 2 bytes  Filter settling time for IIR Filter 2
#define ADDR_MIN_FILTER    20  // 2 byte   For PLL mode, Auto Filter select: Min filter
#define ADDR_MAX_FILTER    22  // 2 byte   For PLL mode, Auto Filter select: Max filter
#define ADDR_FILT_SEL_MODE 24  // 1 byte   Auto Filter Select mode (in PLL mode), Manual or Auto
#define ADDR_MAXMIN_LIMIT  25  // 2 bytes  *** not used ***
#define ADDR_FILTER        27  // 2 bytes  PLL filter selected at Initialization
  



// set the LCD address to 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define software UART, Rx pin 6, Tx pin 11
SoftwareSerial mySerial(SERIAL_IN, SERIAL_OUT);

boolean       first_lcd_update; // identifies first LCD update after initialization
boolean       one_pps_flag;     // signals leading-edge of one_pps_pulse

// frequency-locked loop variables (from original FLL design)
unsigned int  prev_t1_count;
unsigned int  t1_count;
unsigned int  t1_delta;

byte          gpsdo_FLL_state;

long          dac_delta;

int           dac_direction;   // +1 = positive change, -1 = neg. change, 0 = no change.

char          tick_count;
boolean       display_countdown;
unsigned int  prev_countdown;

unsigned long zero_count;
unsigned long prev_count;
boolean       one_flag;
boolean       minus_one_flag;

float         float_zero_count;

int           last_direction;
long          run_error;

float         drift_corr_per_hour;  // drift correction, per hour (not used)


// FLL and PLL variables
unsigned int  dac;            // value sent to DAC
unsigned long long_dac;       // DAC value, in LONG format


// phase-locked loop and Shera variables
unsigned int phase_delta;    // 1-second phase delta value from ADC
unsigned int prev_phase_delta; // previous phase delta
unsigned int adc_max;        // maximum ADC value from Phase Detector
unsigned int dac_start;      // Initialize the DAC output to this value
unsigned int d;              // D, Number of seconds of phase-detector aggregation
float        kv;             // VCO Kv (Hz/V)
float        setpoint;       // setpoint for 30 second sample

float        f1;             // Shera's F1
float        f2;             // Shera's F2
float        kcpu;           // Shera's Kcpu
float        f1_root;        // F1 for the lowest IIR filter (i.e. Filter 2)
float        kcpu_root;      // Kcpu for the lowest IIR filter
unsigned int kcpu_t1;        // Kcpu for Shera's Type 1 filter
unsigned int second_count;   // count seconds to D (30 seconds)
boolean      aggregate_rdy;  // set true when reach D sec.

unsigned int running_sum;    // running sum of inputs
unsigned int d_sample;       // sum of aggregated samples after D counts (e.g. 30 seconds)
float        in0;            // input sample: in(0)
float        in_m1;          // in(-1)
float        out0;           // output sample: out(0)
float        out_m1;         // out(-1)

float        extern_atten;   // external analog attenuation (following "compensated" DAC)
float        pd_error;       // phase-detector error (+/- delta from setpoint)

float        dac_less_offset; // DAC value without the DC offset when using usigned DAC chip

unsigned int max_pd;          // maximum 1-sec phase delta (in a 30 sec aggregated phase delta)
unsigned int min_pd;          // minimum 1-sec phase delta (in a 30 sec aggregated phase delta)
unsigned int mid_pd;          // Average 1-sec phase delta (in a 30 sec aggregated phase delta) (not used)

unsigned int filter;          // IIR filter ID (per Shera, 2 to 7)
unsigned int init_filter;      // Initialization filter, 1-7 (1 = Type 1, 2-7 = IIR)

float        f_adc_max;       // max ADC value, as a float

int          filter_type;     // IIR or TYPE 1 filter.  See Shera article.

// Automatic Filter Select Mode stuff...
byte         filter_select_mode;// AUTO or Manual (via serial port) filter select mode 
unsigned int wraparound_count;  // counts how many times PLL has dropped its filtering to a lower filter due to phase-detector wraparound
long         settling_count;    // Count seconds that of a filter's settling after filter has been changed.
long         settling_root;     // settling time for the lowest IIR filter
long         settling_limit;    // settling root scaled for selected filter
unsigned int max_filter;        // maximum filter to shift up to
unsigned int min_filter;        // minimum filte to shift down to
float        maxmin_limit;      // limits phase-detector error must be within when up-selecting filter
boolean      clear_err_counts;  // true to clear wraparound_count;
float        upper_wraparound_lim;    // phase-detector error limit for wraparound test
float        lower_wraparound_lim;    // phase-detector error limit for wraparound test
bool         wraparound_error;  // true if second-by-second phase detector value is flipping between high and low limits
unsigned int dropback_count;    // counts how many times |pd_error| is > DROPBACK_LIMIT

// misc variables
unsigned long accum_seconds; // count seconds as a measure of elapsed time.
byte          run_state;     // RUN or HOLD (DAC value)
byte          lock_mode;     // PLL or FLL 
int           serialByte_In; // rcvd character from mySerial software UART
long          long_value;    // temporary "long" value
String        inString;      // temporary string (for building strings from mySerial sw UART rx data)
boolean       pll_to_serial; // flag to print seconds, PD error, and DAC to serial port every 30 seconds
boolean       pd_to_serial;  // flag to print PD every second.

// temporary variables
int          int_temp;
unsigned int uint_temp;
float        temp_float;

void setup()
{

  Serial.begin(57600);  // init serial port
  lcd.init();           // initialize the lcd
  InitTimersSafe();     //initialize all timers except for 0, to save time keeping functions

  pinMode(INT0_PIN, INPUT);
  pinMode(TX_IN, INPUT);
  pinMode(ICP1, INPUT);
  pinMode(T1_CLK, INPUT);
  pinMode(GPS_RST, OUTPUT);
  
  pinMode(DIG9, OUTPUT);      // I/O pin used for testing

  analogReference(INTERNAL);  // set analog reference to 1.1V.

  // Reset for 15 msec
  digitalWrite(GPS_RST, HIGH);
  delay(15);
  digitalWrite(GPS_RST, LOW);

  // Set Digital Pin 9 to 0:  (*** Used as output for testing timing
  digitalWrite(DIG9, LOW);

  // pull up the sw uart's RX pin 
  // (so that it doesn't float if unconnected to external TX signal)
  digitalWrite(SERIAL_IN, HIGH);


  //  Print a message to the LCD...
  //
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.print("  Hi, I'm your");
  lcd.setCursor(0, 1);
  lcd.print("  K6JCA GPSDO!");

  mySerial.begin(9600);
  mySerial.println("");
  mySerial.println("Hello Jeff");
  mySerial.println("");

  delay(2000);
  lcd.clear();
  lcd.print("SW Rev: ");
  lcd.print(SW_REV);
  delay(1500);
  lcd.clear();

  lcd.print("  Waiting for   ");
  lcd.setCursor(0, 1);
  lcd.print(" 1 PPS Pulse... ");

  // if eeprom has valid contents, load
  // the pll and fll initialization parameters
  // from eeprom...
  if (EEPROM.read(ADDR_DATA_VALID) == 0x01) {
    // dump eeprom contents
    mySerial.println(F("Initializing from EEPROM... "));
    if (EEPROM.read(ADDR_LOCK_MODE) == PLL) {
            lock_mode = PLL;          
    }
    else {
      lock_mode = FLL;       
    }
    min_filter = read_2bytes_eeprom(ADDR_MIN_FILTER);  
    max_filter = read_2bytes_eeprom(ADDR_MAX_FILTER);  
    init_filter = read_2bytes_eeprom(ADDR_FILTER);         // get filter...
    filter = init_filter;
    if (EEPROM.read(ADDR_FILT_SEL_MODE) == AUTO) {
      // if in auto mode, start at min filter
      filter_select_mode = AUTO;
      filter_type = IIR;
    }
    else {  
      // if in manual filter select mode, start with init_filter from eeprom              
      filter_select_mode = MAN;
      if (filter == 1) {
        filter_type = TYPE1;          
      }
      else {
        filter_type = IIR;
      }
    }
    adc_max = read_2bytes_eeprom(ADDR_ADC_MAX);  
    settling_root = read_2bytes_eeprom(ADDR_SETTLING_ROOT);  
    uint_temp = read_2bytes_eeprom(ADDR_F1_ROOT);
    f1_root = (float) uint_temp;  
    uint_temp = read_2bytes_eeprom(ADDR_F2);  
    f2 = (float) uint_temp;
    uint_temp = read_2bytes_eeprom(ADDR_KCPU_ROOT); 
    kcpu_root = (float) uint_temp; 
    uint_temp = read_2bytes_eeprom(ADDR_KCPU_T1);
    kcpu_t1 = (float) uint_temp;  
    int_temp = (int) read_2bytes_eeprom(ADDR_KV);
    kv = ((float) int_temp)/1000;  
    uint_temp = read_2bytes_eeprom(ADDR_REC_EXT_ATTEN);
    extern_atten = 1/((float) uint_temp);  
    dac_start = read_2bytes_eeprom(ADDR_DAC_START);  

    // *** add maxmin_limit  (below line is temporary)
    maxmin_limit = (float) DELTA_LIMIT_INIT;    

    filter = min_filter;
  }
  else {
    // nothing in eeprom.  Initialize with constants as follows...
    mySerial.println(F(" EEPROM not loaded.  Initialize from program "));
    adc_max   = ADC_MAX_INIT;
    kv        = KV_INIT;
    dac_start = START_DAC;
    max_filter = MAX_IIR_INIT;        
    min_filter = MIN_IIR_INIT;        
    filter = min_filter;
    init_filter = filter;
    filter_select_mode = AUTO;
    f1_root = (float)F1_INIT;
    f2 = (float) F2_INIT;
    kcpu_root = (float) KCPU_INIT;
    kcpu_t1 = (float) KCPU_T1_INIT;
    extern_atten = 1/((float) RECIP_EXT_ATTEN);
    //  lock_mode = FLL;  // init to frequency-lock mode
    lock_mode = PLL;  // init to phase-lock mode
    settling_root = SETTLING_INIT;     
    maxmin_limit = (float) DELTA_LIMIT_INIT;      
    filter_type = IIR;
  }
  f1 = f1_root;
  kcpu = kcpu_root;
  f_adc_max = (float) adc_max;
  settling_limit = settling_root * ((int) (pow((float)2,(float) (filter - min_filter))+0.5));

  dac       = dac_start;
  d         = D_INIT;
  setpoint  = d * adc_max / 2;    // *** store in eeprom?

  upper_wraparound_lim = f_adc_max * 0.875; // i.e. within 12.5% of max 1-sec phase delta value 
  lower_wraparound_lim = f_adc_max * 0.125; // i.e. within 12.5% of min 1-sec phase delta value 
  wraparound_error = false;

  print_current_values();
  
// How accurate are float calculations?
  // calc_floating_point_reciprocals();

  delay(2000);

  one_pps_flag = false;

  first_lcd_update = true;

  tick_count = 0;
  display_countdown = false;
  prev_countdown = 0;


  zero_count = 0;
  accum_seconds = 0;
  dac_direction       = 0;
  gpsdo_FLL_state = WAIT;

  dac_delta = 0;

  one_flag = false;
  minus_one_flag = false;

  last_direction = 0;
  run_error = 0;
  prev_count = 0;

  run_state = RUN;

  inString = "";

  long_value = 0;

  drift_corr_per_hour = OSC_PPB_DRIFT_PER_HOUR / PPB_CORRECTION_PER_DAC_STEP;
  long_dac = long(dac);
  write_to_dac(dac);

  // Shera PLL
  second_count = 0;
  aggregate_rdy = false;

  running_sum = 0;
  d_sample = 0;
  in0 = 0;
  in_m1 = 0;
  out0 =  0;
  out_m1 = 0;

  phase_delta = adc_max/2;   // start phase delta near setpoint.
  pd_error = 0;
  max_pd = 0;
  min_pd = 65535;

  pll_to_serial = false;
  pd_to_serial = false;
  
  wraparound_count = 0;    
  settling_count = 0;    
  clear_err_counts = false;  

  dropback_count = 0;    
  
  // attach interrupt for falling edge of one_pps signal.
  // Note that this falling edge is at the collector of an NPN
  // invertor and it corresponds, in time, to the rising edge of 
  // the 1 PPS pulse from the GPS receiver.
  attachInterrupt(digitalPinToInterrupt(INT0_PIN), one_pps, FALLING);

  // Timer 1: external clock.  Neg. Edge capture.  No noise cancellation.
  timer1_setup (0x00, -1, 0x00, 0x00, 0x00);

 #ifdef PRINT_PLL
  Serial.println(F("seconds,max_pd,min_pd,pd_error,filter,in(-1),out(0),out(-1),DAC-DC,DAC,settling"));
 #endif

#ifdef PRINT_CSV
  Serial.println(F("Total Seconds,DAC,DacDirection,ZeroCount,DacDelta"));
#endif

}

void loop()
{
 
  // If 1 PPS has occured (and a phase-delta sample
  // received via the ADC), do the following///
  // 
  if (one_pps_flag) {
    one_pps_flag = false;
    aggregate_samples();    // 
    if (run_state == RUN) {
      // in RUN mode, so do either the 
      // FLL or the PLL calculations, 
      // depending upon which is selected...
      gpsdo_state_machine();
    }
    else {
      // GPSDO in HOLD state.  So bump the elapsed time counter... 
      accum_seconds++;    // inc seconds counter
    }
    // check if there's serial data from the serial port...
    serialByte_In = mySerial.read();
    if (serialByte_In != -1) {
      // Yes, there's data.  Get it and
      // do something with it...
      get_serial_in(serialByte_In);
    }  
    update_lcd();
  }
}



void update_lcd()
{
  // first, get magnitude of run_error and
  // magnitude of error
  long mag_run_error;
  if (run_error < 0) mag_run_error = -run_error;
  else mag_run_error = run_error;

  // first time into this call, clear the screen.
  // Otherwise, just write w/o clearing.
  if (first_lcd_update == true) {
    lcd.clear();
    first_lcd_update = false;
  }
  tick_count++;
  if (tick_count == 6) {
    tick_count = 0;
  }

  // Print first line of LCD
  lcd.setCursor(0, 0);
  if (lock_mode == FLL) lcd.print("FLL:");
  else lcd.print("PLL:");
  lcd.setCursor(4, 0);
  if (run_state == RUN) lcd.print(" r");
  else lcd.print(" h");
  lcd.print(" DAC: ");
  // Print DAC value and
  // print leading zeroes (normally suppressed
  if (dac < 4096) lcd.print("0");
  if (dac < 256) lcd.print("0");
  if (dac < 16) lcd.print("0");
  lcd.print(dac, HEX);

  if (lock_mode == FLL) {
    // Print second line of LCD
    // start with the current run count of zeroes
    lcd.setCursor(0, 1);
    // Print current value Zero count
    if (zero_count < 10000) lcd.print(" ");
    if (zero_count < 1000) lcd.print(" ");
    if (zero_count < 100) lcd.print(" ");
    if (zero_count < 10) lcd.print(" ");
    lcd.print(zero_count);

    lcd.print(",");

    // Now print the previous run count.
    if (prev_count < 10000) lcd.print(" ");
    if (prev_count < 1000) lcd.print(" ");
    if (prev_count < 100) lcd.print(" ");
    if (prev_count < 10) lcd.print(" ");
    lcd.print(prev_count);

    // Print magnitude of previous run-error.
    if (mag_run_error < 1000) lcd.print(" ");
    if (mag_run_error < 100) lcd.print(" ");
    if (mag_run_error < 10) lcd.print(" ");
    dac_direction_to_lcd();
    lcd.print(mag_run_error);
  }
  else {
    // PLL Mode...
    // Because the PLL data is updated 
    // every 30 seconds, I'll do its LCD updating 
    // in the PLL state machine, further down...
  }

}

void one_pps()
{
  // This is the one pps interrupt routine
  
  prev_phase_delta = phase_delta;   // store previous phase delta
  phase_delta = analogRead(RAMP_IN);// read new phase delta from ADC

#ifdef TEST_ADC_READ_TIMING
  digitalWrite(DIG9, HIGH);  // To test timing from 1pps leading edge...
  digitalWrite(DIG9, HIGH);  // extend...
  digitalWrite(DIG9, HIGH);  // extend...
  digitalWrite(DIG9, LOW);
#endif
  one_pps_flag = true;

#ifdef PRINT_PHASEDELTA
  Serial.print(accum_seconds);
  Serial.print(",");
  Serial.print(phase_delta);
#endif

  // if the following flag is true, print phase-delta 
  // to mySerial port (every second)
  if (pd_to_serial == true) {
    mySerial.print(accum_seconds);
    mySerial.print(",");
    mySerial.println(phase_delta);
  }
}

void timer1_setup (byte mode, int prescale, byte outmode_A, byte outmode_B, byte capture_mode)
{
  // Code for FLL (using Arduino's Timer 1)
  // NOTE:  This code found at:
  // http://sphinx.mythic-beasts.com/~markt/ATmega-timers.html

  // enforce field widths for sanity
  mode &= 15 ;
  outmode_A &= 3 ;
  outmode_B &= 3 ;
  capture_mode &= 3 ;

  byte clock_mode = 0 ; // 0 means no clocking - the counter is frozen.
  switch (prescale)
  {
    case 1: clock_mode = 1 ; break ;
    case 8: clock_mode = 2 ; break ;
    case 64: clock_mode = 3 ; break ;
    case 256: clock_mode = 4 ; break ;
    case 1024: clock_mode = 5 ; break ;
    default:
      if (prescale < 0)
        clock_mode = 7 ; // external clock
  }
  TCCR1A = (outmode_A << 6) | (outmode_B << 4) | (mode & 3) ;
  TCCR1B = (capture_mode << 6) | ((mode & 0xC) << 1) | clock_mode ;
}

void gpsdo_state_machine()
{

  if (lock_mode == FLL) {  
    // do for Frequency Lock Loop...
    switch (gpsdo_FLL_state) {

      case WAIT:
        gpsdo_FLL_state = START;
        break;

      case START:
#ifdef PRINT_CSV
        Serial.print(accum_seconds);
        Serial.print(",");
        Serial.print(dac);
        Serial.println(",");
#endif
        mySerial.print(accum_seconds);
        mySerial.print(",");
        mySerial.print(dac);
        mySerial.print(",");

        write_to_dac(dac);


        gpsdo_FLL_state = SETTLE;
        accum_seconds++;
        break;

      case SETTLE:
        // DAC has been set.  Let's let it settle.
        gpsdo_FLL_state = SETTLE_2;
        accum_seconds++;
        break;

      case SETTLE_2:
        // Let DAC settle for another second.
        // Meanwhile, get current count and
        // set to previous count.
        t1_count = ICR1;
        prev_t1_count = t1_count;
        gpsdo_FLL_state = GO;
        accum_seconds++;
        break;

      case GO:
        accum_seconds++;  // another second has passed
        prev_t1_count = t1_count; // prev. counter snapshot
        t1_count = ICR1;          // new counter snapshot

        // compensate for counter wraparound
        if (t1_count < prev_t1_count) {
          t1_delta = 65536 - (prev_t1_count - t1_count);
        }
        else {
          t1_delta = t1_count - prev_t1_count;
        }
#ifdef PRINT_DELTA
        Serial.print("Seconds: ");
        Serial.print(accum_seconds);
        Serial.print(", zero_count: ");
        Serial.print(zero_count);
        Serial.print(", delta: ");
        Serial.print(t1_count);
        Serial.print(" - ");
        Serial.print(prev_t1_count);
        Serial.print(" = ");
        Serial.print(t1_delta);
        if (minus_one_flag) Serial.println(", minus-flag ");
        else {
          if (one_flag) Serial.println(", plus-flag ");
          else Serial.println(", no-flag");
        }
#endif
        // If snapshot delta equals ideal delta, do
        // nothing except increment the zero-run count.
        // Also, very infrequently the program misses
        // capturing a delta, so on the next 1-second
        // pulse the value is TWICE what it should have been.
        if ((t1_delta == IDEAL_DELTA) || (t1_delta == IDEAL_DELTA << 1)) {
          zero_count++;

        }
        else {
          // Check if the delta is greater than
          // the ideal delta (or  greater than
          // 2x the ideal delta, plus 1,
          // in case a sample was missed).
          if ((t1_delta == IDEAL_DELTA + 1) || (t1_delta == (IDEAL_DELTA << 1) + 1)) {
            if (minus_one_flag) {
              // previously saw a "negative" delta.
              // +1 and -1 cancel so
              // continue counting
              dac_direction = 0;
              zero_count = zero_count + 1;
              minus_one_flag = false;
            }
            else {
              if (!one_flag) {
                // Previous delta was 0, set set flag
                // that this delta is a 1 (i.e. postive).
                // Don't stop counting zeroes yet.
                one_flag = true;
                zero_count = zero_count + 1;
                dac_direction = 0;
              }
              else {
                // Two positive deltas in a row.
                // Run of zeroes is over!
                // Reset flag and indicate direction to
                // change dac.
                // Positive deltas mean increase dac
                // to lower the oscillator frequency.
                dac_direction = +1;
                one_flag = false;
              }
            }
          }
          else {
            // same idea as above, but instead with a
            // negative delta preceeding a pos. delta.
            if ((t1_delta == IDEAL_DELTA - 1) || (t1_delta == (IDEAL_DELTA << 1) - 1)) {
              if (one_flag) {
                // -1 and +1 cancel, so continue with zeroes
                dac_direction = 0;
                zero_count = zero_count + 1;
                one_flag = false;
              }
              else {
                if (!minus_one_flag) {
                  minus_one_flag = true;
                  zero_count = zero_count + 1;
                  dac_direction = 0;
                }
                else {
                  // two minus deltas in a row.
                  // Run of zeroes is over!  Reset flag
                  // and identify direction to move DAC.
                  // Negative deltas mean decrease dac
                  // to raise the oscillator frequency.
                  dac_direction = -1;
                  minus_one_flag = false;
                }
              }
            }
          }
        }
        if (dac_direction != 0) {
          // Run of zeroes has ended.  From run length
          // determine dac correction factor (= run_error).
          // Will also add to that value a "drift" offset
          // (proportional to run length)
          prev_count = zero_count;  // for lcd display purposes
          float_zero_count = float(zero_count);
          // Note that this next calculation now includes a gain factor, 1/extern_atten,
          // to compensate for the additional attenuation added between DAC and VCO 
          // for PLL operation.  I.e. need to increase the dac value to compensate
          // for the additional attenuation.
          run_error = long((1/extern_atten)* MAX_DAC_STEP * (GAIN / (float_zero_count + 1)));  // add one so don't divide by 0.
          if (dac_direction == -1) {
            run_error = -run_error;
            last_direction = -1;  // For lcd display purposes
          }
          else {
            last_direction = +1;
          }
          dac_delta = run_error;

#ifdef PRINT_CSV
          Serial.print(dac_direction);
          Serial.print(",");
          Serial.print(zero_count);
          Serial.print(",");
          Serial.println(dac_delta);
#endif
          mySerial.print(dac_direction);
          mySerial.print(",");
          mySerial.print(zero_count);
          mySerial.print(",");
          mySerial.println(dac_delta);

          // Calculate new dac value.

#ifdef HOLD_DAC
          dac_delta = 0;
#endif

          long_dac = long(dac) + dac_delta;
          if (long_dac > 65535) long_dac = 65535;
          if (long_dac < 0) long_dac = 0;
          dac = uint16_t(long_dac);

          // dac updated.  Start counting 0's anew...
          zero_count = 0;
          dac_direction = 0;
          gpsdo_FLL_state = START;
        }
        break;

      default:
        break;
    }
    // and keep pll stuff in sync, too, (in case we jump to PLL mode)
    // by forcing the integrator's accumulator
    // to follow the current dac value
    //   out0 = (((float)dac)/kcpu)*f_adc_max*((float) d)/(-2304.0);
    out0 = (((float) dac) - 32768.0)*(f_adc_max*((float) d))/(kcpu*(-2304.0));
    out_m1 = out0;
    second_count = 0;
    running_sum = 0;
    aggregate_rdy = false;
    max_pd = 0;
    min_pd = 65535;
    wraparound_error = false;
    //  Serial.println(out0);
    
  }
  else {  
    // in Phase Lock Mode !!!
    // first check if the second-by-second phase delta
    // is flipping between the high and low limits.
    // If so, and the filter is NOT the lowest filter,
    // then want to drop back to the lowest filter.
    if (((phase_delta >= ((unsigned int) upper_wraparound_lim)) && (prev_phase_delta <= ((unsigned int) lower_wraparound_lim))) || ((phase_delta <= ((unsigned int) lower_wraparound_lim)) && (prev_phase_delta >= ((unsigned int) upper_wraparound_lim)))) {
      wraparound_error = true;
    }
    accum_seconds++;
    settling_count++; 
    if (settling_count > settling_limit) settling_count = settling_limit;
    
    if (clear_err_counts) {
      wraparound_count = 0;
      dropback_count = 0;
      clear_err_counts = false;
    }
    // write to dac at 1pps
    // and then calculate new dac
    if (aggregate_rdy == true) {
      aggregate_rdy = false;
      pd_error = (float)d_sample - setpoint;  // subtract the setpoint from the aggregated 30-second sample.

      if (filter_type == IIR) {
        // Shera IIR Filter
        in_m1 = in0;    // shift the last input "in(0)" value in in(-1).
        out_m1 = out0;  // shift the last output "out(0)" value to out(-1)
        in0 = pd_error; // New in(0) equals pd_error.

        // Calculate the new filter output value, out(0), using Shera's formula:
        out0 = ((out_m1 + in0 * (1 / f1 + 1 / f2) + in_m1 * (1 / f1 - 1 / f2)));

        // multiply the filter output value by Kcpu.  Then scale, per the Simulink simulation
        dac_less_offset = (out0 * kcpu * (((float)(-2304.0)) / (f_adc_max * ((float) d))));
       
        // auto filter selection code
        
        if (filter_select_mode == AUTO) {
          // have we experienced a phase-detector wraparound during the 30-sec aggregation period?
          // (if so, want to jump down to a lower filter so that we
          // can try to get the phase-detector re-centered to the setpoint).
          // Note that we do not care about wraparounds if the filter selected is the MINIMUM filter,
          // because we cannot jump back to a lower filter.
          
          if (wraparound_error == true) {
            // a wraparound has occured!  First, reset the settling_count
            // then, drop filter back to the minimum filter
            settling_count = 0;
            wraparound_count++;  // This count, displayed on the LCD, tells us how many wrap-arounds have occured
            filter = min_filter; // Jump down to the minimum filter if there's a wraparound.
            
            second_count = 0;
            running_sum = 0;
            
            // Changed the filter, so update filter values and out0 so that there isn't
            // a discontinuity.
            temp_float = (pow((float)2,(float) (filter - min_filter)));
            settling_limit = settling_root * ((int) (temp_float+0.5));
            f1 = f1_root * temp_float;
            out0 = out0*kcpu/(kcpu_root/temp_float);  // adjust the 'dc' in the integrator to prevent jump
            kcpu = kcpu_root / temp_float;
            aggregate_rdy = false;
            mySerial.println("wraparound!");
 
         }
         else {
            // No wraparound has occurred, but is pd_error too large?
            // If so, will want to backdown to a lower filter so that
            // we recover more quickly back towards the phase delta setpoint.
            // 
            if ((pd_error > DROPBACK_LIMIT) || (pd_error < -DROPBACK_LIMIT)) {
              // big phase delta error, so backdown the IIR filter to a lower one
              // to recover more quickly
              dropback_count++;    // increment the backdown count (for LCD display)
              settling_count = 0;  // Reset settling count timer.
              filter = min_filter; // Simply jump back to the minimum filter if there's a backdown
              
              second_count = 0;
              running_sum = 0;
              
              // Changed the filter (if we weren't at min filter, 
              // so update filter values and out0 so that there isn't
              // a discontinuity in the DAC output.
              temp_float = (pow((float)2,(float) (filter - min_filter)));
              settling_limit = settling_root * ((int) (temp_float+0.5));
              f1 = f1_root * temp_float;
              out0 = out0*kcpu/(kcpu_root/temp_float);  // adjust the 'dc' in the integrator to prevent jump
              kcpu = kcpu_root / temp_float;
              aggregate_rdy = false;
              mySerial.println("backdown!");
            }
            else {
              // Don't backdown, so check if we should select a higher filter...
              // test if we have settled since the previous filter switch
              if (settling_count >= settling_limit) { 
                // (note that the settling time doubles 
                // for each filter step up.)
                // yes, we've settled                                             // and add 0.5 so that int properly truncates
                // Now -- is the phase delta error (which is the same as in0) close to zero?
                // and are we not already at the highest (slowest) filter?
             
                if ((in0 > -maxmin_limit) && (in0 < maxmin_limit)) {
                  // OK, we are near the setpoint, so we can upshift, but first
                  // check that we are not at the highest filter...
                  if (filter < max_filter) {
                    // OK, can select the next filter up
                    filter++;
                    temp_float = (pow((float)2,(float) (filter - min_filter)));
                    settling_limit = settling_root * ((int) (temp_float+0.5));
                    settling_count = 0;
                    second_count = 0;
                    running_sum = 0;
                    f1 = f1_root * temp_float;
                    out0 = out0*kcpu/(kcpu_root/temp_float);  // adjust the 'dc' in the integrator to prevent jump
                    kcpu = kcpu_root / temp_float;
                    aggregate_rdy = false;
    
                  }
                }
                else {
                  // Although we've reached the settling time,
                  // the Phase Delta isn't close enough to the setpoint 
                  // (not within the maxmin_limit window).
                  // So do nothing and continue...
                }            
              }
              else {
                // Haven't setled yet, so do nothing and continue...
              }  
            }        
          }          
        }
      }
      else {
        // Shera Type 1 Filter, not IIR filter
        // calculate DAC input per Simulink model
        dac_less_offset = pd_error * KCPU_T1_INIT * (((float)(-2304.0)) / (f_adc_max * ((float) d)));
        
        // To minimize discontinuities when
        // switching from Type 1 filter to an IIR filter,
        // update the IIR filter's variables while
        // in Type1 filter mode.
        out0 = dac_less_offset * f_adc_max * ((float) d) / (kcpu * ((float)(-2304)));
        out_m1 = out0;
        in0 = pd_error;
        in_m1 = in0;
        settling_count = 0; 
        
      }
      // Clip (i.e. limit) the DAC input (see Simulink model).
      if (dac_less_offset > 0) dac_less_offset = dac_less_offset + 0.5; // round up if positive
      if (dac_less_offset < 0) dac_less_offset = dac_less_offset - 0.5; // round down if negative
      if (dac_less_offset > 32767) dac_less_offset = 32767;             // clip if too positive
      else if (dac_less_offset < -32768) dac_less_offset = -32768;      // clip if too negative
      
      dac = (uint16_t)(dac_less_offset + 0x8000);  // add dac midpoint because DAC is unsigned.
      write_to_dac(dac); // write to the DAC!

#ifdef PRINT_PLL
      Serial.print(accum_seconds);
      Serial.print(",");
      Serial.print(max_pd);
      Serial.print(",");
      Serial.print(min_pd);
      Serial.print(",");
      Serial.print(pd_error);
      Serial.print(",");
      Serial.print(filter);
      Serial.print(",");
      Serial.print(in_m1);
      Serial.print(",");
      Serial.print(out0);
      Serial.print(",");
      Serial.print(out_m1);
      Serial.print(",");
      Serial.print(dac_less_offset);
      Serial.print(",");
      Serial.print(dac);
      Serial.print(",");
      Serial.println(settling_count);

      // update second line of LCD (PLL info)
      lcd.setCursor(0, 1);
      if (max_pd < 100) lcd.print(" ");
      if (max_pd < 10) lcd.print(" ");
      lcd.print(max_pd);
      lcd.print(" ");

//      mid_pd = d_sample/d;
//      if (mid_pd < 100) lcd.print(" ");
//      if (mid_pd < 10) lcd.print(" ");
//      lcd.print(mid_pd);
//      lcd.print(" ");

      if (min_pd < 100) lcd.print(" ");
      if (min_pd < 10) lcd.print(" ");
      lcd.print(min_pd);
      lcd.print(" ");

      // display wraparound count 
      lcd.print("w");
      if (wraparound_count > 9) lcd.print("^"); // print ^ if count > 9
      else lcd.print(wraparound_count);
      lcd.print(" ");
      
      // display dropback count 
      lcd.print("d");
      if (wraparound_count > 9) lcd.print("^"); // print ^ if count > 9
      else lcd.print(dropback_count);
      lcd.print(" ");
      
      lcd.print("F");
      lcd.print(filter);
#endif

      if (pll_to_serial == true) {
        // print to serial port every 30 seconds...
        mySerial.print(accum_seconds);
        mySerial.print(",");
        mySerial.print(pd_error);
        mySerial.print(",");
        mySerial.print(filter);
        mySerial.print(",");
        mySerial.println(dac);       
      }

      
      // reset our max_pd and min_pd values.
      // We will compare the next 30 1-sec
      // PD values against these to find
      // max and min PD in a 30 second 
      // sample aggregation period.
      max_pd = 0;
      min_pd = 65535;

      // clear out wraparound_error flag at the end of the D-sample calcs
      wraparound_error = false;
    }

    // and hold these Frequency-locked loop
    // terms constant:
    zero_count = 0;
    dac_direction = 0;
    gpsdo_FLL_state = START;

  }
}

void write_to_dac(unsigned int dac_value)
{
  // writing 16-bit word to Maxim MAX5217

  byte msbyte;
  byte lsbyte;

  msbyte = byte((dac_value & 0xFF00) >> 8);
  lsbyte = byte(dac_value & 0x00FF);
  Wire.beginTransmission(MAX5217);
  Wire.write(0x01);  // control word
  Wire.write(msbyte);  // MS Byte
  Wire.write(lsbyte);  // LS Byte
  Wire.endTransmission();
}


void dac_direction_to_lcd()
{
  if (last_direction == +1) lcd.print("+");
  else {
    if (last_direction == -1) lcd.print("-");
    else lcd.print(".");
  }
}

long parseLong()
{
  boolean endparse;
  long long_input;
  int inChar;

  endparse = false;
  inString = "";
  long_input = 0;

  while (endparse == false) {
    while (mySerial.available() > 0) {
      inChar = mySerial.read();
      mySerial.write(inChar);   // echo back

      if ((inChar != 0x0D) && (inChar != 0x0A)) {
        // As long as the incoming byte
        // is not a carriage return or a line-feed,
        // convert the incoming byte to a char
        // and add it to the string
        inString += (char)inChar;
      }
      // if received CR or LF, terminate
      // and print the string,
      // then convert string's value to LONG:
      // NOTE:  per Arduino Reference, .toINT
      // actually returns a LONG, not an INT
      // (as there is no .toLong StringObject
      // function).
      else {
        mySerial.println("");
        mySerial.print(F("Input string: "));
        mySerial.println(inString);
        long_input = inString.toInt();
        inString = "";
        endparse = true;
      }
    }
  }
  return (long_input);

}

void print_menu()
{
  mySerial.println("");
  mySerial.println(F(" ********* GPSDO Command Menu *********"));
  mySerial.println(F(" (Values should be entered as INTEGERS.)"));
  mySerial.println("");
  mySerial.println(F("   m:  print Menu"));
  mySerial.println(F("   u:  dump variables"));
  mySerial.println(F("   l:  erase EEPROM"));
  mySerial.println(F("   n:  dump EEPROM"));
  mySerial.println(F("   o:  load EEPROM"));
  mySerial.println(F("   g:  To print 1-sec PD, type 'g1',"));
  mySerial.println(F("       ...to print 30-sec PD_Error, Filter, DAC, type 'g2',")); 
  mySerial.println(F("       ...to STOP, type 'g'. "));
  mySerial.println("");
  mySerial.print(F("   r:  Toggle RUN/HOLD state (currently "));
  if (run_state == RUN) mySerial.println("RUN)");
  else mySerial.println("HOLD)");
  mySerial.print(F("   p:  Toggle PLL/FLL mode (currently "));
  if (lock_mode == PLL) mySerial.println("PLL)");
  else mySerial.println("FLL)");
  mySerial.println("");
  mySerial.println(F("   d:  Write DAC, 0-65535"));
  mySerial.println(F("   b:  Bump DAC, -16386 to 16386"));
  mySerial.println("");
//  mySerial.println(F("   s:  set Setpoint,10-90 (%)"));
  mySerial.println(F("   a:  set ADC Max, 0 to 1023"));
  mySerial.println(F("   k:  set VCO Kv, +/-0 to 10000 milliHz/V"));
  mySerial.println(F("   t:  set Reciprocal of external attenuation, 1 to 100"));
  mySerial.println(F("   v:  set DAC Init, 0-65535"));
  mySerial.println(F("   w:  set F1 (root), 1-32768"));
  mySerial.println(F("   x:  set F2, 1-32768"));
  mySerial.println(F("   y:  set Kcpu (root), for IIR Filters, 1-32768"));
  mySerial.println(F("   z:  set Kcpu for Type 1 Filter, 1-32768"));
  mySerial.println(F("   q:  set Settling Time (root), 1-10000"));
  mySerial.println(F("   h:  set PLL's Initialization Filter (1-7)"));

  mySerial.println("");
  mySerial.print(F("   e:  Toggle PLL Filter Select Mode (Auto/Man, currently: ")); 
  if (filter_select_mode == AUTO) mySerial.println(F("AUTO)"));
  else mySerial.println(F("MANUAL)"));
  mySerial.println(F("   i:    Min IIR FIlter, 2-max_filter)"));
  mySerial.println(F("   j:    Max IIR FIlter, min_filter-7"));
  
  mySerial.println("");
  mySerial.println(F("   0:  set DAC to 0x8000"));
  mySerial.println(F("   1:  select Shera Type 1 Filter"));
  mySerial.println(F("   2-7:  select Shera IIR Filter"));
  mySerial.println(F("   8:  set DAC to 0x0000"));
  mySerial.println(F("   9:  set DAC to 0xFFFF"));

  mySerial.println("");
  mySerial.println(F("   c:  Clear error counts"));
}

void get_serial_in(int input_char)
{
  // echo back character
  mySerial.println("");
  mySerial.write(serialByte_In);

  switch (serialByte_In) {

    case 'a':   // Set ADC Max value 
    case 'A':
      long_value = parseLong();
      if (long_value < 0) long_value = 0;
      if (long_value > 1023) long_value = 1023;
      adc_max = long_value;
      mySerial.println("");
      mySerial.print(F(" ADC Max = "));
      mySerial.println(adc_max);
      break;

    case 'b':  //  bump the dac value
    case 'B':
      long_value = parseLong();
      if (long_value < -16386) long_value = -16386;
      if (long_value > 16386) long_value = 16386;
      mySerial.println("");
      mySerial.print(F(" DAC start = 0x"));
      mySerial.println(dac, HEX);
      mySerial.print(F("  DAC Bump = "));
      mySerial.println(long_value);
      dac = dac + long_value;
      if (dac < 0) dac = 0;
      if (dac > 65535) dac = 65535;
      mySerial.print(F(" DAC end   = 0x"));
      mySerial.println(dac, HEX);
      second_count = 0;
      running_sum = 0;
      settling_count = 0;
      write_to_dac(dac);
      out0 = (((float) dac) - 32768.0)*(f_adc_max*((float) d))/(kcpu*(-2304.0));
      out_m1 = out0;
      in0 = 0;
      in_m1 = 0;
      break;

    case 'c':   // clear any error counters
    case 'C':
      clear_err_counts = true;
      mySerial.println("");
    break;

    case 'd':  // Set the DAC
    case 'D':
      long_value = parseLong();
      if (long_value < 0) long_value = 0;
      if (long_value > 65535) long_value = 65535;
      dac = long_value;
      write_to_dac(dac);
      second_count = 0;
      running_sum = 0;
      settling_count = 0;
      out0 = (((float) dac) - 32768.0)*(f_adc_max*((float) d))/(kcpu*(-2304.0));
      out_m1 = out0;
      in0 = 0;
      in_m1 = 0;
      write_to_dac(dac);
      mySerial.println("");
      mySerial.print(F(" DAC = 0x"));
      mySerial.println(dac, HEX);
      break;

    case 'e':  // Toggle filter select mode (audo vs manual)
    case 'E':
      mySerial.print(F(" Filter Select Mode ="));
      if (filter_select_mode == AUTO) {
        filter_select_mode = MAN;
        mySerial.println(F(" MANUAL"));
      }
      else {
        filter_select_mode = AUTO;
        mySerial.println(F(" AUTO"));
        
      }
    break;

    case 'f':
    case 'F':
    break;

    case 'g':   // enable/disable printing pll run data to mySerial port.
    case 'G':  
      long_value = 0;
      long_value = parseLong();
      if (long_value == 1) {
        pll_to_serial = false;
        pd_to_serial = true;
        mySerial.println(" 1-second print...");
      }
      else {
        if (long_value == 2) {
          pd_to_serial = false;
          pll_to_serial = true;
          mySerial.println(" 30-second print...");
        }
        else {
          pd_to_serial = false;
          pll_to_serial = false;
          mySerial.println(" print OFF");
        }
      }
    break;

    case 'h':
    case 'H': 
      long_value = parseLong();
      if (long_value < 1) long_value = 1;
      if (long_value > 7) long_value = 7;
      init_filter = long_value;
      mySerial.println("");
      mySerial.print(F(" Init Filter = "));
      mySerial.println(init_filter);
    break;

    case 'i':  // set minimum IIR filter
    case 'I':
      long_value = parseLong();
      if (long_value < 2) long_value = 2;
      if (long_value > max_filter) long_value = max_filter; // cannot be greater than max_filter
      if (long_value > 7) long_value = 7;  // cannot be greater than 7
      min_filter = long_value;
      mySerial.println("");
      mySerial.print(F(" Min IIR filter = "));
      mySerial.println(min_filter);
    break;

    case 'j': // set maximum IIR filter
    case 'J':
      long_value = parseLong();
      if (long_value < min_filter) long_value = min_filter; // max filter can't be less than min filter
      if (long_value > 7) long_value = 7;
      max_filter = long_value;
      mySerial.println("");
      mySerial.print(F(" Max IIR filter = "));
      mySerial.println(max_filter);
    break;

    case 'k':  // set Kv for the VCO
    case 'K':
      long_value = parseLong();
      if (long_value < -10000) long_value = -10000;
      if (long_value > 10000) long_value = 10000;
      kv = ((float)long_value) * 0.001;      // input as mHz/V
      mySerial.println("");
      mySerial.print(F(" VCO Kv(Hz/V) = "));
      mySerial.println(kv,3);
      break;

    case 'l':   // erase eeprom
    case 'L':
      EEPROM.update(ADDR_DATA_VALID,0XFF);  // set Data-valid memory to invalid
      mySerial.println(F(" EEPROM erased"));
    break;

    case 'm':   // print command menu
    case 'M':
      print_menu();
    break;

    case 'n':   // read eeprom
    case 'N':
      mySerial.println(F(" EEPROM Contents:"));
      if (EEPROM.read(ADDR_DATA_VALID) == 0x01) {
        // dump eeprom contents
        mySerial.print(F(" Initialize with Lock Mode = "));
        if (EEPROM.read(ADDR_LOCK_MODE) == PLL) {
          mySerial.println(F("PLL"));          
        }
        else {
          mySerial.println(F("FLL"));        
        }
        uint_temp = read_2bytes_eeprom(ADDR_ADC_MAX);  
        mySerial.print(F(" ADC Max = "));
        mySerial.println(uint_temp);
        mySerial.print(F(" Filter Select Mode = "));
        if (EEPROM.read(ADDR_FILT_SEL_MODE) == AUTO) {
          filter_select_mode = AUTO;
          mySerial.println(F("AUTO"));
        }
        else {
          filter_select_mode = MAN;
          mySerial.println(F("MANUAL"));      
        }
        uint_temp = read_2bytes_eeprom(ADDR_SETTLING_ROOT);  
        mySerial.print(F(" Settling Time (Root) = "));
        mySerial.println(uint_temp);
        uint_temp = read_2bytes_eeprom(ADDR_FILTER);  
        mySerial.print(F(" Initialization Filter = "));
        mySerial.println(uint_temp);
        uint_temp = read_2bytes_eeprom(ADDR_MIN_FILTER);  
        mySerial.print(F("   min filter = "));
        mySerial.print(uint_temp);
        uint_temp = read_2bytes_eeprom(ADDR_MAX_FILTER);  
        mySerial.print(F(", max filter = "));
        mySerial.println(uint_temp);
        uint_temp = read_2bytes_eeprom(ADDR_F1_ROOT);  
        mySerial.print(F(" F1 (root) = "));
        mySerial.println(uint_temp);
        uint_temp = read_2bytes_eeprom(ADDR_F2);  
        mySerial.print(F(" F2 = "));
        mySerial.println(uint_temp);
        uint_temp = read_2bytes_eeprom(ADDR_KCPU_ROOT);  
        mySerial.print(F(" Kcpu (root) = "));
        mySerial.println(uint_temp);
        uint_temp = read_2bytes_eeprom(ADDR_KCPU_T1);  
        mySerial.print(F(" Kcpu Type 1 Filter = "));
        mySerial.println(uint_temp);
        int_temp = (int) read_2bytes_eeprom(ADDR_KV);  
        mySerial.print(F(" Kv = "));
        mySerial.println(((float)int_temp)/1000,3);
        uint_temp = read_2bytes_eeprom(ADDR_REC_EXT_ATTEN);  
        mySerial.print(F(" 1/External_Attenuation = "));
        mySerial.println(uint_temp);
        uint_temp = read_2bytes_eeprom(ADDR_DAC_START);  
        mySerial.print(F(" DAC Starting Value "));
        mySerial.println(uint_temp);

      }
      else {
        // nothing in eeprom.
        mySerial.println(F(" EEPROM does not contain valid contents"));
      }
    break;

    case 'o':    // load eeprom
    case 'O':
      EEPROM.update(ADDR_DATA_VALID,0X01);  // set Data-valid memory to valid
      EEPROM.update(ADDR_LOCK_MODE,lock_mode);   // Note: sets initialization of lock mode to CURRENT mode
      write_2bytes_eeprom(ADDR_ADC_MAX,adc_max);
      write_2bytes_eeprom(ADDR_F1_ROOT,(unsigned int) f1_root);
      write_2bytes_eeprom(ADDR_F2,(unsigned int) f2);
      write_2bytes_eeprom(ADDR_KCPU_ROOT,(unsigned int) kcpu_root);
      write_2bytes_eeprom(ADDR_KCPU_T1,(unsigned int) kcpu_t1);
      write_2bytes_eeprom(ADDR_KV,(int) (kv*1000));
      write_2bytes_eeprom(ADDR_REC_EXT_ATTEN,(unsigned int) (1/extern_atten));
      write_2bytes_eeprom(ADDR_DAC_START,(unsigned int) dac_start);
      EEPROM.update(ADDR_FILT_SEL_MODE, filter_select_mode);
      write_2bytes_eeprom(ADDR_FILTER,init_filter);   
      write_2bytes_eeprom(ADDR_SETTLING_ROOT,settling_root);
      write_2bytes_eeprom(ADDR_MIN_FILTER,min_filter);
      write_2bytes_eeprom(ADDR_MAX_FILTER,max_filter);

      mySerial.println(F(" EEPROM loaded"));
//      mySerial.println(extern_atten,6);
//      mySerial.println((unsigned int) (1/extern_atten));

    break;
    
    case 'p':  // toggle gpsdo fll/pll mode
    case 'P':
        if (lock_mode == PLL) {
        lock_mode = FLL;
        mySerial.println("");
        mySerial.println(F(" FLL Mode"));
        
      }
      else {
        lock_mode = PLL;
        mySerial.println("");
        mySerial.println(F(" PLL Mode"));
      }
    break;

    case 'q':   // set Settling_root
    case 'Q':
      long_value = parseLong();
      if (long_value < 1) long_value = 1;
      if (long_value > 10000) long_value = 10000;
      settling_root = long_value;
      mySerial.println("");
      mySerial.print(F(" Settling Time (root) = "));
      mySerial.println(settling_root);
      
    break;

    case 'r':  // Put GPSDO into RUN mode (from HOLD)
    case 'R':
      if (run_state == RUN) {
        run_state = HOLD;
        mySerial.println("HOLD");
        second_count = 0;
        running_sum = 0;
        settling_count = 0;

      }
      else {
        run_state = RUN;
        mySerial.println("RUN");
      }
    break;

    case 's': 
    case 'S':
    break;

    case 't': // set reciprocal of external attenuation (see Simulink Model)
    case 'T':
      long_value = parseLong();
      if (long_value < 0) long_value = 0;
      if (long_value > 100) long_value = 100;
      extern_atten = 1/((float) long_value);
      mySerial.println("");
      mySerial.print(F(" External Attenuation RECIPROCAL = "));
      mySerial.println(1/extern_atten);
      break;

    case 'u': // Dump current operating values
    case 'U':
      print_current_values();
      break;

    case 'v': // set DAC initialization value
    case 'V':
      long_value = parseLong();
      if (long_value < 0) long_value = 0;
      if (long_value > 65535) long_value = 65535;
      dac_start = long_value;
      mySerial.println("");
      mySerial.print(F(" DAC Initialization Value = 0x"));
      mySerial.println(dac_start, HEX);
      break;

    case 'w': // set F1 (root)
    case 'W':
      long_value = parseLong();
      if (long_value < 0) long_value = 0;
      if (long_value > 32768) long_value = 32768;
      f1_root = (float) long_value;
      mySerial.println("");
      mySerial.print(F(" F1 (root) = "));
      mySerial.println(f1_root);
      break;

    case 'x': // set F2
    case 'X':
      long_value = parseLong();
      if (long_value < 0) long_value = 0;
      if (long_value > 32768) long_value = 32768;
      f2 = (float) long_value;
      mySerial.println("");
      mySerial.print(F(" F2 = "));
      mySerial.println(f2);
      break;

    case 'y': // set Kcpu (root)
    case 'Y':
      long_value = parseLong();
      if (long_value < 0) long_value = 0;
      if (long_value > 32768) long_value = 32768;
      kcpu_root = (float) long_value;
      mySerial.println("");
      mySerial.print(F(" Kcpu (root) = "));
      mySerial.println(kcpu_root);
      break;

    case 'z': // set Kcpu for Type 1 filter
    case 'Z':
      long_value = parseLong();
      if (long_value < 0) long_value = 0;
      if (long_value > 32768) long_value = 32768;
      kcpu_t1 = (float) long_value;
      mySerial.println("");
      mySerial.print(F(" Kcpu (Type 1 Filter) = "));
      mySerial.println(kcpu_t1);
      break;

    case '0':    // set DAC to 0x8000, hold mode
      run_state = HOLD;
      mySerial.println("   HOLD");
      dac = 0x8000;
      second_count = 0;
      running_sum = 0;
      settling_count = 0;
      write_to_dac(dac);
    break;

    case '1':    // Shera's Type 1 Filter
      second_count = 0;
      running_sum = 0;
      filter_type = TYPE1;
      aggregate_rdy = false;
      max_pd = 0;
      min_pd = 65535;
      filter = 1;
      mySerial.println("");
      mySerial.print(F(" filter = "));
      mySerial.println(filter);
    break;

    case '2':    // first of Shera's IIR filters
      second_count = 0;
      running_sum = 0;
      settling_count = 0;
      f1 = f1_root;
      out0 = out0*kcpu/(kcpu_root);  // adjust the 'dc' in the integrator to prevent jump (div old kcpu by new kcpu)
      kcpu = kcpu_root;
      filter_type = IIR;
      aggregate_rdy = false;
      max_pd = 0;
      min_pd = 65535;
      filter = 2;
      settling_limit = settling_root * ((int) (pow((float)2,(float) (filter - min_filter))+0.5));
      mySerial.println("");
      mySerial.print(F(" filter = "));
      mySerial.println(filter);
    break;

    case '3':
      second_count = 0;
      running_sum = 0;
      settling_count = 0;
      f1 = f1_root * 2;
      out0 = out0*kcpu/(kcpu_root/2);  // adjust the 'dc' in the integrator to prevent jump
      kcpu = kcpu_root / 2;
      aggregate_rdy = false;
      filter_type = IIR;
      max_pd = 0;
      min_pd = 65535;
      filter = 3;
      settling_limit = settling_root * ((int) (pow((float)2,(float) (filter - min_filter))+0.5));
      mySerial.println("");
      mySerial.print(F(" filter = "));
      mySerial.println(filter);
    break;

    case '4':
      second_count = 0;
      running_sum = 0;
      settling_count = 0;
      f1 = f1_root * 4;
      out0 = out0*kcpu/(kcpu_root/4);  // adjust the 'dc' in the integrator to prevent jump
      kcpu = kcpu_root / 4;
      filter_type = IIR;
      aggregate_rdy = false;
      max_pd = 0;
      min_pd = 65535;
      filter = 4;
      settling_limit = settling_root * ((int) (pow((float)2,(float) (filter - min_filter))+0.5));
      mySerial.println("");
      mySerial.print(F(" filter = "));
      mySerial.println(filter);
    break;

    case '5':
      second_count = 0;
      running_sum = 0;
      settling_count = 0;
      f1 = f1_root * 8;
      out0 = out0*kcpu/(kcpu_root/8);  // adjust the 'dc' in the integrator to prevent jump
      kcpu = kcpu_root / 8;
      filter_type = IIR;
      aggregate_rdy = false;
      max_pd = 0;
      min_pd = 65535;
      filter = 5;
      settling_limit = settling_root * ((int) (pow((float)2,(float) (filter - min_filter))+0.5));
      mySerial.println("");
      mySerial.print(F(" filter = "));
      mySerial.println(filter);
    break;

    case '6':
      second_count = 0;
      running_sum = 0;
      settling_count = 0;
      f1 = f1_root * 16;
      out0 = out0*kcpu/(kcpu_root/16);  // adjust the 'dc' in the integrator to prevent jump
      kcpu = kcpu_root / 16;
      filter_type = IIR;
      aggregate_rdy = false;
      max_pd = 0;
      min_pd = 65535;
      filter = 6;
      settling_limit = settling_root * ((int) (pow((float)2,(float) (filter - min_filter))+0.5));
      mySerial.println("");
      mySerial.print(F(" filter = "));
      mySerial.println(filter);
    break;

    case '7':   // ...last of Shera's filters
      second_count = 0;
      running_sum = 0;
      settling_count = 0;
      f1 = f1_root * 32;
      out0 = out0*kcpu/(kcpu_root/32);  // adjust the 'dc' in the integrator to prevent jump
      kcpu = kcpu_root / 32;
      filter_type = IIR;
      aggregate_rdy = false;
      max_pd = 0;
      min_pd = 65535;
      filter = 7;
      settling_limit = settling_root * ((int) (pow((float)2,(float) (filter - min_filter))+0.5));
      mySerial.println("");
      mySerial.print(F(" filter = "));
      mySerial.println(filter);
    break;

    case '8':   // set DAC to 0x0000
      run_state = HOLD;
      mySerial.println("   HOLD");
      second_count = 0;
      running_sum = 0;
      settling_count = 0;
      dac = 0X0000;
      write_to_dac(dac);
    break;
   
    case '9':    // set DAC to 0xFFFF
      run_state = HOLD;
      mySerial.println("   HOLD");
      second_count = 0;
      running_sum = 0;
      settling_count = 0;
      dac = 0XFFFF;
      write_to_dac(dac);
    break;

    default:
    break;
    
  }

}

void aggregate_samples()
{
  running_sum = running_sum + phase_delta;
  second_count++;
  if (phase_delta > max_pd) max_pd = phase_delta;
  if (phase_delta < min_pd) min_pd = phase_delta;
  if (second_count == d) {
    second_count = 0;
    d_sample = running_sum;
    running_sum = 0;
    aggregate_rdy = true;

  }
}

void write_2bytes_eeprom(uint16_t address, uint16_t int_data) {

  EEPROM.update(address, ((byte) (int_data & 0xFF)));
  int_data = int_data >> 8;
  address = address + 1;
  EEPROM.update(address, ((byte) (int_data & 0xFF)));
  
}


unsigned int read_2bytes_eeprom(uint16_t address) {

  uint8_t byte1;
  uint8_t byte2;
  uint16_t  read_int;
  
  byte1 = EEPROM.read(address);
  address = address + 1;
  byte2 = EEPROM.read(address);

  read_int = byte1 | (((uint16_t) byte2) << 8);

  return(read_int);
}

void print_current_values() {
      mySerial.println(F(" Current Values:"));
      mySerial.print(F(" ADC Max = "));
      mySerial.println(adc_max);
      mySerial.print(F(" DAC Init = "));
      mySerial.println(dac_start);
      mySerial.print(F(" D = "));
      mySerial.println(d);
      mySerial.print(F(" Setpoint = "));
      mySerial.println(setpoint);
      mySerial.print(F(" Lock Mode = "));
      if (lock_mode == PLL) mySerial.println(F("PLL"));
      else mySerial.println(F("FLL"));
      mySerial.print(F(" Initialization Filter = "));
      mySerial.print(init_filter);
      mySerial.print(F(", Current Filter Type = "));
      if (filter_type == IIR) {
        mySerial.print(F("IIR, "));
        mySerial.print(F(" filter: "));
        mySerial.println(filter);
        mySerial.print(F(" Filter Select Mode: "));
        if (filter_select_mode == AUTO) mySerial.println("AUTO");
        else mySerial.println("MANUAL");
        mySerial.print(F("   Min Filter = "));
        mySerial.print(min_filter);
        mySerial.print(F(", Max Filter = "));
        mySerial.println(max_filter);
        mySerial.print(F(" Settling Time (Root): "));
        mySerial.println(settling_root); 
        mySerial.print(F(" Settling Limit: "));
        mySerial.println(settling_limit); 
        mySerial.print(F(" MaxMin Limit = +/-"));
        mySerial.println(maxmin_limit);
        mySerial.print(F(" Upper wraparound limit = "));
        mySerial.print((unsigned int) upper_wraparound_lim);     
        mySerial.print(F(", Lower wraparound limit = "));
        mySerial.println((unsigned int) lower_wraparound_lim); 
        mySerial.print(F(" Dropback limit = "));
        mySerial.println(DROPBACK_LIMIT);    
      }
      else {
        mySerial.println(F("Type 1"));
        
      }
      mySerial.print(F(" F1 = "));
      mySerial.println(f1);
      mySerial.print(F(" F1 (root) = "));
      mySerial.println(f1_root);
      mySerial.print(F(" F2 = "));
      mySerial.println(f2);
      mySerial.print(F(" Kcpu = "));
      mySerial.println(kcpu);
      mySerial.print(F(" Kcpu (root) = "));
      mySerial.println(kcpu_root);
      mySerial.print(F(" Kcpu Type 1 Filter = "));
      mySerial.println(kcpu_t1);
      mySerial.print(F(" Kv = "));
      mySerial.println(kv, 3);
      mySerial.print(F(" 1/External_Attenuation = "));
      mySerial.println(1/extern_atten);
      mySerial.print(F(" DAC = "));
      mySerial.println(dac);
  
}

void calc_floating_point_reciprocals() {
  // just a test to see how accurate
  // the reciprocals of f1 and f2 are,
  // in floating point
  float ff1;
  float ff2;
  float rec_f1;
  float rec_f2;
  float sum_f1f2;
  float diff_f1f2;
  float delta_f1f2;
  
  ff2 = (float) 8;
  rec_f2 = 1/ff2;
  
  int i;
  int ii;
  
  for (i=0; i<=6; i++) {
    ii = (int) (pow((float)2,(float) i) + 0.5);
    ff1 = ((float)2048)*((float)ii);
    rec_f1 = 1/ff1;
    sum_f1f2 = rec_f1 + rec_f2;
    diff_f1f2 = rec_f1 - rec_f2;
    delta_f1f2 = sum_f1f2-diff_f1f2;
  
    mySerial.print(i);
    mySerial.print(",");
    mySerial.print(ii);
    mySerial.print(",");
    mySerial.print(ff1);
    mySerial.print(",");
    mySerial.print(ff2);
    mySerial.print(",");
    mySerial.print(rec_f1,10);
    mySerial.print(",");
    mySerial.print(rec_f2,10);
    mySerial.print(",");
    mySerial.print(sum_f1f2,10);
    mySerial.print(",");
    mySerial.print(diff_f1f2,10);
    mySerial.print(",");
    mySerial.println(delta_f1f2,16);
  
  }
  
}

