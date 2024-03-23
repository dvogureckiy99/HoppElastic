// Developer and maintainer: Ogureckiy Dmitriy, ITMO university
// last update 29.01.24 14:40

#define F_CPU 20000000L

#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include "DynamixelSerial.h"
#include "MPU6050.h"
// #include <avr/cpufunc.h>

#define AX_18A 0
#define AX_12A 1
#define AX_18A_MAX_SPEED 582
#define AX_12A_MAX_SPEED 354

#define MODE_ID   82 // AX18A
#define MOTION_ID 81 // AX18A

// main parameters
#define MOTION_A 60 // [grad] amplitude of sin
#define MOTION_EQUIL 80 // [grad]
#define MOTION_VEL_START AX_18A_MAX_SPEED // [cmd] AX_18A_MAX_SPEED=max, proportional to the FREQ of SINE
#define MOTION_PERIOD_REAL 400
 
#define MODE_A 45 // [grad] amplitude of sin
#define MODE_EQUIL 145 // [grad]
#define MODE_VEL_START AX_18A_MAX_SPEED // [cmd] AX_18A_MAX_SPEED=max, proportional to the FREQ of SINE
#define MODE_PERIOD_REAL MOTION_PERIOD_REAL

#define PHASE_SHIFT_REAL M_PI/2 // [rad]

#define PHASE_SHIFT MODE_PERIOD_REAL*PHASE_SHIFT_REAL/(2*M_PI) // phase shift in msec
#define MODE_PERIOD MODE_PERIOD_REAL/10 //  REAL_PERIOD=MOTION_PERIOD*5 msec
#define MODE_HALFPERIOD MODE_PERIOD/2
#define MOTION_PERIOD MOTION_PERIOD_REAL/10 //  REAL_PERIOD=MOTION_PERIOD*5 msec
#define MOTION_HALFPERIOD MOTION_PERIOD/2

void initMotors(void);
void sendMotionMotorStates(void);
int transform_velocity_fl(int motor,float vel);
float transform_position_comm2ang_fl(int angle);
void RTC_init(void);
int transform_position_ang2comm_fl(float angle);
void communication_cycle(void);

uint8_t flag_RTC_Overflow_happened = 0;
volatile uint16_t PIT_counter = 0; // 1 tick = 0.1220703125 msec = 122.0703125  usec. max value = 8 sec

uint32_t control_cycle_MOTION = 0; // count number of control cycle execution
uint32_t control_cycle_MODE = 0;
uint8_t communication_cycle_cnt = 0;
// uint8_t flag_direction = 1;//1 - clockwise, 0 -counter clockwise, 1 at the start
// uint32_t real_time_counter_4CPUticks = 0; //real time in msec with dt=0.1220703125

// for amplitudes parameters
int position_desired_up_motion ;
int position_desired_down_motion;
int position_desired_up_mode ;
int position_desired_down_mode;

int motion_vel; 
int mode_vel; 
int motion_pos ; // present postition 
int mode_pos ; // present postition 
int motion_load;
int mode_load;
uint64_t time = 0;
// const int buf_size = sizeof(time)+sizeof(motion_pos)+sizeof(mode_pos)+sizeof(motion_load)+sizeof(mode_load)+7;

uint8_t flag_direction_motion = 1;
uint8_t flag_direction_mode = 1;

void setup() {
  delay(2000);

  CLKCTRL.MCLKCTRLB &= ~CLKCTRL_PEN_bm; // disable MAIN_CLOCK PRESCALER

  Serial.begin(256000);
  initMotors();
  RTC_init();

  /* Enable Global Interrupts */
  sei();

  motion_vel = MOTION_VEL_START;
  mode_vel = MODE_VEL_START;

  position_desired_down_motion  =  transform_position_ang2comm_fl(MOTION_EQUIL - MOTION_A);
  position_desired_up_motion    = transform_position_ang2comm_fl(MOTION_EQUIL + MOTION_A) ;
  position_desired_down_mode  = transform_position_ang2comm_fl(MODE_EQUIL - MODE_A);
  position_desired_up_mode    = transform_position_ang2comm_fl(MODE_EQUIL) ;

  Dynamixel.moveSpeed(MOTION_ID, position_desired_up_motion, motion_vel );
  // delay(PHASE_SHIFT); 
  Dynamixel.moveSpeed(MODE_ID, position_desired_up_mode, mode_vel );
  // this need for control sequence doesn't call immidiatelly after entering in main loop
  // cause RTC timer will be overflowed anyway to this time 
  // the same for PIT counter
  PIT_counter = 0;
  RTC.INTFLAGS = RTC_OVF_bm;
  flag_RTC_Overflow_happened = 0;
}

uint8_t counter_of_data_order = 0; // 1-6

void loop() {
    
// ^^^^^^^^^^^^^^^^^^^^^^^ sine control, synchronization and calibration loop ^^^^^^^^^^^^^^^^^^^^^^^
  if(flag_RTC_Overflow_happened){ // __ ms passed
    flag_RTC_Overflow_happened = 0; // flag is to zero until next ___ ms interval passed
    control_cycle_MOTION ++; 
    control_cycle_MODE ++;
    communication_cycle_cnt ++;
    // communication_cycle();
    // motion_pos = transform_position_comm2ang_fl(Dynamixel.readPosition(MOTION_ID));
    // mode_pos = transform_position_comm2ang_fl(Dynamixel.readPosition(MODE_ID));
  }
// ^^^^^^^^^^^^^^^^^^^^^^^ end of sine control and calibration loop ^^^^^^^^^^^^^^^^^^^^^^^

// @@@@@@@@@@@@@@@@@@@@@@@ sine control algorithm @@@@@@@@@@@@@@@@@@@@@@@ 
  if(control_cycle_MOTION >= MOTION_HALFPERIOD){
    if(flag_direction_motion){
      flag_direction_motion = 0;
      Dynamixel.moveSpeed(MOTION_ID, position_desired_down_motion, motion_vel );
    }else{
      flag_direction_motion = 1;
      Dynamixel.moveSpeed(MOTION_ID, position_desired_up_motion, motion_vel );
    }
    control_cycle_MOTION = 0;
  }
  
  // if(control_cycle_MODE >= MODE_HALFPERIOD ){
  //   if(flag_direction_mode){
  //     flag_direction_mode = 0;
  //     Dynamixel.moveSpeed(MODE_ID, position_desired_down_mode , mode_vel );
  //   }else{
  //     flag_direction_mode = 1;
  //     Dynamixel.moveSpeed(MODE_ID, position_desired_up_mode, mode_vel );
  //   }
  //   control_cycle_MODE = 0;
  // }
  // @@@@@@@@@@@@@@@@@@@@@@@ end of sine control algorithm @@@@@@@@@@@@@@@@@@@@@@@

// ------------------- serial communication procedure -----------------  
  if(communication_cycle_cnt > 1){
    time += PIT_counter;
    PIT_counter = 0;
    communication_cycle();
    communication_cycle_cnt = 0;
  }
// ------------------- end of serial communication procedure -----------------  
}

void communication_cycle(void){
  motion_load = Dynamixel.readLoad(MOTION_ID);
  mode_load = Dynamixel.readLoad(MODE_ID);
  motion_pos = Dynamixel.readPosition(MOTION_ID);
  mode_pos = Dynamixel.readPosition(MODE_ID);
  // motion_vel = Dynamixel.readSpeed(MOTION_ID);
  // mode_vel = Dynamixel.readSpeed(MODE_ID);
  sendMotionMotorStates();
}

int transform_velocity_fl(int motor,float vel){
  // input velocity in grad/sec.
  const float gain_vel2com = 1.5;
  float uppper_speed_limit;
  if(motor == AX_18A){  
    uppper_speed_limit = AX_18A_MAX_SPEED;  // 582 grad/sec, 97 rpm, 873 cmd max speed for AX18A
  }else if(motor == AX_12A){
    uppper_speed_limit = AX_12A_MAX_SPEED;
  }
  if(vel >= uppper_speed_limit){
    vel = uppper_speed_limit;
  }else if(vel <= 0){
    vel = 0;
  }
  return round(vel*gain_vel2com);
}

float transform_position_comm2ang_fl(int angle){
  const float gain_comm2ang = 0.2932551319648094;
  return ((float)angle) * gain_comm2ang; // in grad
}

int transform_position_ang2comm_fl(float angle){
  if(angle >= 300){
    angle = 300;
  }else if(angle <= 0){
    angle = 0;
  }
  return round(angle * 3.4099999999999997);
}

void initMotors(void){
  Dynamixel.setSerial(&Serial1);
  Dynamixel.begin(1000000, 2);

  Dynamixel.setTempLimit(MOTION_ID, 0xFF);
  Dynamixel.setMaxTorque(MOTION_ID, 0x03FF);
  Dynamixel.setAngleLimit(MOTION_ID, 0x0000, 0x03FF);
  Dynamixel.setTempLimit(MODE_ID, 0xFF);
  Dynamixel.setMaxTorque(MODE_ID, 0x03FF);
  Dynamixel.setAngleLimit(MODE_ID, 0x0000, 0x03FF);

  delay(2000);
  Dynamixel.moveSpeed(MOTION_ID, transform_position_ang2comm_fl(MOTION_EQUIL), MOTION_VEL_START );
  delay(2000);
  Dynamixel.moveSpeed(MODE_ID, transform_position_ang2comm_fl(MODE_EQUIL), MODE_VEL_START );
  delay(2000);
}

void sendMotionMotorStates(void){
  
  // uint8_t buf[buf_size];
  // for(int i=0;i<8;i++){
  //   buf[i] = time >> (i*8);
  // } 
  // buf[8] = ',';
  // for(int i=0;i<2;i++){
  //   buf[i] = motion_pos >> (i*8);
  // } 
  // buf[11] = ',';
  // for(int i=0;i<2;i++){
  //   buf[i] = mode_pos >> (i*8);
  // } 
  // buf[14] = ',';
  // for(int i=0;i<2;i++){
  //   buf[i] = motion_load >> (i*8);
  // } 
  // buf[17] = ',';
  // for(int i=0;i<2;i++){
  //   buf[i] = mode_load >> (i*8);
  // } 
  // buf[20] = ';';
  // buf[21] = 13;
  // buf[22] = 10;
  // Serial.write(buf,buf_size);


  Serial.print(time);
  Serial.print(", ");
  Serial.print(motion_pos);
  Serial.print(", ");
  Serial.print(mode_pos);
  Serial.print(", ");
  // Serial.print(motion_vel);
  // Serial.print(", ");
  // Serial.print(mode_vel);
  // Serial.print(", ");
  Serial.print(motion_load);
  Serial.print(", ");
  Serial.print(mode_load);
  Serial.println(";"); 
}

void RTC_init(void){   
    uint8_t temp;
    /* Initialize RTC: */
    while (RTC.STATUS > 0)
    {
        ; /* Wait for all register to be synchronized */
    }
    
    /* Set period  1 tick = 0.030517578125  msec */
    RTC.PER = 328; 

    /* Clock Selection 32.768 kHz from OSCULP32K */
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;

    /* Run in debug: disabled */
    temp = RTC.DBGCTRL;
    temp &=  ~RTC_DBGRUN_bm;
    RTC.DBGCTRL = temp;
    
    RTC.PITINTCTRL = RTC_PI_bm; /* Periodic Interrupt: enabled */
    RTC.PITCTRLA = RTC_PERIOD_CYC4_gc /* RTC Clock Cycles 4 */
    | RTC_PITEN_bm; /* Enable: enabled */

    RTC.CTRLA = RTC_PRESCALER_DIV1_gc  /* 32.768 kHz */
    | RTC_RTCEN_bm            /* Enable: enabled */
    & ~RTC_RUNSTDBY_bm;        /* don't run In Standby: enabled */
    
    /* Enable Overflow Interrupt */
    RTC.INTCTRL |= RTC_OVF_bm;
}

ISR(RTC_PIT_vect){
  // run every 4/32768 sec = 0.1220703125 msec = 122.0703125 usec 
  PIT_counter ++;
  // real_time_counter_4CPUticks ++ ; // real time counter in 4 CPU ticks
  // flag_RTC_PIT_happened = 1;
  /* Clear flag by writing '1': */
  RTC.PITINTFLAGS = RTC_PI_bm;
}

ISR(RTC_CNT_vect){
  // run every 32/32768 sec = 0.9765624992  ms
  // run every 98/32768 sec = 2.99072265625  ms
  // run every 132/32768 sec = 4.0283203125   ms  
  // 164 - 5.0048828125 msec
  // 197 - 6.011962890625 msec
  // 229 - 6.988525390625 msec
  // 328 - 10.009765625  msec
  flag_RTC_Overflow_happened = 1;
  /* Clear flag by writing '1': */
  RTC.INTFLAGS = RTC_OVF_bm;
}

