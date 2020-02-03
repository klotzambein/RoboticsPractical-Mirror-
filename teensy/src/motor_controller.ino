//The required includes for the motor controller
#include "Encoder.h" //This library includes the encoder class, such that you can obtain the number of ticks counted so far.
#include <MsTimer2.h>  //This library allows for timer interrupts
#include <ros.h>  //This includes the ROS overlay
#include "PID.h"  //This includes the PID class you will be using


//The folling defines defines constants required for interfacing with the hardware. Please don't edit this.
#define ENCA1 27
#define ENCA2 26
#define ENCB1 39
#define ENCB2 38

#define APHASE 29
#define PWMA 30
#define STBYA 28
#define BPHASE 36
#define PWMB 35
#define STBYB 37

//TODO: Should students obtain these constants themselves?
#define update_rate 20  //in ms. Hence actual rate (Hz) = 1 / (update_rate / 1000). So, for 20 ms => 50 Hz
#define per_rot 1200.0  // number of ticks per full wheel rotation
#define wheel_cir 0.18  //you might have to fine tune this for your vehicle
#define report_count 10


//The encoders to read the number of encoder ticks for the left and right wheel
Encoder motor_left(ENCA1,ENCA2);
Encoder motor_right(ENCB1,ENCB2);

//some global variables you might want to use.
long left_pos = 0;
float vel_left = 0;
long right_pos = 0;
float vel_right = 0;

long last_time = millis();

//The nodehandle as you saw in the ROS intro documentation
ros::NodeHandle nodeHandle;
nodeHandle.initNode();

//add your publisher/subscribe objects here


void setup() {
  
  
  pinMode(APHASE, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BPHASE, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBYA, OUTPUT);
  pinMode(STBYB, OUTPUT);
    
  analogWriteFrequency(PWMA,10000);
  analogWriteFrequency(PWMB,10000);
  analogWriteResolution(12);
  
  Serial.begin(9600);
  MsTimer2::set(vel_time,timed_update_callback); 
  MsTimer2::start();

  //You can do some initialization below here in this function.
}


//Do your stuff from this function
void timed_update_callback(){

  //long new_encoder_ticks_left = motor_left.read();
  //long new_encoder_ticks_right = motor_right.read();
  //drive_motors(left_throttle, right_throttle);

}

//Handles communication with the SerialNode on the Jetson. Don't touch this function.
void loop() {
  nh.spinOnce();
  delay(1);
}
