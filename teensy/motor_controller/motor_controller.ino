//The required includes for the motor controller
#include "Encoder.h" //This library includes the encoder class, such that you can obtain the number of ticks counted so far.
#include "MsTimer2.h"  //This library allows for timer interrupts
#include <ros.h>  //This includes the ROS overlay
#include "my_msgs/Vel.h"
#include <std_msgs/Float64.msg>
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

//These defines define some constants
#define UPATE_RATE 20  //in ms. Hence actual rate (Hz) = 1 / (update_rate / 1000). So, for 20 ms => 50 Hz
#define per_rot 1200.0  // number of ticks per full wheel rotation
#define wheel_cir 0.18   // The wheel circumference in meters. You might have to fine tune this for your vehicle  
#define report_count 10

//The PID controller for the left and right wheel
PID PID_left(0, 0, 0);   //you can put your tuned PID values here. The arguments are P,I,D.
PID PID_right(0, 0, 0);  //you can put your tuned PID values here. The arguments are P,I,D.

//The encoders to read the number of encoder ticks for the left and right wheel
Encoder motor_left(ENCA1,ENCA2);
Encoder motor_right(ENCB1,ENCB2);

// some global variables to store the current and previous encoder values.
long left_pos = 0;
float vel_left = 0;
long right_pos = 0;
float vel_right = 0;

long last_time = millis();

// The nodehandle as you saw in the ROS intro documentation
ros::NodeHandle node_handle;
// These publishers are provided to make visualisation prossible for tuning your PID controllers.
std_msgs::Float64 wheel_msg;
ros::Publisher left_wheel_publisher("left_vel", &wheel_msg);
ros::Publisher right_wheel_publisher("right_vel", &wheel_msg); 

// Update PID settings and target velocities. 
// This callback receives the target wheel velocities you send from the Jetson in Python.
void pid_vel_cb(const my_msgs::Vel& msg)
{
  PID_left.set_terms(msg.kP,msg.kI,msg.kD);
  PID_right.set_terms(msg.kP,msg.kI,msg.kD);
  PID_left.set_target(msg.left_vel);
  PID_right.set_target(msg.right_vel);
  last_time = millis();
}
ros::Subscriber<my_msgs::Vel> pid_sub("PID_vel", &pid_vel_cb);

//add your publisher/subscriber objects here



//This function is called after the Teensy is reprogrammed, or when powered on.
void setup() {
  //This part initializes the ROS node, and tells ROS about the declared publishers and subscribers.
  node_handle.initNode();
  node_handle.subscribe(pid_sub);
  node_handle.advertise(left_wheel_publisher);
  node_handle.advertise(right_wheel_publisher);

  //This part sets the hardware I/O configuration
  pinMode(APHASE, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BPHASE, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBYA, OUTPUT);
  pinMode(STBYB, OUTPUT);
  analogWriteFrequency(PWMA,10000);
  analogWriteFrequency(PWMB,10000);
  analogWriteResolution(12);
  
  //This part sets the PID target velocities to 0
  PID_left.set_target(0);
  PID_right.set_target(0);

  //This part begins a serial communication
  Serial.begin(57600);

  //This part sets up the timed updates
  MsTimer2::set(UPATE_RATE, timed_update_callback); 
  MsTimer2::start();

  //You can do some initialization below here in this function.
}



void timed_update_callback(){
  //This if scope makes sure that if for some reason this function is not updated properly,
  // the PID's get instructions to stop the wheels.
  if (millis() - last_time > 500)
  {
    PID_left.set_target(0);
    PID_right.set_target(0);
    last_time = millis();
  }


  long new_left = motor_left.read();  //get the newest encoder values for the left wheel
  long new_right = motor_right.read(); //get the newest encoder values for the right wheel

  // At the top of this file, the following constants are declared. (line 25 and 26)
  // You can use these constants for you calculations below
  // #define per_rot 1200.0  // number of ticks per full wheel rotation
  // #define wheel_cir 0.18  // The wheel circumference in meters. You might have to fine tune this for your vehicle  

  // When calculating the difference between the current and previous encoder state, 
  // you can use the following variables:
  // - new_left: the current encoder value for left
  // - new_right: the current encoder value for right
  // - left_pos: the previous encoder value for left
  // - right_pos: the previous encoder value for right
  
  // vel_left = ... calculate the wheel velocity for the left wheel here in m/s. 
  // vel_right = ... calculate the wheel velocity for the right wheel here in m/s.


  //The following lines pass the calculated wheel velocities to the PID controller, and updates the motor commands
  float left_throttle = PID_left.update(vel_left);
  float right_throttle = PID_right.update(vel_right);
  drive_motors(left_throttle, right_throttle);

  // The following lines publish the calculated velocities. This is required for the visualisation of current velocities.
	wheel_msg.data = vel_left;
	left_wheel_publisher.publish(&wheel_msg);
	wheel_msg.data = vel_right;
	right_wheel_publisher.publish(&wheel_msg);

  //These lines store the encoder states of this timestep.
  left_pos = new_left;
  right_pos = new_right;
}

//Handles communication with the SerialNode on the Jetson. Don't touch this function.
void loop() {
  node_handle.spinOnce();
  delay(1);
}
