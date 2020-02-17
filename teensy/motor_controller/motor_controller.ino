//The required includes for the motor controller
#include "Encoder.h" //This library includes the encoder class, such that you can obtain the number of ticks counted so far.
#include "MsTimer2.h"  //This library allows for timer interrupts
#include <ros.h>  //This includes the ROS overlay
#include "my_msgs/Vel.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

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


//This defines an enumeration. You can define your states here for Finite State Machine behaviour of your update loop.
enum State
{
  DUMMY_STATE, //you can remove this state, it's just an example on how to add elements in an enumeration
};

State state = State::DUMMY_STATE;  //This is an example on how to set the current state to one of the values from the enumeration.
// For comparison, you can use the following:
// switch(state)
// {
//    case State::DUMMY_STATE:
//      do_stuff();
//      break;
//    case other state.... etc.

// }

//or:

// if (state == State::DUMMY_STATE)
// {
//   do_stuff()
// } else if (state == ....) etc.

// ************* Publishers ***************************
// These publishers are provided to make visualisation prossible for tuning your PID controllers.
std_msgs::Float64 wheel_msg;
ros::Publisher left_wheel_publisher("left_vel", &wheel_msg);
ros::Publisher right_wheel_publisher("right_vel", &wheel_msg); 




// *********** Subscriber callback functions *****************************

//These callbacks receive PID values, and updates the PID controller with these new values;
void P_cb(std_msgs::Float32 const &msg){PID_left.P(msg.data);PID_right.P(msg.data);}
void I_cb(std_msgs::Float32 const &msg){PID_left.I(msg.data);PID_right.I(msg.data);}
void D_cb(std_msgs::Float32 const &msg){PID_left.D(msg.data);PID_right.D(msg.data);}


// Set the left and right wheel velocities in rads/s based on the provided
// body velocities. Where translational is the forward speed in m/s and
// rotational the rotational speed around the z axis in rads/s.
void setTargetVelocity(double translational, double rotational)
{
    double left_wheel_vel = 0;   //put your wheel velocity calculations here (was previously in python)
    double right_wheel_vel = 0;   //put your wheel velocity calculations here (was previously in python)

    PID_left.set_target(left_wheel_vel);
    PID_right.set_target(right_wheel_vel);
}

// This callback receives robot target velocities from outside the Teensy.
void cmd_vel_cb(geometry_msgs::Twist const &msg)
{
   
    double translational = msg.linear.x;
    double rotational = msg.angular.z;
    setTargetVelocity(translational, rotational);
    last_time = millis();
}



void pos_cmd_cb(geometry_msgs::Pose2D const &msg)
{
  // msg contains the target location for position control
  // msg contains:
  // msg.x
  // msg.y
  // msg.theta

  //The received message contains coordinates of the target location in the robot frame of reference.
  //The mathematics behind position control, uses the robot position within the target-location frame of reference though.
  //The following mapping will map the coordinates from the robot frame of reference to the latter frame of reference:

  //express the target location in polar coordinates:
  double distance = sqrt(msg.x * msg.x + msg.y * msg.y);
  double angle = atan2(msg.y, msg.x);

  //Now map to the other frame of reference in polar coordinates:
  angle += 3.14;
  // and add the rotation theta to this frame of reference:
  // The robot angle within the target frame of reference:

  double theta = msg.theta * -1;
  // Rotate the new frame of reference with this angle:
  angle += theta;

  //So now the robot coordinates in the target frame of reference is:
  double x = distance * cos(angle);
  double y = distance * sin(angle);
  // and theta is the angle of the robot in the target frame of reference.
  // Use these coordinates for position control calculations

}

// *************** The subscribers *************************

ros::Subscriber<geometry_msgs::Pose2D> pose_sub("pos_cmd", pos_cmd_cb);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_cb);
ros::Subscriber<std_msgs::Float32> pid_p_sub("PID/P", P_cb);
ros::Subscriber<std_msgs::Float32> pid_i_sub("PID/I", I_cb);
ros::Subscriber<std_msgs::Float32> pid_d_sub("PID/D", D_cb);
//add your publisher/subscriber objects here



//This function is called after the Teensy is reprogrammed, or when powered on.
void setup() {
  //This part initializes the ROS node, and tells ROS about the declared publishers and subscribers.
  node_handle.initNode();
  node_handle.advertise(left_wheel_publisher);
  node_handle.advertise(right_wheel_publisher);
  node_handle.subscribe(cmd_vel_sub);
  node_handle.subscribe(pose_sub);
  node_handle.subscribe(pid_p_sub);
  node_handle.subscribe(pid_i_sub);
  node_handle.subscribe(pid_d_sub);

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

  // When calculating the difference between the current and previous encoder state, 
  // you can use the following variables:
  // - new_left: the current encoder value for left
  // - new_right: the current encoder value for right
  // - left_pos: the previous encoder value for left
  // - right_pos: the previous encoder value for right
  
  // vel_left = ... calculate the wheel velocity for the left wheel here in rads/s.
  // vel_right = ... calculate the wheel velocity for the right wheel here in rads/s.


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
