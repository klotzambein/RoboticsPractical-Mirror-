import rospy
from geometry_msgs.msg import Twist

#declare functions here

# return the left and right wheel velocities in m/s based on the provided
# body velocities. Where translational is the forward speed in m/s and
# rotational the rotational speed around the z axis in radians.
def determine_wheel_command(translational, rotational):
    left_wheel_vel = translational - (rotational * track / 2)
    right_wheel_vel = translational + (rotational * track / 2)

    return left_wheel_vel, right_wheel_vel

## This function is called every time a new velocity command is given
def cmd_vel_callback(data):
    vel_left, vel_right = determine_wheel_commands(data.linear.x, data.angular.z)
    cur_config['translational'] = data.linear.x
    cur_config['rotational'] = data.angular.z
    send(vel_left, vel_right)


## add code here as if it where the main function
if __name__ == "__main__":
    rospy.init_node("velocity_control_node")
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)

    rospy.spin()