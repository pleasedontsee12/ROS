#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

import sys
import select
import termios
import tty

# Constants from OCR (adjust if necessary)
MAX_LIN_VEL = 100.0 # Note: Units might depend on the controller. 100 seems high for m/s.
MAX_ANG_VEL = 1.0   # Radians per second
LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.01

msg = """
---------------------------
Moving around:
   w
a  s  d
   x

w/x : increase/decrease linear velocity ([-{:.1f}, {:.1f}])
a/d : increase/decrease angular velocity ([-{:.1f}, {:.1f}])

space key, s : force stop

CTRL-C to quit
---------------------------
""".format(MAX_LIN_VEL, MAX_LIN_VEL, MAX_ANG_VEL, MAX_ANG_VEL) # Use constants in help message

e = """
Communications Failed
"""

# Function to read a single key press without waiting for Enter
def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    # Use select for non-blocking read with a short timeout
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Function to print current velocities
def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {:.2f}\t angular velocity {:.2f} '.format(
          target_linear_velocity, target_angular_velocity))

# Function to constrain a value within bounds
def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    return input_vel

# Function to check and constrain linear velocity
def check_linear_limit_velocity(velocity):
    # Using abs(MAX_LIN_VEL) for symmetric limits
    return constrain(velocity, -abs(MAX_LIN_VEL), abs(MAX_LIN_VEL))

# Function to check and constrain angular velocity
def check_angular_limit_velocity(velocity):
    # Using abs(MAX_ANG_VEL) for symmetric limits
    return constrain(velocity, -abs(MAX_ANG_VEL), abs(MAX_ANG_VEL))


def main(args=None):
    settings = termios.tcgetattr(sys.stdin) # Get current terminal settings

    rclpy.init(args=args)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0

    try:
        print(msg)
        while(1):
            key = get_key(settings)

            if key == 'w':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'x':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            # OCR Note: The OCR's a/d seems to implement increase/decrease differently than standard ROS
            # 'a' decreases the value, 'd' increases it based on OCR page 45.
            # Standard ROS: positive angular.z = turn left (CCW), negative = turn right (CW)
            # If controller.py uses steer = -angular.z, then:
            #   Positive angular.z -> Negative steer -> turn left (controller.py)
            #   Negative angular.z -> Positive steer -> turn right (controller.py)
            # So, 'a' (turn left) should yield positive angular.z, 'd' (turn right) negative angular.z.
            # The OCR code below might result in 'a' turning right and 'd' turning left depending on the controller.
            # Implementing exactly as OCR shows:
            elif key == 'a':
                 target_angular_velocity =\
                     check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE) # OCR uses MINUS
                 status += 1
                 print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'd':
                 target_angular_velocity =\
                     check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE) # OCR uses PLUS
                 status += 1
                 print_vels(target_linear_velocity, target_angular_velocity)
            elif key == ' ' or key == 's':
                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == '\x03': # CTRL-C
                break
            else:
                # Don't reset velocities if no valid key is pressed, just continue
                # (Unless it's space or 's' handled above)
                if key == '': # Only publish if a command key was pressed or stop keys
                   pass # No key pressed
                #else: # Optional: Handle unexpected keys
                #   print(f"Unhandled key: {key}")


            # Create and publish Twist message
            twist = Twist()
            twist.linear.x = target_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            # Note the angular velocity mapping based on OCR's a/d logic
            twist.angular.z = target_angular_velocity
            pub.publish(twist)

            # Reset status counter (seems intended to print velocity only on change)
            if status == 14: # Arbitrary number from OCR? Print every ~1.4s if timeout is 0.1s
                             # Might be better to print only if velocity *changes*
                print(msg)
                status = 0


    except Exception as ex:
        print(e) # Print communication failed message
        print(ex) # Print the actual exception

    finally:
        # Stop the robot when exiting
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()