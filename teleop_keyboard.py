import os
import select
import sys
import rclpy
from std_msgs.msg import String
import sys, tty, termios
import threading
import time
import Jetson.GPIO as GPIO
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

gun_pin = 13
laser_pin = 22

MAX_LIN_VEL = 1.20
MAX_ANG_VEL = 1.00

LIN_VEL_STEP_SIZE = 0.10
ANG_VEL_STEP_SIZE = 0.10

msg = """
---------------------------
Moving around:
        w
   a    s    d
        x

pole :
   q    -    e
   -    -    -
        -

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    return input_vel


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)


def main():
    settings = None
    laser_on = False
    gun_on = False
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(laser_pin, GPIO.OUT)
    GPIO.setup(gun_pin, GPIO.OUT)
    GPIO.output(laser_pin, GPIO.LOW)
    GPIO.output(gun_pin, GPIO.LOW)
    
    rclpy.init()

    qos = QoSProfile(depth=10)
    node_wheel = rclpy.create_node('teleop_keyboard')
    pub_wheel = node_wheel.create_publisher(Twist, 'cmd_vel', qos)

    pole_msg = String()
    node_pole = rclpy.create_node('control_pole')
    pub_pole = node_pole.create_publisher(String, '/stairs_check', qos)
    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0

    executor = MultiThreadedExecutor()
    executor.add_node(node_wheel)
    executor.add_node(node_pole)
    
    thread = threading.Thread(target=executor.spin)
    thread.start()

    try:
        print(msg)
        while True:
            key = get_key(settings)
            if key == 'w' or key == 'W':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'x' or key == 'X':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'a' or key == 'A':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'd' or key == 'D':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == ' ' or key == 's' or key == 'S':
                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'e' or key == 'E':
                pole_msg.data = 'over'
                pub_pole.publish(pole_msg)
                print("Successfully sent the msg for pole")
            elif key == 'q' or key == 'Q':
                pole_msg.data = 'back'
                pub_pole.publish(pole_msg)
                print("Successfully sent the msg for pole")
            elif key == 'l' or key == 'L':
                if (laser_on):
                    GPIO.output(laser_pin, GPIO.LOW)
                    laser_on = False
                    print("Laser off")
                else:
                    GPIO.output(laser_pin, GPIO.HIGH)
                    laser_on = True
                    print("Laser on")
            elif key == 'g' or key == 'G':
                if (gun_on):
                    GPIO.output(gun_pin, GPIO.LOW)
                    gun_on = False
                    print("Gun off")
                else:
                    GPIO.output(gun_pin, GPIO.HIGH)
                    gun_on = True
                    print("Gun on")
            else:
                if key == '\x03':
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()
            twist.linear.x = target_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = target_angular_velocity

            pub_wheel.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub_wheel.publish(twist)

        node_wheel.destroy_node()
        node_pole.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
