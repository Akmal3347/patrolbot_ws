#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# --- key bindings ---
# i = forward, k = backward, j = left, l = right
# space = stop
settings = termios.tcgetattr(sys.stdin)

msg = """
Control Your Robot!
---------------------------
Moving around:
        i    
   j    k    l

i : move forward
k : move backward
j : turn left
l : turn right
space : force stop

q/z : increase/decrease max speeds
CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rclpy.init()
    node = rclpy.create_node('ijkl_teleop')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    speed = 0.5 # Linear speed
    turn = 1.0  # Angular speed
    x = 0.0
    th = 0.0
    status = 0

    try:
        print(msg)
        while(1):
            key = getKey()
            
            # LOGIC FOR IJKL KEYS
            if key == 'i':
                x = -speed
                th = 0.0
            elif key == 'k':
                x = speed
                th = 0.0
            elif key == 'j':
                x = 0.0
                th = -turn
            elif key == 'l':
                x = 0.0
                th = turn
            elif key == ' ': # Spacebar to stop
                x = 0.0
                th = 0.0
                print("STOP")
            elif key == 'q':
                speed = speed * 1.1
                print(f"Speed: {speed}")
            elif key == 'z':
                speed = speed * 0.9
                print(f"Speed: {speed}")
            elif key == '\x03': # CTRL-C
                break

            # Publish the message
            twist = Twist()
            twist.linear.x = float(x)
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(th)
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # Stop the robot when the script ends
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()