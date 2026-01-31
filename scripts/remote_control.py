#!/usr/bin/env python3
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Control Your Robot!
---------------------------
   w
a  s  d

w/s : forward/backward
a/d : turn left/right
q   : quit

HOLD the key to move. RELEASE to stop.
"""

class RemoteControl(Node):
    def __init__(self):
        super().__init__('remote_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.5
        self.turn = 1.0
        self.timeout = 0.1 # Stop if no key for 0.1 seconds (fast response)

    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        print(msg)
        
        try:
            while rclpy.ok():
                key = self.getKey()
                
                twist = Twist()
                if key == 'w':
                    twist.linear.x = self.speed
                elif key == 's':
                    twist.linear.x = -self.speed
                elif key == 'a':
                    twist.angular.z = self.turn
                elif key == 'd':
                    twist.angular.z = -self.turn
                elif key == 'q':
                    break
                else:
                    # If unknown key or timeout (None), stop
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                self.publisher_.publish(twist)

        except Exception as e:
            print(e)
        finally:
            # proper stop
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        # Wait for input with timeout
        rlist, _, _ = select.select([sys.stdin], [], [], self.timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = RemoteControl()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
