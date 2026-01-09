#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys, select, termios, tty

# --- SETTINGS ---
STEP_SIZE = 0.1
MSG_TYPE = Float64

msg = """
---------------------------
   ARM TELEOP CONTROL
---------------------------
   Joint 1 (Base):   'a' (+)   'z' (-)
   Joint 2 (Shoulder): 's' (+)   'x' (-)
   Joint 3 (Elbow):    'd' (+)   'c' (-)

   q to quit
---------------------------
"""

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop_node')
        self.pub_j1 = self.create_publisher(MSG_TYPE, '/joint_1_cmd', 10)
        self.pub_j2 = self.create_publisher(MSG_TYPE, '/joint_2_cmd', 10)
        self.pub_j3 = self.create_publisher(MSG_TYPE, '/joint_3_cmd', 10)
        self.j1_val = 0.0
        self.j2_val = 0.0
        self.j3_val = 0.0

    def publish_joints(self):
        m = MSG_TYPE()
        m.data = self.j1_val
        self.pub_j1.publish(m)
        m.data = self.j2_val
        self.pub_j2.publish(m)
        m.data = self.j3_val
        self.pub_j3.publish(m)
        print(f"Joints: [{self.j1_val:.2f}, {self.j2_val:.2f}, {self.j3_val:.2f}]")

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleop()
    settings = termios.tcgetattr(sys.stdin)
    
    print(msg)

    try:
        while True:
            key = getKey(settings)
            if key == 'a': node.j1_val += STEP_SIZE
            elif key == 'z': node.j1_val -= STEP_SIZE
            elif key == 's': node.j2_val += STEP_SIZE
            elif key == 'x': node.j2_val -= STEP_SIZE
            elif key == 'd': node.j3_val += STEP_SIZE
            elif key == 'c': node.j3_val -= STEP_SIZE
            elif key == 'q' or key == '\x03': break
            
            node.publish_joints()

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
