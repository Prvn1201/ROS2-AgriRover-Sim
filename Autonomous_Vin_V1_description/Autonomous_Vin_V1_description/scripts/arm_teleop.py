#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys, select, termios, tty

# --- SETTINGS ---
STEP_SIZE = 0.05  # Radians to move per key press
LIMIT_MAX = 1.57  # Max angle (90 degrees)
LIMIT_MIN = -1.57 # Min angle (-90 degrees)

msg = """
---------------------------
   ARM TELEOP CONTROL
---------------------------
   Base Rotation:      'a' (+)    'z' (-)
   Arm Link 1:         's' (+)    'x' (-)
   Arm Link 2:         'd' (+)    'c' (-)

   SPACE : Reset to Zero
   q     : Quit
---------------------------
"""

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop_node')
        # We publish to the controller we defined in controllers.yaml
        self.publisher_ = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        
        # Initial Joint Values [Base, Link1, Link2]
        self.joints = [0.0, 0.0, 0.0]

    def update_joint(self, index, delta):
        new_val = self.joints[index] + delta
        # Clamp values so robot doesn't break
        if new_val > LIMIT_MAX: new_val = LIMIT_MAX
        if new_val < LIMIT_MIN: new_val = LIMIT_MIN
        self.joints[index] = new_val

    def reset_joints(self):
        self.joints = [0.0, 0.0, 0.0]

    def publish_joints(self):
        msg = Float64MultiArray()
        msg.data = self.joints
        self.publisher_.publish(msg)
        print(f"Joints: [{self.joints[0]:.2f}, {self.joints[1]:.2f}, {self.joints[2]:.2f}]")

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
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
            
            updated = False

            # Base Rotation (Index 0)
            if key == 'a': 
                node.update_joint(0, STEP_SIZE)
                updated = True
            elif key == 'z': 
                node.update_joint(0, -STEP_SIZE)
                updated = True
            
            # Arm Link 1 (Index 1)
            elif key == 's': 
                node.update_joint(1, STEP_SIZE)
                updated = True
            elif key == 'x': 
                node.update_joint(1, -STEP_SIZE)
                updated = True
            
            # Arm Link 2 (Index 2)
            elif key == 'd': 
                node.update_joint(2, STEP_SIZE)
                updated = True
            elif key == 'c': 
                node.update_joint(2, -STEP_SIZE)
                updated = True
            
            # Reset
            elif key == ' ':
                node.reset_joints()
                updated = True

            # Quit
            elif key == 'q' or key == '\x03': # \x03 is Ctrl+C
                break
            
            if updated:
                node.publish_joints()

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()