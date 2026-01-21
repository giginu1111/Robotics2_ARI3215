import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# --- SETTINGS ---
msg = """
---------------------------------------
Control Your Robot!
---------------------------------------
Moving around:
   w
a  s  d

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0, 0),
    'a': (0, 0, 0, 1),
    's': (-1, 0, 0, 0),
    'd': (0, 0, 0, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopNode(Node):
    def __init__(self, robot_name):
        super().__init__('teleop_node')
        topic_name = f'/{robot_name}/cmd_vel'
        self.publisher_ = self.create_publisher(Twist, topic_name, 10)
        self.get_logger().info(f'Driving robot: {robot_name} on topic: {topic_name}')
        
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0.0
        self.th = 0.0
        self.status = 0

    def print_status(self):
        print(msg)
        print(f"currently:\tspeed {self.speed}\tturn {self.turn}")

def main():
    settings = termios.tcgetattr(sys.stdin)
    
    # Check arguments for robot name
    if len(sys.argv) < 2:
        print("Error: Please specify robot name.")
        print("Usage: python3 teleop.py mouse  OR  python3 teleop.py cat")
        return

    robot_name = sys.argv[1]
    
    rclpy.init()
    node = TeleopNode(robot_name)
    
    try:
        print(msg)
        while True:
            key = getKey(settings)
            
            # MOVEMENT KEYS
            if key in moveBindings.keys():
                node.x = moveBindings[key][0]
                node.th = moveBindings[key][3]
                
            # SPEED KEYS
            elif key in speedBindings.keys():
                node.speed = node.speed * speedBindings[key][0]
                node.turn = node.turn * speedBindings[key][1]
                print(f"currently:\tspeed {node.speed}\tturn {node.turn}")
                if (node.status == 14):
                    print(msg)
                node.status = (node.status + 1) % 15
                
            # STOP KEYS
            elif key == ' ' or key == 'k':
                node.x = 0.0
                node.th = 0.0
                
            # QUIT
            elif key == '\x03': # Ctrl-C
                break

            # Publish
            twist = Twist()
            twist.linear.x = node.x * node.speed
            twist.angular.z = node.th * node.turn
            node.publisher_.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # Stop robot on exit
        twist = Twist()
        twist.linear.x = 0.0; twist.angular.z = 0.0
        node.publisher_.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()