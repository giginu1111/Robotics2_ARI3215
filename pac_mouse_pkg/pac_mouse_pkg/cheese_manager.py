import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Point
import math

class CheeseManager(Node):
    def __init__(self):
        super().__init__('cheese_manager')

        # Publishers
        self.status_pub = self.create_publisher(String, '/cheese/status', 10)
        self.count_pub = self.create_publisher(Int32, '/cheese/count', 10)

        # Subscribers
        self.mouse_odom_sub = self.create_subscription(
            Odometry, '/mouse/odom', self.mouse_odom_callback, 10)

        # Cheese locations in the maze (x, y, z)
        self.cheese_locations = {
            'cheese_1': (-4.5, 4.5, 0.15),
            'cheese_2': (4.5, 4.5, 0.15),
            'cheese_3': (-4.5, -4.5, 0.15),
            'cheese_4': (4.5, -4.5, 0.15),
            'cheese_5': (0.0, 4.0, 0.15),
            'cheese_6': (0.0, -4.0, 0.15),
            'cheese_7': (-4.0, 0.0, 0.15),
            'cheese_8': (4.0, 0.0, 0.15)
        }

        # Track collected cheese
        self.collected_cheese = set()
        self.total_cheese = len(self.cheese_locations)

        # Mouse position
        self.mouse_pos = (0.0, 0.0)

        # Collection distance threshold
        self.collection_distance = 0.4  # meters

        # Timer for checking collection
        self.timer = self.create_timer(0.1, self.check_collection)

        self.get_logger().info(f"🧀 Cheese Manager: Tracking {self.total_cheese} cheese pieces")

    def mouse_odom_callback(self, msg):
        self.mouse_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def check_collection(self):
        """Check if mouse is close enough to collect cheese"""
        for cheese_name, (cx, cy, cz) in self.cheese_locations.items():
            # Skip if already collected
            if cheese_name in self.collected_cheese:
                continue

            # Calculate distance to mouse
            distance = math.sqrt(
                (self.mouse_pos[0] - cx)**2 + 
                (self.mouse_pos[1] - cy)**2
            )

            # Check if close enough to collect
            if distance < self.collection_distance:
                self.collect_cheese(cheese_name)

    def collect_cheese(self, cheese_name):
        """Mark cheese as collected and remove from simulation"""
        self.collected_cheese.add(cheese_name)
        remaining = self.total_cheese - len(self.collected_cheese)

        self.get_logger().info(
            f"🧀 Cheese collected: {cheese_name} | "
            f"Remaining: {remaining}/{self.total_cheese}"
        )

        # Publish status
        status_msg = String()
        status_msg.data = f"Collected {len(self.collected_cheese)}/{self.total_cheese}"
        self.status_pub.publish(status_msg)

        count_msg = Int32()
        count_msg.data = len(self.collected_cheese)
        self.count_pub.publish(count_msg)

        # Remove cheese from Gazebo using service
        self.remove_cheese_from_gazebo(cheese_name)

    def remove_cheese_from_gazebo(self, cheese_name):
        """Remove cheese model from Gazebo simulation"""
        # Import Gazebo service
        from ros_gz_interfaces.srv import DeleteEntity

        # Create service client
        if not hasattr(self, 'delete_client'):
            self.delete_client = self.create_client(DeleteEntity, '/world/pac_mouse_maze/remove')

        # Wait for service
        if not self.delete_client.service_is_ready():
            self.get_logger().warn("Delete service not ready yet")
            return

        # Create request
        request = DeleteEntity.Request()
        request.entity.name = cheese_name
        request.entity.type = 2  # MODEL type

        # Call service asynchronously
        future = self.delete_client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info(f"✅ Removed {cheese_name} from simulation")
        )

    def get_remaining_cheese(self):
        """Return list of remaining cheese locations"""
        return {
            name: loc for name, loc in self.cheese_locations.items()
            if name not in self.collected_cheese
        }

    def is_all_collected(self):
        """Check if all cheese has been collected"""
        return len(self.collected_cheese) == self.total_cheese

def main(args=None):
    rclpy.init(args=args)
    node = CheeseManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
