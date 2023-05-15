import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriber(Node):
    last_pose = None

    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle5/pose',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        PoseSubscriber.last_pose = msg
        self.get_logger().info('Pose: %.2f %.2f %.2f' % (msg.x, msg.y, msg.theta))

    def get_last_pose(self):
        return PoseSubscriber.last_pose

def main(args=None):
    rclpy.init(args=args)

    pose_subscriber = PoseSubscriber()

    rclpy.spin(pose_subscriber)

    pose_subscriber.destroy_node()
    rclpy.shutdown()


main()
