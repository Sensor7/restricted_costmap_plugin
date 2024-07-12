import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32

class PolygonServer(Node):
    def __init__(self):
        super().__init__('polygon_server')
        self.publisher_ = self.create_publisher(Polygon, 'restricted_area', 10)
        timer_period = 1.0  
        self.timer = self.create_timer(timer_period, self.publish_polygon)

    def publish_polygon(self):
        polygon = Polygon()
        polygon.points = [Point32(x=-1.0, y=-1.0, z=0.0),
                          Point32(x=-1.0, y=5.0, z=0.0),
                          Point32(x=5.0, y=5.0, z=0.0),
                          Point32(x=5.0, y=-1.0, z=0.0)]
        self.publisher_.publish(polygon)
        self.get_logger().info('Publishing polygon area')

def main(args=None):
    rclpy.init(args=args)
    polygon_server = PolygonServer()
    rclpy.spin(polygon_server)
    polygon_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
