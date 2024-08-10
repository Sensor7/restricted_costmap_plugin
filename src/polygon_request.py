

import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
from os.path import join

from stihl_nav_msgs.srv import GetPolygonFromMap
from geometry_msgs.msg import Polygon, Point32
from copy import deepcopy


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(GetPolygonFromMap, "global_costmap/get_polygon")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetPolygonFromMap.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    print(response.polygon)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
