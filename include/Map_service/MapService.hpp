#ifndef MAP_SERVICE_HPP_
#define MAP_SERVICE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "stihl_nav_msgs/srv/get_polygon_from_map.hpp"
#include "stihl_nav_msgs/srv/post_polygon_to_map.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class MapService {
public:
  MapService(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node);
  
  geometry_msgs::msg::Polygon getPolygon() const;
  void setPolygon(const geometry_msgs::msg::Polygon &polygon);
  void addPointToPolygon(const geometry_msgs::msg::Point32 &point);

private:
  void handleGetPolygon(
      const std::shared_ptr<stihl_nav_msgs::srv::GetPolygonFromMap::Request> request,
      std::shared_ptr<stihl_nav_msgs::srv::GetPolygonFromMap::Response> response);
  
  void handlePostPolygon(
      const std::shared_ptr<stihl_nav_msgs::srv::PostPolygonToMap::Request> request,
      std::shared_ptr<stihl_nav_msgs::srv::PostPolygonToMap::Response> response);

  void writePolygonToFile(const std::string &action);

  geometry_msgs::msg::Polygon polygon_;
  std::string csv_file_path_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Service<stihl_nav_msgs::srv::GetPolygonFromMap>::SharedPtr get_polygon_service_;
  rclcpp::Service<stihl_nav_msgs::srv::PostPolygonToMap>::SharedPtr post_polygon_service_;
};

#endif  // MAP_SERVICE_HPP_
