#include "Map_service/MapService.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


MapService::MapService(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node)
{
  //node_ = std::dynamic_pointer_cast<rclcpp::Node>(node);
  csv_file_path_ = ament_index_cpp::get_package_share_directory("ia01_gnss_demo") + "/config/polygon.csv";
  RCLCPP_INFO(node->get_logger(), "CSV file path: %s", csv_file_path_.c_str());

  std::ifstream file(csv_file_path_);
  std::string line;
  geometry_msgs::msg::Point32 last_point;

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string token;
    geometry_msgs::msg::Point32 point;
    
    std::getline(ss, token, ',');
    point.x = std::stof(token);
    std::getline(ss, token, ',');
    point.y = std::stof(token);
    point.z = 0.0;
    
    polygon_.points.push_back(point);
    last_point = point;
  }

  polygon_.points.push_back(last_point);

  get_polygon_service_ = node->create_service<stihl_nav_msgs::srv::GetPolygonFromMap>(
      "/global_costmap/get_polygon", std::bind(&MapService::handleGetPolygon, this, std::placeholders::_1, std::placeholders::_2));

  post_polygon_service_ = node->create_service<stihl_nav_msgs::srv::PostPolygonToMap>(
      "/global_costmap/post_polygon", std::bind(&MapService::handlePostPolygon, this, std::placeholders::_1, std::placeholders::_2));
}

geometry_msgs::msg::Polygon MapService::getPolygon() const {
  return polygon_;
}

void MapService::setPolygon(const geometry_msgs::msg::Polygon &polygon) {
  polygon_ = polygon;
  writePolygonToFile("update_polygon");
}

void MapService::addPointToPolygon(const geometry_msgs::msg::Point32 &point) {
  polygon_.points.push_back(point);
  writePolygonToFile("add_point");
}

void MapService::writePolygonToFile(const std::string &action) {
  std::ofstream file;

  if (action == "update_polygon") {
    file.open(csv_file_path_, std::ofstream::trunc);
    for (const auto &point : polygon_.points) {
      file << point.x << "," << point.y << "\n";
    }
  } else if (action == "add_point") {
    file.open(csv_file_path_, std::ofstream::app);
    const auto &point = polygon_.points.back();
    file << point.x << "," << point.y << "\n";
  }
}

void MapService::handleGetPolygon(
    const std::shared_ptr<stihl_nav_msgs::srv::GetPolygonFromMap::Request> request,
    std::shared_ptr<stihl_nav_msgs::srv::GetPolygonFromMap::Response> response) {
//   RCLCPP_INFO(node_->get_logger(), "Received get polygon request");
  response->polygon = polygon_;
}

void MapService::handlePostPolygon(
    const std::shared_ptr<stihl_nav_msgs::srv::PostPolygonToMap::Request> request,
    std::shared_ptr<stihl_nav_msgs::srv::PostPolygonToMap::Response> response) {
  
  if (request->action == "update_polygon") {
    setPolygon(request->polygon);
    response->polygon = polygon_;
    response->success = true;
    response->message = "Polygon updated successfully.";
  } else if (request->action == "add_point") {
    addPointToPolygon(request->polygon.points.back());
    response->polygon = polygon_;
    response->success = true;
    response->message = "Point added successfully.";
  } else {
    response->success = false;
    response->message = "Unknown action.";
  }
}
