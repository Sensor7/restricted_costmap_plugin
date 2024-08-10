#include "restricted_costmap_plugin/restricted_area_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "boost/geometry.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "boost/geometry/geometries/polygon.hpp"
#include "boost/geometry/strategies/buffer.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace restricted_costmap_plugin
{

RestrictedAreaLayer::RestrictedAreaLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max()),
  client_active_(false)
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
RestrictedAreaLayer::onInitialize()
{ 
  node = node_.lock();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  std::string topic_name = "/restricted_area";
  declareParameter("restricted_area_topic", rclcpp::ParameterValue(topic_name));
  node->get_parameter(name_ + "." + "restricted_area_topic", topic_name);
  std::string active_topic = "/restricted_area/active";
  declareParameter("active_topic", rclcpp::ParameterValue(active_topic));
  node->get_parameter(name_ + "." + "active_topic", active_topic);
  declareParameter("inflation_radius", rclcpp::ParameterValue(inflation_radius_));
  node->get_parameter(name_ + "." + "inflation_radius", inflation_radius_);
  // Set up the timer to periodically call requestPolygon if client_active_ is false
  // request_polygon_timer_ = node->create_wall_timer(
  //   std::chrono::seconds(1),
  //   std::bind(&RestrictedAreaLayer::requestPolygonCallback, this)
  // );
  active_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    active_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&RestrictedAreaLayer::activeCallback, this, std::placeholders::_1));

  restricted_area_sub_ = node->create_subscription<geometry_msgs::msg::Polygon>(
    topic_name, rclcpp::SystemDefaultsQoS(),
    std::bind(&RestrictedAreaLayer::areaCallback, this, std::placeholders::_1));
  need_recalculation_ = false;
  current_ = true;
  // client_node = std::make_shared<rclcpp::Node>("restricted_area_client");
  polygon_client_ = node->create_client<stihl_nav_msgs::srv::GetPolygonFromMap>("/global_costmap/get_polygon");

  calculateBounds();
}

void RestrictedAreaLayer::activeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  client_active_ = msg->data;
  if (client_active_) {
    requestPolygon();
  }else{
    area_= geometry_msgs::msg::Polygon();}
}

void RestrictedAreaLayer::areaCallback(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
  area_= *msg;
  need_recalculation_ = true;
  calculateBounds();
}

void RestrictedAreaLayer::calculateBounds()
{
  if(area_.points.empty()){
    RCLCPP_WARN(rclcpp::get_logger("RestrictedAreaLayer"), "No area points are set");
    return;
  }
  // Calculate the min and max x and y bounds from corners
  min_x_ = std::numeric_limits<double>::max();
  min_y_ = std::numeric_limits<double>::max();
  max_x_ = std::numeric_limits<double>::lowest();
  max_y_ = std::numeric_limits<double>::lowest();

  for (const auto& point: area_.points) {
    if (point.x < min_x_) min_x_ = point.x;
    if (point.x > max_x_) max_x_ = point.x;
    if (point.y< min_y_) min_y_ = point.y;
    if (point.y> max_y_) max_y_ = point.y;
  }
  inflateArea(inflation_radius_);
}

void RestrictedAreaLayer::inflateArea(double inflation_radius)
{
  typedef boost::geometry::model::d2::point_xy<double> BoostPoint;
  typedef boost::geometry::model::polygon<BoostPoint> BoostPolygon;
  // Create Boost Polygon from the area points
  BoostPolygon original_polygon;
  for (const auto& point : area_.points) {
    boost::geometry::append(original_polygon, BoostPoint(point.x, point.y));
  }
  boost::geometry::correct(original_polygon);

  // Inflate the polygon
  std::vector<BoostPolygon> shrunken_polygons;
  using coordinate_type = boost::geometry::coordinate_type<BoostPoint>::type;
  boost::geometry::strategy::buffer::distance_symmetric<coordinate_type> distance_strategy(-inflation_radius);
  boost::geometry::strategy::buffer::join_round join_strategy(12);
  boost::geometry::strategy::buffer::end_round end_strategy(12);
  boost::geometry::strategy::buffer::point_circle point_strategy(12);
  boost::geometry::strategy::buffer::side_straight side_strategy;
  boost::geometry::buffer(original_polygon, shrunken_polygons, distance_strategy, side_strategy, join_strategy, end_strategy, point_strategy);
  if (shrunken_polygons.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("RestrictedAreaLayer"), "Shrunken polygon is empty.");
    return;
  }
  // Inflate the polygon
  inflated_area_.points.clear();
  for (const auto& point : shrunken_polygons.front().outer()) {
    geometry_msgs::msg::Point32 new_point;
    new_point.x = point.x();
    new_point.y = point.y();
    new_point.z = 0.0;
    inflated_area_.points.push_back(new_point);
    RCLCPP_INFO(rclcpp::get_logger("RestrictedAreaLayer"), "Inflated point: %f, %f", new_point.x, new_point.y);
  }
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
RestrictedAreaLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }

}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
void
RestrictedAreaLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) return;

  if (area_.points.empty()) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);
      if (!isPointInPolygon(wx, wy,area_)) {
        master_array[master_grid.getIndex(i, j)] = nav2_costmap_2d::LETHAL_OBSTACLE;
      }
      else if(!isPointInPolygon(wx, wy,inflated_area_)){
        master_array[master_grid.getIndex(i, j)] = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
      }
    }
  }
}

bool RestrictedAreaLayer::isPointInPolygon(double x, double y, const geometry_msgs::msg::Polygon & polygon)
{
  int nvert = polygon.points.size();
  int i, j;
  bool c = false;
  for (i = 0, j = nvert - 1; i < nvert; j = i++) {
    if (((polygon.points[i].y >= y) != (polygon.points[j].y >= y)) &&
        (x <= (polygon.points[j].x - polygon.points[i].x) * (y - polygon.points[i].y) /
                 (polygon.points[j].y - polygon.points[i].y) + polygon.points[i].x)) {
      c = !c;
    }
  }
  return c;
}

void RestrictedAreaLayer::onFootprintChanged()
{
  need_recalculation_ = true;
}

void RestrictedAreaLayer::requestPolygon()
{ 
  
  // if (polygon_client_->wait_for_service(std::chrono::seconds(1))){
  while (!polygon_client_->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }
  auto polygon_request = std::make_shared<stihl_nav_msgs::srv::GetPolygonFromMap::Request>();
  using ServiceResponseFuture = rclcpp::Client<stihl_nav_msgs::srv::GetPolygonFromMap>::SharedFuture;
  auto response_received_callback=[this](ServiceResponseFuture future)
  {
    auto response = future.get();
    if (client_active_) {
    area_ = response->polygon;
    need_recalculation_ = true;
    calculateBounds();
    }
  };
  auto polygon_future = polygon_client_->async_send_request(polygon_request,response_received_callback);

}


}// namespace nav2_gradient_costmap_plugin

// This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(restricted_costmap_plugin::RestrictedAreaLayer, nav2_costmap_2d::Layer)
