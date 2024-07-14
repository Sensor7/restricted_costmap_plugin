#include "restricted_costmap_plugin/restricted_area_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "boost/geometry.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace restricted_costmap_plugin
{

RestrictedAreaLayer::RestrictedAreaLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
RestrictedAreaLayer::onInitialize()
{
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  std::string topic_name = "/restricted_area";
  declareParameter("restricted_area_topic", rclcpp::ParameterValue(topic_name));
  node->get_parameter(name_ + "." + "restricted_area_topic", topic_name);
  declareParameter("inflation_radius", rclcpp::ParameterValue(inflation_radius_));
  node->get_parameter(name_ + "." + "inflation_radius", inflation_radius_);

  restricted_area_sub_ = node->create_subscription<geometry_msgs::msg::Polygon>(
    topic_name, rclcpp::SystemDefaultsQoS(),
    std::bind(&RestrictedAreaLayer::areaCallback, this, std::placeholders::_1));
  need_recalculation_ = false;
  current_ = true;

  // TODO: Load corners from other sources
  geometry_msgs::msg::Point32 p1,p2,p3,p4;
  p1.x = -1.0; p1.y = -1.0;
  p2.x = -1.0; p2.y = 4.0;
  p3.x = 4.0; p3.y = 4.0;
  p4.x = 4.0; p4.y = -1.0;
  area_.points = {p1, p2, p3, p4};
  calculateBounds();
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
  bounds_calculated_ = true;
}

void RestrictedAreaLayer::inflateArea(double inflation_radius)
{
  for (auto &point : area_.points) {
    point.x += (point.x < 0 ? inflation_radius : -inflation_radius);
    point.y += (point.y < 0 ? inflation_radius : -inflation_radius);
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
      if (isPointInPolygon(wx, wy)) {
        costmap_[master_grid.getIndex(i, j)] = nav2_costmap_2d::FREE_SPACE;
      } else {
        costmap_[master_grid.getIndex(i, j)] = nav2_costmap_2d::LETHAL_OBSTACLE;
      }
    }
  }

  updateWithTrueOverwrite(master_grid,min_i,min_j,max_i,max_j);
  
}

bool RestrictedAreaLayer::isPointInPolygon(double x, double y)
{
  int nvert = area_.points.size();
  int i, j;
  bool c = false;
  for (i = 0, j = nvert - 1; i < nvert; j = i++) {
    if (((area_.points[i].y >= y) != (area_.points[j].y >= y)) &&
        (x <= (area_.points[j].x - area_.points[i].x) * (y - area_.points[i].y) /
                 (area_.points[j].y - area_.points[i].y) + area_.points[i].x)) {
      c = !c;
    }
  }
  return c;
}

void RestrictedAreaLayer::onFootprintChanged()
{
  need_recalculation_ = true;
}  
}// namespace nav2_gradient_costmap_plugin

// This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(restricted_costmap_plugin::RestrictedAreaLayer, nav2_costmap_2d::Layer)
