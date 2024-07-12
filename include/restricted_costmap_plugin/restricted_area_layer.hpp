#ifndef RESTRICTED_AREA_LAYER_HPP_
#define RESTRICTED_AREA_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include <vector>

namespace restricted_costmap_plugin
{

class RestrictedAreaLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  RestrictedAreaLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual bool isClearable() {return false;}
  virtual void onFootprintChanged();

private:
  void calculateBounds();
  void areaCallback(const geometry_msgs::msg::Polygon::SharedPtr msg);
  bool isPointInPolygon(double x, double y);
  void inflateArea(double inflation_radius);

  std::vector<geometry_msgs::msg::Point32> corners_;
  geometry_msgs::msg::Polygon area_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr restricted_area_sub_;
  double min_x_, min_y_, max_x_, max_y_;
  bool bounds_calculated_;
  bool need_recalculation_;
  bool enabled_;
  double inflation_radius_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
};

}  // namespace restricted_costmap_plugin

#endif  // RESTRICTED_AREA_LAYER_HPP_
