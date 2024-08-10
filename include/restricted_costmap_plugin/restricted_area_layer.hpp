#ifndef RESTRICTED_AREA_LAYER_HPP_
#define RESTRICTED_AREA_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "stihl_nav_msgs/srv/get_polygon_from_map.hpp" 
#include <vector>
#include "std_msgs/msg/bool.hpp"

namespace restricted_costmap_plugin
{

class RestrictedAreaLayer : public nav2_costmap_2d::Layer
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
  void activeCallback(const std_msgs::msg::Bool::SharedPtr msg);
  bool isPointInPolygon(double x, double y, const geometry_msgs::msg::Polygon & polygon);
  void inflateArea(double inflation_radius);
  void requestPolygon();


  geometry_msgs::msg::Polygon area_;
  geometry_msgs::msg::Polygon inflated_area_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr restricted_area_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_sub_;
  rclcpp::Client<stihl_nav_msgs::srv::GetPolygonFromMap>::SharedPtr polygon_client_;
  rclcpp::TimerBase::SharedPtr request_polygon_timer_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  // std::shared_ptr<rclcpp::Node> client_node;


  double min_x_, min_y_, max_x_, max_y_;
  bool need_recalculation_;
  bool enabled_;
  bool client_active_;
  double inflation_radius_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
};

}  // namespace restricted_costmap_plugin

#endif  // RESTRICTED_AREA_LAYER_HPP_
