#ifndef PROJ1_SETUP_NODE_HPP
#define PROJ1_SETUP_NODE_HPP

#include <baxter_core_msgs/msg/navigator_state.hpp>
#include <baxter_core_msgs/msg/navigator_states.hpp>
#include <chrono>
#include <ecn_baxter/action/points_setup.hpp>
#include <ecn_baxter/game/data/arm_side.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

namespace ecn_baxter {
using baxter_core_msgs::msg::NavigatorState;
using baxter_core_msgs::msg::NavigatorStates;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::TransformStamped;

using namespace rclcpp;
using rclcpp::NodeOptions;
using rclcpp::Subscription;

using namespace rclcpp_action;
using ecn_baxter::action::PointsSetup;
using PtnHandle = rclcpp_action::ServerGoalHandle<PointsSetup>;
template <typename F> using sptr = std::shared_ptr<F>;

using namespace std::chrono_literals;

using Step = unsigned int;

class SetupNode : public rclcpp::Node {
public:
  explicit SetupNode(NodeOptions opts);

  static std::thread action_th;
  static sig_atomic_t stop_cmd;

protected:
  Step current_step = 0;
  std::vector<std::string> names;
  std::vector<game::data::ArmSide> sides;
  bool pushed = false, publish = false;
  Point last_point;

  

  sptr<Subscription<NavigatorStates>> states_sub;

  sptr<Server<PointsSetup>> points_setup;
  GoalResponse handle_goal(const GoalUUID &uuid,
                           sptr<const PointsSetup::Goal> goal);
  CancelResponse handle_cancel(sptr<PtnHandle> goal_handle);
  void handle_accepted(sptr<PtnHandle> goal_handle);
  void ptn_setup_exec(const sptr<PtnHandle> handle);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  void navigator_callback(NavigatorStates::SharedPtr &states);
  void savePose();
};
} // namespace ecn_baxter

#endif // PROJ1_SETUP_NODE_HPP
