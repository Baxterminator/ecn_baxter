#include "ecn_baxter/game/data/arm_side.hpp"
#include <ecn_baxter/setup/setup_node.hpp>
#include <functional>
#include <thread>

namespace ecn_baxter {

std::thread SetupNode::action_th;
sig_atomic_t SetupNode::stop_cmd;


SetupNode::SetupNode(rclcpp::NodeOptions opts)
    : rclcpp::Node("setup_node", opts), tf_buffer(get_clock()),
      tf_listener(tf_buffer) {
  states_sub = create_subscription<NavigatorStates>(
      "/robot/navigators_states", 1,
      [this](NavigatorStates::SharedPtr msg) { navigator_callback(msg); });
  points_setup = create_server<PointsSetup>(
      this, "points_setup",
      [this](const GoalUUID &uuid, sptr<const PointsSetup::Goal> goal) {
        return handle_goal(uuid, goal);
      },
      [this](const sptr<PtnHandle> goal_handle) {
        return handle_cancel(goal_handle);
      },
      [this](const sptr<PtnHandle> goal_handle) {
        handle_accepted(goal_handle);
      });
}

/* =================================================================
 #                              ACTION
 #                            Points setup
 # =================================================================*/
CancelResponse SetupNode::handle_cancel(const sptr<PtnHandle> goal_handle) {
  RCLCPP_INFO(get_logger(), "Request to cancel goal ");
  (void)goal_handle;
  stop_cmd = 1;
  action_th.join();
  return CancelResponse::ACCEPT;
}
GoalResponse
SetupNode::handle_goal([[maybe_unused]]const GoalUUID &uuid,
                       [[maybe_unused]] sptr<const PointsSetup::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  return GoalResponse::ACCEPT_AND_EXECUTE;
}
void SetupNode::handle_accepted(const sptr<PtnHandle> goal_handle) {
  action_th = std::thread{[&]() { ptn_setup_exec(goal_handle); }};
  RCLCPP_INFO(get_logger(), "Launching setup action on thread");
}

void SetupNode::ptn_setup_exec(const sptr<PtnHandle> handle) {

  RCLCPP_INFO(get_logger(), "Start exec");
  RCLCPP_INFO(get_logger(), "Start exec");
  RCLCPP_INFO(get_logger(), "Start exec");
  std::this_thread::sleep_for(2000ms);
  RCLCPP_INFO(get_logger(), "Start exec 2");
  RCLCPP_INFO(get_logger(), "Start exec 2");
  RCLCPP_INFO(get_logger(), "Start exec 2");
  RCLCPP_INFO(get_logger(), "Start exec 2");
  auto const goal = handle->get_goal();
  names = goal->ptns_name;

  RCLCPP_INFO(get_logger(), "Convert sides");
  sides = std::vector<game::data::ArmSide>(0);
  for (bool b : handle->get_goal()->sides) {
    sides.push_back(game::data::bool2side(b));
  }
  current_step = 0;

  RCLCPP_INFO(get_logger(), "Before loop");
  sptr<PointsSetup::Feedback> feedback;
  while (current_step < names.size()) {
    if (publish) {
      RCLCPP_INFO(get_logger(), "If can publish");
      feedback = std::make_shared<PointsSetup::Feedback>();
      feedback->step = (int)current_step;
      feedback->ptn_name = names[current_step];
      feedback->ptn = last_point;

      handle->publish_feedback(feedback);
      current_step++;
      publish = false;
    }

    if (stop_cmd)
      break;
    std::this_thread::sleep_for(200ms);
  }

  if (rclcpp::ok()) {
    auto result = std::make_shared<PointsSetup::Result>();
    result->success = true;
    handle->succeed(result);
  }
}

/* =================================================================
 #                              ACTION
 #                          Points setup
 # =================================================================*/

/**
 * Navigator button handler for saving positions
 * @param states
 */
void SetupNode::navigator_callback(NavigatorStates::SharedPtr &states) {
  if (pushed) {
    for (auto state : states->states) {
      if (state.buttons[0]) {
        return;
      }
      pushed = false;
    }
  } else {
    for (auto state : states->states) {
      if (state.buttons[0]) {
        pushed = true;
        savePose();
        return;
      }
    }
  }
}

void SetupNode::savePose() {
  const std::string base = "base";
  const std::string gripper =
      (sides[current_step] == game::data::ArmSide::RIGHT_ARM) ? "right_gripper"
                                                              : "left_gripper";
  if (!tf_buffer.canTransform(gripper, base, tf2::TimePointZero,
                              tf2::durationFromSec(1.0))) {
    RCLCPP_FATAL(get_logger(), "Can't find corresponding frame");
    return;
  }
  auto trans = tf_buffer.lookupTransform(base, gripper, tf2::TimePointZero,
                                         tf2::durationFromSec(1.0));
  last_point.x = trans.transform.translation.x;
  last_point.y = trans.transform.translation.y;
  last_point.z = trans.transform.translation.z;
  publish = true;
}
} // namespace ecn_baxter

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ecn_baxter::SetupNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  ecn_baxter::SetupNode::stop_cmd = 1;
  if (ecn_baxter::SetupNode::action_th.joinable())
    ecn_baxter::SetupNode::action_th.join();
  return 0;
}