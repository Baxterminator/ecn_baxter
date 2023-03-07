/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS 2 Node part for point setuping
 * @deprecated     :  These functions are only for legacy ROS2 version support
 *(foxy, galactic, ...).
 *========================================================================**/
#include <ecn_baxter/game/ros2/client_points.hpp>

namespace ecn_baxter::game::ros2 {
/// @brief Launch the call to the action for point setuping
/// @deprecated This function is only for legacy ROS2 version support (foxy,
/// galactic, ...).
void SetupPointsClient::launch_point_setup(
    const sptr<game::data::GameProperties> props) {
  using namespace std::placeholders;
  RCLCPP_INFO(logger, "Launching setup phase !");

  // Setup MSG
  goal = PointsSetup::Goal();
  goal.ptns_name = std::vector<std::string>(0);
  goal.sides = std::vector<bool>(0);

  RCLCPP_INFO(logger, "Preparing %zu markers to setup",
              props->setup.needed_points.size());
  for (auto point : props->setup.needed_points) {
    goal.ptns_name.push_back(point.name);
    goal.sides.push_back(game::data::side2bool(point.arm_side));
  }

  goal_options = Client<PointsSetup>::SendGoalOptions();
  goal_options.goal_response_callback =
      [&](std::shared_future<PtnSetupHandler::SharedPtr> future) {
        ptn_setup_goal(future);
      };
  goal_options.feedback_callback =
      [&](PtnSetupHandler::SharedPtr handle,
          const sptr<const PointsSetup::Feedback> feedback) {
        ptn_setup_feedback(handle, feedback);
      };
  goal_options.result_callback =
      [&](const PtnSetupHandler::WrappedResult &result) {
        ptn_setup_result(result);
      };

  RCLCPP_INFO(logger, "Calling action server");
  ptn_setup->async_send_goal(goal, goal_options);
}
} // namespace ecn_baxter::game::ros2