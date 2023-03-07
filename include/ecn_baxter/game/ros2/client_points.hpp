/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS 2 Node part for point setuping
 *========================================================================**/

#ifndef SETUP_PTN_CLIENT_HPP
#define SETUP_PTN_CLIENT_HPP

#include <ecn_baxter/action/points_setup.hpp>
#include <ecn_baxter/game/data/game_properties.hpp>
#include <ecn_baxter/utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace ecn_baxter::game::ros2 {
using ecn_baxter::action::PointsSetup;
using rclcpp_action::Client;
using PtnSetupHandler = rclcpp_action::ClientGoalHandle<PointsSetup>;

/// @brief ROS 2 Node part for point setuping

class SetupPointsClient {
public:
  explicit SetupPointsClient(rclcpp::Logger ros2_logger)
      : logger(ros2_logger) {}

  void launch_point_setup(const sptr<game::data::GameProperties>);

protected:
  rclcpp::Logger logger;
  Client<PointsSetup>::SharedPtr ptn_setup;
  Client<PointsSetup>::SendGoalOptions goal_options;
  PointsSetup::Goal goal;

  /// @brief Callback of the acceptance of the action call
  /// @param future the future linked to the call of the action
  /// @deprecated This function is only for legacy ROS2 version support (foxy,
  /// galactic, ...). Use PtnSetupHandler::SharedPtr from humble and onward
  inline void
  ptn_setup_goal(std::shared_future<PtnSetupHandler::SharedPtr> future) {
    auto handle = future.get();
    ptn_setup_goal(handle);
  }
  void ptn_setup_goal(const PtnSetupHandler::SharedPtr &handle);
  void ptn_setup_feedback(PtnSetupHandler::SharedPtr &handle,
                          const sptr<const PointsSetup::Feedback> feedback);
  void ptn_setup_result(const PtnSetupHandler::WrappedResult &result);
};

} // namespace ecn_baxter::game::ros2
#endif