/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  26/02/2023
 * @description    :  Sub-part of the ROS1 game master for publishing tf
 *                    custom transforms
 *========================================================================**/
#include <ecn_baxter/game/ros1/tf_broadcast.hpp>

namespace ecn_baxter::game::ros1 {

const std::string TFBroadcaster::_tf_topic = "/tf_manual";

void TFBroadcaster::tf_broadcast_init(sptr<ros::NodeHandle> handle) {
  _tf_broadcast_timer = handle->createWallTimer(
      dur, [this]([[maybe_unused]] const ros::WallTimerEvent &event) {
        broadcast_transforms();
      });
  _tf_publisher = handle->advertise<TFMessage>(_tf_topic, _queue_size);
}

/// @brief Publish all saved markers to the TF tree of the robot
void TFBroadcaster::broadcast_transforms() const {
  for (auto tf : _to_broadcast)
    _tf_publisher.publish(tf);
}

/// @brief Add a transform to the broadcast list
void TFBroadcaster::add_tf(const TFMessage &tf) {
  _to_broadcast.push_back(tf);
}
} // namespace ecn_baxter::game::ros1