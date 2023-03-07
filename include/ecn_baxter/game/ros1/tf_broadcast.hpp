/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  26/02/2023
 * @description    :  Sub-part of the ROS1 game master for publishing tf
 *                    custom transforms
 *========================================================================**/
#ifndef TF_BROADCAST
#define TF_BROADCAST

#include "ros/duration.h"
#include "ros/publisher.h"
#include "ros/wall_timer.h"
#include <ecn_baxter/utils.hpp>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/msg/point.hpp>
#include <ros/node_handle.h>
#include <vector>

namespace ecn_baxter::game::ros1 {
using geometry_msgs::msg::Point;
using tf2_msgs::TFMessage;

class TFBroadcaster {
private:
  // Timer
  const ros::WallDuration dur{0.1};

  // Publisher
  ros::Publisher _tf_publisher;
  static const std::string _tf_topic;
  static const int _queue_size = 10;
  std::vector<TFMessage> _to_broadcast;

  void broadcast_transforms() const;

protected:
  explicit TFBroadcaster(){};
  void tf_broadcast_init(sptr<ros::NodeHandle>);

  //* Timer
  ros::WallTimer _tf_broadcast_timer;

public:
  void add_tf(const TFMessage &);
};

} // namespace ecn_baxter::game::ros1

#endif