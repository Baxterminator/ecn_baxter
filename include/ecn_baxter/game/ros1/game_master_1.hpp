/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS1 part of the GameMaster
 *========================================================================**/

#ifndef GAME_MASTER_1_HPP
#define GAME_MASTER_1_HPP

#include "ecn_baxter/game/ros1/tf_broadcast.hpp"
#include <ecn_baxter/game/ros1/bridge_lookup.hpp>
#include <ecn_baxter/utils.hpp>
#include <ros/node_handle.h>

namespace ecn_baxter::game::ros1 {

class GameMaster_1 : public BridgesManager, public TFBroadcaster {
public:
  GameMaster_1();
  static const std::string NODE_NAME;

private:
  sptr<ros::NodeHandle> _handle;
};
} // namespace ecn_baxter::game::ros1

#endif // GAME_MASTER_1_HPP