// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2020 Davide Faconti
// Copyright (c) 2021 RoboTech Vision
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BEHAVIOR_TREE_BT_ROS_NODE_HPP_
#define BEHAVIOR_TREE_BT_ROS_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

namespace BT
{

/** Helper Node which holds reference to ROS node handle
 * inside a BT::ActionNode.
 *
 * Note that the user must implement the methods:
 *
 *  - halt (optionally)
 *
 */
class RosNode : public BT::ActionNodeBase
{
protected:

  RosNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration & conf):
  BT::ActionNodeBase(name, conf), node_(nh)
  {
  }

public:

  using BaseClass  = RosNode;

  RosNode() = delete;

  virtual ~RosNode() = default;

  /// If you override this method, you MUST call this implementation invoking:
  ///
  ///    BaseClass::halt()
  ///
  virtual void halt() override
  {
    setStatus(NodeStatus::IDLE);
  }

protected:

  ros::NodeHandle& node_;
};


/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT> static
  void RegisterRosNode(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID,
                         ros::NodeHandle& node_handle)
{
  NodeBuilder builder = [&node_handle](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(node_handle, name, config );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  factory.registerBuilder( manifest, builder );
}


}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_SUBSCRIBER_NODE_HPP_
