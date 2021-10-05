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

#ifndef BEHAVIOR_TREE_BT_SUBSCRIBER_NODE_HPP_
#define BEHAVIOR_TREE_BT_SUBSCRIBER_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

namespace BT
{

/** Helper Node to process latest ROS message from a topic
 * inside a BT::ActionNode. (TODO: add option to store multiple messages)
 *
 * Note that the user must implement the methods:
 *
 *  - onMessage
 *  - halt (optionally)
 *
 */
template<class MessageT>
class RosSubscriberNode : public BT::ActionNodeBase
{
protected:

  RosSubscriberNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration & conf):
  BT::ActionNodeBase(name, conf), node_(nh)
  {
    std::string topic_name;
    if (!getInput<std::string>("topic", topic_name))
      throw BT::RuntimeError("RosSubscriberNode "+name+": 'topic' attribute must be set");
    subscriber_ = nh.subscribe( topic_name, 1, &BaseClass::messageCallback, this );
  }

public:

  using BaseClass  = RosSubscriberNode<MessageT>;
  using MessageType = MessageT;

  RosSubscriberNode() = delete;

  virtual ~RosSubscriberNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosSubscriber<DeriveClass>()
  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("topic", "name of the topic"),
      };
  }

  /// Method (to be implemented by the user) to process the latest message.
  /// User can decide which NodeStatus it will return (SUCCESS or FAILURE).
  virtual NodeStatus onMessage( const typename MessageType::ConstPtr& msg ) = 0;

  /// If you override this method, you MUST call this implementation invoking:
  ///
  ///    BaseClass::halt()
  ///
  virtual void halt() override
  {
    setStatus(NodeStatus::IDLE);
  }

protected:

  ros::Subscriber subscriber_;

  ros::NodeHandle& node_;

  typename MessageType::ConstPtr message_;

  void messageCallback( const typename MessageType::ConstPtr& msg )
  {
    message_ = msg;
  }

  BT::NodeStatus tick() override
  {
    return onMessage( message_ );
  }
};


/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT> static
  void RegisterRosSubscriber(BT::BehaviorTreeFactory& factory,
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
  const auto& basic_ports = RosSubscriberNode< typename DerivedT::MessageType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );
  factory.registerBuilder( manifest, builder );
}


}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_SUBSCRIBER_NODE_HPP_
