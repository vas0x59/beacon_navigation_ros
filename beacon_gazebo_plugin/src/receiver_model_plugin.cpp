 //
// Created by Vasily Yuryev on 19.06.2021.
//

//#include <map>
//#include <stdio.h>
//// Gazebo
//#include <gazebo/common/common.hh>
//#include <gazebo/physics/physics.hh>
////#include <gazebo_plugins/gazebo_ros_utils.h>
//
//// ROS
//#include <ros/ros.h>
//#include <tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Pose2D.h>
//#include <nav_msgs/Odometry.h>
//#include <sensor_msgs/JointState.h>
//
//// Custom Callback Queue
//#include <ros/callback_queue.h>
//#include <ros/advertise_options.h>
//
//// Boost
////#include <boost/thread.hpp>
////#include <boost/bind.hpp>


// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <algorithm>
#include <assert.h>

//#include <gazebo_plugins/gazebo_ros_diff_drive.h>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

 namespace gazebo
 {
     class ReceiverModelPlugin : public ModelPlugin
     {
     public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
         {
             this->parent = _parent;
             gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "ReceiverModelPlugin" ) );
             // Make sure the ROS node for Gazebo has already been initialized
             gazebo_ros_->isInitialized();
             ROS_INFO_NAMED("ReceiverModelPlugin_Hello")
//             printf("HELLO");
             // Store the pointer to the model
//             this->model = _parent;
//             if (!ros::isInitialized())
//             {
//                 ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
//                                          << "Load the Gazebo system plugin  in the gazebo_ros package)");
//                 return;
//             }
//             ROS_INFO("ReceiverModelPlugin: Hello World!");
//
//             // Listen to the update event. This event is broadcast every
//             // simulation iteration.
//             this->updateConnection = event::Events::ConnectWorldUpdateBegin(
//                     std::bind(&ReceiverModelPlugin::OnUpdate, this));
//             if (_sdf->HasElement("tag_link"))
//             {
//                 std::string receiver_link_name = _sdf->Get<std::string>("tag_link");
//                 this->receiver_link = _parent->GetLink(receiver_link_name);
//
////                 ROS_INFO("Parent name: %s ChildCount: %d", _parent->GetName().c_str(), _parent->GetChildCount());
//
//             }
//             else {
////                 return
//             }
         }

         // Called by the world update start event
     public: void OnUpdate()
     {
         // Apply a small linear velocity to the model.
//             this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
     }

         // Pointer to the model
     private: physics::ModelPtr model;

         // Pointer to the update event connection
     private: event::ConnectionPtr updateConnection;
     private: physics::LinkPtr receiver_link;
     };

     // Register this plugin with the simulator
     GZ_REGISTER_MODEL_PLUGIN(ReceiverModelPlugin)
 }