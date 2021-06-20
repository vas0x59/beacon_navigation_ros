 //
// Created by Vasily Yuryev on 19.06.2021.
//
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
//#include <stdio.h>
#include <string>

 namespace gazebo {
     class BeaconModelPlugin : public ModelPlugin {
     public:
         void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override {
//             this->parent = _parent;
//             gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "ReceiverModelPlugin" ) );
//             // Make sure the ROS node for Gazebo has already been initialized
//             gazebo_ros_->isInitialized();
             if (!ros::isInitialized()) {
                 ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                          << "Load the Gazebo system plugin  in the gazebo_ros package)");
                 return;
             }

             ROS_INFO("BeaconModelPlugin: Hello World!");
//             printf("HELLO");
             // Store the pointer to the model
             this->model = _parent;
//             this->world = _parent->GetWorld();


         }

     private:
         physics::ModelPtr model;
//         event::ConnectionPtr updateConnection;
//         physics::LinkPtr receiver_link;
//         std::string receiver_name;
//         physics::WorldPtr world;
//         float update_rate;
//        common::Time ;
//         common::Time l_u_time;
     };

     // Register this plugin with the simulator
     GZ_REGISTER_MODEL_PLUGIN(BeaconModelPlugin)
 }