 //
// Created by Vasily Yuryev on 19.06.2021.
//
#include <ros/ros.h>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/common/Plugin.hh>
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include <stdio.h>

 namespace gazebo
 {
     class BeaconModelPlugin : public ModelPlugin
     {
     public: BeaconModelPlugin() : ModelPlugin()
         {
         }
//     public: ~GazeboRosDiffDrive() : ~ModelPlugin() {
//
//     }
     public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
         {
             // Store the pointer to the model
             this->model = _parent;

             if (!ros::isInitialized())
             {
                 ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                          << "Load the Gazebo system plugin  in the gazebo_ros package)");
                 return;
             }
             ROS_INFO("BeaconPlugin: Hello World!");

             // Listen to the update event. This event is broadcast every
             // simulation iteration.
             this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                     std::bind(&BeaconModelPlugin::OnUpdate, this));

         }

         // Called by the world update start event
     public: void OnUpdate()
     {
         // Apply a small linear velocity to the model.
             this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
     }

         // Pointer to the model
     private: physics::ModelPtr model;

         // Pointer to the update event connection
     private: event::ConnectionPtr updateConnection;
     };

     // Register this plugin with the simulator
     GZ_REGISTER_MODEL_PLUGIN(BeaconModelPlugin)
 }