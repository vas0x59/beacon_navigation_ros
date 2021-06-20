//
// Created by Vasily Yuryev on 19.06.2021.
//



#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <stdio.h>
namespace gazebo
{
    class HelloPlugin : public WorldPlugin
    {
    public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                         << "Load the Gazebo system plugin  in the gazebo_ros package)");
                return;
            }

            ROS_INFO("HelloPlugin: Hello World!");
        }

    };
    GZ_REGISTER_WORLD_PLUGIN(HelloPlugin)
}

