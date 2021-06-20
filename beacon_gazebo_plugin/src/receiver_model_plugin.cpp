//
// Created by Vasily Yuryev on 19.06.2021.
//


//// Gazebo
//#include <gazebo/common/common.hh>
//#include <gazebo/physics/physics.hh>
//#include <gazebo_plugins/gazebo_ros_utils.h>
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
//#include <boost/thread.hpp>
//#include <boost/bind.hpp>
//#include <algorithm>
//#include <assert.h>

//#include <gazebo_plugins/gazebo_ros_diff_drive.h>

//#include <ignition/math/Angle.hh>
//#include <ignition/math/Pose3.hh>
//#include <ignition/math/Quaternion.hh>
//#include <ignition/math/Vector3.hh>
//#include <sdf/sdf.hh>
//
//#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
//#include <stdio.h>
#include <string>

namespace gazebo {
    class ReceiverModelPlugin : public ModelPlugin {
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

            ROS_INFO("ReceiverModelPlugin: Hello World!");
//             printf("HELLO");
            // Store the pointer to the model
            this->model = _parent;
            this->world = _parent->GetWorld();

//             // Listen to the update event. This event is broadcast every
//             // simulation iteration.
            ROS_INFO("ReceiverModelPlugin: model links");
            for (auto &link : this->model->GetLinks()) {
                std::string link_name = link->GetName();
//                            double d = sqrt(pow(beacon_p.X() - receiver_p.X(), 2) + pow(beacon_p.Y() - receiver_p.Y(), 2) + pow(beacon_p.Z() - receiver_p.Z(), 2));
                printf("\tlink:%s\n",link_name.c_str());
            }
            if (_sdf->HasElement("receiver_link")) {
                std::string receiver_link_name = _sdf->Get<std::string>("receiver_link");
                this->receiver_link = _parent->GetLink(receiver_link_name);

                ROS_INFO("receiver_link_name: %s", receiver_link_name.c_str());
                this->receiver_name = std::string("receiver__") + _parent->GetName();
                if (this->receiver_link == NULL)
                {
                    std::vector<physics::LinkPtr> links = _parent->GetLinks();
                    for (int i = 0; i < links.size(); ++i)
                    {
                        ROS_INFO("Link[%d]: %s", i, links[i]->GetName().c_str());
                    }
                    ROS_ERROR("receiver_link Is NULL");
                    return;
//                    this->useParentAsReference = true;
                }
                ROS_INFO("ReceiverModelPlugin: name: %s", this->receiver_name.c_str());
            } else {
                ROS_ERROR("Haha receiver_link");
                return;
            }
            if (_sdf->HasElement("update_rate")) {
                this->update_rate = _sdf->Get<float>("update_rate");
                ROS_INFO("ReceiverModelPlugin: update_rate: %f", this->update_rate);
            } else {
                ROS_ERROR("Haha update_rate");
                return;
            }

            this->l_u_time = common::Time(0.0);
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&ReceiverModelPlugin::OnUpdate, this, _1));


        }
        void OnUpdate(const common::UpdateInfo &_info) {
            common::Time simTime = _info.simTime;
            common::Time elapsed = simTime - this->l_u_time;
            if (elapsed >= common::Time(1.0/this->update_rate)) {
                this->l_u_time = simTime;
                ROS_INFO("ReceiverModelPlugin: Update, receiver_name: %s", this->receiver_name.c_str());
                physics::Model_V models = this->world->Models();
                for (auto &model : models) {
                    physics::Link_V model_links = model->GetLinks();
                    for (auto &link : model_links) {
                        if (link->GetName().find("beacon") == 0) {
                            std::string beacon_name = std::string("beacon__") + model->GetName();
                            auto beacon_p = link->WorldPose().Pos();
                            auto receiver_p = this->receiver_link->WorldPose().Pos();
//                            double d = 0;
                            double d = sqrt(pow(beacon_p.X() - receiver_p.X(), 2) + pow(beacon_p.Y() - receiver_p.Y(), 2) + pow(beacon_p.Z() - receiver_p.Z(), 2));
                            ROS_INFO("ReceiverModelPlugin:\n\treceiver_name: %s\n\tbeacon_name: %s\n\tdistance: %lf",this->receiver_name.c_str(), beacon_name.c_str(), d);
                        }
                    }
                }
            }
        }
    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        physics::LinkPtr receiver_link;
        std::string receiver_name;
        physics::WorldPtr world;
        float update_rate;
//        common::Time ;
        common::Time l_u_time;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ReceiverModelPlugin)
}