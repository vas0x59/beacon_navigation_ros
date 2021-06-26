//
// Created by Vasily Yuryev on 19.06.2021.
//

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <string>
#include <beacon_gazebo_sim/ReceiverIn.h>
#include <beacon_gazebo_sim/rssi_noise.h>
#include <vector>

namespace gazebo {
    class ReceiverModelPlugin : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override {

            if (!ros::isInitialized()) {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                         << "Load the Gazebo system plugin  in the gazebo_ros package)");
                return;
            }

            ROS_INFO("ReceiverModelPlugin: Hello World!");

            this->model = _parent;
            this->world = _parent->GetWorld();

            ROS_INFO("ReceiverModelPlugin: model links");

            for (auto &link : this->model->GetLinks()) {
                std::string link_name = link->GetName();
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
            ros::NodeHandle n;

            this->receiver_in_msgs_publisher = n.advertise<beacon_gazebo_sim::ReceiverIn>(
                    "/" + this->model->GetName() + "/" + this->receiver_name + "/receiver_in_msgs", 1000);

            this->l_u_time = common::Time(0.0);
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&ReceiverModelPlugin::OnUpdate, this, _1));


        }
        void OnUpdate(const common::UpdateInfo &_info) {
            common::Time simTime = _info.simTime;
            common::Time elapsed = simTime - this->l_u_time;
            if (elapsed >= common::Time(1.0/this->update_rate)) {
//                ROS_INFO("ReceiverModelPlugin: Update, receiver_name: %s", this->receiver_name.c_str());
                physics::Model_V models = this->world->Models();
                for (auto &model : models) {
                    physics::Link_V model_links = model->GetLinks();
                    for (auto &link : model_links) {
                        if (link->GetName().find("beacon") == 0) {
                            std::string beacon_name = std::string("beacon__") + model->GetName();

                            if (this->rssi_noise_generators.find(beacon_name) == this->rssi_noise_generators.end()){
                                this->rssi_noise_generators[beacon_name] = beacon_gazebo_sim::RSSINoise(this->m_rssi);
                            }

                            auto beacon_p = link->WorldPose().Pos();
                            auto receiver_p = this->receiver_link->WorldPose().Pos();

                            double d = std::sqrt(
                                    std::pow(beacon_p.X() - receiver_p.X(), 2) +
                                    std::pow(beacon_p.Y() - receiver_p.Y(), 2) +
                                    std::pow(beacon_p.Z() - receiver_p.Z(), 2)
                                    );

                            beacon_gazebo_sim::ReceiverIn msg;
                            msg.time_stamp = ros::Time::now();
                            msg.rssi = this->rssi_noise_generators[beacon_name].getRSSI(d, elapsed.Double());
                            msg.id = beacon_name;
                            msg.m_rssi = this->m_rssi;
                            this->receiver_in_msgs_publisher.publish(msg);

//                            ROS_INFO("ReceiverModelPlugin:\n\treceiver_name: %s\n\tbeacon_name: %s\n\tdistance: %lf\n\trssi: %lf\n\trssi_m: %lf",
//                                     this->receiver_name.c_str(), beacon_name.c_str(), d, msg.rssi, msg.m_rssi);
                        }
                    }
                }
                this->l_u_time = simTime;
            }
        }
    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        physics::LinkPtr receiver_link;
        std::string receiver_name;
        physics::WorldPtr world;
        float update_rate;
        std::map<std::string, beacon_gazebo_sim::RSSINoise> rssi_noise_generators;
        common::Time l_u_time;
        ros::Publisher receiver_in_msgs_publisher;
        const double m_rssi = -60.0; // TODO: get m_rssi from config or world
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ReceiverModelPlugin)
}