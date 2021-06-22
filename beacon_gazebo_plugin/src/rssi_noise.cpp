//
// Created by Vasily Yuryev on 22.06.2021.
//
#include <beacon_gazebo_plugin/rssi_noise.h>

namespace beacon_gazebo_plugin {
    RSSINoise::RSSINoise() {
        this->m_rssi = -60;
    }
    RSSINoise::RSSINoise(double m_rssi) {
        this->m_rssi = m_rssi;
    }

    double RSSINoise::getRSSI(double distance) {
        return this->get_rssi_from_distance(distance);
    }
    double RSSINoise::get_rssi_from_distance(double distance) { // TODO: add noise simulation
        return this->m_rssi * std::pow(((distance - 0.0) / 1.0), (1.0 / 10.0));
    }

}
