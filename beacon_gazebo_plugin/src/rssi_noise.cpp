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

    double RSSINoise::laplace(double mean, double scale) {
        return (-scale*std::log(this->random_generator())) - (-scale*std::log(this->random_generator())) + mean;
    }

    double RSSINoise::getRSSI(double distance, double delta_time) { // TODO: add noise simulation
        double a = 10;
        double b = 1;
        double c = 0;

        double rssi_1 = this->m_rssi * std::pow(((distance - c) / b), (1.0 / a));
//        std::normal_distribution<double> nd_rssi(rssi_1, 2);

        double rssi_out = laplace(rssi_1, 2); // I think this is the closest distribution to the real one

        return rssi_out;
    }
}
