//
// Created by Vasily Yuryev on 21.06.2021.
//

#pragma once
#include <cmath>
#include <string>
#include <vector>
#include <random>
//#include <boost/math/distributions/laplace.hpp>

namespace beacon_gazebo_plugin {
    class RSSINoise {
    public:
        RSSINoise();
        RSSINoise(double m_rssi);
        double getRSSI(double distance, double delta_time);
    private:
        double m_rssi;
        std::default_random_engine random_generator;

        double laplace(double mean, double scale);
    };
}
