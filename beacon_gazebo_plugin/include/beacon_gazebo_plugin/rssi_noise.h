//
// Created by Vasily Yuryev on 21.06.2021.
//

#pragma once
#include <cmath>
#include <string>
#include <vector>

namespace beacon_gazebo_plugin {
    class RSSINoise {
    public:
        RSSINoise();
        RSSINoise(double m_rssi);
        double getRSSI(double distance);
    private:
        double get_rssi_from_distance(double distance);
        double m_rssi;
    };
}
