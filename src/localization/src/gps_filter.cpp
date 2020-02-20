//
// Created by zhihui on 1/17/20.
//

#include "localization/private/gps_filter.h"

GPSFilter::GPSFilter() {
    GPS_HORI_VAR_THREAS = pow(Config::get<double>("gps.filter.max_hori_std", 1), 2);
    GPS_VERT_VAR_THREAS = pow(Config::get<double>("gps.filter.max_vert_std"), 2);
}

std::vector <GPSReading::Ptr> GPSFilter::filter(const std::vector <GPSReading::Ptr> &input) {
    std::vector <GPSReading::Ptr> vec;
    for (const auto &gps:input) {
        if (gps->covariance(0, 0) < GPS_HORI_VAR_THREAS && gps->covariance(1, 1) < GPS_HORI_VAR_THREAS
            && gps->covariance(2, 2) < GPS_VERT_VAR_THREAS && gps->covariance(0, 0) != 0.0) {
            vec.push_back(gps);
        }
    }
    return vec;
}
