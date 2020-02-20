//
// Created by zhihui on 1/17/20.
//

#ifndef LOCALIZATION_GPS_FILTER_H
#define LOCALIZATION_GPS_FILTER_H

#include <vector>

#include "common.h"

using namespace lins::common;

class GPSFilter {
public:

    GPSFilter();

    std::vector <GPSReading::Ptr> filter(const std::vector <GPSReading::Ptr> &input);

    double GPS_HORI_VAR_THREAS;
    double GPS_VERT_VAR_THREAS;
};


#endif //LOCALIZATION_GPS_FILTER_H
