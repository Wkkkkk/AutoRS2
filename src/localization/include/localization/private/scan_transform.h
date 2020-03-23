//
// Created by zhihui on 3/23/20.
//

#ifndef LOCALIZATION_SCAN_TRANSFORM_H
#define LOCALIZATION_SCAN_TRANSFORM_H

#include "common/scan.h"
#include "message/msg/scan.hpp"

using ScanROS = message::msg::Scan;
using ScanPCL = lins::common::Scan;

ScanROS exportToRosMsg(ScanPCL::Ptr scan);

ScanPCL::Ptr importFromRosMsg(ScanROS scan);

#endif //LOCALIZATION_SCAN_TRANSFORM_H
