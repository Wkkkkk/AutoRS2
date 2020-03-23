//
// Created by zhihui on 3/23/20.
//

#ifndef LOCALIZATION_INIT_H
#define LOCALIZATION_INIT_H

#include "common.hpp"

using Route = lins::common::Route;
using Config = lins::common::Config;

Route::Ptr readGlobalHMap(std::string pcmap_hmap_path);

Route::Ptr init();


#endif //LOCALIZATION_INIT_H
