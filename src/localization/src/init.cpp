//
// Created by zhihui on 3/23/20.
//

#include "localization/init.h"

Route::Ptr readGlobalHMap(std::string pcmap_hmap_path) {
    Route::Ptr activeRoute = Route::Create();
    if (pcmap_hmap_path.empty()) return activeRoute;

    std::ifstream ifs;
    ifs.open(pcmap_hmap_path, std::ios::binary);
    int keyScanSize;
    ifs.read(reinterpret_cast<char *>(&keyScanSize), sizeof(int));

    activeRoute->globalCornerMaps.resize(keyScanSize);
    activeRoute->globalSurfMaps.resize(keyScanSize);

    for (int i = 0; i < keyScanSize; i++) {
        activeRoute->globalCornerMaps[i] = lins::PointRawCloudT::Ptr(new lins::PointRawCloudT);
        activeRoute->globalSurfMaps[i] = lins::PointRawCloudT::Ptr(new lins::PointRawCloudT);
    }

    for (int i = 0; i < keyScanSize; i++) {
        double scan_x, scan_y, scan_z;
        ifs.read(reinterpret_cast<char *>(&scan_x), sizeof(double));
        ifs.read(reinterpret_cast<char *>(&scan_y), sizeof(double));
        ifs.read(reinterpret_cast<char *>(&scan_z), sizeof(double));

        lins::PointMapT scan_center(scan_x, scan_y, scan_z);
        activeRoute->globalPose->push_back(scan_center);

        int surf_size, corner_size;
        ifs.read(reinterpret_cast<char *>(&surf_size), sizeof(int));
        for (int j = 0; j < surf_size; j++) {
            float x, y, z;
            ifs.read(reinterpret_cast<char *>(&x), sizeof(float));
            ifs.read(reinterpret_cast<char *>(&y), sizeof(float));
            ifs.read(reinterpret_cast<char *>(&z), sizeof(float));

            lins::PointRawT pt;
            pt.x = y + scan_y;
            pt.y = z + scan_z;
            pt.z = x + scan_x;
            activeRoute->globalSurf->push_back(pt);

            activeRoute->globalSurfMaps[i]->push_back(pt);
        }
        ifs.read(reinterpret_cast<char *>(&corner_size), sizeof(int));
        for (int j = 0; j < corner_size; j++) {
            float x, y, z;
            ifs.read(reinterpret_cast<char *>(&x), sizeof(float));
            ifs.read(reinterpret_cast<char *>(&y), sizeof(float));
            ifs.read(reinterpret_cast<char *>(&z), sizeof(float));

            lins::PointRawT pt;
            pt.x = y + scan_y;
            pt.y = z + scan_z;
            pt.z = x + scan_x;
            activeRoute->globalCorner->push_back(pt);

            activeRoute->globalCornerMaps[i]->push_back(pt);
        }
    }

    return activeRoute;
}

Route::Ptr init() {
    std::string localization_config = "/home/zhihui/workspace/AutoRS2/localization.yaml";
    Config::setParameterFile(localization_config);

    std::string pcmap_hmap_path = Config::get<std::string>("hmap_file_path");

    Route::Ptr activeRoute = readGlobalHMap(pcmap_hmap_path);

    activeRoute->mapSurfKDTree->setInputCloud(activeRoute->globalSurf);
    activeRoute->mapCornerKDTree->setInputCloud(activeRoute->globalCorner);
    activeRoute->keyPoseKDTree->setInputCloud(activeRoute->globalPose);

    return activeRoute;
}
