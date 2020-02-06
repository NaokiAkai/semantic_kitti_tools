/****************************************************************************
 * Copyright (C) 2019 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * point_cloud_mapping builds sematinc point maps using the SemanticKITTI dataset.
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <VectorLib.h>
#include <SKDAPI.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr filterPC(pcl::PointCloud<pcl::PointXYZ>::Ptr PC) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
    if ((int)PC->points.size() == 0)
        return filteredCloud;
    pcl::VoxelGrid<pcl::PointXYZ> vgf;
    vgf.setInputCloud(PC);
    vgf.setLeafSize(0.1, 0.1, 0.1);
    vgf.filter(*filteredCloud);
    return filteredCloud;
}

void savePCDFile(std::string PCDFilePath, pcl::PointCloud<pcl::PointXYZ>::Ptr PC) {
    if (PC->points.size() > 0) {
        pcl::io::savePCDFileBinary(PCDFilePath.c_str(), *PC);
        printf("%s was saved.\n", PCDFilePath.c_str());
    } else {
        printf("%s was not saved because it is empty\n", PCDFilePath.c_str());
    }
}

int main(int argc, char **argv) {
    if (argv[1] == NULL) {
        fprintf(stderr, "argv[1] must be path to dataset sequence -> /path/to/sequence/00/\n");
        exit(1);
    }
    if (argv[2] == NULL) {
        fprintf(stderr, "argv[2] must be output directory -> /path/to/output/directory/\n");
        exit(1);
    }
    bool isFlatSurface = true;

    std::string datasetDirName = argv[1];
    skd::SKDAPI skdapi(datasetDirName);
    skdapi.getCalibMats();
    std::vector<skd::StampedPose> stampedPoses = skdapi.getVelodyneStampedPoses();

    std::string outputDirName = argv[2];
    std::string roadPCDFilePath = outputDirName + "road.pcd";
    std::string parkingPCDFilePath = outputDirName + "parking.pcd";
    std::string sidewalkPCDFilePath = outputDirName + "sidewalk.pcd";
    std::string otherGroundPCDFilePath = outputDirName + "other_ground.pcd";
    std::string buildingPCDFilePath = outputDirName + "building.pcd";
    std::string fencePCDFilePath = outputDirName + "fence.pcd";
    std::string otherStructurePCDFilePath = outputDirName + "other_structure.pcd";
    std::string laneMarkingPCDFilePath = outputDirName + "lane_marking.pcd";
    std::string vegetationPCDFilePath = outputDirName + "vegetation.pcd";
    std::string trunkPCDFilePath = outputDirName + "trunk.pcd";
    std::string terrainPCDFilePath = outputDirName + "terrain.pcd";
    std::string polePCDFilePath = outputDirName + "pole.pcd";
    std::string trafficSignPCDFilePath = outputDirName + "traffic_sign.pcd";
    std::string otherObjectsPCDFilePath = outputDirName + "other_object.pcd";
    std::string unlabeledPCDFilePath = outputDirName + "unlabeled.pcd";
    std::string staticObjectsPCDFilePath = outputDirName + "static_objects.pcd";
    std::string allObjectsPCDFilePath = outputDirName + "all_objects.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr roadPC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr parkingPC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr sidewalkPC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr otherGroundPC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr buildingPC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr fencePC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr otherStructurePC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr laneMarkingPC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr vegetationPC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr trunkPC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr terrainPC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr polePC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr trafficSignPC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr otherObjectsPC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr unlabeledPC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr staticObjectsPC(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr allObjectsPC(new pcl::PointCloud<pcl::PointXYZ>());

    std::string trajectoryFilePath = outputDirName + "trajectory.txt";
    skdapi.writeVelodyneTrajectory(trajectoryFilePath, stampedPoses);

    int timeIdx = 0;
    std::vector<double> staticPointsRates;
    FILE *fpStaticPointsRate = fopen("/tmp/static_points_rate.txt", "w");
    for (;;) {
        std::vector<skd::VelodynePoint> velodynePoints = skdapi.getVelodynePoints(timeIdx);
        if ((int)velodynePoints.size() == 0)
            break;

        std::vector<float> transMat;
        if (!isFlatSurface)
            transMat = stampedPoses[timeIdx].transMat_;
        else
            transMat = skdapi.getTransMat(stampedPoses[timeIdx].x_, stampedPoses[timeIdx].y_, 0.0f, 0.0f, 0.0f, stampedPoses[timeIdx].yaw_);

        int staticPointsNum = 0;
        for (int i = 0; i < (int)velodynePoints.size(); i++) {
            float x = transMat[0] * velodynePoints[i].x_ + transMat[1] * velodynePoints[i].y_ + transMat[2] * velodynePoints[i].z_ + transMat[3];
            float y = transMat[4] * velodynePoints[i].x_ + transMat[5] * velodynePoints[i].y_ + transMat[6] * velodynePoints[i].z_ + transMat[7];
            float z = transMat[8] * velodynePoints[i].x_ + transMat[9] * velodynePoints[i].y_ + transMat[10] * velodynePoints[i].z_ + transMat[11];
            pcl::PointXYZ point(x, y, z);
            unsigned short label = velodynePoints[i].label_;
            if (label == 40) {
                roadPC->points.push_back(point);
                staticObjectsPC->points.push_back(point);
                staticPointsNum++;
            } else if (label == 44) {
                parkingPC->points.push_back(point);
                staticObjectsPC->points.push_back(point);
                staticPointsNum++;
            } else if (label == 48) {
                sidewalkPC->points.push_back(point);
                staticObjectsPC->points.push_back(point);
                staticPointsNum++;
            } else if (label == 49) {
                otherGroundPC->points.push_back(point);
                staticObjectsPC->points.push_back(point);
                staticPointsNum++;
            } else if (label == 50) {
                buildingPC->points.push_back(point);
                staticObjectsPC->points.push_back(point);
                staticPointsNum++;
            } else if (label == 51) {
                fencePC->points.push_back(point);
                staticObjectsPC->points.push_back(point);
                staticPointsNum++;
            } else if (label == 52) {
                otherStructurePC->points.push_back(point);
                staticObjectsPC->points.push_back(point);
                staticPointsNum++;
            } else if (label == 60) {
                laneMarkingPC->points.push_back(point);
                staticObjectsPC->points.push_back(point);
                staticPointsNum++;
            } else if (label == 70) {
                vegetationPC->points.push_back(point);
                staticObjectsPC->points.push_back(point);
                staticPointsNum++;
            } else if (label == 71) {
                trunkPC->points.push_back(point);
                staticObjectsPC->points.push_back(point);
                staticPointsNum++;
            } else if (label == 72) {
                terrainPC->points.push_back(point);
                staticObjectsPC->points.push_back(point);
                staticPointsNum++;
            } else if (label == 80) {
                polePC->points.push_back(point);
                staticObjectsPC->points.push_back(point);
                staticPointsNum++;
            } else if (label == 81) {
                trafficSignPC->points.push_back(point);
                staticObjectsPC->points.push_back(point);
                staticPointsNum++;
            } else if (label == 99) {
                staticObjectsPC->points.push_back(point);
                otherObjectsPC->points.push_back(point);
                staticPointsNum++;
            } else if (label == 0) {
                unlabeledPC->points.push_back(point);
//                staticObjectsPC->points.push_back(point);
//                staticPointsNum++;
            }
            allObjectsPC->points.push_back(point);
        }

        double staticPointsRate = (double)staticPointsNum / (double)velodynePoints.size();
        staticPointsRates.push_back(staticPointsRate);
        fprintf(fpStaticPointsRate, "%f %lf\n", stampedPoses[timeIdx].time_, staticPointsRate);

        printf("pose size = %d, time idx = %d, velodyne points num = %d, static points rate = %lf [%%]\n", (int)stampedPoses.size(), timeIdx, (int)velodynePoints.size(), staticPointsRate * 100.0);
        timeIdx++;
    }
    fclose(fpStaticPointsRate);

    savePCDFile(roadPCDFilePath, filterPC(roadPC));
    savePCDFile(parkingPCDFilePath, filterPC(parkingPC));
    savePCDFile(sidewalkPCDFilePath, filterPC(sidewalkPC));
    savePCDFile(otherGroundPCDFilePath, filterPC(otherGroundPC));
    savePCDFile(buildingPCDFilePath, filterPC(buildingPC));
    savePCDFile(fencePCDFilePath, filterPC(fencePC));
    savePCDFile(otherStructurePCDFilePath, filterPC(otherStructurePC));
    savePCDFile(laneMarkingPCDFilePath, filterPC(laneMarkingPC));
    savePCDFile(vegetationPCDFilePath, filterPC(vegetationPC));
    savePCDFile(trunkPCDFilePath, filterPC(trunkPC));
    savePCDFile(terrainPCDFilePath, filterPC(terrainPC));
    savePCDFile(polePCDFilePath, filterPC(polePC));
    savePCDFile(trafficSignPCDFilePath, filterPC(trafficSignPC));
    savePCDFile(otherObjectsPCDFilePath, filterPC(otherObjectsPC));
    savePCDFile(unlabeledPCDFilePath, filterPC(unlabeledPC));
    savePCDFile(staticObjectsPCDFilePath, filterPC(staticObjectsPC));
    savePCDFile(allObjectsPCDFilePath, filterPC(allObjectsPC));

    printf("staticPointsRate\n");
    printf("ave = %lf [%%], std = %lf [%%], min = %lf [%%], max = %lf [%%]\n", 
        VectorLib::getAve(staticPointsRates) * 100.0, sqrt(VectorLib::getVar(staticPointsRates)) * 100.0, VectorLib::getMin(staticPointsRates) * 100.0, VectorLib::getMax(staticPointsRates) * 100.0);

    return 0;
}
