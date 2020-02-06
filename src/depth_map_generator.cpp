/****************************************************************************
 * Copyright (C) 2019 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * depth_map_generator generates depth maps.
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <SKDAPI.h>

void yp2uv(float yaw, float pitch, int width, int height, int *u, int *v) {
    float fovUpRad = 3.0f * M_PI / 180.0f;
    float fovDownRad = -25.0f * M_PI / 180.0f;
    *u = width - 1 - (int)((float)width / (2.0f * M_PI) * (yaw + M_PI));
    *v = (int)((float)height / (fovDownRad - fovUpRad) * (pitch - fovUpRad));
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
    float minRange = 3.0f;
    float maxRange = 120.0f;
    int width = 360;
    int height = 32;

    std::string datasetDirName = argv[1];
    std::string outputDirName = argv[2];
    skd::SKDAPI skdapi(datasetDirName);


    int timeIdx = 0;
    for (;;) {
        std::vector<skd::VelodynePoint> velodynePoints = skdapi.getVelodynePoints(timeIdx);
        if ((int)velodynePoints.size() == 0)
            break;

        cv::Mat depthMap = cv::Mat::zeros(height, width, CV_8U);
        cv::Mat intensityMap = cv::Mat::zeros(height, width, CV_8U);
        cv::Mat colorLabelMap = cv::Mat::zeros(height, width, CV_8UC3);
        cv::Mat labelMap = cv::Mat::zeros(height, width, CV_8U);
        cv::Mat pointMap = cv::Mat::zeros(height, width, CV_32FC3);
        for (int i = 0; i < (int)velodynePoints.size(); i++) {
            int label = velodynePoints[i].label_;
            float x = velodynePoints[i].x_;
            float y = velodynePoints[i].y_;
            float z = velodynePoints[i].z_;
            float intensity = velodynePoints[i].intensity_;
            float range = sqrt(x * x + y * y + z * z);
            if (range < minRange)
                continue;
            if (range > maxRange)
                range = maxRange;
            int val = (int)(range / maxRange * 255.0f);
            float yaw = atan2(y, x);
            float pitch = atan2(z, sqrt(x * x + y * y));
            int u, v;
            yp2uv(yaw, pitch, width, height, &u, &v);
            if (0 <= u && u < width && 0 <= v && v < height) {
                if (depthMap.at<uchar>(v, u) == 0 || val < depthMap.at<uchar>(v, u)) {
                    depthMap.at<uchar>(v, u) = val;
                    int intensityVal = (int)(intensity * 255.0f);
                    if (intensityVal > 255)
                        intensityVal = 255;
                    intensityMap.at<uchar>(v, u) = intensityVal;

                    unsigned char b, g, r, labelVal;
                    if (label == 40)
                        b = 255, g = 0, r = 255, labelVal = 1;
                    else if (label == 44)
                        b = 255, g = 150, r = 255, labelVal = 2;
                    else if (label == 48)
                        b = 75, g = 0, r = 75, labelVal = 3;
                    else if (label == 49)
                        b = 75, g = 0, r = 175, labelVal = 4;
                    else if (label == 50)
                        b = 0, g = 200, r = 255, labelVal = 5;
                    else if (label == 51)
                        b = 50, g = 120, r = 255, labelVal = 6;
                    else if (label == 52)
                        b = 0, g = 150, r = 255, labelVal = 7;
                    else if (label == 60)
                        b = 170, g = 255, r = 150, labelVal = 8;
                    else if (label == 70)
                        b = 0, g = 175, r = 0, labelVal = 9;
                    else if (label == 71)
                        b = 0, g = 60, r = 135, labelVal = 10;
                    else if (label == 72)
                        b = 80, g = 240, r = 255, labelVal = 11;
                    else if (label == 80)
                        b = 150, g = 240, r = 255, labelVal = 12;
                    else if (label == 81)
                        b = 0, g = 0, r = 255, labelVal = 13;
                    else if (label == 99)
                        b = 255, g = 255, r = 50, labelVal = 14;
                    else // all of the non-static objects are categorized as the unknown
                        b = 0, g = 0, r = 0, labelVal = 0;
                    colorLabelMap.at<cv::Vec3b>(v, u)[0] = b;
                    colorLabelMap.at<cv::Vec3b>(v, u)[1] = g;
                    colorLabelMap.at<cv::Vec3b>(v, u)[2] = r;
                    labelMap.at<uchar>(v, u) = labelVal;

                    pointMap.at<cv::Vec3f>(v, u)[0] = x;
                    pointMap.at<cv::Vec3f>(v, u)[1] = y;
                    pointMap.at<cv::Vec3f>(v, u)[2] = z;
                }
            }
        }

        std::string depthMapFilePath = outputDirName + "depth_map_" + std::to_string(timeIdx) + ".pgm";
        std::string intensityMapFilePath = outputDirName + "intensity_map_" + std::to_string(timeIdx) + ".pgm";
        std::string colorLabelMapFilePath = outputDirName + "color_label_map_" + std::to_string(timeIdx) + ".ppm";
        std::string labelMapFilePath = outputDirName + "label_map_" + std::to_string(timeIdx) + ".pgm";
        cv::imwrite(depthMapFilePath, depthMap);
        cv::imwrite(intensityMapFilePath, intensityMap);
        cv::imwrite(colorLabelMapFilePath, colorLabelMap);
        cv::imwrite(labelMapFilePath, labelMap);

        std::string pointsFilePath = outputDirName + "points_" + std::to_string(timeIdx) + ".txt";
        FILE *fpPoints = fopen(pointsFilePath.c_str(), "w");
        for (int v = 0; v < height; v++) {
            for (int u = 0; u < width; u++)
                fprintf(fpPoints, "%f %f %f %f\n", pointMap.at<cv::Vec3f>(v, u)[0], pointMap.at<cv::Vec3f>(v, u)[1], pointMap.at<cv::Vec3f>(v, u)[2], (float)intensityMap.at<uchar>(v, u) / 255.0f);
        }
        fclose(fpPoints);

        printf("timeIdx = %d\n", timeIdx);
        timeIdx++;
    }

    return 0;
}
