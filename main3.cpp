//
// Created by liheng on 2018/11/27.
//

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv/cv.hpp>
#include "opencv2/ximgproc.hpp"
#include "LaneDetection.h"
#include "LaneDetectionLSD.h"

using std::cout;
using std::end;

int main()
{

    LaneDetectionLSD laneDetectionLSD;
    char imgPath[256];
    sprintf(imgPath,"/home/liheng/CLionProjects/kitti/01/left/%06d.png",0);
    laneDetectionLSD.initialize_variable(imgPath);

    for(int i=1; i<300; ++i)
    {
        char imgPath[256];
        sprintf(imgPath,"/home/liheng/CLionProjects/kitti/01/left/%06d.png",i);

        cv::Mat srcImage = cv::imread(imgPath,cv::IMREAD_COLOR);

        cv::Mat grayImage;
        cv::Mat edgeImage;
        cv::Mat resImage;
        std::vector<cv::Vec4f> lines;
        laneDetectionLSD.preImage(srcImage,grayImage);
        resImage = grayImage.clone();
        if(1==resImage.channels() )
            cv::cvtColor(resImage,resImage,CV_GRAY2BGR);
        laneDetectionLSD.getEdgeImage(grayImage,edgeImage);
        laneDetectionLSD.getLines(edgeImage,lines);

        laneDetectionLSD.filterLines(lines);

        int length_threshold = 10;
        float distance_threshold = 1.41421356f;
        double canny_th1 = 50.0;
        double canny_th2 = 50.0;
        int canny_aperture_size = 3;
        bool do_merge = false;
        //here I use auto to express -- cv::Ptr<cv::ximgproc::FastLineDetector>
        auto ls = cv::ximgproc::createFastLineDetector(length_threshold,
                                                       distance_threshold, canny_th1, canny_th2, canny_aperture_size,
                                                       do_merge);


        ls->drawSegments(resImage,lines);

//        auto lsd = createLineSegmentDetector(cv::LSD_REFINE_STD, 0.8, 0.6, 2.0, 22.5, 0.0, 0.7, 256);
//        lsd->drawSegments()


        cv::imshow("edgeImage",edgeImage);
        cv::imshow("lsdResult",resImage);
        cv::waitKey(0);


    }

    return  0;
}