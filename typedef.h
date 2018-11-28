//
// Created by liheng on 2018/11/28.
//

#ifndef MULTILANE_TYPEDEF_H
#define MULTILANE_TYPEDEF_H

#include <opencv2/opencv.hpp>
#include <vector>

typedef struct _LineSegment
{
    //the start and end point of the line
    cv::Vec4f line;

    //the line's degree
    float degree;

    //the line param of slope and intercept
    float k;
    float b;

    //the points of lines
    std::vector<cv::Point2f> linePoints;

    _LineSegment()
    {
        for(int i=0; i!=4;++i)
            line[i] = -1;

        //horizonal line
        degree = 0.0f;
        k = 0;
        b = 0;

        std::vector<cv::Point2f>().swap( linePoints );

    };


}LineSegment;


#endif //MULTILANE_TYPEDEF_H
