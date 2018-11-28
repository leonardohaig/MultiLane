//
// Created by liheng on 2018/11/27.
//

#ifndef MULTILANE_LANEDETECTIONLSD_H
#define MULTILANE_LANEDETECTIONLSD_H
#include <opencv2/opencv.hpp>
#include <vector>
#include "typedef.h"


class LaneDetectionLSD
{
private:

    struct LANE_MARKING {
        cv::Point2f str_p;
        cv::Point2f cnt_p;
        cv::Point2f end_p;
        cv::Point2f inn_p;
        int size;
    };


    // Image
    cv::Size img_size;
    cv::Mat m_grayImageROI;
    int img_height;
    int img_width;
    int img_roi_height;
    int img_depth;

    // Lane marking variable
    std::vector<int> max_lw;
    std::vector<int> min_lw;
    std::vector<int> max_lw_d;
    std::vector<int> lm;//save the index

    cv::Mat m_gradientImageROI;
    cv::Mat m_curFramelineMapROI;
    std::vector<cv::Vec4f> m_lines;


public:
    bool initialize_variable(char img_name[256]);
    void preImage(const cv::Mat& InputColorImage,cv::Mat& OutputGrayImage);

    int computeThreshold(const cv::Mat& src);
    void getEdgeImage(const cv::Mat& InputGrayImage,cv::Mat& edgeImage);

    void getLines(const cv::Mat& edgeImage,std::vector<cv::Vec4f>& lines);
    void filterLines(std::vector<cv::Vec4f>& lines);
    int judegLinePNFlag(const cv::Vec4f& line);
    float calLineConfidence(const cv::Vec4f& line);
    void mergeLinesIntoLines(std::vector<cv::Vec4f>& lines);//merge into line,not curve
    float marking_thres(float input);

};


#endif //MULTILANE_LANEDETECTIONLSD_H
