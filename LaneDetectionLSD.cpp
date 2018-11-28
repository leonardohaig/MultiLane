//
// Created by liheng on 2018/11/27.
//

#include <opencv2/ximgproc/fast_line_detector.hpp>
#include <numeric>
#include <algorithm>//for the max() fun
#include "LaneDetectionLSD.h"

#define MAX_LANE_MARKING 4000//2000
#define MAX_LW_N 40		// Max lane width, nearest
#define MAX_LW_F 8		// Max lane width, farest
#define MAX_LW_D 10		// Max lane width, delta
#define MIN_LW_N 20 		// Min lane width, nearest
#define MIN_LW_F 2			// Min lane width, farest
#define SCAN_INTERVAL1 1	//lower
#define SCAN_INTERVAL2 1	//upper


bool LaneDetectionLSD::initialize_variable(char img_name[256])
{

    // Image variable setting
    cv::Mat img_src = cv::imread(img_name);
    if (img_src.empty())
    {
        std::cout << "Err: Cannot find an input image for initialization: " << img_name << std::endl;
        return false;
    }

    img_size = img_src.size();
    img_height = img_src.rows;
    img_width = img_src.cols;
    img_roi_height = (int)(img_size.height*0.5);
    img_depth = img_src.depth();

    max_lw.resize(img_height);
    min_lw.resize(img_height);
    max_lw_d.resize(img_width);

    // Estimated Lane Width
    for (int hh = img_roi_height; hh < img_height; ++hh) {
        max_lw[hh] = (int)((MAX_LW_N - MAX_LW_F)*(hh - img_roi_height) / (img_size.height - img_roi_height) + MAX_LW_F);
        min_lw[hh] = (int)((MIN_LW_N - MIN_LW_F)*(hh - img_roi_height) / (img_size.height - img_roi_height) + MIN_LW_F);
    }

    int w = img_width - 1;
    while (img_width - 1 - w < w) {
        max_lw_d[w] = (int)(MAX_LW_D*(abs(w - (img_width - 1) / 2.0)) / ((img_width - 1) / 2.0));
        max_lw_d[img_width - 1 - w] = (int)(MAX_LW_D*(abs(w - (img_width - 1) / 2.0)) / ((img_width - 1) / 2.0));
        w--;
    }

    return true;
}

void LaneDetectionLSD::preImage(const cv::Mat& InputColorImage,cv::Mat& OutputGrayImage)
{
    //get gray image
    //cv::Mat grayImage;
    cv::cvtColor(InputColorImage.rowRange(img_roi_height,img_height),OutputGrayImage,CV_BGR2GRAY);


    //erode and dilate
    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::erode(OutputGrayImage,OutputGrayImage,element);
    cv::dilate(OutputGrayImage,OutputGrayImage,element);

    cv::Mat blurred;
    cv::Mat temp;
    GaussianBlur(OutputGrayImage, blurred, cv::Size(0, 0), 6);
    addWeighted(OutputGrayImage, 1.5, blurred, -0.5, 0, temp);
    bilateralFilter(temp, OutputGrayImage, 5, 150, 150);
    m_grayImageROI = OutputGrayImage.clone();
}

int LaneDetectionLSD::computeThreshold(const cv::Mat& src)
{
    cv::Mat tmp;

    cv::Mat dx, dy, g2, g;
    Sobel(src, dx, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_REPLICATE);
    Sobel(src, dy, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_REPLICATE);

    g2 = dx.mul(dx) + dy.mul(dy);

    g2.convertTo(g2, CV_64F);
    sqrt(g2, g);
    g.convertTo(g, CV_8UC1);

    return threshold(g, tmp, 0, 255, CV_THRESH_BINARY + CV_THRESH_OTSU);
}
void LaneDetectionLSD::getEdgeImage(const cv::Mat& InputGrayImage,cv::Mat& edgeImage)
{
    cv::Canny(InputGrayImage,edgeImage,100,200);
    cv::threshold(edgeImage,edgeImage,100,200,cv::THRESH_BINARY);

}

void LaneDetectionLSD::getLines(const cv::Mat& edgeImage,std::vector<cv::Vec4f>& lines)
{
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

    std::vector<cv::Vec4f> _lines;
    ls->detect(edgeImage, _lines);

#ifndef NDEBUG
    int nLinesBefor = _lines.size();
#endif
    // Remove horizontal and vertical lines
    // and ensure the line's startPoint is smaller than endPoint
    float horizontalDegree = 5.0f;
    float verticalDegree = 85.0f;
    std::vector<cv::Vec4f>().swap( lines );
    lines.reserve( _lines.size()*2/3 );
    for(int i=0; i!=_lines.size();++i)
    {
        cv::Vec4f& line =  _lines[i];
        float dx = line[0] - line[2];
        float dy = line[1] - line[3];
        float lineDegree = atan(dy/dx) * 180.0f / CV_PI;//cal line's degree

        if( abs(lineDegree) < verticalDegree && abs(lineDegree) >horizontalDegree )
        {
            if( _lines[i][1] > _lines[i][3] )
            {
                std::swap(_lines[i][0],_lines[i][2]);
                std::swap(_lines[i][1],_lines[i][3]);
            }

            lines.push_back(_lines[i]);
        }


    }

    m_lines.assign(lines.begin(),lines.end());

//    //Remove short lines
//    std::vector<float> lengthArray;
//    lengthArray.reserve(lines.size());
//    for(int i=0; i!=lines.size(); ++i)
//    {
//        cv::Vec4f& line =  lines[i];
//        float _length = (line[0]-line[2])*(line[0]-line[2]) + (line[1]-line[3])*(line[1]-line[3]);
//        lengthArray.push_back( sqrt(_length) );
//    }
//
//
//    //remove 1/3 lines that are shorter
//    float avgLength = std::accumulate(lengthArray.begin(),lengthArray.end(),0.0f) / std::max((int)lengthArray.size(),1);
//
//    std::vector<cv::Vec4f>().swap( _lines );
//    _lines.reserve( lines.size()*2/3);
//    for(int i=0; i!=lines.size(); ++i)
//    {
//        if( lengthArray[i] >= avgLength )
//            _lines.push_back(lines[i]);
//    }
//
//    lines.swap( _lines );


#ifndef NDEBUG
    int nLinesAfter = lines.size();
    printf("Lines before = %d;Lines after = %d\r\n",nLinesBefor,nLinesAfter);
#endif


}

void LaneDetectionLSD::filterLines(std::vector<cv::Vec4f>& lines)
{
    m_curFramelineMapROI.create(m_grayImageROI.size(),CV_32SC1);
    m_curFramelineMapROI.setTo(0.0f);

    std::vector<cv::Vec4f> posLines,negLines;
    posLines.reserve( lines.size()/2);
    negLines.reserve(lines.size()/2);

    for(int i=0; i!=lines.size();++i)
    {
        int flag = judegLinePNFlag(lines[i]);
        flag == 1 ? posLines.push_back( lines[i] ) : negLines.push_back( lines[i] );

        cv::Vec4f& line =  lines[i];
        cv::Point startP,endP;
        startP.x = cvRound(line[0]);
        startP.y = cvRound(line[1]);
        endP.x = cvRound(line[2]);
        endP.y = cvRound(line[3]);

        flag == 1 ? cv::line(m_curFramelineMapROI,startP,endP,cv::Scalar(i)) : cv::line(m_curFramelineMapROI,startP,endP,cv::Scalar(-1*i));
    }


#ifndef NDEBUG
#if 0

    //show the pos and neg lines
    cv::Mat imgPosNegLines;
    cv::cvtColor(m_grayImageROI,imgPosNegLines,CV_GRAY2BGR);

    cv::Scalar color(0,255,0);
    for(int i=0; i<posLines.size(); ++i)
    {
        cv::Point startP,endP;
        startP.x = cvRound(posLines[i][0]);
        startP.y = cvRound(posLines[i][1]);
        endP.x = cvRound(posLines[i][2]);
        endP.y = cvRound(posLines[i][3]);
        cv::line(imgPosNegLines,startP,endP,color,2);
    }

    color = cv::Scalar(0,0,255);
    for(int i=0; i<negLines.size(); ++i)
    {
        cv::Point startP,endP;
        startP.x = cvRound(negLines[i][0]);
        startP.y = cvRound(negLines[i][1]);
        endP.x = cvRound(negLines[i][2]);
        endP.y = cvRound(negLines[i][3]);
        cv::line(imgPosNegLines,startP,endP,color,2);
    }

    cv::imshow("imgPosNegLines",imgPosNegLines);
#endif
#endif


    //cal lines confidence
    std::vector<float> confidenceArray;
    confidenceArray.resize(posLines.size());
    for(int i=0; i!=posLines.size();++i)
    {
        confidenceArray[i] = calLineConfidence(posLines[i]);
    }



    //show lines confidence
#ifndef NDEBUG
#if 1

    //show the pos and neg lines
    cv::Mat imgLinesConfidence;
    cv::cvtColor(m_grayImageROI,imgLinesConfidence,CV_GRAY2BGR);

    for(int i=0; i<posLines.size(); ++i)
    {
        cv::Point startP,endP;
        startP.x = cvRound(posLines[i][0]);
        startP.y = cvRound(posLines[i][1]);
        endP.x = cvRound(posLines[i][2]);
        endP.y = cvRound(posLines[i][3]);

        cv::Scalar color(0,255,0);
        if(confidenceArray[i]>=0.05 && confidenceArray[i]<0.3 )
            color = cv::Scalar(0,255,0);
        else if(confidenceArray[i]<0.05)
            color = cv::Scalar(0,0,255);
        else
            color = cv::Scalar(255,255,0);

        cv::line(imgLinesConfidence,startP,endP,color,2);
    }

    cv::imshow("imgLinesConfidence",imgLinesConfidence);

#endif
#endif








#ifndef NDEBUG
#if 0
    double min,max;
    cv::minMaxLoc(lineMap,&min,&max);
    cv::convertScaleAbs(lineMap,lineMap,255/(max-min),255/min);
    cv::imshow("lineMap",lineMap);
#endif
#endif






    for (int h = 0; h < m_curFramelineMapROI.rows;)
    {
        // half size of the filter
        int hf_size = 2 + 8 * (h - 0 + 1) / (m_curFramelineMapROI.rows - 0);
        std::vector<int> scan_line(img_width);

        // Edge Extraction
        for (int w = hf_size + 1; w < img_width - hf_size - 1; w++)
        {

            // left edge value, right edge value
            int l_val = 0;
            int r_val = 0;

            for (int i = -hf_size; i<0; i++)
            {
                l_val = l_val + m_grayImageROI.at<uchar>(h, w + i);
            }
            for (int i = 1; i <= hf_size; i++)
            {
                r_val = r_val + m_grayImageROI.at<uchar>(h, w + i);
            }
            if (((float)(r_val - l_val) / (float)hf_size)>marking_thres((float)l_val / (float)hf_size))
            {
                scan_line[w] = 1; // left edge = 1;
            }

            if (((float)(l_val - r_val) / (float)hf_size)>marking_thres((float)r_val / (float)hf_size))
            {
                scan_line[w] = -1; // right edge = -1;
            }

        }// for (int w = hf_size + 1; w < img_width - hf_size - 1; w++)


        // Edge Centering
        int e_flag = 0; // edge flag
        for (int w = hf_size + 1; w < img_width - hf_size - 2; w++)
        {
            if (scan_line[w] == 1)
            {
                if (e_flag >= 0)
                {
                    e_flag++;
                }
                else
                {
                    scan_line[w - (int)(e_flag / 2.0)] = -10;
                    e_flag = 0;
                }
            }
            else if (scan_line[w] == -1)
            {
                if (e_flag <= 0)
                {
                    e_flag--;
                }
                else
                {
                    scan_line[w + (int)(e_flag / 2.0)] = 10;
                    e_flag = 0;
                }
            }
            else
            {
                if (e_flag > 0) {
                    scan_line[w - (int)(e_flag / 2.0)] = 10;
                    e_flag = 0;
                }
                else if (e_flag < 0)
                {
                    scan_line[w + (int)(e_flag / 2.0)] = -10;
                    e_flag = 0;
                }
            }
        }//// Edge Centering

        // Extracting Lane Markings - marking flag
        cv::Point2i l_pt, r_pt;
        int m_flag = 0;

        for (int w = hf_size + 1; w < img_width - hf_size - 1; w++)
        {
            if (scan_line[w] == 10)
            {
                m_flag = 1;
                l_pt.x = w;
                l_pt.y = h;
            }
            if (m_flag == 1)
            {
                if (scan_line[w] == -10)
                {
                    m_flag = 2;
                    r_pt.x = w;
                    r_pt.y = h;
                }
            }
            if (m_flag == 2)
            {
                if (((r_pt.x - l_pt.x) >= min_lw[h+img_roi_height]) && ((r_pt.x - l_pt.x) <= (max_lw[h+img_roi_height] + max_lw_d[w+img_roi_height]))) {

                    // lane update
                    LANE_MARKING lm_new;
                    lm_new.str_p = l_pt;
                    lm_new.end_p = r_pt;
                    lm_new.cnt_p.x = (int)((l_pt.x + r_pt.x) / 2.0);
                    lm_new.cnt_p.y = r_pt.y;
                    if (lm_new.cnt_p.x > (int)(img_size.width / 2))
                    {
                        lm_new.inn_p = l_pt;
                    }
                    else
                    {
                        lm_new.inn_p = r_pt;
                    }
                    lm_new.size = r_pt.x - l_pt.x;

                    //lm.push_back(lm_new);

                    int idx = m_curFramelineMapROI.at<int>(l_pt.y,l_pt.x);
                    lm.push_back( idx );

                    w = r_pt.x + 5;
                    m_flag = 0;
                    if (lm.size() >= MAX_LANE_MARKING - 1) {
                        break;
                    }
                }
                m_flag = 0;
            }
        }//// Extracting Lane Markings

        if (lm.size() >= MAX_LANE_MARKING - 1) {
            break;
        }

        //if (h < 120) {
        //	h += SCAN_INTERVAL1;
        //}
        //else {
        //	h += SCAN_INTERVAL2;
        //}
        h += SCAN_INTERVAL1;


    }//for (int h = 0; h < lineMap.rows;)


    //Remove duplicate items
    sort(lm.begin(),lm.end());
    lm.erase(unique(lm.begin(), lm.end()), lm.end());

//    //show the lines
//    cv::Mat _temp;
//    cv::cvtColor(m_grayImageROI,_temp,CV_GRAY2BGR);
//    for(int i=1; i<lm.size(); ++i)
//    {
//        int idx = lm[i];
//        cv::Point startP,endP;
//        cv::Scalar color(0,0,255);
//        startP.x = cvRound(lines[idx][0]);
//        startP.y = cvRound(lines[idx][1]);
//        endP.x = cvRound(lines[idx][2]);
//        endP.y = cvRound(lines[idx][3]);
//        cv::line(_temp,startP,endP,color,2);
//    }
//
//    cv::imshow("lines after filter",_temp);




}

int LaneDetectionLSD::judegLinePNFlag(const cv::Vec4f& line)
{
    //separate the lines into pos lines and neg lines
    cv::Mat kern = (cv::Mat_<float>(3,3)<<-1,0,1,
            -1,0,1,
            -1,0,1);
    cv::filter2D(m_grayImageROI,m_gradientImageROI,CV_32FC1,kern);

    int flag = -1;
//    //--------------------------------------------------debug-show-right_points
//    cv::Mat show_vp;
//    cv::cvtColor(m_grayImageROI,show_vp,CV_GRAY2BGR);
//    //--------------------------------------------------debug-show-right_points

    //计算x增量
    cv::Point startP,endP;
    startP.x = line[0];
    startP.y = line[1];
    endP.x = line[2];
    endP.y = line[3];

    float vx = (endP.x - startP.x) / (endP.y - startP.y);

    if (endP.y >= m_grayImageROI.rows - 1) endP.y = m_grayImageROI.rows - 2;
    if (endP.y <= 0) endP.y = 1;
    if (startP.y >= m_grayImageROI.rows - 1) startP.y = m_grayImageROI.rows - 2;
    if (startP.y <= 0) startP.y = 1;


    int i_num = (int)abs(endP.y - startP.y);

    int pn_flag = (int)(abs(endP.y - startP.y) / (endP.y - startP.y));

    int p_num = 0;
    int n_num = 0;
    int iw = m_grayImageROI.cols;
    int ih = m_grayImageROI.rows;

    float* fdata = (float*)m_gradientImageROI.data;
    for (int i = 0; i < i_num; i++)
    {
        //计算x,y
        float cur_x = vx*(float)(i*pn_flag) + startP.x;

        int cur_y = (int)startP.y + i*pn_flag;

        int ws = std::max(2, min_lw[cur_y+img_roi_height]);//window size

        //防止越界
        if ((int)(cur_x + ws)>iw-1 || (int)(cur_x - ws + 1) <0)
        {
            continue;
        }

        int temp_v = 0;

        //取极值点位置
        float max_abs_v = 0;

        float min_abs_v = 0;

        int tar_maxX = cur_x + ws;

        int tar_minX = cur_x + ws;

        int tar_x = cur_x;

        int tar_y = cur_y;

        for (int j = (int)cur_x-ws; j < (int)cur_x+ws+1; j++)
        {
            float cur_v = (fdata[cur_y*iw + j]);
            if (cur_v> max_abs_v  )
            {
                max_abs_v = cur_v;

                tar_maxX = j;
            }

            if (cur_v< min_abs_v)
            {
                min_abs_v = cur_v;

                tar_minX = j;
            }

        }//for(j)

//        cv::Point _sp,_ep;
//        _sp.x = cur_x-ws;
//        _sp.y = cur_y;
//        _ep.x = cur_x+ws;
//        _ep.y = cur_y;
//        cv::line(show_vp,_sp,_ep,cv::Scalar(0,255,255));

        if (abs((float)tar_maxX-cur_x)<abs((float)tar_minX-cur_x) && abs(max_abs_v)>abs(min_abs_v/2))
        {
            tar_x = tar_maxX;
        }
        else if(abs((float)tar_maxX - cur_x)>abs((float)tar_minX - cur_x) && abs(max_abs_v/2)<abs(min_abs_v))
        {
            tar_x = tar_minX;
        }
        cv::Point tpp(tar_x, tar_y);
        if (tar_x== tar_maxX)
        {
            p_num++;
            //circle(show_vp, tpp, 2, cv::Scalar(0, 255, 0));//--------------------------------------------------debug-show-right_points
        }
        else if(tar_x == tar_minX)
        {
            n_num++;
            //circle(show_vp, tpp, 2, cv::Scalar(0, 0, 255));//--------------------------------------------------debug-show-right_points
        }

    }//for(i)

//    //--------------------------------------------------debug-show-right_points
//    if (p_num>10)
//    {
//    	imshow("vppshow", show_vp);
//    	cv::waitKey(0);
//    }
//    //--------------------------------------------------debug-show-right_points

    if ((float)(p_num - n_num)>0)
    {
        flag = 1;
    }
    else
    {
        flag = -1;
    }


    return flag;
}

float LaneDetectionLSD::calLineConfidence(const cv::Vec4f& line)
{
    std::vector<cv::Point3i> searchArea;//x,y--curPos,z:searchWidth

    //计算x增量
    cv::Point startP,endP;
    startP.x = line[0];
    startP.y = line[1];
    endP.x = line[2];
    endP.y = line[3];

    float vx = (endP.x - startP.x) / (endP.y - startP.y);

    if (endP.y >= m_grayImageROI.rows - 1) endP.y = m_grayImageROI.rows - 2;
    if (endP.y <= 0) endP.y = 1;
    if (startP.y >= m_grayImageROI.rows - 1) startP.y = m_grayImageROI.rows - 2;
    if (startP.y <= 0) startP.y = 1;


    int i_num = (int)abs(endP.y - startP.y)+1;
    searchArea.reserve(i_num);

    int pn_flag = (int)(abs(endP.y - startP.y) / (endP.y - startP.y));

    int iw = m_grayImageROI.cols;
    int ih = m_grayImageROI.rows;

    for (int i = 0; i < i_num; i++)
    {
        //计算x,y
        float cur_x = vx*(float)(i*pn_flag) + startP.x;
        int cur_y = (int)startP.y + i*pn_flag;

        int ws = std::max(2, (min_lw[cur_y+img_roi_height]+max_lw[cur_y+img_roi_height])/2);//window size

        //防止越界
        if ((int)(cur_x + ws)>=iw || (int)(cur_x - ws) <=0)
            continue;


        searchArea.push_back(cv::Point3i(cur_x,cur_y,ws));

    }//for(i)


    //debug
    std::vector<cv::Point> t_rps, t_lps;
    //debug

    float m_angle_threshold = 5.0f;
    int t_valid_num = 0;

    float curLineDegree = atan( (startP.y-endP.y)*1.0/(startP.x-endP.x) );

    for (int j = 0; j < searchArea.size(); j++)
    {

        //------------------------------------debug-------------------------------------//
        {
            //int tx = temp_lineROI[j].x;
            //int ty = temp_lineROI[j].y;
            //int tw = temp_lineROI[j].width;
            //cv::Point p1(tx - tw / 2, ty);
            //cv::Point p2(tx + tw / 2, ty);
            //cv::line(temp_show, p1, p2, cv::Scalar(0, 255, 0));
        }
        //------------------------------------debug-------------------------------------//

        int tt_x = searchArea[j].x;

        int tt_y = searchArea[j].y;

        int tt_minw = (int)std::max(1, min_lw[tt_y+img_roi_height]);

        int tt_maxw = (int)std::max(tt_minw, max_lw[tt_y+img_roi_height]);

        float* fdata = (float*)m_gradientImageROI.data;

        int icols = m_gradientImageROI.cols;

        if ((tt_x - tt_minw) < 0) tt_minw = tt_x - 0;

        if ((tt_x + tt_minw) > icols - 1) tt_minw = icols - 1 - tt_x;

        if ((tt_x + tt_maxw) > icols - 1) tt_maxw = icols - 1 - tt_x;

        int t_right_x = -1;

        int t_left_x = -1;

        float t_max_gv = 8;//车道线与地面的梯度阈值

        float t_min_gv = -8;

        //找左点的梯度极值坐标
        for (int k = tt_x-tt_minw; k < tt_x + tt_minw+1; k++)
        {
            if (fdata[tt_y*icols+k]>t_max_gv)
            {
                t_left_x = k;

                t_max_gv = fdata[tt_y*icols + k];
            }//if

        }//for(k)

        bool t_have_rp = false;
        if (t_left_x>0)
        {
            //找右点的梯度极值点，且找到右线的点
            short* sdata = (short*)m_curFramelineMapROI.data;

            for (int k = tt_x + tt_minw; k < tt_x + tt_maxw + 1; k++)
            {
                if (fdata[tt_y*icols + k]<t_min_gv)
                {
                    t_right_x = k;

                    t_min_gv = fdata[tt_y*icols + k];

                }//if

                if (sdata[tt_y*icols + k] != 0)
                {
                    int tt_lid = (int)abs(sdata[tt_y*icols + k]);
                    float refLineDegree = atan( (m_lines[tt_lid][1]-m_lines[tt_lid][3])*1.0 / (m_lines[tt_lid][0]-m_lines[tt_lid][2])) ;
                    if ((curLineDegree- refLineDegree) < (m_angle_threshold*2))//m_angle_threshold:线段合并时角度差异阈值，弧度 //==右线要与左线保持“平行”
                    {
                        t_have_rp = true;
                    }//if
                }//if

            }//for(k)

        }//if(t_right_x>0)

        if (t_right_x > 0 && t_have_rp == true)
        {
            t_valid_num++;
            t_lps.push_back(cv::Point(t_left_x, tt_y));
            t_rps.push_back(cv::Point(t_right_x, tt_y));

        }//if(t_left_x > 0)

    }//for(j)

    float confidence = (float)(t_valid_num) / (float)(searchArea.size());     //wx. 置信度：负线/正线的长度


    return  confidence;
}

void LaneDetectionLSD::mergeLinesIntoLines(std::vector<cv::Vec4f>& lines)
{
    //generate lines points
    std::vector<LineSegment> lineSegmentArray;
    lineSegmentArray.resize( lines.size() );
    for(int i=0; i!=lines.size();++i)
    {
        cv::Vec4f& line =  lines[i];
        cv::Point2f startP,endP;
        startP.x = line[0];
        startP.y = line[1];
        endP.x = line[2];
        endP.y = line[3];

        lineSegmentArray[i].line = lines[i];
        float k = (endP.y-startP.y)*1.0 / (endP.x-startP.x);//divde
        float b = endP.y - k*endP.x;
        float degree = atan(k)*180.0 / CV_PI;

        lineSegmentArray[i].k = k;
        lineSegmentArray[i].b = b;
        lineSegmentArray[i].degree = degree;

        int nPointNum = abs(endP.y-startP.y)+1;
        lineSegmentArray[i].linePoints.reserve( nPointNum );
        for(int curY=cvRound(startP.y); curY<=endP.y; ++curY )
        {
            float curX = (curY-b)*1.0 / k;
            lineSegmentArray[i].linePoints.push_back( cv::Point2f(curX,curY));

        }


    }



    //generate the lines map
    int lineMapInitValue = -1;
    cv::Mat curLinesMap(m_grayImageROI.size(),CV_32SC1,cv::Scalar(lineMapInitValue));

    for(int i=0; i!=lines.size();++i)
    {
        cv::Vec4f& line =  lines[i];
        cv::Point startP,endP;
        startP.x = cvRound(line[0]);
        startP.y = cvRound(line[1]);
        endP.x = cvRound(line[2]);
        endP.y = cvRound(line[3]);
        cv::line(curLinesMap,startP,endP,cv::Scalar(i));
    }

    int mapHeight = curLinesMap.rows;
    int mapWidth = curLinesMap.cols;

    //merge lines
    float angleThrehold = 5.0;//unit:degree
    int searchDis = 10;//unit:pixel
    int windowSize = 3;//the line's left and right 3pixel to find line to merge
    for(int i=0; i!=lines.size(); ++i)
    {
        const cv::Vec4f& masterLine = lines[i];
        cv::Point2f masterStartP,masterEndP;
        masterStartP.x = masterLine[0];
        masterStartP.y = masterLine[1];
        masterEndP.x = masterLine[2];
        masterEndP.y = masterLine[3];
        float masterLineDegree = lineSegmentArray[i].degree;
        float k = lineSegmentArray[i].k;
        float b = lineSegmentArray[i].b;

        for(int curY=cvRound(masterStartP.y-searchDis*1.0/sin(atan(k))); curY<masterStartP.y;++curY)
        {
            if( curY<0 || curY>=mapHeight ) continue;

            float x = (curY-b) / k;
            for(int curX=cvRound(x-windowSize);curX<x+windowSize;++x)
            {
                if(curX<0 || curX>=mapWidth ) continue;

                //if find line in line map,then merge it,and upgrate the lineMap,lineSegmentArray ,etc
                int slaveIdx = curLinesMap.at<int>(curY,curX);
                if( slaveIdx != lineMapInitValue )//merge the two line to master line
                {

                    //judeg the two lines's degree is right or not
                    const cv::Vec4f& slaveLine = lines[slaveIdx];
                    cv::Point2f slaveStartP,slaveEndP;
                    slaveStartP.x = slaveLine[0];
                    slaveStartP.y = slaveLine[1];
                    slaveEndP.x = slaveLine[2];
                    slaveEndP.y = slaveLine[3];
                    float k = (slaveEndP.y-slaveStartP.y) / (slaveEndP.x-slaveStartP.x);
                    float b = slaveEndP.y - k*slaveEndP.x;
                    float slaveLineDegree = atan( k)*180.0 / CV_PI;

                    if( abs(masterLineDegree-slaveLineDegree) < angleThrehold*2 )
                    {
                        //get the two lines points,and get the new line by RANSAC
                        LineSegment newLineSegMent;
                        getMergedLineParam(lineSegmentArray[i],lineSegmentArray[slaveIdx],newLineSegMent);

                        if( abs(newLineSegMent.degree-masterLineDegree) < angleThrehold*2 &&
                                abs(newLineSegMent.degree-slaveLineDegree) < angleThrehold*2 )
                        {

                            //update the variables
                            lines[i] = newLineSegMent.line;
                            lineSegmentArray[i] = newLineSegMent;

                            std::vector<cv::Vec4f>::iterator itLines = (lines.begin()+slaveIdx);
                            lines.erase(itLines);

                            std::vector<LineSegment>::iterator itLineSegment = (lineSegmentArray.begin() + slaveIdx);
                            lineSegmentArray.erase( itLineSegment );

                            //this is error!!!!
                            curLinesMap.setTo(i,curLinesMap==slaveIdx);//linemap need to update

                            --i;

                            break;
                        }




                    }






                }



            }


        }

        //merge the two line to master line
        cv::Point2f newLineStartP,newLineEndP;
        if( masterStartP.y < slaveStartP.y )
            ;
        lines[i][0] = 0;











    }
}
bool LaneDetectionLSD::fitLineRansac(const std::vector<cv::Point2f>& vecLinePoint, cv::Vec4f& vec4fLine)
{
    int iterations = 100;//迭代次数
    double sigma = 1.;
    double a_max = 7.;

    int n = vecLinePoint.size();
    //cout <<"point size : "<< n << endl;
    if (n<2)
    {
        printf("Points must be more than 2 EA\n");
        return false;
    }

    cv::RNG rng;
    double bestScore = -1.;
    for (int k = 0; k<iterations; k++)
    {
        int i1 = 0, i2 = 0;
        double dx = 0;
        while (i1 == i2)
        {
            i1 = rng(n);
            i2 = rng(n);
        }
        cv::Point2f p1 = vecLinePoint[i1];
        cv::Point2f p2 = vecLinePoint[i2];

        cv::Point2f dp = p2 - p1;
        dp *= 1. / norm(dp);
        double score = 0;

        if (fabs(dp.x / 1.e-5f) && fabs(dp.y / dp.x) <= a_max)
        {
            for (int i = 0; i<n; i++)
            {
                cv::Point2f v = vecLinePoint[i] - p1;
                double d = v.y*dp.x - v.x*dp.y;
                score += exp(-0.5*d*d / (sigma*sigma));
            }
        }
        if (score > bestScore)
        {
            vec4fLine = cv::Vec4f(dp.x, dp.y, p1.x, p1.y);
            bestScore = score;
        }
    }
    return true;
}

bool LaneDetectionLSD::getMergedLineParam(const LineSegment& line1,const LineSegment& line2,LineSegment& newLine)
{
    int n = line1.linePoints.size();
    int m = line2.linePoints.size();

    std::vector<cv::Point2f> vecLinePoint;
    vecLinePoint.reserve( n+m );
    vecLinePoint.assign(line1.linePoints.begin(),line1.linePoints.end());
    vecLinePoint.insert(vecLinePoint.end(),line2.linePoints.begin(),line2.linePoints.end());

    cv::Vec4f vec4fLine;
    bool bCheck = fitLineRansac(vecLinePoint,vec4fLine);
    if( bCheck )
    {
        cv::Point2f startP,endP;
        startP.x = vec4fLine[0];
        startP.y = vec4fLine[1];
        endP.x = vec4fLine[2];
        endP.y = vec4fLine[3];

        newLine.line = vec4fLine;
        float k = (endP.y-startP.y)*1.0 / (endP.x-startP.x);//divde
        float b = endP.y - k*endP.x;
        float degree = atan(k)*180.0 / CV_PI;

        newLine.k = k;
        newLine.b = b;
        newLine.degree = degree;

        int nPointNum = abs(endP.y-startP.y)+1;
        newLine.linePoints.reserve( nPointNum );
        for(int curY=cvRound(startP.y); curY<=endP.y; ++curY )
        {
            float curX = (curY-b)*1.0 / k;
            newLine.linePoints.push_back( cv::Point2f(curX,curY));

        }
    }

    return bCheck;


}


float LaneDetectionLSD::marking_thres(float input)
{

    float thres = 0;

    /*if(input<50){
    thres = (int)(input/10);
    }else{
    thres = (int)(15+input/200*10);
    }*/
    //return thres;

    return input / 10 + 4;
}