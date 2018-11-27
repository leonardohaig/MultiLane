////
//// Created by liheng on 2018/11/27.
////
//
//
////#include "opencv2/imgcodecs.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include <iostream>
//#include <stdio.h>
//#define SMAX 70
//#define SMIN 100
//#define PMAX 160
//#define PMIN 10
//using namespace cv;
//using namespace std;
//
////  Global variables
//int l=50,u=200;
///*  General variables */
//Mat src, edges;
//Mat src_gray;
//Mat standard_hough, probabilistic_hough;
//int min_threshold = 50;
//int max_trackbar = 200;
//
//const char* standard_name = "Standard Hough Lines Demo";
//const char* probabilistic_name = "Probabilistic Hough Lines Demo";
//
//int s_trackbar = max_trackbar;
//int p_trackbar = max_trackbar;
//
///// Function Headers
//void help();
//void Standard_Hough( int , void* );
//void Probabilistic_Hough( int , void* );
//
///*
// */
//int main( int argc, char**argv )
//{
//    /// Read the image
//    argv[1] = "../../kitti/01/left/000273.png";
//    src = imread( argv[1], 1 );
//
//    if( src.empty() )
//    { help();
//        return -1;
//    }
//
//    /// Pass the image to gray
//    cvtColor( src, src_gray, COLOR_RGB2GRAY );
//
//    /// Apply Canny edge detector
//
//
//    /// Create Trackbars for Thresholds
//    char thresh_label[50];
//    sprintf( thresh_label, "Thres: %d + input", min_threshold );
//
//    namedWindow( standard_name, WINDOW_AUTOSIZE );
//    createTrackbar( thresh_label, standard_name, &s_trackbar, max_trackbar,Standard_Hough);
//    createTrackbar("upper",standard_name,&u,300,Standard_Hough);    //delete after thresholding and update u,l in canny func accordingly
//    createTrackbar("lower",standard_name,&l,300,Standard_Hough);    //
//
//    namedWindow( probabilistic_name, WINDOW_AUTOSIZE );
//    createTrackbar( thresh_label, probabilistic_name, &p_trackbar, max_trackbar,Probabilistic_Hough);
//    createTrackbar("upper",probabilistic_name,&u,300,Probabilistic_Hough);    //delete after thresholding and update u,l in canny func accordingly
//    createTrackbar("lower",probabilistic_name,&l,300,Probabilistic_Hough);    //
//
//    /// Initialize
//
//    Standard_Hough(0,0);
//    Probabilistic_Hough(0,0);
//    waitKey(0);
//    return 0;
//}
//
///**/
//void help()
//{
//    printf("\t Hough Transform to detect lines \n ");
//    printf("\t---------------------------------\n ");
//    printf(" Usage: ./HoughLines_Demo <image_name> \n");
//}
//
///**/
//void Standard_Hough( int , void*)
//{
//    vector<Vec2f> s_lines;
//    Canny( src_gray, edges, l, u, 3 );
//    cvtColor( edges, standard_hough, COLOR_GRAY2BGR );
//    float r_sum_p=0.0,t_sum_p=0.0;
//    float r_avg_p=0.0,t_avg_p=0.0;
//    float r_sum_n=0.0,t_sum_n=0.0;
//    float r_avg_n=0.0,t_avg_n=0.0;
//
//    float x_i,y_i;
//
//    int count_p=0;
//    int count_n=0;
//
//    /// 1. Use Standard Hough Transform
//    HoughLines( edges, s_lines, 1, CV_PI/180, min_threshold + s_trackbar, 0, 0 );
//
//    /// Show the result
//    for( size_t i = 0; i < s_lines.size(); i++ )
//    {
//        float r = s_lines[i][0], t = s_lines[i][1];
//
//        //cout << "theta is "<< t*180/3.14 << endl;
//        double cos_t = cos(t)  ;  double sin_t = sin(t);
//        double x0 = r*cos_t, y0 = r*sin_t;
//        double alpha = 1000;
//
//        Point pt1( cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t) );
//        Point pt2( cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t) );
//
//        //cout << "slope is " << slope <<endl;
//        if ((t*180/3.14 > SMIN ) || ( t*180/3.14 < SMAX) )
//        {
//            if (s_lines[i][1]>0 && s_lines[i][1]<1.57)
//            {
//                r_sum_p+=s_lines[i][0] ; t_sum_p+=s_lines[i][1]; //cout << "theta is " << s_lines[i][1] <<endl;
//                count_p++;
//            }
//            if (s_lines[i][1]<0 || s_lines[i][1]>1.57)
//            {
//                r_sum_n+=s_lines[i][0] ; t_sum_n+=s_lines[i][1]; //cout << "theta is " << s_lines[i][1] <<endl;
//                count_n++;
//            }
//            line( standard_hough, pt1, pt2, Scalar(255,0,0), 2, CV_AA);
//        }
//    }
//
//    if (count_p>0)
//    {
//        t_avg_p=(float)t_sum_p/count_p;r_avg_p=(float)r_sum_p/count_p;
//        //cout << "avg is " << t_avg <<endl;
//        double cos_t_avg = cos(t_avg_p)  ;  double sin_t_avg = sin(t_avg_p);
//        double x0a = r_avg_p*cos_t_avg, y0a = r_avg_p*sin_t_avg;
//        double alpha = 1000;
//
//        Point pt3( cvRound(x0a + alpha*(-sin_t_avg)), cvRound(y0a + alpha*cos_t_avg) );
//        Point pt4( cvRound(x0a - alpha*(-sin_t_avg)), cvRound(y0a - alpha*cos_t_avg) );
//
//        line( standard_hough, pt3, pt4, Scalar(0,255,0), 2, CV_AA);
//    }
//
//
//    if (count_n>0)
//    {
//        t_avg_n=(float)t_sum_n/count_n;r_avg_n=(float)r_sum_n/count_n;
//        //cout << "avg is " << t_avg <<endl;
//        double cos_t_avg = cos(t_avg_n)  ;  double sin_t_avg = sin(t_avg_n);
//        double x0a = r_avg_n*cos_t_avg, y0a = r_avg_n*sin_t_avg;
//        double alpha = 1000;
//
//        Point pt3( cvRound(x0a + alpha*(-sin_t_avg)), cvRound(y0a + alpha*cos_t_avg) );
//        Point pt4( cvRound(x0a - alpha*(-sin_t_avg)), cvRound(y0a - alpha*cos_t_avg) );
//
//        line( standard_hough, pt3, pt4, Scalar(0,255,0), 2, CV_AA);
//    }
//
//    x_i=((r_avg_n)*(sin(t_avg_p))-(r_avg_p)*(sin(t_avg_n)))/( - cos(t_avg_p)*sin(t_avg_n) + cos(t_avg_n)*sin(t_avg_p));
//    y_i=(r_avg_p*cos(t_avg_n)-r_avg_n*cos(r_avg_p))/( -cos(t_avg_p)*sin(t_avg_n) + cos(t_avg_n)*sin(t_avg_p));
//
//    cout << "Intersection is ("<< x_i<< "," << y_i << ")" << endl;
//
//    imshow( standard_name, standard_hough );
//}
//
///* */
//void Probabilistic_Hough( int , void*)
//{
//    vector<Vec4i> p_lines;
//    Canny( src_gray, edges, l, u, 3 );
//    cvtColor( edges, probabilistic_hough, COLOR_GRAY2BGR );
//
//    /// 2. Use Probabilistic Hough Transform
//    HoughLinesP( edges, p_lines, 1, CV_PI/180, min_threshold + p_trackbar, 30, 5 );
//
//    /// Show the result
//    for( size_t i = 0; i < p_lines.size(); i++ )
//    {
//        Vec4i l = p_lines[i];
//        float slope = ((float)l[3]-(float)l[1])/((float)l[2]-(float)l[0]);
//        //cout << "slope is " << slope << endl;
//
//        if ( slope > 0)
//        {
//            line( probabilistic_hough, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 2, CV_AA);
//        }
//        else
//        {
//            line( probabilistic_hough, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, CV_AA);
//        }
//
////        if ( slope > tan(PMIN*3.143/180) || slope < tan(PMAX*3.143/180))
////        {
////            line( probabilistic_hough, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 2, CV_AA);
////        }
////        else if ( slope < -tan(PMIN*3.143/180) || slope > -tan(PMAX*3.143/180))
////        {
////            line( probabilistic_hough, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, CV_AA);
////        }
//    }
//
//    imshow( probabilistic_name, probabilistic_hough );
//}