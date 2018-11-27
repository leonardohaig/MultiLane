//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/imgproc/imgproc_c.h"
//#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include <opencv2/videoio/videoio.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui/highgui.hpp>
//
//#include <stdlib.h>
//#include <stdio.h>
//#include <iostream>
//
//
//
//using namespace std;
//using namespace cv;
//
//int hough(Mat &src, Mat &Main_Lane, Mat &Main_Lane_NoBlur, Mat &Sub_Lane);
//int video(string);
//void Merge_Function(Mat &, Mat &);
//bool intersection(Point2f, Point2f, Point2f, Point2f, Point2f &);
//int FramePreproceesing(Mat&, Mat &, Mat &, Mat &);
//void CannyThreshold_side(int, void*);
//void CannyThreshold(int, void*);
//
//
//int Calib_flag = 0;
//int ROI_paveY = 360;
//int ROI_paveX = 300;
//int ROI_Y = 350;
//int ROI_Y_NextFrame = 350;
//int ROI_X = 0;
//int ypoint = 0;
//int xpoint = 0;
//int lowThreshold = 0;
//int lowThresholdside = 0;
//int kernel_size = 3, ratio = 3;
//
//Mat _merge;
//Mat _image, dest;
//Mat _merge_pavement;
//
//int main(int argc, char *argv[])
//{
//
//    //const string VideoFileName = argv[1];
//    const string VideoFileName = "../../kitti/01/left/%06d.png";
//
//    int return_cmdVideo = video(VideoFileName);
//    cout << "##### Info: Program completed #####";
//
//    return return_cmdVideo;
//}
//
//int video(string VideoFileName)
//{
//
//    /************ The calibration is performed by taking the 1st frame of the video. ***************/
//
//
//    VideoCapture calib(VideoFileName);
//
//    if (!calib.isOpened())
//    {
//        cout << "##### ERROR: error at videocapture calib - could not open file" << VideoFileName << "#####" << endl;
//        return -1;
//    }
//
//    /**************** initializing default values for region of interest. This will be modified later***************/
//
//    Mat frame_calib;
//    calib >> frame_calib;
//
//    //cout << frame_calib.rows / 2;
//
//    ROI_paveY = frame_calib.rows / 2;
//    ROI_paveX = frame_calib.cols / 2;
//    ROI_Y = frame_calib.rows / 2;
//
//    _merge = frame_calib.clone();
//    Mat Main_Lane = _merge.clone();
//    Mat Main_Lane_NoBlur = _merge.clone();
//    Mat Sub_Lane = _merge.clone();
//
//    FramePreproceesing(frame_calib, Main_Lane, Main_Lane_NoBlur, Sub_Lane);
//    hough(frame_calib, Main_Lane, Main_Lane_NoBlur, Sub_Lane);
//    cout << "##### info: Calibration Completed #####" << endl;
//    waitKey(0);
//
//
//    /**************** END of initializing default values for region of interest***************/
//
//    /*********************** END of calibration  *************************/
//
//    Calib_flag = 1;
//
//    /************************************ Main video run ***********************************/
//
//    Size S = Size(frame_calib.cols, frame_calib.rows);			// size of of the video frame for updating the output video
//    VideoCapture capture(VideoFileName);
//
//    if (!capture.isOpened())
//    {
//        cout << "##### ERROR: error at videocapture capture - could not open file";
//        return -1;
//    }
//
//    //VideoWriter outputVideo;
//    //outputVideo.open("Output_Video.avi", -1, capture.get(CV_CAP_PROP_FPS), S, true);
//
//    //if (!outputVideo.isOpened())
//    //{
//    //    cout << "##### ERROR: error at VideoWriter outputVideo - could not open file to write";
//    //    return -1;
//    //}
//
//    while (1)
//    {
//        Mat frame;
//        capture >> frame;
//
//        if (frame.empty())
//        {
//            cout << "#### Info: Frame empty #####";
//            break;
//        }
//
//        _merge = frame.clone();
//        _merge_pavement = frame.clone();
//
//        FramePreproceesing(frame, Main_Lane, Main_Lane_NoBlur, Sub_Lane);
//        hough(frame, Main_Lane, Main_Lane_NoBlur, Sub_Lane);
//
//        /*** Reference taken from http://docs.opencv.org/2.4/doc/tutorials/highgui/video-write/video-write.html ***/
//
//        //outputVideo << _merge;						// saving the merged data into a new .avi file
//    }
//
//    cout << "##### info Video Run completed #####";
//    waitKey(0);
//
//    /************************************ End Main video run ***********************************/
//    return 0;
//
//}
//
//void CannyEdgeSeg(Mat& image, int)
//{
//    _image = image.clone();
//    dest = image.clone();
//
//    cvtColor(image, _image, CV_BGR2GRAY);
//    namedWindow("Canny_Filter", CV_WINDOW_NORMAL);
//
//    if (Calib_flag == 0)														// Only for initial calibration
//    {
//        createTrackbar("Min Threshold:", "Canny_Filter", &lowThreshold, 700, CannyThreshold, &kernel_size);
//        CannyThreshold(lowThreshold, 0);
//        createTrackbar("kernel_size:", "Canny_Filter", &kernel_size, 4, CannyThreshold, &lowThreshold);
//        CannyThreshold(kernel_size, 0);
//        waitKey(0);
//    }
//    else																		// For the normal video run
//    {
//        CannyThreshold(lowThreshold, 0);
//    }
//
//}
//
//void CannyThreshold(int, void*)
//{
//    Mat EdgeDetected = _image.clone();
//    blur(_image, EdgeDetected, Size(2 * kernel_size + 1, 2 * kernel_size + 1));
//    Canny(EdgeDetected, EdgeDetected, lowThreshold, lowThreshold*ratio, 2 * kernel_size + 1);
//    imshow("Canny_Filter", EdgeDetected);
//    dest = EdgeDetected.clone();
//}
//
//void CannyEdgeSeg_side(Mat& image_side, int)
//{
//    _image = image_side.clone();			// initializing _image
//    dest = image_side.clone();				// initializing dest
//
//    cvtColor(image_side, _image, CV_BGR2GRAY);
//    namedWindow("Canny_Filter_side", CV_WINDOW_NORMAL);
//
//    if (Calib_flag == 0)					// Only for initial calibration
//    {
//        createTrackbar("Min Threshold:", "Canny_Filter_side", &lowThresholdside, 700, CannyThreshold_side, &kernel_size);
//        CannyThreshold_side(lowThresholdside, 0);
//        createTrackbar("kernel_size:", "Canny_Filter_side", &kernel_size, 4, CannyThreshold_side, &lowThresholdside);
//        CannyThreshold_side(kernel_size, 0);
//        waitKey(0);
//    }
//    else									// For the normal video run
//    {
//        CannyThreshold_side(lowThresholdside, 0);
//    }
//
//}
//
//void CannyThreshold_side(int, void*)
//{
//    Mat EdgeDetected = _image.clone();
//    blur(_image, EdgeDetected, Size(2 * kernel_size + 1, 2 * kernel_size + 1));
//    Canny(EdgeDetected, EdgeDetected, lowThresholdside, lowThresholdside*ratio, 2 * kernel_size + 1);
//    namedWindow("Canny_Filter_side", CV_WINDOW_AUTOSIZE);
//    imshow("Canny_Filter_side", EdgeDetected);
//    dest = EdgeDetected.clone();
//}
//int FramePreproceesing(Mat &src, Mat &Main_Lane, Mat &Main_Lane_NoBlur, Mat &Sub_Lane)
//{
//    /******* For removing noise if any. The processing also helped further in edge detection. The canny worked better as compared to one without the Blur*******/
//
//    GaussianBlur(src, src, Size(7, 7), 1.5, 1.5);
//
//    /***************END of GaussianBlur **************/
//
//    if (Calib_flag == 0)
//    {
//        ROI_Y_NextFrame = ROI_Y;														// For the initial frame for calibration a default ROI is taken as the vanishing point is not known.
//    }
//
//    if (((src.cols - ROI_X - 100) < 0) || ((src.rows - ROI_Y_NextFrame) < 0) || (ROI_X < 0) || (ROI_Y_NextFrame < 0) || (ROI_Y < 0))
//    {
//        cout << "##### Error: The Value of ROI is not in the limits of the image. ROI cannot be obtained";
//        return -1;
//    }
//
//    cout << " #### write info: ROI_Y_NextFrame =" << ROI_Y_NextFrame<<" #####" << endl;
//    Rect roi(ROI_X, ROI_Y_NextFrame, src.cols - ROI_X - 100, src.rows - ROI_Y_NextFrame);
//
//    // Initially the Region Of Interest is taken as the bottom half of the image. It is then adjusted according to the vanishing point which is calculated in the calibration process.
//
//    Main_Lane = src(roi);
//    Main_Lane_NoBlur = _merge(roi);
//
//    Rect roi_pave(0, ROI_paveY, ROI_paveX, src.rows - ROI_paveY);						// Left side lane and pavement detection
//
//    Sub_Lane = _merge(roi_pave);
//
//    int operation = 2;																	// for Close function morphology
//
//    Mat element = getStructuringElement(0, Size(2 * 15 + 1, 2 * 15 + 1), Point(15, 15));//  A 31 x 31 Matrix is taken for the morphological operation
//
//    // A closing operation is performed for removing the false positives which causes unwanted lanes. Eg: removal of shadows
//
//    morphologyEx(Main_Lane, Main_Lane, 3, element);
//
//    return 0;
//}
//
//int hough(Mat &src,Mat &Main_Lane, Mat &Main_Lane_NoBlur, Mat &Sub_Lane)
//{
//    Point2f MainLane_intersectionpoint;					// to find the vanishing point during calibration
//    vector<Vec2f> MainLane_Lines, SubLanes_Lines;		// Detects the number of lines in the image.
//    Point pt1[10000], pt2[10000];						// Maximum lines that can be taken into consideration is 10000. This is done to gt the approximation of the vanishing point.
//
//    int hough_threshold_main = 30;						// The initial threshold for the Lane in which the vehicle is present.
//    int hough_threshold_sub = 50;						// The left Sub Lane for which the vehicle is present
//
//    int Line_increment=0;								// For saving the number of lines that are generated by the hough Transform.
//
//    CannyEdgeSeg(Main_Lane, Calib_flag);
//
//
//    /************		 Performs the Hough Transform for getting the Main Lanes				***************/
//    /************ The algorithm starts with the initial or default threshold and varies the		***************/
//    /************ threshold with a step of 5 for each of the run until the number of lines		***************/
//    /************ generated becomes less than 10. This is in assumption that the road and the	***************/
//    /************ lane have a good edge and this can be detected even with less number of threshold ************/
//
//    do
//    {
//        HoughLines(dest, MainLane_Lines, 1, CV_PI / 180, hough_threshold_main);		// gets the number of hough lines for the Main Lane
//
//        //cout << "MainLane_Lines.size() = " << MainLane_Lines.size() << endl;
//
//        hough_threshold_main = hough_threshold_main + 5;
//
//    } while (MainLane_Lines.size() > 10);
//
//    /*		The theta is checked for each of the lines generated and those which fall in the required range is drawn to the main image		*/
//    /* Reference from : https://marcosnietoblog.wordpress.com/2011/12/27/lane-markings-detection-and-vanishing-point-detection-with-opencv/ */
//
//    for (size_t i = 0; i < MainLane_Lines.size(); i++)
//    {
//        float rho = MainLane_Lines[i][0], theta = MainLane_Lines[i][1];
//
//        // 73 to 78 will work for left lane detection but very crude. To get a wider angle of the lanes 10 to 80 degree and 100 to 170 degree is taken
//
//        if ((theta > CV_PI / 180 * 10 && theta < CV_PI / 180 * 80) || (theta > CV_PI / 180 * 100 && theta < CV_PI / 180 * 170))
//        {
//            double a = cos(theta), b = sin(theta);				// The formula used is: rho = x0 * cos (theta) + y0 * sin (theta)
//            double x0 = a*rho, y0 = b*rho;
//
//            /************* For getting the end points of the line segment *************/
//
//            pt1[Line_increment].x = cvRound(x0 + 1000 * (-b));
//            pt1[Line_increment].y = cvRound(y0 + 1000 * (a));
//            pt2[Line_increment].x = cvRound(x0 - 1000 * (-b));
//            pt2[Line_increment].y = cvRound(y0 - 1000 * (a));
//            line(Main_Lane_NoBlur, pt1[Line_increment], pt2[Line_increment], Scalar(0, 0, 255), 3, CV_AA);
//            Line_increment++;
//
//            /*************************END: end points of the line segment *************************/
//        }
//    }
//    imshow("Main_Lane_NoBlur", Main_Lane_NoBlur);
//
//    /********* A rough estimate of calculating the vanishing point for the image. This is called only during th calibration phase ********/
//    /*********	Reference: http://docs.opencv.org/trunk/d6/d6e/group__imgproc__draw.html#ga482fa7b0f578fcdd8a174904592a6250		  ********/
//
//    if (Calib_flag == 0)
//    {
//        int i_loop = 0;
//        int j_loop = 0;
//        do
//        {
//            intersection(pt1[i_loop], pt2[i_loop], pt1[i_loop + 1], pt2[i_loop + 1], MainLane_intersectionpoint);
//
//            xpoint = (int)MainLane_intersectionpoint.x;
//            ypoint = (int)MainLane_intersectionpoint.y;
//
//            i_loop++;
//            j_loop++;
//        } while (ypoint >= src.rows / 2);
//
//        ROI_Y_NextFrame = Main_Lane_NoBlur.rows + ypoint + 30;		// The intersection point is taken as the vanishing point. So the image is reduced to 30 pixel less than the vanishing point
//        // NOTE: This is a rough approximation only of the vanishing point.
//
////        cout << "#####info: size of merge = " << size(_merge)<<" #####" << endl;
////        cout << "#####info: size of src = " << size(src) << " #####" << endl;
////        cout << "#####info: size of MainLane_NoBlur" << size(Main_Lane_NoBlur) << " #####" << endl;
////        cout << "#####info: size of main lane" << size(Main_Lane) << " #####" << endl;
//        drawMarker(_merge, MainLane_intersectionpoint, 0, 20, 1, 8);
//    }
//
//    /********* END: A rough estimate of calculating the vanishing point for the image. This is called only during th calibration phase ********/
//
//    /************ HoughLinesP are required t0 get the images which have dashed Lanes *************/
//
//    vector<Vec4i> lines;										// Line segments for HoughTransformP.
//    Point2f MainLane_intersectionpoint_houghP[2];				// TO get the intersection point
//
//    // Apply canny edge
//
//    HoughLinesP(dest, lines, 1, 2 * CV_PI / 180, 40, 20, 100);	// The threshold = 40,minLineLength =20, maxLineGap =100
//
//    for (size_t i = 0; i < lines.size(); i++)
//    {
//
//        float rho = lines[i][0], theta = lines[i][1];
//
//        // Point P1 is represented as (x1,y1) and P2 is represented as (x2,y2)
//
//        MainLane_intersectionpoint_houghP[0].x = lines[i][0];		// x1
//        MainLane_intersectionpoint_houghP[0].y = lines[i][1];		// y1
//        MainLane_intersectionpoint_houghP[1].x = lines[i][2];		// x2
//        MainLane_intersectionpoint_houghP[1].y = lines[i][3];		// y2
//
//        float angle = atan2(MainLane_intersectionpoint_houghP[0].y - MainLane_intersectionpoint_houghP[1].y, MainLane_intersectionpoint_houghP[0].x - MainLane_intersectionpoint_houghP[1].x);
//        angle = 180 * angle / CV_PI;
//        //cout << angle << endl;
//        if ((angle < -100 && angle > -160) || (angle > 100 && angle < 160))	// Angle limits
//        {
//            line(Main_Lane_NoBlur, MainLane_intersectionpoint_houghP[0], MainLane_intersectionpoint_houghP[1], Scalar(0, 0, 255), 3, CV_AA);
//        }
//    }
//
//    /************ END: HoughLinesP are required t0 get the images which have dashed Lanes *************/
//
//    // Calculation of hough transform for Side Lanes
//
//    CannyEdgeSeg_side(Sub_Lane, Calib_flag);
//
//    do
//    {
//        HoughLines(dest, SubLanes_Lines, 1, CV_PI / 180, hough_threshold_sub);
//        hough_threshold_sub = hough_threshold_sub + 10;					// The step update is 10 for the threshold
//
//        //cout << "SubLanes_Lines.size() = "<< SubLanes_Lines.size() << endl;
//
//    } while (SubLanes_Lines.size() > 10);
//
//    /*********	Reference: http://docs.opencv.org/trunk/d6/d6e/group__imgproc__draw.html#ga482fa7b0f578fcdd8a174904592a6250		  ********/
//
//    for (size_t i = 0; i < SubLanes_Lines.size(); i++)
//    {
//
//        float rho = SubLanes_Lines[i][0], theta = SubLanes_Lines[i][1];
//        // 73 to 78 will work for left lane detection but very crude// normal lane detction works with 55 to 60
//        if ((theta > CV_PI / 180 * 70 && theta < CV_PI / 180 * 80) || (theta > CV_PI / 180 * 120 && theta < CV_PI / 180 * 130))
//        {
//            double a = cos(theta), b = sin(theta);
//            double x0 = a*rho, y0 = b*rho;
//            pt1[Line_increment].x = cvRound(x0 + 1000 * (-b));
//            pt1[Line_increment].y = cvRound(y0 + 1000 * (a));
//            pt2[Line_increment].x = cvRound(x0 - 1000 * (-b));
//            pt2[Line_increment].y = cvRound(y0 - 1000 * (a));
//            line(src, pt1[Line_increment], pt2[Line_increment], Scalar(0, 0, 255), 3, CV_AA);
//            Line_increment++;
//        }
//    }
//
//    Merge_Function(Main_Lane_NoBlur, Sub_Lane);
//    return 0;
//
//    /********************************** Hough Transform End***********************************/
//
//}
//
///***************  Finds the intersection of two lines, or returns false. The lines are defined by (o1, p1) and (o2, p2). ***************/
///*************** http://stackoverflow.com/questions/7446126/opencv-2d-line-intersection-helper-function ***************/
//
//bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r)
//{
//    Point2f x = o2 - o1;
//    Point2f d1 = p1 - o1;
//    Point2f d2 = p2 - o2;
//
//    float cross = d1.x*d2.y - d1.y*d2.x;
//    if (abs(cross) < /*EPS*/1e-8)
//        return false;
//
//    double t1 = (x.x * d2.y - x.y * d2.x) / cross;
//    r = o1 + d1 * t1;
//    return true;
//}
//
///*************** END: Intersection function**************/
//
///******************* For merging of the image with the Lane to the normal image*****************/
//void Merge_Function(Mat& Main_Lane, Mat& Sub_Lane)
//{
//    int i = 0;
//    int j = 0;
//
//    if (Calib_flag == 0)
//    {
//        for (i = 0; i < Main_Lane.rows; i++)
//        {
//            for (j = 0; j < Main_Lane.cols; j++)
//            {
//                _merge.at<cv::Vec3b>(i + ROI_Y, j + ROI_X)[0] = Main_Lane.at<cv::Vec3b>(i, j)[0];
//                _merge.at<cv::Vec3b>(i + ROI_Y, j + ROI_X)[1] = Main_Lane.at<cv::Vec3b>(i, j)[1];
//                _merge.at<cv::Vec3b>(i + ROI_Y, j + ROI_X)[2] = Main_Lane.at<cv::Vec3b>(i, j)[2];
//            }
//
//        }
//    }
//    else
//    {
//        for (i = 0; i < Main_Lane.rows; i++)
//        {
//            for (j = 0; j < Main_Lane.cols; j++)
//            {
//                _merge.at<cv::Vec3b>(i + ROI_Y_NextFrame, j + ROI_X)[0] = Main_Lane.at<cv::Vec3b>(i, j)[0];
//                _merge.at<cv::Vec3b>(i + ROI_Y_NextFrame, j + ROI_X)[1] = Main_Lane.at<cv::Vec3b>(i, j)[1];
//                _merge.at<cv::Vec3b>(i + ROI_Y_NextFrame, j + ROI_X)[2] = Main_Lane.at<cv::Vec3b>(i, j)[2];
//            }
//
//        }
//    }
//    imshow("_merge", _merge);
//    waitKey(1);
//
//}
