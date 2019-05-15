/**
  @file   detection.h
  @brief  Turbine detection
  @author Thijs de Jong
  @email  thijsdejong21@gmail.com
  @date   15th of May, 2019
*/

#pragma once
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace std; 
using namespace cv;
using namespace cv::xfeatures2d; 

//====== Turbine detection class ========



class Detector 
{ 
    public:
    Detector() //Initializer
    {
        bw2_Treshold    = 180;
        bw4_Rho         = 1;
        bw4_Theta       = 180;
        bw4_Treshold    = 80;
        bw4_srn         = 30;
        bw4_stn         = 10;

        //create trackbars
        namedWindow(    "bw2 mask",1);
        createTrackbar( "bw2_Treshold",     "bw2 mask", &bw2_Treshold, 255, NULL );

        namedWindow(    "bw4: Hough lines",1);
        createTrackbar( "bw4_Rho",          "bw4: Hough lines", &bw4_Rho,       10, NULL );
        createTrackbar( "bw4_Theta",        "bw4: Hough lines", &bw4_Theta,     360, NULL );
        createTrackbar( "bw4_Treshold",     "bw4: Hough lines", &bw4_Treshold,  255, NULL );
        createTrackbar( "bw4_srn",          "bw4: Hough lines", &bw4_srn,       60, NULL );
        createTrackbar( "bw4_stn",          "bw4: Hough lines", &bw4_stn,       30, NULL );

        
        window_capture_name = "Video Capture";
        window_detection_name = "Object Detection";
        low_H = 0, low_S = 0, low_V = 0;
        high_H = max_value_H, high_S = max_value, high_V = max_value;

        namedWindow(window_detection_name);
        // Trackbars to set thresholds for HSV values
        createTrackbar("Low H" , window_detection_name, &low_H , max_value_H, NULL);
        createTrackbar("High H", window_detection_name, &high_H, max_value_H, NULL);
        createTrackbar("Low S" , window_detection_name, &low_S , max_value  , NULL);
        createTrackbar("High S", window_detection_name, &high_S, max_value  , NULL);
        createTrackbar("Low V" , window_detection_name, &low_V , max_value  , NULL);
        createTrackbar("High V", window_detection_name, &high_V, max_value  , NULL);
    };

    private: //Private variables
    
    
    
    int  bw2_Treshold;    
    int  bw4_Rho     ;    
    int  bw4_Theta   ;   
    int  bw4_Treshold;  
    int  bw4_srn     ; 
    int  bw4_stn     ;
    static const int max_value_H = 360/2;
    static const int max_value = 255;
    String window_capture_name;
    String window_detection_name;
    int low_H, low_S, low_V;
    int high_H, high_S , high_V;

    private: //Private functions

    

    
    public: //Public variables
  
    
    string name; //dummy var

    public: //Functions

    void printname() //dummy function
    { 
       cout << "Geekname is: " << name; 
    } 

    void locate(Mat frame)
    {   
        //C++: bool solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, 
        //          InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int flags=ITERATIVE )¶
        
        // show live and wait for a key with timeout long enough to show images
        imshow("Live", frame);

        // -------- gray scaling ----------

        


      

        Mat frame_HSV; Mat frame_threshold;
        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        imshow("HSV",frame_HSV);
        imshow(window_detection_name, frame_threshold);

        // Set up the detector with default parameters.
        
        
        // Detect blobs.
        std::vector<KeyPoint> keypoints;
        

        // Setup SimpleBlobDetector parameters.
        SimpleBlobDetector::Params params;
        
        // Change thresholds
        params.minThreshold = 0;
        params.maxThreshold = 255;
        
        // Filter by Area.
        params.filterByArea = false;
        params.minArea = 1500;
        
        // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = 0.8;
        
        // Filter by Convexity
        params.filterByConvexity = false;
        params.minConvexity = 0.87;
        
        // Filter by Inertia
        params.filterByInertia = false;
        params.minInertiaRatio = 0.01;
        
        Mat beforeBlob = frame_threshold < 200;

        cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params); 
        detector->detect( beforeBlob, keypoints );

        // Draw detected blobs as red circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        
        
        Mat im_with_keypoints;
        drawKeypoints( beforeBlob, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        
        // Show blobs
        imshow("keypoints", im_with_keypoints );

        for( int i = 0 ; i < keypoints.size(); i++)
        {
            cout << "Blob " << i << " : " << keypoints[i].pt << "\t";

        }
        
        cout << "\n\n";

        std::vector<cv::Point2d> image_points;

        if(keypoints.size() == 4 || true)
        {
            
            image_points.push_back( cv::Point2d(154, 213) );    // Nose tip
            image_points.push_back( cv::Point2d(292, 235) );    // Chin
            image_points.push_back( cv::Point2d(422, 133) );     // Left eye left corner
            image_points.push_back( cv::Point2d(290, 66) ); 
            image_points.push_back( cv::Point2d(293, 400) ); 

            // image_points.push_back( cv::Point2d(keypoints[0].pt.x, keypoints[0].pt.y) );    // Nose tip
            // image_points.push_back( cv::Point2d(keypoints[1].pt.x, keypoints[1].pt.y) );    // Chin
            // image_points.push_back( cv::Point2d(keypoints[2].pt.x, keypoints[2].pt.y) );     // Left eye left corner
            // image_points.push_back( cv::Point2d(keypoints[3].pt.x, keypoints[3].pt.y) ); 
            // image_points.push_back( cv::Point2d(keypoints[4].pt.x, keypoints[4].pt.y) ); 
            // 2D image points. If you change the image, you need to change vector
           // Right eye right corner
        //image_points.push_back( cv::Point2d(345, 465) );    // Left Mouth corner
        //image_points.push_back( cv::Point2d(453, 469) );    // Right mouth corner
        
        // 3D model points.
        std::vector<cv::Point3d> model_points;
        model_points.push_back(cv::Point3d(  0.0f,  0.0f, 0.0f));  //Center           
        model_points.push_back(cv::Point3d(  0.0f,-40.0f, 0.0f));  //Bottom         
        model_points.push_back(cv::Point3d(-20.0f,-10.0f, 0.0f));  //left bottom    
        model_points.push_back(cv::Point3d(-20.0f, 20.0f, 0.0f));  //left top
        model_points.push_back(cv::Point3d( 25.0f,  0.0f, 0.0f));  //right      
        //model_points.push_back(cv::Point3d(-150.0f, -150.0f, -125.0f));      // Left Mouth corner
        //model_points.push_back(cv::Point3d(150.0f, -150.0f, -125.0f));       // Right mouth corner
        
        // Camera internals
        double focal_length = frame.cols; // Approximate focal length.
        Point2d center = cv::Point2d(frame.cols/2,frame.rows/2);
        cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
        cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion
        
        cout << "Camera Matrix " << endl << camera_matrix << endl ;
        // Output rotation and translation
        cv::Mat rotation_vector; // Rotation in axis-angle form
        cv::Mat translation_vector;
        
        // Solve for pose
        cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
    
        
        // Project a 3D point (0, 0, 1000.0) onto the image plane.
        // We use this to draw a line sticking out of the nose
        
        vector<Point3d> nose_end_point3D;
        vector<Point2d> nose_end_point2D;
        nose_end_point3D.push_back(Point3d(100,0,0));
        nose_end_point3D.push_back(Point3d(0,100,0));
        nose_end_point3D.push_back(Point3d(0,0,100));
        
        projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
        
        
        for(int i=0; i < image_points.size(); i++)
        {
            circle(frame, image_points[i], 3, Scalar(0,0,255), -1);
        }
        
        cv::line(frame,image_points[0], nose_end_point2D[0], cv::Scalar(255,0,0), 2);
        cv::line(frame,image_points[0], nose_end_point2D[1], cv::Scalar(0,255,0), 2);
        cv::line(frame,image_points[0], nose_end_point2D[2], cv::Scalar(0,0,255), 2);
        
        cout << "Rotation Vector " << endl << rotation_vector << endl;
        cout << "Translation Vector" << endl << translation_vector << endl;
        
        cout <<  nose_end_point2D << endl;
        
        // Display image.
        
        }

        cv::imshow("Output", frame);

    }

    //Hello opencv, shows plain input
    void showVideo(Mat frame)
    {
            
        imshow("live",frame);

    }

    //Turbine detector using key points
    void detectKeys(bool video, Mat frame)
    {

        // show live and wait for a key with timeout long enough to show images
        imshow("Live", frame);

        // -------- gray scaling ----------

        Mat BW1;
        cvtColor(frame, BW1, CV_BGR2GRAY);
        imshow("bw1 grey", BW1);

        // -------- Masking ----------
        Mat BW2 = BW1 > bw2_Treshold;
    
        imshow("bw2 mask", BW2);

        // ---------Detecting keypoints ----

        //-- Step 1: Detect the keypoints using SURF Detector
        int minHessian = 400;
        
        Ptr<SURF> detector = SURF::create( minHessian );
        
        std::vector<KeyPoint> keypoints_1;
        
        detector->detect( BW2, keypoints_1 );
        
        //-- Draw keypoints
        Mat img_keypoints_1;
        drawKeypoints( BW2, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
        
        //-- Show detected (drawn) keypoints
        imshow("Keypoints 1", img_keypoints_1 );

    }

    //Turbine detector
    void detect(bool video, Mat frame)
    {
        // show live and wait for a key with timeout long enough to show images
        imshow("Live", frame);

        // -------- gray scaling ----------

        Mat BW1;
        cvtColor(frame, BW1, CV_BGR2GRAY);
        imshow("bw1 grey", BW1);

        // -------- Masking ----------
        Mat BW2 = BW1 > bw2_Treshold;
    
        imshow("bw2 mask", BW2);


        // -------- Skeleton ----------

        Mat skel(BW2.size(), CV_8UC1, Scalar(0));
        Mat temp;
        Mat eroded;
        Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
        Mat BW3 = BW2;
        bool done;
        int iterations=0;

        do
        {
        erode(BW3, eroded, element);
        dilate(eroded, temp, element);
        subtract(BW3, temp, temp);
        bitwise_or(skel, temp, skel);
        eroded.copyTo(BW3);

        done = (countNonZero(BW3) == 0);
        iterations++;
        
        } while (!done && (iterations < 100));

        //cout << "iterations=" << iterations << endl;
        
        imshow("bw3: skeleton", skel);

        //-------Hough lines----------//
        //C++: void HoughLines(InputArray image, OutputArray lines, double rho, double theta, int threshold, double srn=0, double stn=0 )

        Mat BW4 = frame;
        vector<Vec4i> lines;
        int Xmax = frame.size[0]/2;
        int Xmin = frame.size[0]/2;
        int Ymax = frame.size[1]/2;
        int Ymin = frame.size[1]/2;

        Rect boundingBox;

        HoughLinesP( skel, lines, bw4_Rho, CV_PI/bw4_Theta, bw4_Treshold, bw4_srn, bw4_stn );
        cout << "BW4 \t Found hough lines: \n";
        for( size_t i = 0; i < lines.size(); i++ )
        {
            line( BW4, Point(lines[i][0], lines[i][1]),
                Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
            
            

            //Create bounding box
            if (Xmax < lines[i][0] || Xmax < lines[i][2] )
            {
                if( lines[i][0] > lines[i][2])
                {
                    Xmax = lines[i][0];
                }
                else
                {
                    Xmax = lines[i][2];
                }
            }
            if (Xmin > lines[i][0] || Xmin > lines[i][2] )
            {
                if( lines[i][0] < lines[i][2])
                {
                    Xmin = lines[i][0];
                }
                else
                {
                    Xmin = lines[i][2];
                }
            }
            if (Ymax < lines[i][1] || Ymax < lines[i][3] )
            {
                if( lines[i][1] > lines[i][3])
                {
                    Ymax = lines[i][1];
                }
                else
                {
                    Ymax = lines[i][3];
                }
            }
            if (Ymin > lines[i][1] || Ymin > lines[i][3] )
            {
                if( lines[i][1] < lines[i][3])
                {
                    Ymin = lines[i][1];
                }
                else
                {
                    Ymin = lines[i][3];
                }
            }
            
            boundingBox.x       = Xmin;
            boundingBox.y       = Ymin;
            boundingBox.width   = Xmax - Xmin; 
            boundingBox.height  = Ymax - Ymin;
            
            float  slope = atan( (float)( (int)lines[i][3] - (int)lines[i][1]) / (float)((int)lines[i][2] - (int)lines[i][0]) ) * (180 / 3.1415);


            cout << "BW4 \t Line: " << i << "\t P1_x, P1_y: (" << lines[i][0] << "," << lines[i][1] << ") \t P2_x, P2_y: ";
            cout << lines[i][2] << "," << lines[i][3]  << "\t Slope: " << slope << "\n";
        }

        cout << "BW4 \t box: " << 1 << "\t Xmin, Xmax: (" << Xmin << "," << Xmax << ") \t Ymin,Ymax: (" << Ymin << "," << Ymax  << ")\n";
        rectangle(BW4, boundingBox, Scalar( 0, 255, 255 ));
        imshow("bw4: Hough lines", BW4);
        //Mat& img, Rect rec, const Scalar& color,

    }    

}; 
