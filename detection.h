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
//#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <stdio.h>

using namespace std; 
using namespace cv;
//using namespace cv::xfeatures2d; 

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
    };

    private: //Private variables
    

    
    int  bw2_Treshold;    
    int  bw4_Rho     ;    
    int  bw4_Theta   ;   
    int  bw4_Treshold;  
    int  bw4_srn     ; 
    int  bw4_stn     ;

    private: //Private functions

    

    
    public: //Public variables
  
    
    string name; //dummy var

    public: //Functions

    void printname() //dummy function
    { 
       cout << "Geekname is: " << name; 
    } 

    void locate()
    {   
        //C++: bool solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, 
        //          InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int flags=ITERATIVE )¶
        
        
        //Object points: Cirlce prefered set of points
        std::vector<Point2f> vec;
        // points or a circle
        for( int i = 0; i < 30; i++ )
            vec.push_back(Point2f((float)(100 + 30*cos(i*CV_PI*2/5)),
                                (float)(100 - 30*sin(i*CV_PI*2/5))));

        //Image points

        //camera Matrix

        //dist Coeffs

        //rvec

        //tvec


    }

    //Hello opencv, shows plain input
    void showVideo(Mat frame)
    {
            
        imshow("live",frame);

    }

    

    //Turbine detector
    void detect(bool video, Mat frame)
    {
        // show live and wait for a key with timeout long enough to show images
        //imshow("Live", frame);

        // -------- gray scaling ----------

        Mat BW1;
        cvtColor(frame, BW1, CV_BGR2GRAY);
        //imshow("bw1 grey", BW1);

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
        
        //imshow("bw3: skeleton", skel);

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
