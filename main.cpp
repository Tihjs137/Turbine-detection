/**
  @file   main.cpp
  @brief  Turbine detection
  @author Thijs de Jong
  @email  thijsdejong21@gmail.com
  @date   14th of May, 2019
*/
#pragma once
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include "detection.h"

using namespace std; 
using namespace cv; 




int main(int argc, char** argv)
{
    Detector d;
    
    Mat frame;
    VideoCapture cap;
    bool video;


    if( argc == 1)
    {
        video = true;
        cout <<"Using video capture" << endl;
        //--- INITIALIZE VIDEOCAPTURE
        

        // open the default camera using default API
        // cap.open(0);
        // OR advance usage: select any API backend

        int deviceID = 0;             // 0 = open default camera
        int apiID = cv::CAP_ANY;      // 0 = autodetect default API

        // open selected camera using selected API
        cap.open(deviceID + apiID);

        // check if we succeeded
        if (!cap.isOpened()) 
        {
            cerr << "ERROR! Unable to open camera\n";
            return -1;
        }
    }
    else
    {
        cout << "Using image: " << argv[1] << "n";
        video = false;
        frame = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

        if(! frame.data )                              // Check for invalid input
        {
            cout <<  "Could not open or find the image" << std::endl ;
            return -1;
        }
    }

    //--- GRAB AND WRITE LOOP
    cout << "Start grabbing" << endl
        << "Press any key to terminate" << endl;
    

    // ======= Main Vision Loop =======
    for(;;)
    {
        if(video) //1 frame or frame feed?
        {
            // wait for a new frame from camera and store it into 'frame'
            cap.read(frame);
            // check if we succeeded
            if (frame.empty()) 
            {
                cerr << "ERROR! blank frame grabbed\n";
                break;
            }
        }
        
        
        
        
        

        //Run the detector
        d.detect(video, frame);
        

        //-------Wait key -----------
        if(!video)
        {
            for(;;)
            {
                if (waitKey(5) >= 0)
                    return 0;


            }

        }

        if (waitKey(5) >= 0)
        {
            return 0;
        }    
    }            

    
    // the camera will be deinitialized automatically in VideoCapture destructor
    
    return 0;
}
