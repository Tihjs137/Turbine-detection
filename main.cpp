/**
  @file     main.cpp
  @project  Turbine detection
  @author   Thijs de Jong
  @email    thijsdejong21@gmail.com
  @date     16th of May, 2019
*/



//Library headers
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>

//Class headers
#include "detection.h"

//Namespace decl.
using namespace std; 
using namespace cv; 




int main(int argc, char* argv[])
{
    //Init the Detector object
    Detector d;
    
    //Mat to store the frame(s)
    Mat frame;

    //Video source object
    VideoCapture cap;
    
    //Are we using video or an image?
    bool video;

    //When no arguments are given
    if( argc == 1)
    {
        cout << "Error in syntax \nUsage: main source(path to image / video) (video source) \n";
        return 0;
    }
    
    //Store the argument in "Parm"
    string Parm = argv[1]; 

    //When Parm == video; init video capture
    if(Parm == (char*)"video")
    {
        //Set the video selector
        video = true;
        cout <<"Using video capture" << endl;
        //--- INITIALIZE VIDEOCAPTURE
        

        // open the default camera using default API
        // cap.open(0);
        // OR advance usage: select any API backend
        
        int deviceID = 1;             // 0 = open default camera
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
        //Debug feature, confirm the image name
        cout << "Using image: " << Parm << "\n";
        
        //Set image selector
        video = false;

        // Read the file
        frame = imread(argv[1], CV_LOAD_IMAGE_COLOR);   

        // Check for invalid input
        if(! frame.data )                              
        {
            cout <<  "Could not open or find the image" << std::endl ;
            return -1;
        }
    }

    //--- GRAB AND WRITE LOOP
    cout << "Start grabbing" << endl
        << "Press any key to terminate" << endl;
    
   
    
    /* Rotation using rodrigues */
    

    // ======= Main Vision Loop =======
    for(;;)
    {
        //Video of image? 
        //Video will be loaded here. unless image is selected, then this statement will be ignored
        if(video) 
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
        

        //Run the detector (detect houghlinees)
        //d.detect(frame);
        
        //Run the PnP function
        d.locate(frame);

        //Run keypoint finder
        //d.detectKeys(video,frame);


        


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
